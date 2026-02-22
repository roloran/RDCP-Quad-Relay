#include "rdcp-incoming.h"
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-entrypoint.h"
#include "rdcp-blockdevice.h"
#include "rdcp-scheduler.h"
#include "rdcp-forward.h"
#include "rdcp-neighbors.h"
#include "rdcp-memory.h"
#include "rdcp-commands.h"
#include "rdcp-csv.h"

lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
extern da_config CFG;
extern runtime_da_data DART;

int32_t bad_crc_counter;
uint16_t last_origin[NUMCHANNELS] = { RDCP_ADDRESS_SPECIAL_ZERO, RDCP_ADDRESS_SPECIAL_ZERO };
uint16_t last_seqnr[NUMCHANNELS]  = { RDCP_SEQUENCENR_SPECIAL_ZERO, RDCP_SEQUENCENR_SPECIAL_ZERO };
bool currently_in_fetch_mode = false;
char serial_info[INFOLEN];

void rdcp_handle_incoming_lora_message(void)
{
    cpu_fast(); 
    
    /* Check whether it could be an RDCP message at all */
    if (current_lora_message.payload_length < RDCP_HEADER_SIZE)
    {
        serial_writeln("INFO: LoRa packet too small - not an RDCP message, not processing");
        // NB: Received non-RDCP LoRa packets have no influence on CFEst, so there is nothing else to do.
        return;
    }

    /* Copy the message to process into the rdcp_msg_in data structure */
    memcpy(&rdcp_msg_in.header, &current_lora_message.payload, RDCP_HEADER_SIZE);
    for (int i=RDCP_HEADER_SIZE; i<current_lora_message.payload_length; i++) rdcp_msg_in.payload.data[i-RDCP_HEADER_SIZE] = current_lora_message.payload[i];
    
    /* Verify the CRC-16 checksum */
    if (!rdcp_check_crc_in(current_lora_message.payload_length))
    {
        serial_writeln("INFO: RDCP checksum mismatch - not processing");
        // NB: Any RDCP Header or RDCP Payload field may have been corrupted,
        //     so we do not process anything further, including updates to CFEst.
        bad_crc_counter++;
        if (bad_crc_counter % 100 == 0)
        {
            /* 
                Bad CRC usually is the result of poor reception or other devices sending 
                non-RDCP LoRA packets on the same channel. However, we re-initialize our 
                LoRa radios every now and then in case it might be hardware-related. 
            */
            serial_writeln("WARNING: Bad CRC counter exceeded threshold - consider additional countermeasures!");
            setup_radio();
        }
        return;
    }

    /* Update the CFEst since we received an RDCP Message */
    rdcp_update_cfest_in(rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number);

    /* Stop any TX events on the current channel as long as it is busy */
    if ((rdcp_msg_in.header.origin != last_origin[current_lora_message.channel]) && (rdcp_msg_in.header.sequence_number != last_seqnr[current_lora_message.channel]))
    {
        last_origin[current_lora_message.channel] = rdcp_msg_in.header.origin;
        last_seqnr[current_lora_message.channel] = rdcp_msg_in.header.sequence_number;
        rdcp_reschedule_on_busy_channel(current_lora_message.channel);
    }

    /* RDCPCSV output */
    print_rdcp_csv();

    /* Check the RDCP Message duplicate status */
    bool duplicate = rdcp_check_duplicate_message(rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number);

    /* On the 433 MHz channel, we may be a designated relay even if it is a duplicate */
    if (current_lora_message.channel == CHANNEL433)
    {
        /* If it is a duplicate, we need to remember whether we already relayed the 
           message. We relay it at most once. */
        int relay_delay = rdcp_check_relay_designation();
        if (relay_delay > -1)
        {
            /* 
                We could add further checks here whether the message should really be relayed, 
                e.g., based on Message Type (e.g., Fetch) and Origin (e.g., blocked device). 
                However, for now we assume that we are only assigned as relay if we should do so.
            */
            if (!rdcp_check_has_already_relayed()) 
            {
                rdcp_schedule_relayed_message(relay_delay);
                DART.num_rdcp_tx++;
            }
        }
    }

    /* On the 868 MHz channel, we have to process MG HEARTBEAT messages (always dupes). */
    /* We also register Sender-based RSSI and SNR values for any RDCP Messages. */
    bool heartbeat = false;
    bool explicit_refnr = false;
    uint16_t latest_refnr = RDCP_OA_REFNR_SPECIAL_ZERO;
    uint16_t roamingrec = RDCP_ADDRESS_SPECIAL_ZERO;
    if ((current_lora_message.channel == CHANNEL868) && 
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_HEARTBEAT) && 
        (rdcp_msg_in.header.sequence_number == RDCP_SEQUENCENR_SPECIAL_ZERO) &&
        (rdcp_msg_in.header.origin == rdcp_msg_in.header.sender))
    { 
        heartbeat = true;
        if (rdcp_msg_in.header.rdcp_payload_length == RDCP_PAYLOAD_SIZE_MG_HEARTBEAT)
        {
            explicit_refnr = true; 
            latest_refnr = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];
            roamingrec   = rdcp_msg_in.payload.data[2] + 256 * rdcp_msg_in.payload.data[3];
        }
    }
    rdcp_neighbor_register_rx(current_lora_message.channel, rdcp_msg_in.header.sender, 
                              current_lora_message.rssi, current_lora_message.snr, 
                              current_lora_message.timestamp, heartbeat, 
                              explicit_refnr, latest_refnr, roamingrec);

    /* Only perform the following actions if the message is not a duplicate */
    if (!duplicate)
    {
        DART.num_rdcp_rx++;

        if (current_lora_message.channel == CHANNEL433)
        {
            if (rdcp_check_forward_868_relevance()) 
            {   
                rdcp_forward_schedule(FORWARD_DELAY_PROPORTIONAL);
            }
            else 
            {
                serial_writeln("INFO: Message received on 433 MHz channel not relevant for 868-forwarding");
            }
            if (rdcp_check_forward_da_relevance()) rdcp_msg_to_da_via_serial();
        }
        else /* RDCP Message received on CHANNEL868 */
        {
            /* Forward the RDCP Message on 433 MHz if we are the Entry Point
               unless there are reasons not to forward it. */
            if (rdcp_check_entrypoint_designation())
            {
                if (rdcp_check_entrypoint_messagetype_valid() && 
                    rdcp_relay_allowed_for_device(rdcp_msg_in.header.origin))
                {
                    /* If it is a CIRE, we also have to send an ACK back to the MG */
                    if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT)
                    {
                        // Re-schedule other entries on 868 MHz so we get the ACK out first 
                        rdcp_txqueue_reschedule(CHANNEL868, CFG.corridor_basetime * SECONDS_TO_MILLISECONDS);
                        rdcp_send_ack_unsigned(CFG.rdcp_address, rdcp_msg_in.header.origin, 
                                               rdcp_msg_in.header.sequence_number);
                    }

                    /* Forward the message on the 433 MHz channel unless we are the destination */
                    if (rdcp_msg_in.header.destination != CFG.rdcp_address)
                    { 
                        rdcp_entrypoint_schedule();
                        DART.num_rdcp_tx++;
                    
                        /* 
                            If the HQ used us as entry point, send the message also on the 868 MHz channel 
                            so everyone in our area gets it even if they do not hear the HQ device directly.
                        */
                        if ((rdcp_msg_in.header.sender < RDCP_ADDRESS_BBKDA_LOWERBOUND) &&
                            (rdcp_msg_in.header.message_type != RDCP_MSGTYPE_HEARTBEAT)) // don't echo back heartbeats
                        {
                                rdcp_forward_schedule(FORWARD_DELAY_SHORT);
                        }
                        /*
                            The same applies to messages sent by other MGs so they reach the HQ on 868 MHz
                            if it is in our area. 
                        */
                        if ((rdcp_msg_in.header.sender >= RDCP_ADDRESS_MG_LOWERBOUND) &&
                            (rdcp_msg_in.header.message_type != RDCP_MSGTYPE_HEARTBEAT)) // don't echo back heartbeats
                        {
                            if (rdcp_check_forward_868_relevance()) 
                                rdcp_forward_schedule(FORWARD_DELAY_SHORT);
                        }
                    }

                    /* Forward relevant messages to the DA even if we did not broadcast it. */
                    if (rdcp_check_forward_da_relevance()) rdcp_msg_to_da_via_serial();
                }
            }
            else 
            {
                /*
                    We are not the designated Entry Point but we got a new (not duplicate) message 
                    on 868 MHz first. If we receive it later on 433 MHz, we will consider it a 
                    duplicate. While we still may relay it then, we would not forward it on 868 MHz. 
                    Thus, we have to forward in on 868 MHz and to our DA here. 
                */
                if (rdcp_check_forward_868_relevance() &&
                    (rdcp_msg_in.header.message_type != RDCP_MSGTYPE_HEARTBEAT)) // don't shadow-forward heartbeats
                { 
                    /* 
                        If the received message is a CIRE sent by an MG, we need to keep the 
                        868 MHz channel free so the EP has a chance to send its ACK with
                        priority; thus, consider the channel busy longer. 
                    */
                    if ((rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT) && 
                        (rdcp_msg_in.header.sender >= RDCP_ADDRESS_MG_LOWERBOUND))
                    { // was 2* corridor_basetime
                        rdcp_update_channel_free_estimation(CHANNEL868, rdcp_get_channel_free_estimation(CHANNEL868) + CFG.corridor_basetime * SECONDS_TO_MILLISECONDS);
                        rdcp_txqueue_reschedule(CHANNEL868, 0); // re-schedule based on CFEst
                    }
                    rdcp_forward_schedule(FORWARD_DELAY_PROPORTIONAL); // add a delay
                }
                else if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_HEARTBEAT)
                { // Forward Heartbeats only if their origin is another DA, not an MG
                    if ((rdcp_msg_in.header.origin >= RDCP_ADDRESS_BBKDA_LOWERBOUND) &&
                        (rdcp_msg_in.header.origin <= RDCP_ADDRESS_MG_LOWERBOUND)) 
                        rdcp_forward_schedule(FORWARD_DELAY_PROPORTIONAL); // add a delay
                }
                if (rdcp_check_forward_da_relevance()) rdcp_msg_to_da_via_serial();
            }
        }

        /* If we are the destination of a message, fulfill relevant requests */
        if (rdcp_matches_any_of_my_addresses(rdcp_msg_in.header.destination))
        {
           serial_writeln("INFO: Handling incoming command");
           rdcp_handle_command();
        }

        /* Store relevant RDCP Messages for periodic transmission on 868 MHz */
        if ((rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE))
        {
            if ((rdcp_msg_in.header.destination == RDCP_BROADCAST_ADDRESS) || 
                ((rdcp_msg_in.header.destination >= RDCP_ADDRESS_MULTICAST_LOWERBOUND) && (rdcp_msg_in.header.destination <= RDCP_ADDRESS_MULTICAST_UPPERBOUND))) 
            {
                rdcp_memory_remember();
            }
        }
    }
    else
    { // RDCP Message is a duplicate, but certain messages must be processed even then.
        if (
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_RESET) ||
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_REBOOT) ||
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_INFRASTRUCTURE_RESET)
        )
        {
            if (rdcp_matches_any_of_my_addresses(rdcp_msg_in.header.destination))
            {
                serial_writeln("INFO: Fulfilling request despite sent via duplicate RDCP message");
                rdcp_handle_command();
            }    
        }
        else if ((currently_in_fetch_mode) && (current_lora_message.channel == CHANNEL433))
        {
            if ((rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || 
                (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE))
            {
                rdcp_memory_remember();
                rdcp_msg_to_da_via_serial();
            }    
        }
        else 
        {
            snprintf(serial_info, INFOLEN, "INFO: Ignoring duplicate %04X-%04X sent by %04X", rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number, rdcp_msg_in.header.sender);
            serial_writeln(serial_info);
        }
    }

    return;
}

/* EOF */