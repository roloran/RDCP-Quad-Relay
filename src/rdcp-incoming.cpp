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
#include "lorawan-tunnel.h"
#include "rdcp-roaming-support.h"
#include "rdcp-dupetable.h"
#include "rdcp-callbacks.h"

lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
extern da_config CFG;
extern runtime_da_data DART;

int32_t bad_crc_counter;
uint16_t last_origin[NUMCHANNELS] = { RDCP_ADDRESS_SPECIAL_ZERO, RDCP_ADDRESS_SPECIAL_ZERO, RDCP_ADDRESS_SPECIAL_ZERO, RDCP_ADDRESS_SPECIAL_ZERO };
uint16_t last_seqnr[NUMCHANNELS]  = { RDCP_SEQUENCENR_SPECIAL_ZERO, RDCP_SEQUENCENR_SPECIAL_ZERO, RDCP_SEQUENCENR_SPECIAL_ZERO, RDCP_SEQUENCENR_SPECIAL_ZERO };
bool currently_in_fetch_mode = false;
char serial_info[INFOLEN];
extern int64_t last_heartbeat_sent;
int64_t timestamp_last_hqdev_seen_via_ep = RDCP_TIMESTAMP_ZERO;

extern callback_chain CC[NUM_TX_CALLBACKS];

void rdcp_handle_incoming_lora_message(void)
{
    cpu_fast();

    int64_t now = my_millis();
    
    if (current_lora_message.channel == CHANNEL868LW)
    {
        /* LoRa packets received on this channel must be LoRaWAN traffic, not RDCP Messages. */
        lorawan_tunnel_incoming();
        return;
    }
    
    /* Check whether it could be an RDCP message at all */
    if (current_lora_message.payload_length < RDCP_HEADER_SIZE)
    {
        serial_writeln("INFO: LoRa packet too small - not an RDCP message, not processing");
        // NB: Received non-RDCP LoRa packets have no influence on CFEst, so there is nothing else to do.
        return;
    }
    if (current_lora_message.payload_length > RDCP_MAX_LORA_PAYLOAD_SIZE)
    {
        serial_writeln("INFO: LoRa packet too large - not an RDCP message, not processing");
        return;
    }

    /* Copy the message to process into the rdcp_msg_in data structure */
    memset(&rdcp_msg_in.payload.data, 0, RDCP_MAX_INNER_PAYLOAD_SIZE);
    memcpy(&rdcp_msg_in.header, &current_lora_message.payload, RDCP_HEADER_SIZE);
    for (int i=RDCP_HEADER_SIZE; i<current_lora_message.payload_length; i++) rdcp_msg_in.payload.data[i-RDCP_HEADER_SIZE] = current_lora_message.payload[i];
    
    /* Verify the CRC-16 checksum */
    if (!rdcp_check_crc_in(current_lora_message.payload_length))
    {
        serial_writeln("INFO: RDCP checksum mismatch - not processing");
        // NB: Any RDCP Header or RDCP Payload field may have been corrupted,
        //     so we do not process anything further, including updates to CFEst.
        bad_crc_counter++;
        if (bad_crc_counter % BAD_CRC_COUNTER_THRESHOLD == 0)
        {
            /* 
                Bad CRC usually is the result of poor reception or other devices sending 
                non-RDCP LoRA packets on the same channel. However, we re-initialize our 
                LoRa radios every now and then in case it might be hardware-related. 
            */
            serial_writeln("WARNING: Bad CRC counter exceeded threshold - consider additional countermeasures!");
            bad_crc_counter = COUNT_ZERO;
            reset_radio(current_lora_message.channel);
        }        
        return;
    }

    if (rdcp_msg_in.header.rdcp_payload_length > RDCP_MAX_INNER_PAYLOAD_SIZE)
    {
        serial_writeln("WARNING: Stated RDCP payload length exceeds maximum allowed size, ignoring");
        return;
    }

    if ((rdcp_msg_in.header.rdcp_payload_length + RDCP_HEADER_SIZE) != current_lora_message.payload_length)
    {
        serial_writeln("WARNING: Mismatch between LoRa packet length and RDCP header information, ignoring");
        return;
    }

    /* Completely ignore selected messages on selected channels */
    if (current_lora_message.channel == CHANNEL868DA)
    {
        /* Ignore area-local messages such as DA-ACKs and Roaming Beacons sent by other DAs on CHANNEL868DA */
        if ((rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ROAMINGBEACON) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK))
        {
            if ((rdcp_msg_in.header.origin >= RDCP_ADDRESS_BBKDA_LOWERBOUND) && (rdcp_msg_in.header.origin < RDCP_ADDRESS_MG_LOWERBOUND))
            {
                if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK) print_rdcp_csv(); // preserve telemetry data for DA ACKs
                return;
            }
        }
        /*
            MGs are expected to uplink to us on the other channel, CHANNEL868MG, not here on CHANNEL868DA.
            Thus, we want to strictly ignore HQ devices sending on the wrong channel, and mostly ignore other MGs
            on this wrong channel as well, but still shadow-forward or EP-process legacy MGs sending a CIRE.
        */
       if ((rdcp_msg_in.header.origin == rdcp_msg_in.header.sender) && 
           (rdcp_msg_in.header.origin <= RDCP_ADDRESS_HQ_UPPERBOUND))
        {
            serial_writeln("WARNING: HQ device is sending an uplink message on downlink channel, ignoring");
            print_rdcp_csv(); // preserve telemetry data
            return;
        }
       if ((rdcp_msg_in.header.origin == rdcp_msg_in.header.sender) && 
           (rdcp_msg_in.header.origin >= RDCP_ADDRESS_MG_LOWERBOUND))
        {
            if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT)
            {
                serial_writeln("WARNING: Legacy MG device sends CIRE on downlink channel instead of uplink channel. Processing it."); 
            }
            else 
            {
                serial_writeln("WARNING: Legacy MG device is sending a non-CIRE uplink message on downlink channel, ignoring");
                print_rdcp_csv(); // preserve telemetry data
                return;
            }
        }
    }
    if (current_lora_message.channel == CHANNEL868MG)
    {
        /*
          To avoid RDCP Message reordering issues for messages by HQ devices, we ignore them unless we are either
          the designated entry point or we deem shadow-forwarding them necessary based on the assumption that the
          HQ has not chosen a working entry point.
        */
        if ((rdcp_msg_in.header.origin == rdcp_msg_in.header.sender) && // received directly from origin and 
            (rdcp_msg_in.header.origin <= RDCP_ADDRESS_HQ_UPPERBOUND))  // origin presumably is an HQ device
        {
            if (rdcp_check_entrypoint_designation() == false) // we are NOT the designated entry point for this message
            {
                if (now - timestamp_last_hqdev_seen_via_ep > TIMEOUT_ASSUME_NONWORKING_EP_FOR_HQ * MINUTES_TO_MILLISECONDS)
                { // We have not seen a properly forwarded message for a too long time, so we proceed as usual
                    serial_writeln("INFO: Assuming that RDCP Message from HQ device requires shadow propagation");
                }
                else 
                { // Proper forwarding of HQ messages appears to work fine, so we completely ignore the received messages 
                  // by returning from this function, without registering the message for dupe table/SPW and further processing it.
                    serial_writeln("INFO: Skipping shadow propagation for HQ message, expecting proper propagation following");
                    print_rdcp_csv(); // preserve telemetry data
                    return;
                }
            }
        }
    }

    /* Register RDCP Messages sent by an HQ device as Origin that are properly forwarded on 433 MHz */
    if (current_lora_message.channel == CHANNEL433)
    {
        if (rdcp_msg_in.header.origin <= RDCP_ADDRESS_HQ_UPPERBOUND)
        {
            timestamp_last_hqdev_seen_via_ep = now;
        }
    }

    /* Check for Red CIRE Button activity */
    if (CFG.my_cire_button > RDCP_ADDRESS_SPECIAL_ZERO)
    if ((rdcp_msg_in.header.origin == CFG.my_cire_button) || (rdcp_msg_in.header.destination == CFG.my_cire_button))
    {
        if ((rdcp_msg_in.header.origin == CFG.my_cire_button) && (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT))
        {
            serial_writeln("INFO: Red CIRE Button associated with this MERLIN-Base has sent a CIRE!");
            serial_writeln("DA_CIREBUTTON_PRESSED");
        }
        if ((rdcp_msg_in.header.destination == CFG.my_cire_button) && (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK))
        {
            if (rdcp_msg_in.header.origin <= RDCP_HQ_MULTICAST_ADDRESS)
            {
                serial_writeln("INFO: Red CIRE Button associated with this MERLIN-Base has received an ACK from HQ!");
                serial_writeln("DA_CIREBUTTON_HQACK");
            }
        }
    }

    /* Update the CFEst since we received an RDCP Message; parameters for 433 MHz propagation cycle tracking only */
    rdcp_update_cfest_in(rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number);

    /* 
        Stop any TX events on the current channel as long as it is busy. 
        However, approximate limiting rescheduling to only once for each RDCP Message 
        and not for each of its retransmissions. Thus, if we have seen the same Origin 
        and SeqNr just before, do not reschedule again. 
        Rescheduling this way will be based on the offset "CFEst - now()", so it 
        will converge to zero when called repeatedly for alternating RDCP Message 
        retransmissions. In such cases of indeterministic propagation cycle clashes,
        backing off from the channel may contribute to resolving the clash, and 
        TX Queue compression will step in when the channel is free for some time.
        Note that in our multi-channel setup, this simplified check is not 
        equivalent to duplicate checking, as we need to reschedule on a per-channel
        and not a per-message basis.
    */
    if ((rdcp_msg_in.header.origin == last_origin[current_lora_message.channel]) && 
        (rdcp_msg_in.header.sequence_number == last_seqnr[current_lora_message.channel]))
    {
        /* Do not reschedule again */
    }
    else
    {
        last_origin[current_lora_message.channel] = rdcp_msg_in.header.origin;
        last_seqnr[current_lora_message.channel] = rdcp_msg_in.header.sequence_number;
        rdcp_reschedule_on_busy_channel(current_lora_message.channel);
    }

    /* RDCPCSV output */
    print_rdcp_csv();

    /* TXQueue clean-up */
    rdcp_txqueue_clean();

    /* Check the RDCP Message duplicate status */
    bool duplicate = rdcp_check_duplicate_message(rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number);

    /* If we are currently sending periodics, stop doing so if we hear a CIRE or ACK going on. */
    if (CC[TX_CALLBACK_PERIODIC868].in_use)
    {
        if ((rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT) ||
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK))
            {
                serial_writeln("INFO: Interrupting periodics chain due to ongoing CIRE/ACK");
                rdcp_chain_callback(TX_CALLBACK_PERIODIC868, true);
            }
    }

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
    if ((current_lora_message.channel == CHANNEL868MG) && 
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
        roaming_support_register_mg_heartbeat();
    }
    rdcp_neighbor_register_rx(current_lora_message.channel, rdcp_msg_in.header.sender, 
                              current_lora_message.rssi, current_lora_message.snr, 
                              current_lora_message.timestamp, heartbeat, 
                              explicit_refnr, latest_refnr, roamingrec);

    /* Only perform the following actions if the message is not a duplicate */
    if (!duplicate)
    {
        DART.num_rdcp_rx++;

        if (!rdcp_relay_allowed_for_device(rdcp_msg_in.header.origin))
        { /* Avoid forwarding and shadow-propagation for blocked devices even as non-EP */
            serial_writeln("WARNING: Not processing message from blocked device origin any further");
            return;
        }

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
        else /* RDCP Message received on CHANNEL868DA or CHANNEL868MG */
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
                        rdcp_txqueue_reschedule(CHANNEL868DA, CFG.corridor_basetime * SECONDS_TO_MILLISECONDS);
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
                    Thus, we have to forward in on 868 MHz (including HQ) and to our DA here. 
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
                    { 
                        int64_t cfest_max = rdcp_get_channel_free_estimation(CHANNEL868DA); 
                        now = my_millis(); // refresh timestamp
                        if (now > cfest_max) cfest_max = now;
                        rdcp_update_channel_free_estimation(CHANNEL868DA, cfest_max + CFG.corridor_basetime * SECONDS_TO_MILLISECONDS);
                        rdcp_txqueue_reschedule(CHANNEL868DA, 0);
                    }
                    rdcp_forward_schedule(FORWARD_DELAY_PROPORTIONAL); // add a delay
                }
                else if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_HEARTBEAT)
                { // Forward Heartbeats only if their origin is another DA, not an MG
                    if ((rdcp_msg_in.header.origin >= RDCP_ADDRESS_BBKDA_LOWERBOUND) &&
                        (rdcp_msg_in.header.origin < RDCP_ADDRESS_MG_LOWERBOUND)) 
                        rdcp_forward_schedule(FORWARD_DELAY_PROPORTIONAL); // add a delay
                }
                if (rdcp_check_forward_da_relevance()) rdcp_msg_to_da_via_serial();

                /* 
                    If we hear a DA sending an ACK, consider a full CIRE-ACK-loop in progress 
                    and avoid sending trivial messages such as Heartbeats.
                */
                if ((rdcp_msg_in.header.origin >= RDCP_ADDRESS_BBKDA_LOWERBOUND) &&
                    (rdcp_msg_in.header.origin < RDCP_ADDRESS_MG_LOWERBOUND) && 
                    (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK))
                {
                    last_heartbeat_sent += 10 * MINUTES_TO_MILLISECONDS;
                }
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