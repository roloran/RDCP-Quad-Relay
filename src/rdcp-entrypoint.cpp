#include "rdcp-incoming.h"
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-entrypoint.h"
#include "rdcp-blockdevice.h"
#include "rdcp-scheduler.h"

extern rdcp_message rdcp_msg_in;
extern da_config CFG;

bool rdcp_check_entrypoint_designation(void)
{
    /*
       We accept messages as Entry Point if 
         - they were received on the 868 MHz channel (this function is not called otherwise)
         - Sender equals Origin
         - Sender has an RDCP Address in the MG or HQ range 
         - Header field Relay1 designates us as Relay with Delay 0
         - Header fields Relay2 and Relay3 are 0xEE each 
    */
    if (rdcp_msg_in.header.origin == rdcp_msg_in.header.sender)
    {
        if ( 
            ((rdcp_msg_in.header.sender >= RDCP_ADDRESS_MG_LOWERBOUND) && 
             (rdcp_msg_in.header.sender <= RDCP_ADDRESS_MG_UPPERBOUND))
            ||
            ((rdcp_msg_in.header.sender >= RDCP_ADDRESS_HQ_LOWERBOUND) && 
             (rdcp_msg_in.header.sender <= RDCP_ADDRESS_HQ_UPPERBOUND))
        )
        {
            if ((rdcp_msg_in.header.relay2 == RDCP_HEADER_RELAY_MAGIC_EP) && 
                (rdcp_msg_in.header.relay3 == RDCP_HEADER_RELAY_MAGIC_EP))
            {
                uint8_t my_designation = (CFG.relay_identifier << 4);
                if (my_designation == rdcp_msg_in.header.relay1)
                {
                    char output[INFOLEN];
                    snprintf(output, INFOLEN, "INFO: Positive entry point designation check for %04X-%04X", 
                        rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number);
                    serial_writeln(output);
                    return true;
                }
            }
        }
    }

    return false;
}

bool rdcp_check_entrypoint_messagetype_valid(void)
{
    /*
        We have to accept pretty much any message type as Entry Point 
        because also the HQ sends its messages on the 868 MHz channel. 
        However, a few message types should not be sent with designated 
        Entry Points on 868 MHz so we can reject them here. 
        It would also be good to use allow-listing instead of block-listing 
        and to filter unknown message types, so this can be added later.
    */
    if (
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_HEARTBEAT) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DA_STATUS_RESPONSE) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_FETCH_ALL_NEW_MESSAGES) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DELIVERY_RECEIPT)
       )
    {
        serial_writeln("WARNING: Rejected being Entry Point based on Message Type");
        return false;
    }

    return true;
}

void rdcp_entrypoint_schedule(void)
{
    /* Prepare outgoing message by copying the received one */
    rdcp_message r;
    memcpy(&r, &rdcp_msg_in.header, RDCP_HEADER_SIZE);
    for (int i=0; i<r.header.rdcp_payload_length; i++) r.payload.data[i] = rdcp_msg_in.payload.data[i];

    /* Update header fields for the outgoing message */
    r.header.sender = CFG.rdcp_address;
    r.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(r.header.message_type);

    uint8_t my_relay1 = CFG.oarelays[0]; // Default direction is to spread in the mesh
    uint8_t my_relay2 = CFG.oarelays[1];
    uint8_t my_relay3 = CFG.oarelays[2];
    if (r.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT)
    { // Direction for those messages is to bring them back to the HQ
        my_relay1 = CFG.cirerelays[0];
        my_relay2 = CFG.cirerelays[1];
        my_relay3 = CFG.cirerelays[2];
    }
    r.header.relay1 = (my_relay1 << 4) + 0; // Delay 0
    r.header.relay2 = (my_relay2 << 4) + 1; // Delay 1
    r.header.relay3 = (my_relay3 << 4) + 2; // Delay 2

    /* Update CRC header field */
    uint8_t data_for_crc[INFOLEN];
    if (r.header.rdcp_payload_length > RDCP_MAX_INNER_PAYLOAD_SIZE) 
        r.header.rdcp_payload_length = RDCP_MAX_INNER_PAYLOAD_SIZE;
    memcpy(&data_for_crc, &r.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < r.header.rdcp_payload_length; i++) data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = r.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + r.header.rdcp_payload_length);
    r.header.checksum = actual_crc;
    
    /* Schedule the message for sending if the Entry Point functionality is enabled for this DA */
    if (CFG.ep_enabled)
    {
        uint8_t data_for_scheduler[INFOLEN];
        memcpy(&data_for_scheduler, &r.header, RDCP_HEADER_SIZE);
        for (int i=0; i < r.header.rdcp_payload_length; i++) 
            data_for_scheduler[i + RDCP_HEADER_SIZE] = r.payload.data[i];

        bool important = false;
        if ( /* Determine whether to set the "important" flag in the TXQ */
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_RESET) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_REBOOT) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_MAINTENANCE) 
        )
        {
            important = true;
        }

        int64_t schedtime = 0 - CFG.sf_multiplier * SECONDS_TO_MILLISECONDS; // history: TX_WHEN_CF
#ifdef NEUHAUS202609
        /* Prioritize HQ RDCP Messages by scheduling them to CFEst instead of appending them to the queue */
        if (CFG.hqprio_433 && (rdcp_msg_in.header.origin <= RDCP_ADDRESS_HQ_UPPERBOUND))
        {
            schedtime = TX_WHEN_CF;
        }
#endif
        rdcp_txqueue_add(CHANNEL433, data_for_scheduler, RDCP_HEADER_SIZE + r.header.rdcp_payload_length,
          important, NOFORCEDTX, TX_CALLBACK_ENTRY, schedtime);
    }

    return;
}

extern int64_t most_recent_mg_sender_end;

void rdcp_send_ack_unsigned(uint16_t origin, uint16_t destination, uint16_t seqnr)
{
    struct rdcp_message rm;
    uint8_t data[INFOLEN];
    char info[INFOLEN];

    snprintf(info, INFOLEN, "INFO: Sending DA ACK to %04X for SeqNr %04X", destination, seqnr);
    serial_writeln(info);
  
    rm.header.origin = origin;
    rm.header.sender = origin;
    rm.header.destination = destination;
    rm.header.message_type = RDCP_MSGTYPE_ACK;
    rm.header.counter = 2;
    rm.header.sequence_number = get_next_rdcp_sequence_number(origin);
    rm.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rm.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rm.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
  
    rm.payload.data[0] = seqnr % 256;
    rm.payload.data[1] = seqnr / 256;
    rm.payload.data[2] = rdcp_get_ack_from_infrastructure_status();
  
    rm.header.rdcp_payload_length = RDCP_PAYLOAD_SIZE_ACK_UNSIGNED;
  
    /* Finalize the RDCP Header by calculating the checksum */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &rm.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < rm.header.rdcp_payload_length; i++)
    {
      data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rm.payload.data[i];
    }
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + rm.header.rdcp_payload_length);
    rm.header.checksum = actual_crc;
  
    /* Schedule the crafted message for sending */
    memcpy(&data, &rm.header, RDCP_HEADER_SIZE);
    for (int i=0; i<rm.header.rdcp_payload_length; i++) data[i + RDCP_HEADER_SIZE] = rm.payload.data[i];

    /* ACK needs to be sent after MG has switched back to CHANNEL868DA */
    int64_t forced_time = most_recent_mg_sender_end;
    forced_time += 3 * SECONDS_TO_MILLISECONDS;

    rdcp_txqueue_add(CHANNEL868DA, data, RDCP_HEADER_SIZE + rm.header.rdcp_payload_length, 
        IMPORTANT, FORCEDTX, TX_CALLBACK_ACK, forced_time);

    return;
}

#define CIRE_FILTER_NUM_ENTRIES 128
uint16_t cire_filter_devices[CIRE_FILTER_NUM_ENTRIES] = { RDCP_ADDRESS_SPECIAL_ZERO };
int64_t  cire_filter_timestamps[CIRE_FILTER_NUM_ENTRIES] = { RDCP_TIMESTAMP_ZERO };

bool rdcp_cire_filter_check_and_set(uint16_t origin, uint64_t timestamp)
{
    bool was_already_known = false;
    char filter_info[INFOLEN];

    if (CFG.cirefilter_time == COUNT_ZERO) return was_already_known;
    if (origin == RDCP_ADDRESS_SPECIAL_ZERO) return was_already_known;

    /* Check whether already known */
    for (int i=COUNT_ZERO; i < CIRE_FILTER_NUM_ENTRIES; i++)
    {
        if (cire_filter_devices[i] == origin)
        {
            // Entry found, but might be expired
            if (cire_filter_timestamps[i] + CFG.cirefilter_time * MINUTES_TO_MILLISECONDS < timestamp)
            {
                snprintf(filter_info, INFOLEN, "INFO: CIRE Filter for %04X was expired, setting again");
                serial_writeln(filter_info);
                cire_filter_timestamps[i] = timestamp;
                return was_already_known;
            }
            else 
            {
                snprintf(filter_info, INFOLEN, "INFO: CIRE Filter for %04X present and not expired");
                serial_writeln(filter_info);
                was_already_known = true;
            }
        }
    }

    if (!was_already_known)
    {
        // Find suitable insertion position
        int pos = RDCP_INDEX_NONE;
        for (int i=COUNT_ZERO; i < CIRE_FILTER_NUM_ENTRIES; i++)
        {
            if (cire_filter_devices == RDCP_ADDRESS_SPECIAL_ZERO)
            {
                pos = i;
                break;
            }
        }

        if (pos == RDCP_INDEX_NONE)
        {
            serial_writeln("INFO: CIRE Filter table overflow, cannot add more CIRE origins");
        }
        else 
        {
            cire_filter_devices[pos] = origin;
            cire_filter_timestamps[pos] = timestamp;
            snprintf(filter_info, INFOLEN, "INFO: CIRE Filter added for device %04X at position %d",
                origin, pos);
            serial_writeln(filter_info);
        }
    }

    return was_already_known;
}

void rdcp_cire_filter_reset_open_state(uint16_t destination)
{
    if (destination == RDCP_ADDRESS_SPECIAL_ZERO) return;
    char filter_info[INFOLEN];

    for (int i=COUNT_ZERO; i < CIRE_FILTER_NUM_ENTRIES; i++)
    {
        if (cire_filter_devices[i] == destination)
        {
            cire_filter_devices[i] = RDCP_ADDRESS_SPECIAL_ZERO;
            cire_filter_timestamps[i] = RDCP_TIMESTAMP_ZERO;
            snprintf(filter_info, INFOLEN, "INFO: Lifting CIRE filter for device %04X at position %d based on HQ ACK",
              destination, i);
            serial_writeln(filter_info);
        }
    }
    return;
}

/* EOF */