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
#include "hal.h"

extern rdcp_message rdcp_msg_in;
extern da_config CFG;
extern lora_message current_lora_message;

bool rdcp_check_forward_868_relevance(void)
{
    /* 
        When receiving a new RDCP Message on the 433 MHz channel, forward it 
        on the 868 MHz channel by default, unless it is certainly not relevant 
        for MGs or specifically addressed to our own device.
    */
    if (
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DA_STATUS_REQUEST) ||
        // (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DA_STATUS_RESPONSE) || // NB: needs to reach HQ over 868 MHz
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_FETCH_ALL_NEW_MESSAGES) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_FETCH_MESSAGE) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DELIVERY_RECEIPT) ||
        // (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_HEARTBEAT) || // NB: needs to reach HQ over 868 MHz
        (rdcp_msg_in.header.destination == CFG.rdcp_address)
    ) return false;

    return true;
}

bool rdcp_check_forward_da_relevance(void)
{
    /*
        While we output any received LoRa packets via RX lines to Serial/UART, 
        RDCP Messages deemed relevant for the DA are separately printed. 
    */
    if (
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_BLOCK_ALERT) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_TIMESTAMP) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_RESET) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_REBOOT) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_MAINTENANCE) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_ACK) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DELIVERY_RECEIPT) ||
        (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE) ||
        (rdcp_msg_in.header.destination == CFG.rdcp_address)
    ) return true;

    return false;
}

void rdcp_forward_schedule(int add_random_delay)
{
    /* Do not forward messages we have sent ourselves before */
    if (rdcp_msg_in.header.origin == CFG.rdcp_address)
    {
        serial_writeln("INFO: Not forwarding my own message on 868 MHz after receiving it from someone else");
        return;
    }

    /* Prepare outgoing message by copying the received one */
    rdcp_message r;
    memcpy(&r, &rdcp_msg_in.header, RDCP_HEADER_SIZE);
    for (int i=0; i<r.header.rdcp_payload_length; i++) r.payload.data[i] = rdcp_msg_in.payload.data[i];

    /* Update header fields for the outgoing message */
    r.header.sender = CFG.rdcp_address;
    r.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(r.header.message_type);
    r.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

    /* Update CRC header field */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &r.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < r.header.rdcp_payload_length; i++) data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = r.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + r.header.rdcp_payload_length);
    r.header.checksum = actual_crc;
    
    /* Schedule the message for sending if the forwarding functionality is enabled for this DA */
    if (CFG.forward_enabled)
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
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_DEVICE_REBOOT) || 
            (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_MAINTENANCE) 
        )
        {
            important = true;
        }

        int64_t forced_time = TX_WHEN_CF;
        if (add_random_delay == FORWARD_DELAY_SHORT)
        { // used as EP
          // delay long enough to have a good chance to start distributing on 433 MHz channel first
          // forced_time = 0 - my_random_in_range(2000, 5000); // 2 to 5 seconds
          forced_time = -4000; // fixed value for 433 MHz channel headstart
        }
        else if (add_random_delay == FORWARD_DELAY_PROPORTIONAL)
        { // used as non-EP
          // start with upper bound of random EP delay
          forced_time -= 5000; 
          // add time proportional to timeslot duration (message length, retransmissions) and own relay id 
          forced_time -= (1 + CFG.relay_identifier) * rdcp_get_timeslot_duration(CHANNEL868, data_for_scheduler);
        }
        
        char info[INFOLEN];
        snprintf(info, INFOLEN, "INFO: Preparing 868 forward of %04X-%04X heard from %04X", 
            rdcp_msg_in.header.origin, rdcp_msg_in.header.sequence_number, rdcp_msg_in.header.sender);
        serial_writeln(info);

        rdcp_txqueue_add(CHANNEL868, data_for_scheduler, RDCP_HEADER_SIZE + r.header.rdcp_payload_length,
          important, NOFORCEDTX, TX_CALLBACK_FORWARD, forced_time);
    }

    return;
}

void rdcp_msg_to_da_via_serial(void)
{
    serial_write("DA_RDCP ");
    serial_write_base64((char *)current_lora_message.payload, current_lora_message.payload_length, true);
    return;
}

/* EOF */