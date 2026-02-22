#include "rdcp-incoming.h"
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-scheduler.h"

extern lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
extern da_config CFG;
relay_memory_entry relay_memory[1];

int rdcp_check_relay_designation(void)
{
    int result = -1; // do not relay by default
    uint8_t my_mask = CFG.relay_identifier << 4;

    /* Explicit designation */
    if ((rdcp_msg_in.header.relay1 & 0xF0) == my_mask)
    { 
        /* 
            We are designated in the Relay1 header field.
            However, we need to filter RDCP Messages sent to the EP on the wrong channel 
            based on empirical evidence. 
        */
        if ((rdcp_msg_in.header.relay2 == RDCP_HEADER_RELAY_MAGIC_NONE) && // no relay2
            (rdcp_msg_in.header.relay3 == RDCP_HEADER_RELAY_MAGIC_NONE) && // no relay3
            ((rdcp_msg_in.header.relay1 & 0x0F) == 0)) // delay for relay1 is 0 
        {
            serial_writeln("WARNING: Message sent to EntryPoint on wrong channel, not relaying.");
        }
        else 
        {
            result = (rdcp_msg_in.header.relay1 & 0x0F);
        }
    }
    if ((rdcp_msg_in.header.relay2 & 0xF0) == my_mask) result = (rdcp_msg_in.header.relay2 & 0x0F);
    if ((rdcp_msg_in.header.relay3 & 0xF0) == my_mask) result = (rdcp_msg_in.header.relay3 & 0x0F);

    /* Catch-all magic value */
    if ((rdcp_msg_in.header.relay1 == 0xFF) || (rdcp_msg_in.header.relay2 == 0xFF) || (rdcp_msg_in.header.relay3 == 0xFF))
        result = 0; // relay immediately
    if ((rdcp_msg_in.header.relay1 == 0xF3) && (rdcp_msg_in.header.relay2 == 0xEE) && (rdcp_msg_in.header.relay3 == 0xEE))
        result = 3; // valid in TS4 only, all send in TS8

    if (result > -1)
    {
        char info[INFOLEN]; 
        snprintf(info, INFOLEN, "INFO: Relay designation with a delay of %d timeslots", result); 
        serial_writeln(info);
    }

    return result;
}

bool rdcp_check_has_already_relayed(void)
{
    /* Currently, we only remember the most recently relayed message to avoid relaying it twice. */
    /* This might be extended to a larger ring-buffer memory depending on how RDCP collissions are handled. */
    int index = 0; /* may be changed to loop later */

    uint16_t relay12 = (rdcp_msg_in.header.relay1 << 8) + rdcp_msg_in.header.relay2; // combined relay1 and relay2 header fields

    if ((relay_memory[index].origin  == rdcp_msg_in.header.origin) &&
        (relay_memory[index].sender  == rdcp_msg_in.header.sender) &&
        (relay_memory[index].relay12 == relay12) &&
        (relay_memory[index].seqnr   == rdcp_msg_in.header.sequence_number)) 
    {
        // Duplicate, do not relay again
        return true;
    }
    else 
    {
        // Remember this message for future checks 
        relay_memory[index].sender  = rdcp_msg_in.header.sender; // by checking the sender, we relay the same message multiple times if designated as relay by different senders
        relay_memory[index].origin  = rdcp_msg_in.header.origin;
        relay_memory[index].seqnr   = rdcp_msg_in.header.sequence_number;
        relay_memory[index].relay12 = relay12;
        // Signal back that it was a new message 
        return false;
    }
    return false;
}

int rdcp_derive_timeslot_from_in(void)
{
    int ts = -1;

    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x00) ts = 0; // First Hop 1 assigned with Delay 0
    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x02) ts = 1; // Second Hop 1 assigned with Delay 2
    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x03) ts = 4; // Third Hop 1 assigned
    if ((rdcp_msg_in.header.relay2 & 0x0F) == 0x04) ts = 2; // Second Hop 4 assigned with Delay 4, overrides previous
    if (rdcp_msg_in.header.relay1 == 0xE4) ts = 3;
    if (rdcp_msg_in.header.relay1 == 0xE2) ts = 5;
    if (rdcp_msg_in.header.relay1 == 0xE1) ts = 6;
    if (rdcp_msg_in.header.relay1 == 0xE0) ts = 7;
    if (rdcp_msg_in.header.relay1 == 0xEE) ts = 8;

    if (rdcp_msg_in.header.relay1 == 0xFF) ts = 7; // Third Hop variant

    if (ts == -1)
    {
        serial_writeln("ERROR: Cannot derive propagation cycle timeslot from incoming RDCP message");
    }

    return ts;
}

void rdcp_schedule_relayed_message(int relay_delay)
{
    if (relay_delay < 0)
    {
        serial_writeln("ERROR: Negative relay delay cannot be used for relay scheduling");
        return;
    }

    rdcp_header h;
    memcpy(&h, &rdcp_msg_in.header, RDCP_HEADER_SIZE);
    int64_t tx_delay_in_ms = relay_delay * rdcp_get_timeslot_duration(current_lora_message.channel, (uint8_t*) &h);

    /* 
        The message we have to relay might still be repeated (retransmission counter) by the sender 
        we got it from. This must be considered when deriving the absolute timestamp for our own timeslot.
    */
    int64_t previous_timeslot_rest = rdcp_msg_in.header.counter * 
                        (RDCP_TIMESLOT_BUFFERTIME + airtime_in_ms(current_lora_message.channel, 
                            RDCP_HEADER_SIZE+rdcp_msg_in.header.rdcp_payload_length));

    int64_t timeslot_syncer_after_rx = RDCP_TIMESLOT_BUFFERTIME - (my_millis() - current_lora_message.timestamp);

    int64_t my_timeslot_begin = previous_timeslot_rest + current_lora_message.timestamp + 
                                timeslot_syncer_after_rx + tx_delay_in_ms - TRANSMISSION_PROCESSING_TIME;

    /* Prepare outgoing message */
    rdcp_message r;
    memcpy(&r, &rdcp_msg_in.header, RDCP_HEADER_SIZE);
    for (int i=0; i<r.header.rdcp_payload_length; i++) r.payload.data[i] = rdcp_msg_in.payload.data[i];

    r.header.sender = CFG.rdcp_address;
    r.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(r.header.message_type);

    /* Which timeslot are we going to send in? */
    int myts = rdcp_derive_timeslot_from_in() + relay_delay + 1; // Zero delay means sending in next timeslot

    if (myts > 8)
    {
        serial_writeln("ERROR: Assigned relaying timeslot exceeds propagation cycle");
        return;
    }

    uint8_t my_relay1 = CFG.oarelays[0]; // Default direction is to spread in the mesh
    uint8_t my_relay2 = CFG.oarelays[1];
    /* Don't assign the sender we got the message from as next relay */
    if (my_relay1 == rdcp_msg_in.header.sender & 0x000F) my_relay1 = CFG.oarelays[2];
    if (my_relay2 == rdcp_msg_in.header.sender & 0x000F) my_relay2 = CFG.oarelays[2];

    if ((r.header.message_type == RDCP_MSGTYPE_DA_STATUS_RESPONSE) ||
        (r.header.message_type == RDCP_MSGTYPE_CITIZEN_REPORT) ||
        (r.header.message_type == RDCP_MSGTYPE_HEARTBEAT))
        { // Direction for those messages is to bring them back to the HQ
            my_relay1 = CFG.cirerelays[0];
            my_relay2 = CFG.cirerelays[1];
            if (my_relay1 == rdcp_msg_in.header.sender & 0x000F) my_relay1 = CFG.cirerelays[2];
            if (my_relay2 == rdcp_msg_in.header.sender & 0x000F) my_relay2 = CFG.cirerelays[2];
        }

    if (myts == 1) // cannot be < 1 or > 8
    {
        r.header.relay1 = (my_relay1 << 4) + 2; 
        r.header.relay2 = (my_relay2 << 4) + 3;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 2)
    {
        r.header.relay1 = (my_relay1 << 4) + 3; 
        r.header.relay2 = (my_relay2 << 4) + 4;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 3)
    {
        r.header.relay1 = 0xE4;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 4)
    {
        r.header.relay1 = (my_relay1 << 4) + 3; 
        if (CFG.ts4allones) r.header.relay1 = 0xF3;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 5)
    {
        r.header.relay1 = 0xE2;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 6)
    {
        r.header.relay1 = 0xE1;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 7)
    {
        r.header.relay1 = CFG.ts7relay1;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }
    else if (myts == 8)
    {
        r.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        r.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    }

    /* Update CRC header field */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &r.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < r.header.rdcp_payload_length; i++) data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = r.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + r.header.rdcp_payload_length);
    r.header.checksum = actual_crc;

    /* Pass the message to the scheduler */
    if (CFG.relay_enabled)
    {
        uint8_t data_for_scheduler[INFOLEN];
        memcpy(&data_for_scheduler, &r.header, RDCP_HEADER_SIZE);
        for (int i=0; i < r.header.rdcp_payload_length; i++) 
            data_for_scheduler[i + RDCP_HEADER_SIZE] = r.payload.data[i];

        bool forcedtx = FORCEDTX;
        bool important = IMPORTANT;
        /*
          If we are yet about to relay another message (TXQ has an FORCEDTX entry) or know that 
          another propagation cycle is still ongoing, we are a designated relay for a "collision" 
          RDCP Message. This is the place to handle such collisions.
        */
        bool collision = rdcp_txqueue_has_forced_entry(CHANNEL433) || rdcp_propagation_cycle_duplicate(); 
        if (collision)
        {
            if (rdcp_get_number_of_tracked_propagation_cycles() > 1)
            {
                serial_writeln("WARNING: Relay collision (hard - Relay designation to multiple propagation cycles)");
                forcedtx = NOFORCEDTX;
                important = NOTIMPORTANT;
                int64_t now = my_millis();
                my_timeslot_begin = 0 - ((my_timeslot_begin - now) + (rdcp_get_channel_free_estimation(CHANNEL433) - now));
                char info[INFOLEN];
                snprintf(info, INFOLEN, "INFO: Postponing relaying of conflicting message by %" PRId64 " ms after CFEst433", 
                    -1 * my_timeslot_begin);
                serial_writeln(info);
            }
            else 
            {
                serial_writeln("INFO: Relay collision (soft - repeated Relay designation to same tracked propagation cycle)");
            }
        }

        rdcp_txqueue_add(CHANNEL433, data_for_scheduler, RDCP_HEADER_SIZE + r.header.rdcp_payload_length,
          important, forcedtx, TX_CALLBACK_RELAY, my_timeslot_begin);
    }

    return;
}

/* EOF */