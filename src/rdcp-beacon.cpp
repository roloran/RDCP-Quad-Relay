#include "rdcp-beacon.h"
#include "lora.h"
#include "hal.h"
#include "rdcp-scheduler.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"

extern da_config CFG;
int64_t time_of_last_beacon[NUMCHANNELS] = {0, 0};
int32_t number_of_last_beacon[NUMCHANNELS] = {0, 0};

void rdcp_beacon_send(uint8_t channel, int32_t beacon_number)
{
    char message[INFOLEN];
    snprintf(message, INFOLEN, "RDCP-Beacon %" PRId32 " by %04X (%s) on CHANNEL%d",
        beacon_number, CFG.rdcp_address, CFG.name, channel == CHANNEL433 ? 433 : 868);
    serial_writeln("INFO: Scheduling RDCP-Beacon");

    rdcp_message rm;

    /* Prepare RDCP Header (except CRC) */
    rm.header.origin = CFG.rdcp_address;
    rm.header.sender = CFG.rdcp_address;
    rm.header.destination = RDCP_BROADCAST_ADDRESS;
    rm.header.counter = NRT_LEVEL_LOW;
    rm.header.sequence_number = get_next_rdcp_sequence_number(CFG.rdcp_address);
    rm.header.message_type = RDCP_MSGTYPE_TEST;
    rm.header.rdcp_payload_length = strlen(message);
    rm.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rm.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rm.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

    /* Prepare RDCP Payload */
    for (int i=0; i<strlen(message); i++) rm.payload.data[i] = message[i];

    /* Prepare CRC Header field */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &rm.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < rm.header.rdcp_payload_length; i++) 
        data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rm.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + rm.header.rdcp_payload_length);
    rm.header.checksum = actual_crc;

    /* Schedule for sending on free channel */
    uint8_t data_for_scheduler[INFOLEN];
    memcpy(&data_for_scheduler, &rm.header, RDCP_HEADER_SIZE);
    for (int i=0; i < rm.header.rdcp_payload_length; i++) 
        data_for_scheduler[i + RDCP_HEADER_SIZE] = rm.payload.data[i];
    int64_t my_delay = 0 - (1 * SECONDS_TO_MILLISECONDS + 100 * (CFG.relay_identifier + 1)) * CFG.sf_multiplier;
    rdcp_txqueue_add(channel, data_for_scheduler, RDCP_HEADER_SIZE + rm.header.rdcp_payload_length,
      NOTIMPORTANT, NOFORCEDTX, TX_CALLBACK_NONE, my_delay);

    return;
}

void rdcp_beacon(void)
{
    if (CFG.beacon_interval[CHANNEL433] > 0)
    {
        if (get_num_txq_entries(CHANNEL433) == 0) // skip if channel is busy otherwise
        {
            int64_t now = my_millis();
            if (now > time_of_last_beacon[CHANNEL433] + CFG.beacon_interval[CHANNEL433])
            {
                time_of_last_beacon[CHANNEL433] = now; 
                number_of_last_beacon[CHANNEL433]++;
                rdcp_beacon_send(CHANNEL433, number_of_last_beacon[CHANNEL433]);
            }
        }
    }

    if (CFG.beacon_interval[CHANNEL868] > 0)
    {
        if (get_num_txq_entries(CHANNEL868) == 0) // skip if channel is busy otherwise
        {
            int64_t now = my_millis();
            if (now > time_of_last_beacon[CHANNEL868] + CFG.beacon_interval[CHANNEL868])
            {
                time_of_last_beacon[CHANNEL868] = now; 
                number_of_last_beacon[CHANNEL868]++;
                rdcp_beacon_send(CHANNEL868, number_of_last_beacon[CHANNEL868]);
            }
        }
    }

    return;
}

/* EOF */