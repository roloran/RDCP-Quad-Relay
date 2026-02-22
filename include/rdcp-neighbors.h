#ifndef _RDCP_NEIGHBORS
#define _RDCP_NEIGHBORS

#include <Arduino.h> 
#include "lora.h"

#define MAX_NEIGHBORS 128

/**
 * Add or update a neighbor table entry based on the most recently received RDCP Message. 
 * @param channel CHANNEL433 or CHANNEL868 
 * @param sender RDCP Address of neighbor 
 * @param rssi RSSI value of received LoRa packet 
 * @param snr SNR value of received LoRa packet 
 * @param timestamp Timestamp of when LoRa packet was received 
 * @param heartbeat true if explicit RDCP Heartbeat, false otherwise 
 */
void rdcp_neighbor_register_rx(uint8_t channel, uint16_t sender, double rssi, double snr, int64_t timestamp, bool heartbeat, bool explicit_refnr, uint16_t latest_refnr, uint16_t roamingrec);

/**
 * List the current neighbor table on Serial. 
 */
void rdcp_neighbor_dump(void);

struct neighbor_table_entry {
    uint8_t channel   = CHANNEL433;
    uint16_t sender   = RDCP_ADDRESS_SPECIAL_ZERO;
    double rssi       = 0.0;
    double snr        = 0.0;
    int64_t timestamp = RDCP_TIMESTAMP_ZERO;
    bool heartbeat    = false;      // has sent an explitit Heartbeat  
    bool counted      = false;      // has been counted for DA Status Response
    bool explicit_refnr = false;    // has sent an explicit latest OA RefNr
    uint16_t latest_refnr = RDCP_OA_REFNR_SPECIAL_ZERO; // OA RefNr reported by MG
    uint16_t roamingrec = RDCP_ADDRESS_SPECIAL_ZERO; // Roaming Recommendation reported by MG
};

#endif 
/* EOF */