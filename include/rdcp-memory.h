#ifndef _RDCP_MEMORY
#define _RDCP_MEMORY

#include <Arduino.h>
#include "lora.h"
#include "rdcp-common.h"

#define MAX_STORED_MSGS 32

struct rdcp_memory_entry {
    bool slot_used            = false; 
    uint8_t payload[MAX_LORA_PAYLOAD_SIZE];
    uint8_t payload_length    = 0;
    uint16_t reference_number = RDCP_OA_REFNR_SPECIAL_ZERO;
    int64_t timestamp_added   = RDCP_TIMESTAMP_ZERO;
    bool used_in_fetch_single = false;
    bool used_in_fetch_all    = false;
    bool used_in_periodic868  = false;
};

struct rdcp_memory_table {
    rdcp_memory_entry entries[MAX_STORED_MSGS];
    int idx_first = -1;
};

struct runtime_da_data {
    uint8_t battery1 = 255;
    uint8_t battery2 = 255;
    uint16_t num_rdcp_rx = 0;
    uint16_t num_rdcp_tx = 0;
};

/**
 * Remember the currently processed OA/Signature. 
 */
void rdcp_memory_remember(void);

/**
 * Forget all memories. 
 */
void rdcp_memory_forget(void);

/**
 * Print memories on Serial. 
 */
void rdcp_memory_dump(void);

/**
 * Store all current memories in FFat. 
 */
void rdcp_memory_persist(void);

/**
 * Restore persisted memories from FFat. 
 */
void rdcp_memory_restore(void);

#endif 
/* EOF */