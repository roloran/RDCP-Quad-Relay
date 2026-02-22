#ifndef _RDCP_CALLBACKS
#define _RDCP_CALLBACKS

#include <Arduino.h>
#include "rdcp-common.h"

struct callback_chain {
    bool     in_use        = false;  /// chain currently in use?
    int64_t  timeout       = RDCP_DURATION_ZERO;         /// timeout timestamp if callback isn't triggered
    int64_t  activity      = RDCP_TIMESTAMP_ZERO;        /// when did we start the most recent activity?
    uint16_t refnr         = RDCP_OA_REFNR_SPECIAL_ZERO; /// relevant OA reference number
    uint16_t destination   = RDCP_ADDRESS_SPECIAL_ZERO;  /// RDCP address of destination, if any
    int      last_mem_idx  = RDCP_INDEX_NONE;            /// "memory" index of last sent message
};

/**
 * Start a new callback-chained transmission. 
 * @param callback_to_use Number of callback to use, e.g., TX_CALLBACK_PERIODIC868
 * @param starter Index of memory to start the chain with, or -1 if none 
 * @param destination RDCP Address of destination (e.g., single device or broadcast)
 * @param refnr A Reference Number related to this chained transmission
 */
void rdcp_chain_starter(uint8_t callback_to_use, int starter, uint16_t destination, uint16_t refnr);

/**
 * Callback function for chained transmissions.
 * Called when TX of a chained memory has finished or a timeout occured. 
 * @param callback_type Choice of callback, e.g., TX_CALLBACK_FETCH_ALL 
 * @param has_timeout true if called due to timeout, false if called regularily 
 */
void rdcp_chain_callback(uint8_t callback_type, bool has_timeout);

/**
 * Starts a fresh Periodic868 chain. 
 */
void rdcp_periodic_kickstart(void);

#endif 
/* EOF */