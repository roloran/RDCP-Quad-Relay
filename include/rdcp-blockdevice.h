#ifndef _RDCP_BLOCKDEVICE 
#define _RDCP_BLOCKDEVICE

#include <Arduino.h> 

#define MAX_DEVICE_BLOCKS 128

struct device_block_entry {
    uint16_t address   = RDCP_ADDRESS_SPECIAL_ZERO; /// affected device
    uint16_t duration  = RDCP_DURATION_ZERO;        /// block duration in minutes
    int64_t  timestamp = RDCP_TIMESTAMP_ZERO;       /// timestamp of when entry was added/updated
};

/**
 * Check whether we should relay messages from a specific origin 
 * @param origin RDCP Address to check 
 * @return true if relaying is OK, false if origin is currently blocked 
 */
bool rdcp_relay_allowed_for_device(uint16_t origin);

/**
 * Add a new device block. 
 * @param target RDCP Address of device to block 
 * @param duration Duration in minutes to block the device. Must be > 0. 
 */
void rdcp_device_block_add(uint16_t target, uint16_t duration);

/**
 * Delete a device block entry. Must be used for "duration == 0" operations. 
 * @param target RDCP Address of the device to unblock
 */
void rdcp_device_block_remove(uint16_t target);

/**
 * Remove all current device block entries, e.g., during reset operations.
 */
void rdcp_device_block_clear(void);

#endif 
/* EOF */