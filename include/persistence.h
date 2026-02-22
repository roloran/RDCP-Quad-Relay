#ifndef _DEVICE_PERSISTENCE
#define _DEVICE_PERSISTENCE

#include <Arduino.h>

/**
 * Initialize persistent storage on the T-Deck.
 * To be called once after power-on.
 */
void setup_persistence(void);

/**
 * Store a Serial command to be automatically executed again on next device power-on.
 * @param s String with a Serial command to automatically replay on next power-on.
 */
void persist_serial_command_for_replay(String s);

/**
 * Replay persisted Serial commands (= update configuration during device power-on).
 * Usually called once after power-on and persistence_setup().
 */
void persistence_replay_serial(void);

/**
 * Reset/delete the list of Serial commands replayed on device power-on.
 */
void persistence_reset_replay_serial(void);

/**
 * Get the next RDCP Sequence Number to use for a specific RDCP address as Origin. 
 * @param origin RDCP Address of the device that needs the SequenceNumber as Origin 
 * @return uint16_t SequenceNumber as used in RDCP Header
 */
uint16_t get_next_rdcp_sequence_number(uint16_t origin);

/**
 * Set the next RDCP Sequence NUmber to use for a specific RDCP address. Used internally 
 * as well as for testing purposes. 
 * @param origin RDCP Address or Origin 
 * @param seq Next SequenceNumber to use for this Otigin 
 * @return the sequence number that has been set
 */
uint16_t set_next_rdcp_sequence_number(uint16_t origin, uint16_t seq);

/**
 * @return true if the device has FFat available, false otherwise
 */
bool hasStorage(void);

/**
 * Check for nonce validity on management RDCP messages. 
 * @param name Name of the nonce type 
 * @param nonce Received nonce 
 * @return true if nonce is valid, false otherwise
 */
bool persistence_checkset_nonce(char *name, uint16_t nonce);

#endif 
/* EOF */