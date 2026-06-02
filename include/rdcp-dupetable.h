#ifndef _RDCP_DUPETABLE
#define _RDCP_DUPETABLE

#include <Arduino.h>
#include <rdcp-common.h>

/**
  * Data structure for Duplicate Table entries
  */
struct rdcp_dup_table_entry {
  uint16_t origin = RDCP_ADDRESS_SPECIAL_ZERO; //< Origin from the RDCP Header
  uint16_t sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO; //< SequenceNumber from the RDCP Header
  int64_t last_seen = RDCP_TIMESTAMP_ZERO; //< Timestamp of when the entry was last updated
};
  
#define SPW_MAXIMUM_DIFFERENCE 16

#define NUM_DUPETABLE_ENTRIES 256
/**
  * Data structure for the overall Duplicate Table
  */
struct rdcp_dup_table {
  unsigned short num_entries = 0;              //< Number of currently stored entries
  struct rdcp_dup_table_entry tableentry[NUM_DUPETABLE_ENTRIES]; //< Array of Duplicate Table entries
};
  
/* Number of entries should be chosen as at least 
   SPW_MAXIMUM_DIFFERENCE * (number_of_das_in_scenario + number_of_hq_devices) 
   if memory/free RAM permits. However, it is fail-safe if chosen too small 
   because duplicate checking according to RDCP specs will still be in place.
*/
#define SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES 256
struct sliding_prune_window_entry {
  uint16_t origin = RDCP_ADDRESS_SPECIAL_ZERO;
  uint16_t sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO;
};

struct sliding_prune_window {
  bool initialized = false;
  int front = COUNT_ZERO;
  struct sliding_prune_window_entry windowentries[SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES];
};

/**
  * Resets the Duplicate Table by clearing all entries
  */
void rdcp_reset_duplicate_message_table(void);

/**
 * Restore a persisted duplicate table
 */
void rdcp_duplicate_table_restore(void);

/**
 * Delete the duplicate table file
 */
void rdcp_duplicate_table_delete_file(void);

/**
 * Remove a specific origin from the duplicate table
 * @param origin RDCP Address of entry to remove
 */
void rdcp_duplicate_table_delete_entry(uint16_t origin);

/**
 * Set a specific last-seen sequence number for an RDCP address in the duplicate table
 * @param origin RDCP Address (Origin) to change setting for
 * @param seqnr New most recently seen to set for this Origin
 */
void rdcp_duplicate_table_set_entry(uint16_t origin, uint16_t seqnr);

/**
 * Clear the duplicate table (all entries)
 */
void rdcp_duplicate_table_delete_all_entries(void);

/**
 * Persist the current duplicate table
 */
void rdcp_duplicate_table_persist(void);

/**
  * Checks whether an RDCP Message with Origin and SequenceNumber given as parameters
  * should be treated as duplicate (returns true) or new (returns false).
  * @param origin RDCP Origin address of the RDCP Message to check for duplicate 
  * @param sequence_number RDCP Header SequenceNumber of the RDCP Message to check for duplicate 
  * @return true if the message is a duplicate, false if it was not seen before
  */
bool rdcp_check_duplicate_message(uint16_t origin, uint16_t sequence_number);
 

/**
 * Dump the current duplicate table via Serial.
 */
void rdcp_dump_duplicate_message_table(void);

#endif