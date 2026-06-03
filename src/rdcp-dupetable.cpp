#include "rdcp-dupetable.h"
#include "rdcp-common.h"
#include "lora.h"
#include "hal.h"
#include "serial.h"
#ifdef ROLORAN_USE_FFAT
#include "FFat.h"
#else
#include <LittleFS.h>
#endif

struct rdcp_dup_table dupe_table;              // One global RDCP Message Duplicate Table
#define FILENAME_DUPETABLE "/dupetable"
#define FILENAME_SPW       "/dupespw"
bool do_not_persist_dupetable = false;

struct sliding_prune_window spw;

/**
 * Initialize the Sliding Prune Window by zeroing all entries
 */
void rdcp_spw_init(void)
{
  for (int i=0; i != SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES; i++)
  {
    spw.windowentries[i].origin = RDCP_ADDRESS_SPECIAL_ZERO;
    spw.windowentries[i].sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO;
  }
  spw.front = COUNT_ZERO;
  spw.initialized = true;
  serial_writeln("INFO: SPW initialized");
  return;
}

/**
 * Persist the Sliding Prune Window via file system
 */
void rdcp_spw_persist(void)
{
  if (do_not_persist_dupetable == true)
  {
    serial_writeln("INFO: Refusing to persist SPW");
    return;
  }

  serial_writeln("INFO: Persisting SPW");
#ifdef ROLORAN_USE_FFAT
  FFat.remove(FILENAME_SPW);
  File f = FFat.open(FILENAME_SPW, FILE_WRITE);
#else
  LittleFS.remove(FILENAME_SPW);
  File f = LittleFS.open(FILENAME_SPW, FILE_WRITE);
#endif
  if (!f) return;
  f.write((uint8_t *) &spw, sizeof(spw));
  f.close();

  return;
}

/**
 * Reset the Sliding Prune Window by initializing it (again) and persisting the empty state
 */
void rdcp_spw_reset(void)
{
  rdcp_spw_init();
  rdcp_spw_persist();
  return;
}

/**
 * Read the persisted Sliding Prune Window from file system or initialize it if not persisted
 */
void rdcp_spw_restore(void)
{
  serial_writeln("INFO: Restoring SPW");
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(FILENAME_SPW, FILE_READ);
#else
  File f = LittleFS.open(FILENAME_SPW, FILE_READ);
#endif
  if (!f) return;
  f.read((uint8_t *) &spw, sizeof(spw));
  f.close();
  return;
}

void rdcp_spw_delete_file(void)
{
  serial_writeln("INFO: Deleting SPW file");
  LittleFS.remove(FILENAME_SPW);
  return;
}

void rdcp_spw_delete_entry(uint16_t origin)
{
  for (int i=0; i != SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES; i++)
  {
    if (spw.windowentries[i].origin == origin)
    {
      spw.windowentries[i].sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO;
      spw.windowentries[i].origin = RDCP_ADDRESS_SPECIAL_ZERO;
    }
  }
  return;
}

void rdcp_spw_set_entry(uint16_t origin, uint16_t seqnr)
{
  bool found = false;
  for (int i=0; i != SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES; i++)
  {
    if (spw.windowentries[i].origin == origin)
    {
      spw.windowentries[i].sequence_number = seqnr;
      found = true;
    }
  }
  if (!found)
  {
    spw.windowentries[spw.front].origin = origin;
    spw.windowentries[spw.front].sequence_number = seqnr;
    spw.front = (spw.front + 1) % SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES;
  }
  return;
}

void rdcp_spw_dump(void)
{
  char info[INFOLEN];
  for (int i=0; i != SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES; i++)
  {
    if (spw.windowentries[i].origin == RDCP_ADDRESS_SPECIAL_ZERO) continue;
    snprintf(info, INFOLEN, "INFO: SPW entry %i: %04X with seqnr %04X",
      i,
      spw.windowentries[i].origin,
      spw.windowentries[i].sequence_number);
    serial_writeln(info);
  }
  return;
}

/** 
  Check whether we have exactly this tuple of (origin, seqnr) in our SPW already.
  If so, a duplicate was found and true is returned.
  If we did not find the exact tuple, add it to the SPW ring buffer 
  (overwriting oldest entry if none is free), and return false.
*/
bool rdcp_spw_check_and_add(uint16_t origin, uint16_t seqnr)
{
  bool result = true; // By default, mark as duplicate, as this was the previous fixed return value of our single caller
  bool found = false; // Matching tuple was not yet found in the SPW

  for (int i=0; i != SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES; i++)
  {
    if (spw.windowentries[i].origin == RDCP_ADDRESS_SPECIAL_ZERO) continue; // Skip unused entries
    if (spw.windowentries[i].origin == origin)
    {
      if (spw.windowentries[i].sequence_number == seqnr)
      {
        found = true;
        break;
      }
    }
  }

  /* 
    If the entry was found, a message with this origin/seqnr was already processed previously, 
    so there is nothing else to do but to return the dupe status as "true". 
    Otherwise, we add the tuple to the SPW and return "false".
   */

  if (!found)
  {
    result = false;
    spw.windowentries[spw.front].origin = origin;
    spw.windowentries[spw.front].sequence_number = seqnr;
    spw.front = (spw.front + 1) % SLIDING_PRUNE_WINDOW_NUMBER_OF_ENTRIES;
  }

  return result;
}

void rdcp_reset_duplicate_message_table(void)
{
  dupe_table.num_entries = 0;
  for (int i=0; i != NUM_DUPETABLE_ENTRIES; i++)
  {
    dupe_table.tableentry[i].origin = RDCP_ADDRESS_SPECIAL_ZERO;
    dupe_table.tableentry[i].sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO;
    dupe_table.tableentry[i].last_seen = RDCP_TIMESTAMP_ZERO;
  }
  rdcp_duplicate_table_persist();

  /* Also clear and persist the SPW */
  rdcp_spw_reset();

  return;
}

void rdcp_dump_duplicate_message_table(void)
{
  char info[INFOLEN];
  for (int i=0; i != NUM_DUPETABLE_ENTRIES; i++)
  {
    if (dupe_table.tableentry[i].origin == RDCP_ADDRESS_SPECIAL_ZERO) continue;
    snprintf(info, INFOLEN, "INFO: Dupe table entry %i: %04X with seqnr %04X",
      i,
      dupe_table.tableentry[i].origin,
      dupe_table.tableentry[i].sequence_number);
    serial_writeln(info);
  }

  rdcp_spw_dump();

  return;
}

void rdcp_duplicate_table_restore(void)
{
  serial_writeln("INFO: Restoring dupe table");
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(FILENAME_DUPETABLE, FILE_READ);
#else
  File f = LittleFS.open(FILENAME_DUPETABLE, FILE_READ);
#endif
  if (!f) return;
  f.read((uint8_t *) &dupe_table, sizeof(dupe_table));
  f.close();

  rdcp_spw_restore();

  return;
}

void rdcp_duplicate_table_delete_file(void)
{
  serial_writeln("INFO: Deleting duplicate table file");
  LittleFS.remove(FILENAME_DUPETABLE);
  rdcp_spw_delete_file();
  return;
}

void rdcp_duplicate_table_delete_entry(uint16_t origin)
{
  for (int i=0; i != dupe_table.num_entries; i++)
  {
    if (dupe_table.tableentry[i].origin == origin)
    {
      dupe_table.tableentry[i].sequence_number = 0;
      dupe_table.tableentry[i].last_seen = my_millis();
      serial_writeln("INFO: Duplicate table entry was reset for given origin");
    }
  }
  rdcp_spw_delete_entry(origin);
  return;
}

void rdcp_duplicate_table_set_entry(uint16_t origin, uint16_t seqnr)
{
  for (int i=0; i != dupe_table.num_entries; i++)
  {
    if (dupe_table.tableentry[i].origin == origin)
    {
      dupe_table.tableentry[i].sequence_number = seqnr;
      dupe_table.tableentry[i].last_seen = my_millis();
      serial_writeln("INFO: Duplicate table entry was set for given origin");
    }
  }
  rdcp_spw_set_entry(origin, seqnr);
  return;
}

void rdcp_duplicate_table_delete_all_entries(void)
{
  for (int i=0; i != dupe_table.num_entries; i++)
  {
    dupe_table.tableentry[i].sequence_number = 0;
    dupe_table.tableentry[i].last_seen = my_millis();
  }
  serial_writeln("INFO: Duplicate table entry was reset for all entries");
  rdcp_spw_init();
  return;
}

void rdcp_duplicate_table_persist(void)
{
  if (do_not_persist_dupetable == true)
  {
    serial_writeln("INFO: Refusing to persist duplicate table");
    return;
  }

  serial_writeln("INFO: Persisting dupe table");
#ifdef ROLORAN_USE_FFAT
  FFat.remove(FILENAME_DUPETABLE);
  File f = FFat.open(FILENAME_DUPETABLE, FILE_WRITE);
#else
  LittleFS.remove(FILENAME_DUPETABLE);
  File f = LittleFS.open(FILENAME_DUPETABLE, FILE_WRITE);
#endif
  if (!f) return;
  f.write((uint8_t *) &dupe_table, sizeof(dupe_table));
  f.close();

  /* Also persist the SPW */
  rdcp_spw_persist();

  return;
}

bool rdcp_check_duplicate_message(uint16_t origin, uint16_t sequence_number)
{
  if (!spw.initialized) rdcp_spw_init();

  int pos = RDCP_INDEX_NONE;
  for (int i=0; i != dupe_table.num_entries; i++)
  {
    if (dupe_table.tableentry[i].origin == origin) pos = i;
  }

  if (pos == RDCP_INDEX_NONE) // new entry
  {
    if (dupe_table.num_entries > NUM_DUPETABLE_ENTRIES-1)
    {
      Serial.println("WARNING: RDCP duplicate table overflow - increase size!");
      return false;
    }

    dupe_table.tableentry[dupe_table.num_entries].origin = origin;
    dupe_table.tableentry[dupe_table.num_entries].sequence_number = sequence_number;
    dupe_table.tableentry[dupe_table.num_entries].last_seen = my_millis();
    dupe_table.num_entries++;

    rdcp_spw_set_entry(origin, sequence_number); // Add seen message to SPW

    return false;
  }
  else
  {
    dupe_table.tableentry[pos].last_seen = my_millis();
    if (dupe_table.tableentry[pos].sequence_number < sequence_number)
    { // update highest sequence number
      dupe_table.tableentry[pos].sequence_number = sequence_number;
      rdcp_spw_set_entry(origin, sequence_number); // Add seen message to SPW
      return false;
    }
    else
    { /* 
        Duplicate found according to dupe table.
        Apply additional checks to avoid omissions due to RDCP Message reordering issues.
      */

      /*
        1)
        If sequence number is exactly the same as the one in the dupe table, 
        this is a duplicate for sure, no need to check SPW.
      */
      if (dupe_table.tableentry[pos].sequence_number == sequence_number) 
      {
        rdcp_spw_set_entry(origin, sequence_number); // Add seen message to SPW
        return true;
      }

      /* 
        2)
        MGs are exempted from SPW checking as CIRE reordering railguards are in place. 
      */
      if (RDCP_ADDRESS_MG_LOWERBOUND < origin) 
      {
        return true; // no need to add to SPW as SPW check will always be skipped for devices in higher RDCP address range
      }

      /*
        3)
        If the sequence number to check is way too old, the message is considered a sure duplicate.
      */
      if (sequence_number + SPW_MAXIMUM_DIFFERENCE < dupe_table.tableentry[pos].sequence_number) 
      {
        return true; // no need to pollute SPW with sequence numbers that are too old anyway
      }

      /*  Finally check SPW, implicitly store the new message metadata, and use the check's result. */
      bool spw_state = rdcp_spw_check_and_add(origin, sequence_number);
      return spw_state;
    }
  }
  return false;
}

/* EOF rdcp-dupetable.cpp */