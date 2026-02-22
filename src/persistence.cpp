#include "persistence.h"
#include "serial.h"

#ifdef ROLORAN_USE_FFAT
#include <FFat.h>
#else 
#include "FS.h"
#include <LittleFS.h>
#endif

#define FILENAME_SERIAL_REPLAY "/config.txt"
#define FILENAME_PREFIX_SEQNR "/seqnr_"

bool hasFFat = false;

bool hasStorage(void)
{
  return hasFFat;
}

void setup_persistence(void)
{
#ifdef ROLORAN_USE_FFAT
  // FFat.format(); // unbrick device :-)
  if(FFat.begin(true)) hasFFat = true;
#else 
  if(LittleFS.begin(true, "/littlefs", 10U, "ffat")) hasFFat = true;
#endif
  return;
}

void persist_serial_command_for_replay(String s)
{
  if (!hasFFat) return;
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(FILENAME_SERIAL_REPLAY, FILE_APPEND);
#else 
  File f = LittleFS.open(FILENAME_SERIAL_REPLAY, FILE_APPEND);
#endif
  if (!f) return;
  char cline[INFOLEN];
  s.toCharArray(cline, INFOLEN);
  f.printf("%s\n", cline);
  f.close();
  return;
}

void persistence_replay_serial(void)
{
  if (!hasFFat) return;
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(FILENAME_SERIAL_REPLAY, FILE_READ);
#else
  File f = LittleFS.open(FILENAME_SERIAL_REPLAY, FILE_READ);
#endif
  if (!f) return;
  while (f.available())
  {
    String line = f.readStringUntil('\n');
    serial_process_command(line, "REPLAY: ", false);
  }
  f.close();
  return;
}

void persistence_reset_replay_serial(void)
{
#ifdef ROLORAN_USE_FFAT
  if (hasFFat) FFat.remove(FILENAME_SERIAL_REPLAY);
#else
  if (hasFFat) LittleFS.remove(FILENAME_SERIAL_REPLAY);
#endif
  return;
}

uint16_t get_next_rdcp_sequence_number(uint16_t origin)
{
  uint16_t seq = 1;
  if (!hasFFat) return seq;
  char fn[INFOLEN];
  snprintf(fn, INFOLEN, "%s%04X", FILENAME_PREFIX_SEQNR, origin);
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(fn, FILE_READ);
#else
  File f = LittleFS.open(fn, FILE_READ);
#endif
  if (!f)
  {
    serial_writeln("WARNING: Missing sequence number file, starting with defaults");
    set_next_rdcp_sequence_number(origin, 2);
    return seq;
  }
  String line = "0";
  if (f.available())
  {
    line = f.readString();
  }
  seq = line.toInt();
  f.close();
  if (seq == 0) serial_writeln("WARNING: Existing sequence number file yielded 0");
  set_next_rdcp_sequence_number(origin, seq+1);
  return seq;
}

uint16_t set_next_rdcp_sequence_number(uint16_t origin, uint16_t seq)
{
  if (!hasFFat) return seq;
  char fn[INFOLEN];
  snprintf(fn, INFOLEN, "INFO: Persisting next-up seqnr %u for %04X", seq, origin);
  serial_writeln(fn);
  snprintf(fn, INFOLEN, "%s%04X", FILENAME_PREFIX_SEQNR, origin);
#ifdef ROLORAN_USE_FFAT
  FFat.remove(fn);
  File f = FFat.open(fn, FILE_WRITE);
#else
  LittleFS.remove(fn);
  File f = LittleFS.open(fn, FILE_WRITE);
#endif
  if (!f) return seq;
  char content[INFOLEN];
  snprintf(content, INFOLEN, "%" PRIu16 "\n", seq);
  f.print(content);
  f.close();
  delay(1);
  return seq;
}

bool persistence_checkset_nonce(char *name, uint16_t nonce)
{
  if (!hasFFat) return false;
  bool is_valid = false;
  char filename[INFOLEN];
  snprintf(filename, INFOLEN, "/%s.nce", name);
#ifdef ROLORAN_USE_FFAT
  File f = FFat.open(filename, FILE_READ);
#else
  File f = LittleFS.open(filename, FILE_READ);
#endif
  if (!f)
  {
    is_valid = true; // never seen a nonce for this type before
  }
  else
  {
    String line = "0";
    if (f.available())
    {
      line = f.readString();
    }
    uint16_t old_nonce = line.toInt();
    f.close();
    if (old_nonce < nonce)
    {
      is_valid = true;
    }
    else 
    {
      char info[INFOLEN];
      snprintf(info, INFOLEN, "WARNING: Old nonce == %" PRIu16 ", new nonce == %" PRIu16, old_nonce, nonce);
      serial_writeln(info);
    }
  }
  if (is_valid)
  {
#ifdef ROLORAN_USE_FFAT
    f = FFat.open(filename, FILE_WRITE);
#else
    f = LittleFS.open(filename, FILE_WRITE);
#endif
    if (!f) 
    { 
      is_valid = false; // cannot persist nonce, don't trust it
      serial_writeln("ERROR: Cannot persist nonce");
    }
    char content[INFOLEN];
    snprintf(content, INFOLEN, "%" PRIu16 "\n", nonce);
    f.print(content);
    f.close();
  }
  return is_valid;
}

/* EOF */