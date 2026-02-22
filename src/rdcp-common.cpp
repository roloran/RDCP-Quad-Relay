#include "rdcp-common.h"
#include "lora.h"
#include "hal.h"
#include "serial.h"
#ifdef ROLORAN_USE_FFAT
#include "FFat.h"
#else
#include <LittleFS.h>
#endif

rdcp_message rdcp_msg_in;
int64_t CFEst[NUMCHANNELS] = {0, 0}; 
extern da_config CFG; 
extern lora_message current_lora_message;
struct rdcp_dup_table dupe_table;              // One global RDCP Message Duplicate Table

uint16_t most_recent_airtime = RDCP_TIMESTAMP_ZERO;
uint8_t  most_recent_future_timeslots = 0;
int64_t  contributed_propagation_cycle_end = RDCP_TIMESTAMP_ZERO;

tracked_propagation_cycle propagation_cycles[MAX_TRACKED_PCS];

#define FILENAME_DUPETABLE "/dupetable"
bool do_not_persist_dupetable = false;

int64_t rdcp_get_channel_free_estimation(uint8_t channel)
{
  return CFEst[channel];
}

bool rdcp_set_channel_free_estimation(uint8_t channel, int64_t new_value)
{
  CFEst[channel] = new_value;
  return true; 
}

bool rdcp_update_channel_free_estimation(uint8_t channel, int64_t new_value)
{
  if (rdcp_get_channel_free_estimation(channel) < new_value)
  {
    rdcp_set_channel_free_estimation(channel, new_value);
    return true;
  }
  return false;
}

uint8_t rdcp_get_default_retransmission_counter_for_messagetype(uint8_t mt)
{
  uint8_t nrt = NRT_LEVEL_LOW;
  if ( (mt == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) || (mt == RDCP_MSGTYPE_ACK) ||
       (mt == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) ) nrt = NRT_LEVEL_MIDDLE;
  if ( (mt == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || (mt == RDCP_MSGTYPE_CITIZEN_REPORT) ||
       (mt == RDCP_MSGTYPE_SIGNATURE) ) nrt = NRT_LEVEL_HIGH;
  return nrt;
}

void rdcp_update_cfest_in(uint16_t origin, uint16_t seqnr)
{
  uint16_t airtime = airtime_in_ms(current_lora_message.channel, RDCP_HEADER_SIZE + rdcp_msg_in.header.rdcp_payload_length);
  uint16_t airtime_with_buffer = airtime + RDCP_TIMESLOT_BUFFERTIME;

  uint32_t remaining_current_sender_time = airtime_with_buffer * rdcp_msg_in.header.counter;

  uint8_t nrt = NRT_LEVEL_LOW;
  uint8_t mt = rdcp_msg_in.header.message_type;
  if ( (mt == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) || (mt == RDCP_MSGTYPE_ACK) ||
       (mt == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) ) nrt = NRT_LEVEL_MIDDLE;
  if ( (mt == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || (mt == RDCP_MSGTYPE_CITIZEN_REPORT) ||
       (mt == RDCP_MSGTYPE_SIGNATURE) ) nrt = NRT_LEVEL_HIGH;

  uint32_t timeslot_duration = (nrt+1) * airtime_with_buffer;

  uint8_t future_timeslots = 0;

  if ((rdcp_msg_in.header.sender < RDCP_ADDRESS_MG_LOWERBOUND) && (rdcp_msg_in.header.sender >= RDCP_ADDRESS_BBKDA_LOWERBOUND))
  { // DA or BBK sending
    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x00) future_timeslots = 8;
    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x02) future_timeslots = 7;
    if ((rdcp_msg_in.header.relay1 & 0x0F) == 0x03) future_timeslots = 4; // Third Hop 1 assigned with Delay 3
    if ((rdcp_msg_in.header.relay2 & 0x0F) == 0x04) future_timeslots = 6; // Second Hop 4 assigned with Delay 4
    if (rdcp_msg_in.header.relay1 == 0xE4) future_timeslots = 5;
    if (rdcp_msg_in.header.relay1 == 0xE2) future_timeslots = 3;
    if (rdcp_msg_in.header.relay1 == 0xE1) future_timeslots = 2;
    if (rdcp_msg_in.header.relay1 == 0xE0) future_timeslots = 1;
    if (rdcp_msg_in.header.relay1 == 0xEE) future_timeslots = 0;
  }
  else
  { // other device sending, not leading to relay on same channel
    future_timeslots = 0;
  }

  uint32_t channel_free_after = remaining_current_sender_time + future_timeslots * timeslot_duration;
  int64_t channel_free_at = my_millis() + channel_free_after;
  most_recent_future_timeslots = future_timeslots;

  rdcp_update_channel_free_estimation(current_lora_message.channel, channel_free_at);
  if (current_lora_message.channel == CHANNEL433) rdcp_track_propagation_cycles(channel_free_at, origin, seqnr, PC_STATUS_KNOWN);

  char buf[INFOLEN];
  snprintf(buf, INFOLEN, "INFO: Channel %d CFEst4current (in): +%zu ms, @%llu ms (airtime %u ms, retrans %zu ms, timeslot %zu ms, %d fut ts)", 
    (current_lora_message.channel == CHANNEL433) ? 433 : 868, 
    channel_free_after, channel_free_at, airtime, remaining_current_sender_time, timeslot_duration, future_timeslots);
  serial_writeln(buf);

  return;
}

void rdcp_track_propagation_cycles(int64_t channel_free_at, uint16_t origin, uint16_t seqnr, uint8_t status)
{
  int index = RDCP_INDEX_NONE;
  int64_t now = my_millis();
  bool newpc = false;
  char info[INFOLEN];

  for (int i=0; i < MAX_TRACKED_PCS; i++)
  {
    if (propagation_cycles[i].timestamp_end < now) propagation_cycles[i].status = PC_STATUS_NONE;
    if ((origin == propagation_cycles[i].origin) && 
        (seqnr == propagation_cycles[i].seqnr)   &&
        (PC_STATUS_NONE != propagation_cycles[i].status)) index = i;
  }

  if (index == RDCP_INDEX_NONE)
  {
    newpc = true;
    for (int i=0; i < MAX_TRACKED_PCS; i++)
    {
      if (propagation_cycles[i].status == PC_STATUS_NONE) { index = i; break; }
    }
  }

  if (index == RDCP_INDEX_NONE)
  {
    serial_writeln("WARNING: Failed to track propagation cycle");
    return;
  }

  if (channel_free_at <= now) return; 

  propagation_cycles[index].origin = origin;
  propagation_cycles[index].seqnr = seqnr;
  if (newpc)
  { 
    propagation_cycles[index].timestamp_known = now;
    propagation_cycles[index].status = status;
    propagation_cycles[index].timestamp_end = channel_free_at;
  }
  else 
  {
    if (status > propagation_cycles[index].status) propagation_cycles[index].status = status;
    if (channel_free_at > propagation_cycles[index].timestamp_end) propagation_cycles[index].timestamp_end = channel_free_at;
  }

  for (int i=0; i < MAX_TRACKED_PCS; i++)
  {
    if (propagation_cycles[i].status != PC_STATUS_NONE)
    {
      snprintf(info, INFOLEN, "INFO: Propagation cycle i%d %04X-%04X ongoing for %" PRId64 " ms", 
        i, 
        propagation_cycles[i].origin, propagation_cycles[i].seqnr,
        propagation_cycles[i].timestamp_end - now);
      serial_writeln(info);
    }
  }

  return;
}

int rdcp_get_number_of_tracked_propagation_cycles(void)
{
  int result = 0;
  int64_t now = my_millis();
  char info[INFOLEN];

  for (int i=0; i < MAX_TRACKED_PCS; i++)
  {
    if ((propagation_cycles[i].status != PC_STATUS_NONE) &&
        (propagation_cycles[i].timestamp_end > now))
    {
      result++;
      snprintf(info, INFOLEN, "INFO: Tracking propagation cycle i%d %04X-%04X, %" PRId64 " ms, as %s (known for %" PRId64 " ms)",
        i, propagation_cycles[i].origin, propagation_cycles[i].seqnr, 
        propagation_cycles[i].timestamp_end - now,
        propagation_cycles[i].status == PC_STATUS_CONTRIBUTOR ? "contributor" : "listener", 
        now - propagation_cycles[i].timestamp_known);
      serial_writeln(info);
    }
  }

  return result;
}
void rdcp_update_cfest_out(uint8_t channel, uint8_t len, uint8_t rcnt, uint8_t mt, uint8_t relay1, uint8_t relay2, uint8_t relay3, uint16_t origin, uint16_t seqnr)
{
  uint16_t airtime = airtime_in_ms(channel, len);
  uint16_t airtime_with_buffer = airtime + RDCP_TIMESLOT_BUFFERTIME;

  uint32_t remaining_current_sender_time = airtime_with_buffer * (rcnt+1);

  uint8_t nrt = NRT_LEVEL_LOW;
  if ( (mt == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) || (mt == RDCP_MSGTYPE_ACK) ||
       (mt == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) ) nrt = NRT_LEVEL_MIDDLE;
  if ( (mt == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || (mt == RDCP_MSGTYPE_CITIZEN_REPORT) ||
       (mt == RDCP_MSGTYPE_SIGNATURE) ) nrt = NRT_LEVEL_HIGH;

  uint32_t timeslot_duration = (nrt+1) * airtime_with_buffer;

  uint8_t future_timeslots = 0;

  if (channel == CHANNEL433)
  { // consider propagation cycle
    if ((relay1 & 0x0F) == 0x00) future_timeslots = 8;
    if ((relay1 & 0x0F) == 0x02) future_timeslots = 7;
    if ((relay1 & 0x0F) == 0x03) future_timeslots = 4; // Third Hop 1 assigned with Delay 3
    if ((relay2 & 0x0F) == 0x04) future_timeslots = 6; // Second Hop 4 assigned with Delay 4
    if (relay1 == 0xE4) future_timeslots = 5;
    if (relay1 == 0xE2) future_timeslots = 3;
    if (relay1 == 0xE1) future_timeslots = 2;
    if (relay1 == 0xE0) future_timeslots = 1;
    if (relay1 == 0xEE) future_timeslots = 0;
  }
  else
  { // no propagation cycle to consider
    future_timeslots = 0;
  }

  uint32_t channel_free_after = remaining_current_sender_time + future_timeslots * timeslot_duration;
  int64_t channel_free_at = my_millis() + channel_free_after;

  contributed_propagation_cycle_end = channel_free_at;

  rdcp_update_channel_free_estimation(channel, channel_free_at);
  if (channel == CHANNEL433) rdcp_track_propagation_cycles(channel_free_at, origin, seqnr, PC_STATUS_CONTRIBUTOR);

  char buf[INFOLEN];
  snprintf(buf, INFOLEN, "INFO: Channel %d CFEst4current (out): +%zu ms, @%llu ms (airtime %u ms, retrans %zu ms, timeslot %zu ms, %d fut ts)", 
    (channel == CHANNEL433) ? 433 : 868, 
    channel_free_after, channel_free_at, airtime, remaining_current_sender_time, timeslot_duration, future_timeslots);
  serial_writeln(buf);

  return;
}

bool rdcp_propagation_cycle_duplicate(void)
{
  int64_t now = my_millis();
  char info[INFOLEN];

  /* 
     We are within an ongoing propagation cycle if we actively contributed to it 
     by relaying an RDCP Message and are not close to the CFEst value determined 
     when we sent it.
  */
  if ((now + 1 * SECONDS_TO_MILLISECONDS) < contributed_propagation_cycle_end)
  {
    snprintf(info, INFOLEN, "INFO: Contributed propagation cycle ends in %" PRId64 " ms", contributed_propagation_cycle_end - now);
    serial_writeln(info);
    return true;
  }

  if (rdcp_get_number_of_tracked_propagation_cycles() > 1) 
  {
    serial_writeln("INFO: Multiple parallel propagation cycles tracked");
    return true;
  }

  return false;
}

uint16_t airtime_in_ms(uint8_t channel, uint8_t payload_size)
{
  uint16_t time_for_packet = 0;

  uint32_t bandwidth_in_hz = (uint32_t) CFG.lora[channel].bw * 1000;
  uint8_t  low_data_rate_optimization = 1;
  uint8_t  implicit_header_mode = 0;
  uint8_t  coding_rate = CFG.lora[channel].cr - 4;
  uint8_t  SF = CFG.lora[channel].sf;

  double time_per_symbol = pow(2, SF) / bandwidth_in_hz;

  /* Calculate the airtime for the preamble */
  uint8_t number_of_preamble_symbols = CFG.lora[channel].pl;
  double time_for_preamble = (number_of_preamble_symbols + 4.25) * time_per_symbol;

  /* Calculate the airtime for the payload */
  double number_of_payload_symbols = 8 + max((coding_rate + 4)*ceil((8 * payload_size - 4 * SF + 28 + 16 - 20 * implicit_header_mode) / (4*(SF - 2*low_data_rate_optimization))), 0.0);
  double time_for_payload = number_of_payload_symbols * time_per_symbol;

  /* Sum it up, converting from seconds to milliseconds and from Double to Int */
  time_for_packet = (uint16_t) (1000 * (time_for_preamble + time_for_payload));
  most_recent_airtime = time_for_packet;

  return time_for_packet;
}

int64_t rdcp_get_timeslot_duration(uint8_t channel, uint8_t *data)
{
  int64_t duration = 0;

  struct rdcp_header h;
  memcpy(&h, data, RDCP_HEADER_SIZE);

  uint16_t airtime = airtime_in_ms(channel, RDCP_HEADER_SIZE + h.rdcp_payload_length);
  uint16_t airtime_with_buffer = airtime + RDCP_TIMESLOT_BUFFERTIME;

  uint8_t nrt = NRT_LEVEL_LOW;
  uint8_t mt = h.message_type;
  if ( (mt == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) || (mt == RDCP_MSGTYPE_ACK) ||
       (mt == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) ) nrt = NRT_LEVEL_MIDDLE;
  if ( (mt == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) || (mt == RDCP_MSGTYPE_CITIZEN_REPORT) ||
       (mt == RDCP_MSGTYPE_SIGNATURE) ) nrt = NRT_LEVEL_HIGH;

  duration = (nrt+1) * airtime_with_buffer;

  return duration;
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
  return;
}

void rdcp_dump_duplicate_message_table(void)
{
  char info[INFOLEN];
  for (int i=0; i != NUM_DUPETABLE_ENTRIES; i++)
  {
    if (dupe_table.tableentry[i].origin == 0) continue;
    snprintf(info, INFOLEN, "INFO: Dupe table entry %i: %04X with seqnr %04X",
      i,
      dupe_table.tableentry[i].origin,
      dupe_table.tableentry[i].sequence_number);
    serial_writeln(info);
  }
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
  return;
}

void rdcp_duplicate_table_delete_file(void)
{
  serial_writeln("INFO: Deleting duplicate table file");
  LittleFS.remove(FILENAME_DUPETABLE);
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
  return;
}

bool rdcp_check_duplicate_message(uint16_t origin, uint16_t sequence_number)
{
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
    return false;
  }
  else
  {
    dupe_table.tableentry[pos].last_seen = my_millis();
    if (dupe_table.tableentry[pos].sequence_number < sequence_number)
    { // update highest sequence number
      dupe_table.tableentry[pos].sequence_number = sequence_number;
      return false;
    }
    else
    { // duplicate found
      return true;
    }
  }
  return false;
}

bool rdcp_check_crc_in(uint8_t real_packet_length)
{
  uint8_t data_for_crc[INFOLEN];

  /* Copy RDCP header and payload into data structure for CRC calculation */
  memcpy(&data_for_crc, &rdcp_msg_in.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
  for (int i=0; i < real_packet_length - RDCP_HEADER_SIZE; i++)
  {
    data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rdcp_msg_in.payload.data[i];
  }

  /* Calculate and check CRC */
  uint16_t actual_crc = crc16(data_for_crc, real_packet_length - RDCP_CRC_SIZE);

  if (actual_crc == rdcp_msg_in.header.checksum)
  {
    return true;
  }

  return false;
}

uint16_t crc16(uint8_t *data, uint16_t len)
{
    uint16_t lookup[] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108,
        0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
        0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B,
        0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
        0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE,
        0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
        0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D,
        0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5,
        0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
        0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4,
        0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
        0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13,
        0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
        0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E,
        0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1,
        0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
        0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0,
        0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
        0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657,
        0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
        0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882,
        0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E,
        0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
        0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D,
        0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
        0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

    uint16_t crc = 0xFFFF;
    for (int i=0; i < len; i++)
    {
        uint8_t b = data[i];
        crc = (crc << 8) ^ lookup[(crc >> 8) ^ b];
        crc &= 0xFFFF;
    }
    return crc;
}

uint8_t rdcp_get_ack_from_infrastructure_status(void)
{
    if (CFG.infrastructure_status == RDCP_INFRASTRUCTURE_MODE_NONCRISIS) 
        return RDCP_ACKNOWLEDGMENT_NEGATIVE;
    if (CFG.infrastructure_status == RDCP_INFRASTRUCTURE_MODE_CRISIS_NOSTAFF)
        return RDCP_ACKNOWLEDGMENT_POSNEG;
    return RDCP_ACKNOWLEDGMENT_POSITIVE;
}

bool rdcp_matches_any_of_my_addresses(uint16_t rdcpa)
{
    if (
          (rdcpa == CFG.rdcp_address) || 
          (rdcpa == RDCP_BROADCAST_ADDRESS) || 
          ((CFG.multicast[0] > 0) && (rdcpa == CFG.multicast[0])) ||
          ((CFG.multicast[1] > 0) && (rdcpa == CFG.multicast[1])) ||
          ((CFG.multicast[2] > 0) && (rdcpa == CFG.multicast[2])) ||
          ((CFG.multicast[3] > 0) && (rdcpa == CFG.multicast[3])) ||
          ((CFG.multicast[4] > 0) && (rdcpa == CFG.multicast[4])) 
       )
        return true;
    return false;
}
/* EOF */