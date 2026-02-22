#include <Arduino.h>
#include "serial.h"
#include "Base64ren.h"
#include "persistence.h"
#include "hal.h"
#include "lora.h"
#include "rdcp-memory.h"
#include "rdcp-neighbors.h"
#include "rdcp-commands.h"
#include "rdcp-scheduler.h"
#include "BluetoothSerial.h"
#include "rdcp-csv.h"
// #include <Preferences.h>

lora_message lorapacket_in_sim;
extern da_config CFG;
extern runtime_da_data DART;
BluetoothSerial SerialBT;
bool bt_on = false;
extern int64_t last_dasresp_sent;
// Preferences preferences;

void setup_serial(void)
{
  Serial.begin(115200);
  Serial.setTimeout(10);
  return;
}

void enable_bt(void)
{
  char devicename[NONCENAMESIZE];
  // preferences.begin("RDCP-RL", false);
  // unsigned int btuid = preferences.getUInt("btuid", 1);
  // preferences.putUInt("btuid", btuid+1);
  // preferences.end();
  // snprintf(devicename, NONCENAMESIZE, "RDCP-RL-%04X-%u", CFG.rdcp_address, btuid);
  snprintf(devicename, NONCENAMESIZE, "RDCP-RL-%04X", CFG.rdcp_address);
  SerialBT.begin(devicename);
  SerialBT.setTimeout(10);
  CFG.bt_enabled = true;
  bt_on = true;
  serial_writeln("INFO: Enabling BT access");
  return;
}

void disable_bt(void)
{
  serial_writeln("INFO: Disabling BT access");
  SerialBT.end();
  CFG.bt_enabled = false;
  bt_on = false;
  return;
}

void serial_write(String s, bool use_prefix)
{
  if (use_prefix == true)
  {
    Serial.print(SERIAL_PREFIX + s);
    if (CFG.bt_enabled) SerialBT.print(SERIAL_PREFIX + s);
  }
  else
  {
    Serial.print(s);
    if (CFG.bt_enabled) SerialBT.print(s);
  }
  Serial.flush();
  if (CFG.bt_enabled) SerialBT.flush();
  return;
}

void serial_writeln(String s, bool use_prefix)
{
  serial_write(s + "\n", use_prefix);
  return;
}

void serial_write_base64(char *data, uint8_t len, bool add_newline)
{
  int encodedLength = Base64ren.encodedLength(len);
  char encodedString[encodedLength + 1];
  Base64ren.encode(encodedString, (char *) data, len);
  if (add_newline == true)
  {
    serial_writeln(encodedString, false);
  }
  else
  {
    serial_write(encodedString, false);
  }
  return;
}

String serial_readln(void)
{
  if (CFG.bt_enabled)
  {
    if (SerialBT.available()) 
    {
      return SerialBT.readString();
    }
  }
  return Serial.readString();
}

void serial_banner(void)
{
  Serial.println(SERIAL_PREFIX "INFO: Firmware for the scenario " FW_SCENARIO ", RDCP " FW_RDCP ", build " FW_VERSION ", " __DATE__ " " __TIME__);
  char buf[INFOLEN];
  snprintf(buf, INFOLEN, "%sINFO: Device RDCP address    : %04X (relay id %01X, %s)\0", SERIAL_PREFIX, CFG.rdcp_address, CFG.relay_identifier, CFG.name); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device RDCP multicast  : %04X %04X %04X %04X %04X\0", SERIAL_PREFIX, CFG.multicast[0], CFG.multicast[1], CFG.multicast[2], CFG.multicast[3], CFG.multicast[4]); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device RDCP OA relays  : %01X %01X %01X\0",           SERIAL_PREFIX, CFG.oarelays[0], CFG.oarelays[1], CFG.oarelays[2]); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device RDCP CIRE relays: %01X %01X %01X\0",           SERIAL_PREFIX, CFG.cirerelays[0], CFG.cirerelays[1], CFG.cirerelays[2]); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%SINFO: Device TS4 all ones    : %s",                         SERIAL_PREFIX, CFG.ts4allones ? "enabled" : "disabled"); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%SINFO: Device TS7Relay1 value : %02X",                       SERIAL_PREFIX, CFG.ts7relay1); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device RDCP Fetch from : %04X\0",                     SERIAL_PREFIX, CFG.neighbor_for_fetch); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa frequency  : %.3f MHz, %.3f MHz\0",       SERIAL_PREFIX, CFG.lora[CHANNEL433].freq, CFG.lora[CHANNEL868].freq); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa bandwidth  : %3.0f kHz, %3.0f kHz\0",     SERIAL_PREFIX, CFG.lora[CHANNEL433].bw, CFG.lora[CHANNEL868].bw); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa SF         : %2d, %2d\0",                 SERIAL_PREFIX, CFG.lora[CHANNEL433].sf, CFG.lora[CHANNEL868].sf); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa CR         : 4/%d, 4/%d\0",               SERIAL_PREFIX, CFG.lora[CHANNEL433].cr, CFG.lora[CHANNEL868].cr); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa syncword   : 0x%02X, 0x%02X\0",           SERIAL_PREFIX, CFG.lora[CHANNEL433].sw, CFG.lora[CHANNEL868].sw); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa TX power   : %d dBm, %d dBm\0",           SERIAL_PREFIX, CFG.lora[CHANNEL433].pw, CFG.lora[CHANNEL868].pw); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device LoRa preamble   : %2d symbols, %2d symbols\0", SERIAL_PREFIX, CFG.lora[CHANNEL433].pl, CFG.lora[CHANNEL868].pl); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%sINFO: Device Options         : Relay %s, EP %s, Fwd %s, Fetch %s, Per868 %s, Send %s, BT %s", SERIAL_PREFIX,
    CFG.relay_enabled    ? "+" : "DISABLED",
    CFG.ep_enabled       ? "+" : "DISABLED", 
    CFG.forward_enabled  ? "+" : "DISABLED", 
    CFG.fetch_enabled    ? "+" : "DISABLED", 
    CFG.periodic_enabled ? "+" : "DISABLED", 
    CFG.send_enabled     ? "+" : "DISABLED",
    CFG.bt_enabled       ? "+" : "DISABLED"
  ); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  snprintf(buf, INFOLEN, "%SINFO: Device settings        : HI %" PRId32 "m, MPA %" PRId64 "h, PI %" PRId64 "m", SERIAL_PREFIX,
    CFG.heartbeat_interval / MINUTES_TO_MILLISECONDS, 
    CFG.max_periodic868_age / HOURS_TO_MILLISECONDS, 
    CFG.periodic_interval / MINUTES_TO_MILLISECONDS); Serial.println(buf); if (CFG.bt_enabled) SerialBT.println(buf);
  return;
}

void serial_process_command(String s, String processing_mode, bool persist_selected_commands)
{
  s.trim();
  String s_uppercase = String(s);
  s_uppercase.toUpperCase();
  char info[2*INFOLEN];

  /* Don't echo the Serial command if it starts with '!' */
  if (s.startsWith("!"))
  {
    s = s.substring(1);
    s_uppercase = s_uppercase.substring(1);
  }
  else
  {
    if (s.length() > 1)
    { // echo the command back
      serial_writeln(processing_mode + s);
    }
    else
    { // empty command == show current status
      char status[INFOLEN];
      int64_t now = my_millis();
      int32_t free_heap = ESP.getFreeHeap();
      int32_t min_free_heap = ESP.getMinFreeHeap();
      rdcp_dump_txq(CHANNEL433);
      rdcp_dump_txq(CHANNEL868);

      int32_t uptime_sec = (int32_t) (now/1000);
      int days = uptime_sec / (24 * 3600);
      int seconds_within_today = (uptime_sec - days * 24 * 3600);
      int hours = seconds_within_today / 3600; 
      int seconds_within_this_hour = (seconds_within_today - hours * 3600);
      int minutes = seconds_within_this_hour / 60;
      int seconds = seconds_within_this_hour % 60;

      snprintf(status, INFOLEN, "STATUS: Uptime %" PRId64 " ms (%d days %02d hours %02d minutes %02d seconds), Heap %d/%d", 
        now, days, hours, minutes, seconds, min_free_heap, free_heap);
      serial_writeln(status);
      serial_writeln("READY");
    }
  }

  if (s_uppercase.startsWith("RESET "))
  {
    String p1= s_uppercase.substring(6);
    if (p1.equals(String("CONFIG")))
    {
      serial_writeln("INFO: Removing persisted Serial commands for replay");
      persistence_reset_replay_serial();
    }
    else if (p1.equals(String("RADIO")))
    {
      serial_writeln("INFO: Re-initalizing the radio with current configuration");
      setup_radio();
    }
    else if (p1.equals(String("DUPETABLE")))
    {
      serial_writeln("INFO: Resetting duplicate table");
      rdcp_reset_duplicate_message_table();
    }
    else if (p1.equals(String("SEQNR")))
    {
      serial_writeln("INFO: Resetting own used sequence number");
      set_next_rdcp_sequence_number(CFG.rdcp_address, 1); // Reset own sequence numbers
    }
  } // ^ RESET
 else if (s_uppercase.startsWith("SHOW "))
  {
    String p1= s_uppercase.substring(5);
    if (p1.equals(String("CONFIG")))
    {
      serial_banner();
    }
    else if (p1.equals(String("NEIGHBORS")))
    {
      rdcp_neighbor_dump();
    }
    else if (p1.equals(String("MEMORIES")))
    {
      rdcp_memory_dump();
    }
    else if (p1.equals(String("DUPETABLE")))
    {
      rdcp_dump_duplicate_message_table();
    }
  } // ^ SHOW
  else if (s_uppercase.startsWith("LORAFREQ "))
  {
    // LORAFREQ 433.175 869.525
    // 012345678901234567890123
    String p1 = s.substring(9, 16); // 9 -- 15   == 433 MHz frequency
    String p2 = s.substring(17);    // 17 -- end == 868 MHz frequency
    float f433 = p1.toFloat();
    float f868 = p2.toFloat();
    if ((f433 > 400.0) && (f868 > 800.0))
    {
      CFG.lora[CHANNEL433].freq = f433;
      CFG.lora[CHANNEL868].freq = f868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa frequencies to %.3f MHz and %.3f MHz",
        CFG.lora[CHANNEL433].freq, CFG.lora[CHANNEL868].freq);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORAFREQ command syntax");
    }
  }
  else if (s_uppercase.startsWith("LORABW "))
  {
    // LORABW 125 250
    // 01234567890123
    String p1 = s.substring(7, 10); //  7 -- 9   == 433 MHz bandwidth 
    String p2 = s.substring(11);    // 11 -- end == 868 MHz bandwidth
    float b433 = p1.toFloat();
    float b868 = p2.toFloat();
    if ((b433 > 100) && (b868 > 100))
    {
      CFG.lora[CHANNEL433].bw = b433;
      CFG.lora[CHANNEL868].bw = b868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa bandwidth to %3.0f kHz and %3.0f kHz",
        CFG.lora[CHANNEL433].bw, CFG.lora[CHANNEL868].bw);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORABW command syntax");
    }
  }
  else if (s_uppercase.startsWith("LORASF "))
  {
    // LORASF 07 12
    // 012345678901
    String p1 = s.substring(7, 9);
    String p2 = s.substring(10);
    int s433 = p1.toInt();
    int s868 = p2.toInt();

    if ((s433 > 5) && (s868 > 5))
    {
      CFG.lora[CHANNEL433].sf = s433;
      CFG.lora[CHANNEL868].sf = s868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa spreading factors to %d and %d",
        CFG.lora[CHANNEL433].sf, CFG.lora[CHANNEL868].sf);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORASF command syntax");
    }
  }
  else if (s_uppercase.startsWith("LORACR "))
  {
    // LORACR 8 5
    // 0123456789
    String p1 = s.substring(7,8);
    String p2 = s.substring(9);
    int c433 = p1.toInt();
    int c868 = p2.toInt();
    if ((c433 > 4) && (c868 > 4))
    {
      CFG.lora[CHANNEL433].cr = c433;
      CFG.lora[CHANNEL868].cr = c868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa coding rates to %d and %d",
        CFG.lora[CHANNEL433].cr, CFG.lora[CHANNEL868].cr);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORACR command syntax");
    }
  }
  else if (s_uppercase.startsWith("LORASW "))
  {
    // LORASW 12 34
    // 012345678901
    char buffer1[32], buffer2[32];
    String p1 = s.substring(7,9);
    String p2 = s.substring(10);
    p1.toCharArray(buffer1, 32);
    p2.toCharArray(buffer2, 32);
    uint8_t s433 = strtol(buffer1, NULL, 16);
    uint8_t s868 = strtol(buffer2, NULL, 16);
    if ((s433 > 0) && (s868 > 0))
    {
      CFG.lora[CHANNEL433].sw = s433;
      CFG.lora[CHANNEL868].sw = s868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa sync words to 0x%02X and 0x%02X",
        CFG.lora[CHANNEL433].sw, CFG.lora[CHANNEL868].sw);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORASW command syntax");
    }
  }
  else if (s_uppercase.startsWith("LORAPW "))
  {
    // LORAPW 10 14
    // 012345678901
    String p1 = s.substring(7,9);
    String p2 = s.substring(10);
    int p433 = p1.toInt();
    int p868 = p2.toInt();
    if ((p433 > -15) && (p868 > -15))
    {
      CFG.lora[CHANNEL433].pw = p433;
      CFG.lora[CHANNEL868].pw = p868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa TX power to %d dBm and %d dBm",
        CFG.lora[CHANNEL433].pw, CFG.lora[CHANNEL868].pw);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORAPW command syntax");
    }    
  }
  else if (s_uppercase.startsWith("LORAPL "))
  {
    // LORAPL 15 15
    // 012345678901
    String p1 = s.substring(7, 9);
    String p2 = s.substring(10);
    int p433 = p1.toInt();
    int p868 = p2.toInt();
    if ((p433 > -1) && (p868 > -1))
    {
      CFG.lora[CHANNEL433].pl = p433;
      CFG.lora[CHANNEL868].pl = p868;
      snprintf(info, 2*INFOLEN, "INFO: Changed this device's LoRa preamble length to %d symbols and %d symbols",
        CFG.lora[CHANNEL433].pl, CFG.lora[CHANNEL868].pl);
      serial_writeln(info);
      setup_radio();
      if (persist_selected_commands) persist_serial_command_for_replay(s);
    }
    else 
    {
      serial_writeln("ERROR: Check LORAPL command syntax");
    }    
  }
  else if (s_uppercase.startsWith("RESTART") || s_uppercase.startsWith("REBOOT"))
  {
    serial_writeln("INFO: Restarting device");
    ESP.restart();
  }
  else if (s_uppercase.startsWith("RDCPADDR "))
  {
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_rdcp_address = strtol(buffer, NULL, 16);
    CFG.rdcp_address = new_rdcp_address;
    snprintf(info, INFOLEN, "INFO: Changed this device's RDCP address to %04X", CFG.rdcp_address);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPNEFF "))
  {
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_rdcp_address = strtol(buffer, NULL, 16);
    CFG.neighbor_for_fetch = new_rdcp_address;
    snprintf(info, INFOLEN, "INFO: Changed this device's neighbor to fetch from to %04X", CFG.neighbor_for_fetch);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPRLID "))
  { // RDCPRLID 0
    // Sets own relay identified to 0 ; single-hex-digit required
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint8_t new_relay_id = strtol(buffer, NULL, 16);
    CFG.relay_identifier = new_relay_id;
    snprintf(info, INFOLEN, "INFO: Changed this device's relay identifier to %01X", CFG.relay_identifier);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPNUMRL "))
  { // RDCPNUMRL 10
    // Sets total number of relays in RDCP scenario
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint8_t new_relay_num = strtol(buffer, NULL, 10);
    CFG.scenario_num_relays = new_relay_num;
    snprintf(info, INFOLEN, "INFO: Changed number of relays in RDCP scenario to %d", CFG.scenario_num_relays);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPRLOA "))
  { // RDCPRLOA 123
    // Sets the other relays to use for OAs; 3-hex-digit (other relay's identifiers)
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint32_t oa_relays = strtol(buffer, NULL, 16);
    CFG.oarelays[0] = (oa_relays & 0x00000F00) >> 8;
    CFG.oarelays[1] = (oa_relays & 0x000000F0) >> 4;
    CFG.oarelays[2] = (oa_relays & 0x0000000F);
    snprintf(info, INFOLEN, "INFO: Changed this device's OA relay peers to %01X, %01X, %01X", CFG.oarelays[0], CFG.oarelays[1], CFG.oarelays[2]);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPRLCR "))
  { // RDCPRLCR 34E
    // Sets the other relays to use for CIREs; 3-hex-digit (other relay's identifiers)
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint32_t cire_relays = strtol(buffer, NULL, 16);
    CFG.cirerelays[0] = (cire_relays & 0x00000F00) >> 8;
    CFG.cirerelays[1] = (cire_relays & 0x000000F0) >> 4;
    CFG.cirerelays[2] = (cire_relays & 0x0000000F);
    snprintf(info, INFOLEN, "INFO: Changed this device's CIRE relay peers to %01X, %01X, %01X", CFG.cirerelays[0], CFG.cirerelays[1], CFG.cirerelays[2]);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPTS7R "))
  { // RDCPTS7R 0E
    // 01234567890
    // Sets the relay1 header field value to use when relaying in timeslot 7
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint8_t ts7r = strtol(buffer, NULL, 16);
    CFG.ts7relay1 = ts7r;
    snprintf(info, INFOLEN, "INFO: Changed this device's Timeslot7 Relay1 value to %02X", CFG.ts7relay1);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPTS4R"))
  { // Toggles the relay1 header field value to use when relaying in timeslot 4
    CFG.ts4allones = !CFG.ts4allones;
    if (CFG.ts4allones)
    {
      serial_writeln("INFO: Enabled all-relay in final timeslot based on ts4-relaying");
    }
    else 
    {
      serial_writeln("INFO: Disabled all-relay in final timeslot based on ts4-relaying");
    }
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("MULTICAST "))
  { // MULTICAST B000 B001 B002 0000 0000
    // 0123456789012345678901234567890123
    // 0         1         2         3
    String p1 = s.substring(10, 14);
    String p2 = s.substring(15, 19);
    String p3 = s.substring(20, 24);
    String p4 = s.substring(25, 29);
    String p5 = s.substring(30);

    char buffer1[32], buffer2[32], buffer3[32], buffer4[32], buffer5[32];
    p1.toCharArray(buffer1, 32);
    p2.toCharArray(buffer2, 32);
    p3.toCharArray(buffer3, 32);
    p4.toCharArray(buffer4, 32);
    p5.toCharArray(buffer5, 32);
    CFG.multicast[0] = strtol(buffer1, NULL, 16);
    CFG.multicast[1] = strtol(buffer2, NULL, 16);
    CFG.multicast[2] = strtol(buffer3, NULL, 16);
    CFG.multicast[3] = strtol(buffer4, NULL, 16);
    CFG.multicast[4] = strtol(buffer5, NULL, 16);
    snprintf(info, INFOLEN, "INFO: Set multicast addresses to %04X, %04X, %04X, %04X, %04X",
      CFG.multicast[0], CFG.multicast[1], CFG.multicast[2], CFG.multicast[3], CFG.multicast[4]);
    serial_writeln(info);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("TOGGLE"))
  { // TOGGLE RELAY
    // 0123456789...
    String p1 = s_uppercase.substring(7);
    bool failed = false;
    if (p1.equals(String("RELAY")))
    {
      CFG.relay_enabled = !CFG.relay_enabled;
    }
    else if (p1.equals(String("EP")))
    {
      CFG.ep_enabled = !CFG.ep_enabled;
    }
    else if (p1.equals(String("FORWARD")))
    {
      CFG.forward_enabled = !CFG.forward_enabled;
    }
    else if (p1.equals(String("STATUS")))
    {
      CFG.status_enabled = !CFG.status_enabled;
    }
    else if (p1.equals(String("FETCH")))
    {
      CFG.fetch_enabled = !CFG.fetch_enabled;
    }
    else if (p1.equals(String("PERIODIC")))
    {
      CFG.periodic_enabled = !CFG.periodic_enabled;
    }
    else if (p1.equals(String("SEND")))
    {
      CFG.send_enabled = !CFG.send_enabled;
    }
    else 
    {
      failed = true;
      serial_writeln("ERROR: Unknown TOGGLE subcommand");
    }
    if (persist_selected_commands) if (!failed) persist_serial_command_for_replay(s);
    serial_banner();
  }
  else if (s_uppercase.startsWith("BTENABLE"))
  {
    enable_bt();
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("BTDISABLE"))
  {
    if (bt_on)
    {
      disable_bt();
    }
    else 
    {
      CFG.bt_enabled = false;
    }
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("CSVLOGENABLE"))
  {
    rdcpcsv_logfile_set_status(true);
  }
  else if (s_uppercase.startsWith("CSVLOGDISABLE"))
  {
    rdcpcsv_logfile_set_status(false);
  }
  else if (s_uppercase.startsWith("CSVLOGDELETE"))
  {
    rdcpcsv_logfile_delete();
  }
  else if (s_uppercase.startsWith("CSVLOGDUMP"))
  {
    rdcpcsv_logfile_dump();
  }
  else if (s_uppercase.startsWith("HEARTBEAT "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_heartbeat_interval = strtol(buffer, NULL, 10);
    CFG.heartbeat_interval = new_heartbeat_interval * MINUTES_TO_MILLISECONDS;
    serial_writeln("INFO: Changed DA Heartbeat interval to " + p1 + " minutes");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("OMRETRANS "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.memory_retransmissions = new_value;
    serial_writeln("INFO: Changed old memory retransmission counter to " + p1);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("MAXPERAGE "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.max_periodic868_age = new_value * HOURS_TO_MILLISECONDS;
    serial_writeln("INFO: Changed maximum age for Periodic868 to " + p1 + " hours");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("PERIODICS "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.periodic_interval = new_value * MINUTES_TO_MILLISECONDS;
    serial_writeln("INFO: Changed Periodic868 interval to " + p1 + " minutes");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("UNSOLICIT "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.unsolicited_dasrep_timer = new_value * MINUTES_TO_MILLISECONDS;
    serial_writeln("INFO: Changed unsolicited DA Status Response interval to " + p1 + " minutes");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("CORRIDOR "))
  {
    String p1 = s.substring(9);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.corridor_basetime = new_value;
    serial_writeln("INFO: Changed corridor basetime to " + p1 + " seconds");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPSFMUL "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.sf_multiplier = new_value;
    serial_writeln("INFO: Changed RDCP SF multiplier to " + p1);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPSEQNR "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 16);
    set_next_rdcp_sequence_number(CFG.rdcp_address, new_value);
    serial_writeln("INFO: Changed RDCP Sequence Number to " + p1);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("RDCPDUPETABLERESET"))
  {
    rdcp_duplicate_table_delete_all_entries();
  }
  else if (s_uppercase.startsWith("RDCPDUPETABLEDELETE"))
  {
    rdcp_duplicate_table_delete_file();
  }
  else if (s_uppercase.startsWith("RDCPDUPETABLEZAP "))
  {
    String p1 = s.substring(17);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t origin = strtol(buffer, NULL, 16);
    rdcp_duplicate_table_delete_entry(origin);
  }
  else if (s_uppercase.startsWith("RDCPDUPETABLESET "))
  {
    char buffer[32];

    String p1 = s.substring(17, 21);
    p1.toCharArray(buffer, 32);
    uint16_t origin = strtol(buffer, NULL, 16);

    String p2 = s.substring(22);
    p2.toCharArray(buffer, 32);
    uint16_t seqnr = strtol(buffer, NULL, 16);

    rdcp_duplicate_table_set_entry(origin, seqnr);
  }
  else if (s_uppercase.startsWith("MAINTENANCE"))
  {
    serial_writeln("INFO: Preparing manual switch to MAINTENANCE MODE");
    enable_bt();
    serial_writeln("DA_MAINTENANCE");
  }
  else if (s_uppercase.startsWith("SERIAL "))
  {
    String p1 = s.substring(7);
    serial_writeln(p1);
  }
  else if (s_uppercase.startsWith("BEACON433 "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.beacon_interval[CHANNEL433] = new_value * SECONDS_TO_MILLISECONDS;
    serial_writeln("INFO: Changed Beacon433 interval to " + p1 + " seconds");
  }
  else if (s_uppercase.startsWith("BEACON868 "))
  {
    String p1 = s.substring(10);
    char buffer[32];
    p1.toCharArray(buffer, 32);
    uint16_t new_value = strtol(buffer, NULL, 10);
    CFG.beacon_interval[CHANNEL868] = new_value * SECONDS_TO_MILLISECONDS;
    serial_writeln("INFO: Changed Beacon868 interval to " + p1 + " seconds");
  }
  else if (s_uppercase.startsWith("SIMRX "))
  { // SIMRX 433 base64here
    // 01234567890
    String p1 = s.substring(6);
    String p2 = s.substring(10);

    int simfreq = p1.toInt();
    if ((simfreq != 433) && (simfreq != 868))
    {
      serial_writeln("ERROR: Check SIMRX syntax");
      return;
    }

    char buffer[512];
    p2.toCharArray(buffer, 512);
    int b64msg_len = strlen(buffer);
    int decoded_length = Base64ren.decodedLength(buffer, b64msg_len);
    char decoded_string[decoded_length + 1];
    Base64ren.decode(decoded_string, buffer, b64msg_len);

    if (decoded_length > 0)
    {
      uint8_t channel = CHANNEL433;
      if (simfreq == 868) channel = CHANNEL868;
      lorapacket_in_sim.available = true;
      lorapacket_in_sim.channel = channel;
      lorapacket_in_sim.rssi = 100;
      lorapacket_in_sim.snr = 0;
      lorapacket_in_sim.timestamp = my_millis();
      lorapacket_in_sim.payload_length = decoded_length;
      for (int i=0; i != decoded_length; i++) lorapacket_in_sim.payload[i] = decoded_string[i];

      serial_writeln("INFO: LoRa Radio SIM received packet.");
      char serialtext[INFOLEN];
      snprintf(serialtext, INFOLEN, "RXMETA %d %d %d %.3f\0", decoded_length, 0, 100, CFG.lora[channel].freq);
      serial_writeln(serialtext);
      snprintf(serialtext, INFOLEN, "RX %s\0", buffer);
      serial_writeln(serialtext);
    }
    else
    {
      serial_writeln("WARNING: SIMRX empty packet - ignored.");
    }
  }
  else if (s_uppercase.startsWith("RDCPCIRE "))
  {
    // RDCPCIRE 01 12EF str#uctu#red#text
    // 012345678901234567890
    String p1 = s.substring(9, 11);  // Subtype
    String p2 = s.substring(12, 16); // Reference number
    String p3 = s.substring(17);     // Text

    char b1[32], b2[32], b3[256];
    p1.toCharArray(b1, 32);
    p2.toCharArray(b2, 32);
    p3.toCharArray(b3, 256);

    uint8_t subtype = (uint8_t)  strtol(b1, NULL, 16);
    uint16_t refnum = (uint16_t) strtol(b2, NULL, 16);

    char b[INFOLEN];
    snprintf(b, INFOLEN, "INFO: Preparing to send CITIZEN REPORT (subtype %d, refnr %d)", subtype, refnum);
    serial_writeln(b);
    rdcp_send_cire(subtype, refnum, b3);
  }
  else if (s_uppercase.startsWith("RDCPFETCH "))
  {
    // RDCPFETCH 12EF
    // 01234567890123
    String p1 = s.substring(10);
    char b1[32];
    p1.toCharArray(b1, 32);
    uint16_t refnum = (uint16_t) strtol(b1, NULL, 16);
    rdcp_command_fetch_one_from_neighbor(refnum);
  }
  else if (s_uppercase.startsWith("NAME "))
  {
    String p1 = s.substring(5);
    char daname[INFOLEN];
    p1.toCharArray(daname, INFOLEN);
    char buffer[INFOLEN];
    snprintf(buffer, INFOLEN, "INFO: Device name set to %s", daname);
    serial_writeln(buffer);
    snprintf(CFG.name, 64, "%s", daname);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("HQPUBKEY "))
  {
    String p1 = s.substring(9);
    char buffer[INFOLEN];
    p1.toCharArray(buffer, INFOLEN);
    snprintf(CFG.hqpubkey, INFOLEN, "%s", buffer);
    snprintf(buffer, INFOLEN, "INFO: HQ Public Key set");
    serial_writeln(buffer);
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("SHAREDSECRET "))
  {
    String p1 = s.substring(13);
    uint8_t secret[32];
    for (int i=0; i<32; i++) secret[i] = 0;

    char input[128];
    p1.toCharArray(input, 128);

    char hexbyte[3];
    hexbyte[2] = 0;
    for (int i=0; i<32; i++)
    {
      hexbyte[0] = toupper(input[2*i]);
      hexbyte[1] = toupper(input[2*i+1]);
      secret[i] = (uint8_t) strtol(hexbyte, NULL, 16);
    }
    for (int i=0; i<32; i++) CFG.hqsharedsecret[i] = secret[i];

    serial_writeln("INFO: Shared secret with HQ established");
    if (persist_selected_commands) persist_serial_command_for_replay(s);
  }
  else if (s_uppercase.startsWith("BATTERY "))
  {
    // BATTERY 012 034
    // 012345678901234
    String p1 = s.substring(8, 11); //  8 -- 10  == Battery1 value
    String p2 = s.substring(12);    // 12 -- end == Battery2 value
    uint8_t bat1 = p1.toInt();
    uint8_t bat2 = p2.toInt();
    DART.battery1 = bat1;
    DART.battery2 = bat2; 
    snprintf(info, INFOLEN, "INFO: Update reportable battery status to %d and %d", DART.battery1, DART.battery2);
    serial_writeln(info);
  }
  else
  {
    if (s_uppercase.length() > 0) serial_writeln("WARNING: Unknown command");
  }

  return;
}

/* EOF */