#include <Arduino.h>
#include <RadioLib.h>
#include "Base64ren.h"
#include <SPI.h>
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "hal.h"
#include "rdcp-incoming.h"
#include "rdcp-scheduler.h"
#include "rdcp-neighbors.h"
#include "rdcp-memory.h"
#include "rdcp-callbacks.h"
#include "rdcp-commands.h"
#include "rdcp-beacon.h"

SET_LOOP_TASK_STACK_SIZE(16*1024); // default of 8 kb is not enough

extern da_config CFG;             // Configuration data

/**
 * Initial setup after power-on
 */
void setup() 
{
  delay(100);
  setup_serial();                 // Set up the Serial/UART connection
  delay(900);
  setup_lora_hardware();          // Set up the SPI-connected SX126x chips
  setup_radio();                  // Initialize RadioLib
  setup_persistence();            // Set up FFat persistence
  persistence_replay_serial();    // Set up device configuration based on stored commands
  if (CFG.bt_enabled) enable_bt();// Set up BT access
  rdcp_memory_restore();          // Load persisted memories
  rdcp_duplicate_table_restore(); // Load persisted duplicate table entries
  serial_banner();                // Show current device configuration over Serial
  serial_writeln("READY");        // Signal LoRa modem readiness
}

/**
 * Keep some information globally to simplify loop task stack
 */
int64_t  minute_timer = 0;
int      minute_counter = 0;
String   serial_string;
bool     shall_process_new_message = false;
int64_t  reboot_requested = RDCP_TIMESTAMP_ZERO;
bool     seqnr_reset_requested = false;
uint16_t new_delivery_receipt_from = RDCP_ADDRESS_SPECIAL_ZERO;
int64_t  last_periodic_chain_finish = RDCP_TIMESTAMP_ZERO;
bool     has_initially_fetched = false;
int32_t  old_free_heap = ESP.getFreeHeap();
int32_t  old_min_free_heap = ESP.getMinFreeHeap();
int32_t  free_heap = 0;
int32_t  min_free_heap = 0;
char     info[INFOLEN];
bool     rinse_and_repeat = false;

extern lora_message lorapacket_in_sim, lorapacket_in_433, lorapacket_in_868, current_lora_message;
extern callback_chain CC[NUM_TX_CALLBACKS];
extern da_config CFG;
extern bool currently_in_fetch_mode;
extern bool rtc_active;
extern int64_t last_dasresp_sent;

/**
 * Loop task for periodic actions
 */
void loop() 
{
  loop_radio(); // periodically let the LoRa radios do their work

  serial_string = serial_readln(); 
  if (serial_string.length() != 0) serial_process_command(serial_string); // Process any new Serial commands
  
  if (lorapacket_in_433.available)
  {
    memcpy(&current_lora_message, &lorapacket_in_433, sizeof(lora_message));
    lorapacket_in_433.available = false;
    rdcp_handle_incoming_lora_message();
    rinse_and_repeat = true;
  }

  if (lorapacket_in_868.available)
  {
    memcpy(&current_lora_message, &lorapacket_in_868, sizeof(lora_message));
    lorapacket_in_868.available = false;
    rdcp_handle_incoming_lora_message();
    rinse_and_repeat = true;
  }

  if (lorapacket_in_sim.available)
  {
    memcpy(&current_lora_message, &lorapacket_in_sim, sizeof(lora_message));
    lorapacket_in_sim.available = false;
    rdcp_handle_incoming_lora_message();
    rinse_and_repeat = true;
  }

  if (rinse_and_repeat) 
  { 
    delay(1); 
    rinse_and_repeat = false;
    return; 
  }

  rdcp_txqueue_loop(); // Periodically let the TX scheduler do its work

  if ((minute_timer == 0) || (my_millis() - minute_timer > (60*SECONDS_TO_MILLISECONDS)))
  {
    cpu_fast();
    minute_timer = my_millis();
    rdcp_check_heartbeat(); // Check whether we should send a DA Heartbeat
    rdcp_periodic_kickstart(); // Check whether we should start a periodic868 chain
    minute_counter++;
    if (minute_counter == 2)
    { /* Warning: Executed after 2, 32, ... minutes, not only once, as minute_counter is reset every 30 minutes. */
      if (!has_initially_fetched)
      {
        has_initially_fetched = true;
        if (CFG.fetch_enabled)
        {
          currently_in_fetch_mode = true;
          rdcp_command_fetch_from_neighbor(); // Fetch All New Messages from configured neighbor
        }
      }
      if ((CFG.unsolicited_dasrep_timer > 0) && (my_millis() > last_dasresp_sent + CFG.unsolicited_dasrep_timer))
      {
        rdcp_cmd_send_da_status_response(true); // send unsolicited DA Status Response
      }
    }
    if (minute_counter == 30)
    {
      minute_counter = 0;
      rdcp_memory_persist(); // Store current memories in case of power loss 
      rdcp_duplicate_table_persist();

      free_heap = ESP.getFreeHeap();
      min_free_heap = ESP.getMinFreeHeap();
      if ((free_heap < old_free_heap) || (min_free_heap < old_min_free_heap))
      {
        snprintf(info, INFOLEN, "WARNING: Free heap dropped from %d/%d to %d/%d", 
          old_min_free_heap, old_free_heap, min_free_heap, free_heap);
        serial_writeln(info);
        old_min_free_heap = min_free_heap;
        old_free_heap = free_heap;
        if (free_heap < 8192)
        {
          serial_writeln("ERROR: OUT OF MEMORY - restarting as countermeasure");
          delay(5 * SECONDS_TO_MILLISECONDS);
          ESP.restart();
        }
      }
    }
    if (currently_in_fetch_mode) rdcp_check_fetch_timeout();
  }
  
  /* Check all callback chains for timeouts and trigger the callbacks */
  if ((CC[TX_CALLBACK_FETCH_SINGLE].in_use) &&
      (CC[TX_CALLBACK_FETCH_SINGLE].timeout < my_millis()))
        rdcp_chain_callback(TX_CALLBACK_FETCH_SINGLE, true);
  if ((CC[TX_CALLBACK_FETCH_ALL].in_use) &&
      (CC[TX_CALLBACK_FETCH_ALL].timeout < my_millis()))
        rdcp_chain_callback(TX_CALLBACK_FETCH_ALL, true);
  if ((CC[TX_CALLBACK_PERIODIC868].in_use) &&
      (CC[TX_CALLBACK_PERIODIC868].timeout < my_millis()))
        rdcp_chain_callback(TX_CALLBACK_PERIODIC868, true);

  /* Delayed restart triggered by RDCP Infrastructure Reset */
  if ((reboot_requested > 0) && (my_millis() > reboot_requested))
  {
    if (seqnr_reset_requested) set_next_rdcp_sequence_number(CFG.rdcp_address, 1); // Reset own sequence numbers
    delay(1 * SECONDS_TO_MILLISECONDS);
    ESP.restart();
  }

  delay(1); // for background tasks such as watchdogs
  if (rtc_active) rdcp_cmd_check_rtc();
  rdcp_beacon();

  cpu_slow();
  return;
}

/* EOF */