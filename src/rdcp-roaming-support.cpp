#include <Arduino.h>
#include "rdcp-roaming-support.h"
#include "serial.h"
#include "rdcp-common.h"
#include "rdcp-commands.h"
#include "rdcp-scheduler.h"
#include "hal.h"

int64_t timestamp_last_mg_heartbeat = RDCP_TIMESTAMP_ZERO;
int64_t timestamp_last_own_tx_channel868da = RDCP_TIMESTAMP_ZERO;
int64_t timestamp_last_beacon_triggered = RDCP_TIMESTAMP_ZERO;

extern da_config CFG;

void roaming_support_register_mg_heartbeat(void)
{
  timestamp_last_mg_heartbeat = my_millis();
  return;
}

void roaming_support_register_own_tx(void)
{
  timestamp_last_own_tx_channel868da = my_millis();
  return;
}

void roaming_support_check_and_send_beacon(void)
{
  /* No roaming beacons in non-crisis mode */
  if (CFG.infrastructure_status == RDCP_INFRASTRUCTURE_MODE_NONCRISIS) return;

  /* Roaming beacons disabled? */
  if (CFG.roaming_beacon_enabled == false) return;

  int64_t now = my_millis();

  /* Do not send roaming beacon if we transmitted less than a minute ago ourselves */
  if (now - timestamp_last_own_tx_channel868da < 1 * MINUTES_TO_MILLISECONDS) return;

  /* Do not send roaming beacon if no MG has sent a hearbeat for clearly more than half an hour */
  if (now - timestamp_last_mg_heartbeat > 35 * MINUTES_TO_MILLISECONDS) return;

  /* Do not send roaming beacons too often */
  if (now - timestamp_last_beacon_triggered < 55 * SECONDS_TO_MILLISECONDS) return;

  timestamp_last_beacon_triggered = now;

  /* Skip sending the roaming beacon if other transmissions on CHANNEL868DA are upcoming already anyway */
  if (get_num_txq_entries(CHANNEL868DA) > 0) return;

  rdcp_send_roaming_beacon();

  return;
}

/* EOF */