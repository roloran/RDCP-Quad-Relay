#ifndef _RDCP_ROAMING_SUPPORT
#define _RDCP_ROAMING_SUPPORT

#include <Arduino.h>

/**
 * Keep track of when we received the last Heartbeat message from any MG.
 */
void roaming_support_register_mg_heartbeat(void);

/**
 * Keep track of when we most recently sent on CHANNEL868DA.
 */
void roaming_support_register_own_tx(void);

/**
 * Check whether we should send a Roaming Beacon, and do so if necessary.
 */
void roaming_support_check_and_send_beacon(void);

#endif

/* EOF */