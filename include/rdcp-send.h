#ifndef _RDCP_SEND 
#define _RDCP_SEND 

#include <Arduino.h>

/**
 * Start gracefully transmitting the currently `tx_ongoing` RDCP Message
 * by applying Channel Activity Detection first, which might seriously
 * delay the start of the transmission.
 */
void rdcp_send_message_cad(uint8_t channel);

/**
 * Send the `tx_ongoing` RDCP Message from TX Queue now. 
 * This is implicitly used for `force_tx`-scheduled messages or when
 * Channel Activity Detection has delayed for too long without
 * non-important messages getting dropped in the process.
 * @param channel Either CHANNEL433 or CHANNEL868
 */
void rdcp_send_message_force(uint8_t channel);

/**
 * Callback when a LoRa TX event has finished. This is an additional
 * RDCP-specific callback function to be called by the underlying
 * LoRa library. It automatically re-transmits an RDCP Message if its
 * previous retransmission counter was > 0 (after adjusting its Checksum)
 * or removes it from the TX Queue at the end of a timeslot. 
 */
void rdcp_callback_txfin(uint8_t channel);

/**
 * Callback when a LoRa CAD event has results. If the channel is free,
 * transmission of the `tx_ongoing` RDCP Message starts. Otherwise,
 * repeated CAD attempts and re-scheduling processes are triggered
 * according to RDCP specifications.
 * @return true if the message is being sent now, false if CAD delays
 */
bool rdcp_callback_cad(uint8_t channel, bool cad_busy);

#endif 
/* EOF */