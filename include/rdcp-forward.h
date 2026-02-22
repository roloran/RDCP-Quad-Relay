#ifndef _RDCP_FORWARD
#define _RDCP_FORWARD

#include <Arduino.h>

/**
 * Check whether a received RDCP Message should be forwarded on the 868 MHz channel. 
 * @return true if forwarding is recommended, false otherwise 
 */
bool rdcp_check_forward_868_relevance(void);

/**
 * Check whether a received RDCP Message should be sent explicitly to the DA via Serial. 
 * @return true if explicit sending is recommended, false otherwise 
 */
bool rdcp_check_forward_da_relevance(void);

#define FORWARD_DELAY_NONE         0
#define FORWARD_DELAY_SHORT        1
#define FORWARD_DELAY_PROPORTIONAL 2
/**
 * Schedule a received RDCP Message for forwarding on the 868 MHz channel. 
 * @param add_random_delay 0 to send ASAP, 1 for short delay, 2 for proportional delay
 */
void rdcp_forward_schedule(int add_random_delay);

/**
 * Send the RDCP Message expliticly to the DA via Serial/UART. 
 */
void rdcp_msg_to_da_via_serial(void);

#endif 
/* EOF */