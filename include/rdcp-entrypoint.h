#ifndef _RDCP_ENTRYPOINT
#define _RDCP_ENTRYPOINT

#include <Arduino.h>

/**
 * Check whether the currently received RDCP Message designates us as Entry Point. 
 * @return true if we are designated EP, false otherwise 
 */
bool rdcp_check_entrypoint_designation(void);

/**
 * Check whether the currently received RDCP Message has a Message Type that 
 * qualifies for being relayed by an Entry Point. 
 * @return true if valid message type, false otherwise 
 */
bool rdcp_check_entrypoint_messagetype_valid(void);

/**
 * Schedule an RDCP Message received on 868 MHz for relaying on the 433 MHz 
 * channel given that we are the designated Entry Point. 
 */
void rdcp_entrypoint_schedule(void);

/**
 * Send an unsigned DA ACK, e.g., in response to an MG's CIRE. 
 * @param origin Origin RDCP Address to use for the ACK 
 * @param destination Destination RDCP Address, who gets the ACK 
 * @param seqnr Acknowledged Sequence Number (e.g., the CIRE's)
 */
void rdcp_send_ack_unsigned(uint16_t origin, uint16_t destination, uint16_t seqnr);

#endif 
/* EOF */