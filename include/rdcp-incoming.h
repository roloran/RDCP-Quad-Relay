#ifndef _RDCP_INCOMING
#define _RDCP_INCOMING

/**
 * Main processing function. Handle an incoming LoRa packet. 
 * Called whenever something was received on either channel 
 * for real or via SIMRX. 
 */
void rdcp_handle_incoming_lora_message(void);

#endif
/* EOF */