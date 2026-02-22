#ifndef _RDCP_COMMANDS
#define _RDCP_COMMANDS 

#include <Arduino.h>

/**
 * Handle the command contained in the most recent received RDCP Message.
 */
void rdcp_handle_command(void);

/**
 * Check whether it is time to send a DA Heartbeat message and do so. 
 */
void rdcp_check_heartbeat(void);

/**
 * After power-on, fetch missing OAs and Signatures from designated neighbor. 
 */
void rdcp_command_fetch_from_neighbor(void);

/**
 * When fetching memories from a neighbor, check for timeouts.
 */
void rdcp_check_fetch_timeout(void);

/**
 * Send an RDCP Citizen Report to the HQ. 
 * 
 */
void rdcp_send_cire(uint8_t subtype, uint16_t refnr, char *content);

/**
 * Fetch a single message from designated neighbor. 
 * @param refnr Reference Number of the message to fetch 
 */
void rdcp_command_fetch_one_from_neighbor(uint16_t refnr);

/**
 * Send a DA Status Response.
 * @param unsolicited true if sent without request, false if in response to DA Status Request
 */
void rdcp_cmd_send_da_status_response(bool unsolicited);

struct rtc_entry {
    bool active     = false;
    int64_t alarm   = 0;
    uint8_t restart = 0;
    uint8_t persist = 0;
    char rtc[256];
};

#define MAX_RTC 16

/**
 * Check for RTCs.
 */
void rdcp_cmd_check_rtc(void);

#endif 
/* EOF */