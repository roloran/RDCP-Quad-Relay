#include <Arduino.h>

/**
 * Print an RDCPCSV line for the most recent message.
 */
void print_rdcp_csv(void);
void print_rdcp_csv_out(uint8_t channel, int txqidx);

/**
 * Enable or disable RDCPCSV logging.
 */
void rdcpcsv_logfile_set_status(bool enabled);

/**
 * Delete the current RDCPCSV logfile.
 */
void rdcpcsv_logfile_delete(void);

/**
 * Dump the current RDCPCSV logfile.
 */
void rdcpcsv_logfile_dump(void);

/* EOF */