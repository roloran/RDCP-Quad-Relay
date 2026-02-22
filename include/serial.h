#ifndef _ROLORAN_DA_SERIAL
#define _ROLORAN_DA_SERIAL

#include <Arduino.h>
#include "lora.h"

#define FW_SCENARIO   "Neuhaus v2.0"
#define FW_RDCP       "v0.4"
#define FW_VERSION    "002"

#define SERIAL_PREFIX "DA: "

#define INFOLEN 256

/**
 * Open the Serial port for I/O with default settings.
 * To be called once after power-on. 
 */
void setup_serial(void);

/**
 * Print a String on the Serial port.
 * Adds device prefix (SERIAL_PREFIX) if second parameter is set to true.
 * Returns false if Serial port is in "silent mode", true otherwise.
 * @param s String to print on Serial/UART connection to PC
 * @param use_prefix true if the default device prefix is to be used, false for no prefix 
 * @return true if the string was printed on Serial, false if it was suppressed (e.g. due to SERIAL_MODE_SILENT)
 */
void serial_write(String s, bool use_prefix=true);

/**
 * Print a String on the Serial port and add a newline symbol.
 * Same as serial_write(), but with a newline at the end and no return value.
 * @param s String to print on Serial/UART 
 * @param use_prefix true if the default device prefix is to be used, false for no prefix
 */
void serial_writeln(String s, bool use_prefix=true);

/**
 * Convert binary data to Base64 and print it on the Serial port.
 * @param data Binary data to print 
 * @param len Length of binary data in number of bytes 
 * @param add_newline true if a trailing newline should be printed 
 */
void serial_write_base64(char *data, uint8_t len, bool add_newline=false);

/**
 * Read a String from Serial port.
 * Input is expected to consist of one line of text (with a newline symbol at the end).
 * An empty returned String indicates that no input was available (timeout).
 * @return String read from Serial/UART port; empty String if no input is available 
 */
String serial_readln(void);

/**
 * Process a command received via Serial / UART.
 * @param s String with command to process 
 * @param processing_mode String with processing mode, such as "ECHO: " or "REPLAY: "
 * @param persist_selected_commands true if certain commands may be persisted, false otherwise (e.g., during replay of already persisted commands)
 */
void serial_process_command(String s, String processing_mode="ECHO: ", bool persist_selected_commands=true);

/**
 * Print a text banner with some device settings on Serial
 */
void serial_banner(void);

/**
 * Enable BT access
 */
void enable_bt(void);

/**
 * Disable BT access
 */
void disable_bt(void);

#endif
/* EOF */