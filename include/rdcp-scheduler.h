#ifndef _RDCP_SCHEDULER
#define _RDCP_SCHEDULER

#include <Arduino.h> 
#include "rdcp-common.h"

/**
  * Data structure for a TX Queue entry.
  */
struct txqueue_entry {
  uint8_t payload[RDCP_HEADER_SIZE + RDCP_MAX_PAYLOAD_SIZE];      //< data of the outgoing message
  uint8_t payload_length = 0;                   //< length of the outgoing message
  int64_t currently_scheduled_time = RDCP_TIMESTAMP_ZERO;         //< timestamp when to transmit
  int64_t originally_scheduled_time = RDCP_TIMESTAMP_ZERO;        //< timestamp when originally planned to transmit
  uint8_t num_of_reschedules = 0;               //< how often the entry has already been rescheduled
  bool important = false;                       //< message is important and should not be dropped even it if takes longer
  bool force_tx = false;                        //< indicator whether message should be sent independend of CAD status
  uint8_t callback_selector = TX_CALLBACK_NONE; //< which callback function to use when TX is finished
  int64_t timeslot_duration = RDCP_DURATION_ZERO;                 //< timeslot duration in milliseconds including retransmissions
  uint8_t cad_retry = 0;                        //< CAD retry attempt number
  bool waiting = false;                         //< message is still waiting to be sent
  bool in_process = false;                      //< this message is currently being processed
};
  
/// Keep the TX Queue small on purpose. We don't want single devices to block the channel for too long.
#define MAX_TXQUEUE_ENTRIES 32
  
/**
  * Data structure for the overall TX Queue.
  */
struct txqueue {
  uint8_t num_entries = 0;
  struct txqueue_entry entries[MAX_TXQUEUE_ENTRIES];
};
  
/**
  * Data structure for a TX Ahead Queue entry.
  */
struct txaheadqueue_entry {
  uint8_t payload[RDCP_MAX_PAYLOAD_SIZE];       //< data of the outgoing message
  uint8_t payload_length = 0;                   //< length of the outgoing message
  int64_t scheduled_time = RDCP_TIMESTAMP_ZERO; //< timestamp when to move to the TX Queue
  bool important = false;                       //< message is important and should not be dropped even it if takes longer
  bool force_tx = false;                        //< indicator whether message should be sent independend of CAD status
  uint8_t callback_selector = TX_CALLBACK_NONE; //< which callback function to use when TX is finished
  bool waiting = false;                         //< message is still waiting to be sent
};
  
/// The length of the TX Ahead Queue is kept to a reasonable minimum as this queue is currently not in use
#define MAX_TXAHEADQUEUE_ENTRIES 4
  
/**
  * Data structure for the overall TX Ahead Queue.
  */
struct txaheadqueue {
  uint8_t num_entries = 0;
  struct txaheadqueue_entry entries[MAX_TXAHEADQUEUE_ENTRIES];
};

/**
  * Add an outgoing RDCP Message to the TX Queue (TXQ).
  * TXQ is used for messages that are up for transmission very soon, unlike the
  * TX Ahead Queue, which is used for intermediate-term scheduling. 
  * The LoRa payload is given as data along with its length. Parameter `important`
  * is set to true if the message should not be dropped even if re-scheduled too often
  * or delayed too long. `force_tx` prohibits re-scheduling. The `callback_selector`
  * determines which function is called when the message has been sent including all of
  * its retransmissions. If `force_tx` is used, the `forced_time` should be given. 
  * @param channel Either CHANNEL433 or CHANNEL868
  * @param data Complete RDCP Message (header+payload) to schedule
  * @param len Length of data (RDCP Message) in bytes
  * @param important True if the message is so important that it must not be dropped even after multiple re-schedules
  * @param force_tx True if the message must be sent at its scheduled time, i.e., must not be re-scheduled
  * @param callback_selector Number of the callback to trigger after TX, e.g. TX_CALLBACK_NONE
  * @param forced_time Used in combination with force_tx to specify the TX start time
  * @return true if message was accepted, false otherwise (e.g., queue full)
  */
 bool rdcp_txqueue_add(uint8_t channel, uint8_t *data, uint8_t len, bool important, bool force_tx, uint8_t callback_selector, int64_t forced_time);

 /**
   * Re-schedule the entries in the TX Queue because CFEst has changed meanwhile (offset=0) or by a given offset.
   * Returns true if entries were dropped due to resecheduling, or false if all entries are still there.
   * Calling this function over and over again implicitly drops messages not marked as important and can
   * free up space in the TX Queue.
   * @param offset Relative time offset to re-schedule all entries by, or 0 if CFEst has changed and serves as baseline
   * @return true if it at least one scheduled message was dropped due to excessive re-scheduling or postponing 
   */
 bool rdcp_txqueue_reschedule(uint8_t channel, int64_t offset);
 
 /**
   * RDCP TX Queue Loop to be called periodically to start sending outgoing
   * messages when the channel is free. This should be called as a part of an overall
   * Arduino-style loop() for the whole device; otherwise, no TX ever starts. 
   * @return true If a message is prepared to be sent now; false if no TX is up ahead.
   */
 bool rdcp_txqueue_loop(void);
 
 /**
  * Schedule an outgoing RDCP Message in the "ahead of time" queue.
  * This is intended for intermediate-term scheduling, like sending a SIGNATURE after a corresponding RDCP Message. 
  * Periodic retransmissions should not be added to any queue unless the LoRa channel is considered free currently. 
  * @param channel CHANNEL433 or CHANNEL868
  * @param data LoRa packet payload (i.e., RDCP Header + RDCP Payload)
  * @param len Length of data in bytes 
  * @param important IMPORTANT if the message must not be dropped, otherwise NOTIMPORTANT 
  * @param forcetx FORCEDTX if the message has to be sent at its scheduled time, NOTFORCEDTX if it may be re-scheduled 
  * @param callback_selector Number of the callback to use, e.g., TX_CALLBACK_NONE
  * @param delay_in_ms Relative monotonic clock timestamp of when to move the message to the TX queue (in milliseconds)
  * @return true if the message was accepted in the queue, false otherwise (e.g., queue full)
  */
bool rdcp_txaheadqueue_add(uint8_t channel, uint8_t *data, uint8_t len, bool important, bool force_tx, uint8_t callback_selector, int64_t delay_in_ms);

/**
  * RDCP TX Ahead Queue Loop to be called implicitly by the TX Queue Loop.
  * Calling this manually should be used scarcely. 
  * @return true if a messages was moved to the TX queue, false otherwise
  */
bool rdcp_txaheadqueue_loop(void);

/**
 * @return Number of messages currently in the TX Queue. 
 */
int get_num_txq_entries(uint8_t channel);

/**
 * @return Number of messages currently in the TX Ahead Queue. 
 */
int get_num_txaq_entries(uint8_t channel);

/**
 * Print the current TX Queue on Serial. 
 * @param channel CHANNEL433 or CHANNEL868
 */
void rdcp_dump_txq(uint8_t channel);

/**
 * Reschedule current TXQ entries due to channel being busy. 
 * @param channel Affected channel, CHANNEL433 or CHANNEL868
 */
void rdcp_reschedule_on_busy_channel(uint8_t channel);

/**
 * Does TXQ have at least one FORCEDTX entry?
 * @param channel CHANNEL433 or CHANNEL868
 * @return bool true if hard-scheduled entry exists, false otherwise
 */
bool rdcp_txqueue_has_forced_entry(uint8_t channel);

#endif 
/* EOF */