#include "rdcp-scheduler.h"
#include "rdcp-incoming.h"
#include "lora.h"
#include "hal.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-scheduler.h"
#include "rdcp-send.h"

extern lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
extern da_config CFG;

txqueue txq[NUMCHANNELS];
txaheadqueue txaq[NUMCHANNELS];

int tx_ongoing[NUMCHANNELS] = {-1, -1};      // Index of TXQ entry currently up for transmission
int64_t tx_process_start[NUMCHANNELS] = {0, 0};
int retransmission_count[NUMCHANNELS] = {0, 0};
int64_t last_tx_activity[NUMCHANNELS] = {0, 0};

bool rdcp_txqueue_add(uint8_t channel, uint8_t *data, uint8_t len, bool important, bool force_tx, uint8_t callback_selector, int64_t forced_time)
{
    if (txq[channel].num_entries == MAX_TXQUEUE_ENTRIES)
    {
      serial_writeln("WARNING: rdcp_txqueue_add() failed -- TX Queue is full");
      return false;
    }

    int64_t now = my_millis();

    for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
    {
      if (txq[channel].entries[i].waiting == false)
      {
        txq[channel].num_entries += 1;
        txq[channel].entries[i].waiting = true;
        txq[channel].entries[i].in_process = false;
        txq[channel].entries[i].timeslot_duration = rdcp_get_timeslot_duration(channel, data);
        txq[channel].entries[i].callback_selector = callback_selector;
        txq[channel].entries[i].force_tx = force_tx;
        txq[channel].entries[i].important = important;
        if (forced_time == 0)
        { /* No time given, schedule as early as possible when channel is free */
          txq[channel].entries[i].originally_scheduled_time = rdcp_get_channel_free_estimation(channel);
          /* Don't schedule into the past if channel is currently assumed free. */
          if (txq[channel].entries[i].originally_scheduled_time < now) txq[channel].entries[i].originally_scheduled_time = now;
        }
        else if (forced_time < 0)
        { /* Negative relative time => append */
          int64_t highest_timestamp = rdcp_get_channel_free_estimation(channel);
          if (now > highest_timestamp) highest_timestamp = now; // don't schedule into the past
          for (int j=0; j < MAX_TXQUEUE_ENTRIES; j++)
          {
            if (txq[channel].entries[j].waiting)
            {
              if (txq[channel].entries[j].currently_scheduled_time > highest_timestamp)
                highest_timestamp = txq[channel].entries[j].currently_scheduled_time;
            }
          }
          // Add the (negative) relative time to last entry's time
          txq[channel].entries[i].originally_scheduled_time = highest_timestamp - forced_time;
        }
        else
        { /* Positive absolute time */
          txq[channel].entries[i].originally_scheduled_time = forced_time;
        }
        txq[channel].entries[i].currently_scheduled_time = txq[channel].entries[i].originally_scheduled_time;
        txq[channel].entries[i].num_of_reschedules = 0;
        txq[channel].entries[i].payload_length = len;
        txq[channel].entries[i].in_process = false;
        txq[channel].entries[i].cad_retry = 0;
        for (int j=0; j < len; j++) txq[channel].entries[i].payload[j] = data[j];

        char buf[INFOLEN];
        snprintf(buf, INFOLEN, "INFO: Outgoing message scheduled -> TXQ%di %d, len %d, TSd %" PRId64 ", @%" PRId64 ", ft%" PRId64,
            channel == CHANNEL433 ? 4 : 8, i, len, txq[channel].entries[i].timeslot_duration, txq[channel].entries[i].currently_scheduled_time, forced_time);
        serial_writeln(buf);

        rdcp_dump_txq(channel);

        return true; // found free spot and added entry, exit loop here.
      }
    }

    return false;
}

bool rdcp_txqueue_reschedule(uint8_t channel, int64_t offset)
{
    char info[INFOLEN];
    int64_t now = my_millis();
    int64_t cfest = rdcp_get_channel_free_estimation(channel);
    int64_t delta = cfest - now;
    int64_t rescheduled_by = 0;
    if (delta < 0) delta = 0; // do not schedule back in time
    bool dropped = false;

    if (offset != 0) delta = offset;
    /*
        We might return false here if delta == 0 as it would not
        actually re-schedule anything.
        However, using this function may still drop less important
        messages to keep the TQX short, which may actually be
        desired when the channel is somewhat busy.
    */

    /* Look for the 'currently scheduled' timestamp of the earliest
       TXQ entry to be sent before channel becomes free */
    int64_t next_timestamp = cfest;
    for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
    {
        if (!txq[channel].entries[i].waiting)      continue; // only waiting entries are relevant
        if (txq[channel].entries[i].in_process)    continue; // skip if currently in process
        if (txq[channel].entries[i].force_tx)      continue; // skip if it has a forced time
        if (txq[channel].entries[i].currently_scheduled_time  < next_timestamp)
          next_timestamp = txq[channel].entries[i].currently_scheduled_time;
    }
    int64_t maximum_diff_to_cfest = cfest - next_timestamp;

    for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
    {
      if (txq[channel].entries[i].waiting)
      {
        if (txq[channel].entries[i].in_process) continue;
        if (txq[channel].entries[i].force_tx) continue;

        txq[channel].entries[i].num_of_reschedules++;
        int reschedule_mode = 0;
        if (delta >= 0)
        {
          txq[channel].entries[i].currently_scheduled_time += delta;
          rescheduled_by = delta;
          reschedule_mode = 1;
        }
        else
        {
          delta = -1 * delta;
          if (txq[channel].entries[i].currently_scheduled_time < (cfest + delta))
          {
            txq[channel].entries[i].currently_scheduled_time += maximum_diff_to_cfest + delta;
            rescheduled_by = maximum_diff_to_cfest + delta;
            reschedule_mode = 2;
          }
        }

        if (rescheduled_by > 0)
        {
          snprintf(info, INFOLEN, "INFO: TXQ%d entry %d re-scheduled (mode %d) by %" PRId64 " ms, r%" PRId64 "ms, CFr%" PRId64 "ms",
           channel == CHANNEL433 ? 4 : 8, i, reschedule_mode, rescheduled_by, txq[channel].entries[i].currently_scheduled_time - now, cfest-now);
          serial_writeln(info);
        }

        if (txq[channel].entries[i].currently_scheduled_time < now)
        {
          if (offset < 0) offset = -1 * offset;
          txq[channel].entries[i].currently_scheduled_time = now + offset;
          snprintf(info, INFOLEN, "INFO: TXQ%d entry %d re-scheduled from past to %" PRId64 " ms, r%" PRId64 "ms, CFr%" PRId64 "ms",
            channel == CHANNEL433 ? 4 : 8, i, txq[channel].entries[i].currently_scheduled_time, txq[channel].entries[i].currently_scheduled_time - now, cfest-now);
          serial_writeln(info);
        }

        if (txq[channel].entries[i].important) continue;
        if ( (txq[channel].entries[i].num_of_reschedules > 50) ||
             (txq[channel].entries[i].currently_scheduled_time - txq[channel].entries[i].originally_scheduled_time > 10 * MINUTES_TO_MILLISECONDS) )
        {
          snprintf(info, INFOLEN, "WARNING: Dropped TXQ%d entry %d based on %d re-schedules and %" PRId64 " ms delay.",
            channel == CHANNEL433 ? 4 : 8, i, txq[channel].entries[i].num_of_reschedules,
            txq[channel].entries[i].currently_scheduled_time - txq[channel].entries[i].originally_scheduled_time);
          serial_writeln(info);
          txq[channel].entries[i].waiting = false; // drop message due to excessive delay when trying to send
          txq[channel].entries[i].payload_length = 0;
          txq[channel].entries[i].in_process = false;
          txq[channel].num_entries--;
          dropped = true;
        }
      }
    }

    rdcp_dump_txq(channel);

    return dropped;
}

void rdcp_txqueue_compress(void)
{
  int64_t now = my_millis();

  for (int channel=0; channel <= 1; channel++)
  {
    if (now > rdcp_get_channel_free_estimation(channel) + 2 * MINUTES_TO_MILLISECONDS)
    { /* Channel is unused for more than two minutes; check whether we have something to send earlier. */
      bool has_forced_entry = false;
      int earliest = RDCP_INDEX_NONE;
      for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
      {
        if (txq[channel].entries[i].waiting)
        {
          if ((txq[channel].entries[i].force_tx) ||
              (txq[channel].entries[i].in_process))
          {
            has_forced_entry = true;
          }
          if ((earliest == RDCP_INDEX_NONE) ||
              (txq[channel].entries[i].currently_scheduled_time < txq[channel].entries[earliest].currently_scheduled_time))
          {
            earliest = i;
          }
        }
      }
      if (has_forced_entry) continue; // Skip compression to avoid clash with hard-scheduled messages
      if (earliest == RDCP_INDEX_NONE) continue;   // No entry found to send earlier

      int64_t delta = txq[channel].entries[earliest].currently_scheduled_time - now;
      if (delta > 30 * SECONDS_TO_MILLISECONDS)
      {
        for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
        {
          if (txq[channel].entries[i].waiting)
          {
            if (txq[channel].entries[i].force_tx) { continue; }   // should not happen if we reach this code
            if (txq[channel].entries[i].in_process) { continue; } // same
            txq[channel].entries[i].currently_scheduled_time -= delta;
            char info[INFOLEN];
            snprintf(info, INFOLEN, "INFO: Compressed TXQ%d entry %d by moving it %" PRId64 " ms",
              channel == CHANNEL433 ? 4 : 8, i, delta);
            serial_writeln(info);
          }
        }
      }
    }
  }

  return;
}

bool rdcp_txqueue_has_forced_entry(uint8_t channel)
{
  for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
  {
    if (txq[channel].entries[i].waiting)
    {
      if (txq[channel].entries[i].force_tx)
      {
        char info[INFOLEN];
        snprintf(info, INFOLEN, "INFO: Previous 433 FORCETX entry TXQi %d in %" PRId64 " ms",
          i, txq[channel].entries[i].currently_scheduled_time - my_millis());
        serial_writeln(info);
        return true;
      }
    }
  }
  return false;
}

bool rdcp_txqueue_loop(void)
{
    int64_t now = my_millis();
    bool result = false;

    rdcp_txqueue_compress();

    for (int channel=0; channel <= 1; channel++)
    {
        /* Skip if any transmission is already ongoing */
        if (tx_ongoing[channel] != -1)
        {
          if (now - last_tx_activity[channel] > 180000)
          {
            serial_writeln("WARNING: TX Activity Timeout, restarting TXQ processing");
            txq[channel].entries[tx_ongoing[channel]].in_process = false;
            txq[channel].entries[tx_ongoing[channel]].cad_retry = 0;
            tx_ongoing[channel] = -1;
            setup_radio();
          }
          continue;
        }

        last_tx_activity[channel] = now;

        /* Feed fresh messages into our queue */
        rdcp_txaheadqueue_loop();

        for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
        {
            if (txq[channel].entries[i].waiting)
            {
                if (txq[channel].entries[i].currently_scheduled_time <= now)
                {
                    cpu_fast();
                    if (tx_ongoing[channel] == -1)
                    { // found a first message to start send-processing now
                        tx_ongoing[channel] = i;
                    }
                    else
                    { // already had found a suitable message, but maybe chose another one
                        if (txq[channel].entries[i].currently_scheduled_time < txq[channel].entries[tx_ongoing[channel]].currently_scheduled_time)
                            tx_ongoing[channel] = i; // try to keep the order
                        if (txq[channel].entries[i].force_tx) tx_ongoing[channel] = i; // but prioritize hard-scheduled messages even more
                    }
                }
            }
        }
        if (tx_ongoing[channel] != -1) { result = true; } else { continue; }

        /* Double-check for non-force-tx scheduling clashes */
        if (!txq[channel].entries[tx_ongoing[channel]].force_tx)
        { // Entry is not force-tx...
          if (now < rdcp_get_channel_free_estimation(channel))
          { // ... but channel is currently known not to be free ...
            serial_writeln("WARNING: Resolving scheduling clash");
            tx_ongoing[channel] = -1;
            result = false;
            rdcp_txqueue_reschedule(channel, 0); // re-schedule based on channel's CFest
            return false;
          }
        }

        txq[channel].entries[tx_ongoing[channel]].in_process = true;
        tx_process_start[channel] = now;

        char buf[INFOLEN];
        snprintf(buf, INFOLEN, "INFO: Outgoing message up for send-processing -> TXQ%di %d, len %d, TSd %" PRId64 ", @%" PRId64 ", =%" PRId64,
             channel == CHANNEL433 ? 4:8, tx_ongoing[channel], txq[channel].entries[tx_ongoing[channel]].payload_length,
             txq[channel].entries[tx_ongoing[channel]].timeslot_duration,
             txq[channel].entries[tx_ongoing[channel]].currently_scheduled_time, now);
        serial_writeln(buf);

        if (txq[channel].entries[tx_ongoing[channel]].force_tx == false)
        {
            rdcp_send_message_cad(channel);
        }
        else
        {
            rdcp_send_message_force(channel);
        }
    }

    return result;
}

bool rdcp_txaheadqueue_add(uint8_t channel, uint8_t *data, uint8_t len, bool important, bool force_tx, uint8_t callback_selector, int64_t delay_in_ms)
{
    if (txaq[channel].num_entries == MAX_TXAHEADQUEUE_ENTRIES)
    {
      serial_writeln("WARNING: rdcp_txaheadqueue_add() failed -- TX Ahead Queue is full");
      return false;
    }

    int64_t now = my_millis();

    for (int i=0; i < MAX_TXAHEADQUEUE_ENTRIES; i++)
    {
      if (txaq[channel].entries[i].waiting == false)
      {
        txaq[channel].num_entries += 1;
        txaq[channel].entries[i].waiting = true;
        txaq[channel].entries[i].callback_selector = callback_selector;
        txaq[channel].entries[i].force_tx = force_tx;
        txaq[channel].entries[i].important = important;
        txaq[channel].entries[i].scheduled_time = now + delay_in_ms;
        txaq[channel].entries[i].payload_length = len;
        for (int j=0; j < len; j++) txaq[channel].entries[i].payload[j] = data[j];

        char buf[INFOLEN];
        snprintf(buf, INFOLEN, "INFO: Delayed message scheduled -> TXAQ%di %d, len %d, @%" PRId64, channel == CHANNEL433 ? 4 : 8, i, len, txaq[channel].entries[i].scheduled_time);
        serial_writeln(buf);

        return true; // found free spot and added entry, exit loop here.
      }
    }

    return false;
}

bool rdcp_txaheadqueue_loop(void)
{
    for (int channel = 0; channel <= 1; channel++)
    {
        if (txq[channel].num_entries == MAX_TXQUEUE_ENTRIES) continue;

        int64_t now = my_millis();

        for (int i=0; i < MAX_TXAHEADQUEUE_ENTRIES; i++)
        {
            if ((txaq[channel].entries[i].waiting == true) && (txaq[channel].entries[i].scheduled_time <= now))
            {
                if (rdcp_txqueue_add(channel, txaq[channel].entries[i].payload, txaq[channel].entries[i].payload_length,
                        txaq[channel].entries[i].important, txaq[channel].entries[i].force_tx,
                        txaq[channel].entries[i].callback_selector, txaq[channel].entries[i].scheduled_time) == true)
                {
                    txaq[channel].entries[i].waiting = false;
                    txaq[channel].num_entries--;
                    return true;
                }
                else
                { // Moving to TXQ failed, probably full. Try again next time.
                     continue;
                }
            }
        }
    }

    return false;
}

int get_num_txq_entries(uint8_t channel)
{
    return txq[channel].num_entries;
}

int get_num_txaq_entries(uint8_t channel)
{
    return txaq[channel].num_entries;
}

void rdcp_dump_txq(uint8_t channel)
{
  int64_t now = my_millis();
  char info[INFOLEN];
  snprintf(info, INFOLEN, "INFO: Listing TXQ%d @ %" PRId64 " ms", channel == CHANNEL433 ? 4 : 8, now);
  serial_writeln(info);

  for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
  {
    if (txq[channel].entries[i].waiting)
    {
      int64_t timediff = txq[channel].entries[i].currently_scheduled_time - now;
      int32_t td = (int32_t) timediff;

      snprintf(info, INFOLEN, "INFO: TXQ%d i%02d t%03.3fms l%03d o%" PRId64 "ms c%" PRId64 "ms d%" PRId64 "ms r%" PRId64 "ms",
        channel == CHANNEL433 ? 4 : 8,
        i,
        td / 1000.0,
        txq[channel].entries[i].payload_length,
        txq[channel].entries[i].originally_scheduled_time,
        txq[channel].entries[i].currently_scheduled_time,
        txq[channel].entries[i].currently_scheduled_time - txq[channel].entries[i].originally_scheduled_time,
        timediff);
      serial_writeln(info);
    }
  }

  int64_t cfest = rdcp_get_channel_free_estimation(channel);
  int32_t relcfest32 = (int32_t) (cfest-now);
  snprintf(info, INFOLEN, "INFO: Listing TXQ%d ends, CFEst r%03.3f @%" PRId64 "ms",
    channel == CHANNEL433 ? 4 : 8,
    (relcfest32) / 1000.0,
    cfest);
  serial_writeln(info);

  return;
}

void rdcp_reschedule_on_busy_channel(uint8_t channel)
{
  for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++) txq[channel].entries[i].cad_retry = 0;
  if (tx_ongoing[channel] != -1)
  {
      serial_writeln("INFO: Postponing current transmission due to RDCP Message reception");
      txq[channel].entries[tx_ongoing[channel]].in_process = false;
      tx_ongoing[channel] = -1;
  }

  int64_t timediff = rdcp_get_channel_free_estimation(channel) - my_millis();
  if (timediff > 0)
  {
    char info[INFOLEN];
    snprintf(info, INFOLEN, "INFO: Rescheduling CHANNEL%d by %d ms due to timediff CFEst-now", channel == CHANNEL433 ? 433:868, timediff);
    serial_writeln(info);

    rdcp_txqueue_reschedule(channel, timediff);
  }

  return;
}

 /* EOF */
