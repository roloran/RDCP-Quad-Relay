#include "rdcp-send.h"
#include "rdcp-scheduler.h"
#include "rdcp-incoming.h"
#include "lora.h"
#include "hal.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-scheduler.h"
#include "Base64ren.h"
#include "rdcp-callbacks.h"

extern txqueue txq[NUMCHANNELS];
extern txaheadqueue txaq[NUMCHANNELS];
extern int tx_ongoing[NUMCHANNELS];
extern da_config CFG;
extern int64_t last_tx_activity[NUMCHANNELS];
extern int retransmission_count[NUMCHANNELS];
extern int64_t CFEst[NUMCHANNELS];
int64_t tx_start[NUMCHANNELS];
int64_t tx_latency[NUMCHANNELS];

void rdcp_queue_postpone_for_retransmission(uint8_t channel, int highlander, int64_t notbefore)
{
    char info[INFOLEN];
    for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++)
    {
        if (i == highlander) continue; // don't postpone the one we want to send
        if (txq[channel].entries[i].waiting)
        {
            while (txq[channel].entries[i].currently_scheduled_time < notbefore)
            {
                snprintf(info, INFOLEN, "INFO: TXQ%d entry %d must be re-scheduled due to retransmission, hl %d, nb %" PRId64 ", TSd %" PRId64 " ms",
                  channel == CHANNEL433 ? 4 : 8, i, highlander, notbefore, txq[channel].entries[highlander].timeslot_duration);
                serial_writeln(info);
                txq[channel].entries[i].currently_scheduled_time += 
                    txq[channel].entries[highlander].timeslot_duration;
            }
        }
    }
    return;
}

void rdcp_send_message_cad(uint8_t channel)
{
    radio_start_cad(channel); // actual sending happens when CAD reports channel free via rdcp_callback_cad()
    return;
}

void rdcp_send_message_force(uint8_t channel)
{
    int64_t now = my_millis();
    int64_t timediff = now - 
                       txq[channel].entries[tx_ongoing[channel]].originally_scheduled_time - 
                       retransmission_count[channel] * 
                            (airtime_in_ms(channel, txq[channel].entries[tx_ongoing[channel]].payload_length) + 
                            RDCP_TIMESLOT_BUFFERTIME);
    char buf[INFOLEN];
    snprintf(buf, INFOLEN, "INFO: TXStart for TXQ%di %d, len %d, TSd %" PRId64 "ms, latency %" PRId64 " ms", 
        channel == CHANNEL433 ? 4 : 8, tx_ongoing[channel], txq[channel].entries[tx_ongoing[channel]].payload_length, 
        txq[channel].entries[tx_ongoing[channel]].timeslot_duration, timediff);
    serial_writeln(buf);
    snprintf(buf, INFOLEN, "INFO: (cont.) os %" PRId64 ", cs %" PRId64 ", now %" PRId64, 
        txq[channel].entries[tx_ongoing[channel]].originally_scheduled_time, 
        txq[channel].entries[tx_ongoing[channel]].currently_scheduled_time,
        now);
    serial_writeln(buf);
  
    snprintf(buf, INFOLEN, "TXMETA %d %" PRId64 " %3.3f", 
        txq[channel].entries[tx_ongoing[channel]].payload_length, now, CFG.lora[channel].freq);
    serial_writeln(buf);
  
    int encodedLength = Base64ren.encodedLength(txq[channel].entries[tx_ongoing[channel]].payload_length);
    char b64msg[encodedLength + 1];
    Base64ren.encode(b64msg, (char *) txq[channel].entries[tx_ongoing[channel]].payload, 
                     txq[channel].entries[tx_ongoing[channel]].payload_length);
  
    snprintf(buf, INFOLEN, "TX %s", b64msg);
    serial_writeln(buf);

    tx_start[channel] = my_millis();
    tx_latency[channel] = timediff > 0 ? timediff : 0;

    send_lora_message_binary(channel, txq[channel].entries[tx_ongoing[channel]].payload, 
                             txq[channel].entries[tx_ongoing[channel]].payload_length);
  
    uint16_t origin = txq[channel].entries[tx_ongoing[channel]].payload[2] + 256 * txq[channel].entries[tx_ongoing[channel]].payload[3];
    uint16_t seqnr  = txq[channel].entries[tx_ongoing[channel]].payload[4] + 256 * txq[channel].entries[tx_ongoing[channel]].payload[5];
    uint8_t mt      = txq[channel].entries[tx_ongoing[channel]].payload[8];
    uint8_t rcnt    = txq[channel].entries[tx_ongoing[channel]].payload[10];
    uint8_t relay1  = txq[channel].entries[tx_ongoing[channel]].payload[11];
    uint8_t relay2  = txq[channel].entries[tx_ongoing[channel]].payload[12];
    uint8_t relay3  = txq[channel].entries[tx_ongoing[channel]].payload[13];

    rdcp_update_cfest_out(channel, txq[channel].entries[tx_ongoing[channel]].payload_length, 
      rcnt, mt, relay1, relay2, relay3, origin, seqnr);
    rdcp_txqueue_reschedule(channel, -1);

    return; 
}

void rdcp_callback_txfin(uint8_t channel)
{
    char buf[INFOLEN];

    last_tx_activity[channel] = my_millis();
    int num_waiting = -1;
    for (int i=0; i < MAX_TXQUEUE_ENTRIES; i++) if (txq[channel].entries[i].waiting) num_waiting++;
  
    int num_retransmissions = 0;
    struct rdcp_message rm;
    memcpy(&rm.header, txq[channel].entries[tx_ongoing[channel]].payload, RDCP_HEADER_SIZE);
    for (int i=0; i < rm.header.rdcp_payload_length; i++) 
        rm.payload.data[i] = txq[channel].entries[tx_ongoing[channel]].payload[RDCP_HEADER_SIZE + i];
    num_retransmissions = rm.header.counter;
  
    snprintf(buf, INFOLEN, "INFO: TXFIN 4 TXQ%di %d, %d retransmissions ahead, %d/%d more messages waiting", 
        channel == CHANNEL433 ? 4 : 8, tx_ongoing[channel], num_retransmissions, num_waiting, txq[channel].num_entries);
    serial_writeln(buf);
  
    txq[channel].entries[tx_ongoing[channel]].cad_retry = 0;
  
    if (num_retransmissions > 0)
    { // same RDCP Message needs retransmission based on counter in RDCP Header
      rm.header.counter -= 1;
  
      uint8_t data_for_crc[INFOLEN];
      memcpy(&data_for_crc, &rm.header, RDCP_HEADER_SIZE - 2);
      for (int i=0; i < rm.header.rdcp_payload_length; i++) data_for_crc[i + RDCP_HEADER_SIZE - 2] = rm.payload.data[i];
      uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - 2 + rm.header.rdcp_payload_length);
      rm.header.checksum = actual_crc;
      memcpy(txq[channel].entries[tx_ongoing[channel]].payload, &rm.header, RDCP_HEADER_SIZE); // no need to overwrite RDCP payload
  
      retransmission_count[channel]++;
      /* 
        The timestamp for sending the next retransmission from plain RDCP specs is the
        timestamp of the previous transmission + its airtime + the 1000 ms buffer time. 
        However, we schedule the retransmission a bit earlier due to processing time, 
        approximated by a constant value for now. Additionally, we consider the latency
        accumulated before the previous transmission with an upper bound of 250 ms for now. 
      */
      int64_t next_timestamp = tx_start[channel] + 
                               airtime_in_ms(channel, txq[channel].entries[tx_ongoing[channel]].payload_length) + 
                               RDCP_TIMESLOT_BUFFERTIME;
      next_timestamp -= RETRANSMISSION_PROCESSING_TIME;
      next_timestamp -= tx_latency[channel] < 250 ? tx_latency[channel] : 250;

      txq[channel].entries[tx_ongoing[channel]].currently_scheduled_time = next_timestamp;
      txq[channel].entries[tx_ongoing[channel]].force_tx = true;
      txq[channel].entries[tx_ongoing[channel]].important = true;
      txq[channel].entries[tx_ongoing[channel]].waiting = true;
      txq[channel].entries[tx_ongoing[channel]].in_process = true; //?
      uint8_t highlander = tx_ongoing[channel];
      tx_ongoing[channel] = -1; 

      rdcp_queue_postpone_for_retransmission(channel, highlander, next_timestamp);
    }
    else
    { // last transmission for this RDCP Message completed
      if (txq[channel].entries[tx_ongoing[channel]].callback_selector == TX_CALLBACK_NONE)
      {
        // Nothing to do; no callback necessary.
      }
      else if (txq[channel].entries[tx_ongoing[channel]].callback_selector == TX_CALLBACK_CIRE)
      {
        serial_writeln("DA_CIRESENT");
      }
      else if (txq[channel].entries[tx_ongoing[channel]].callback_selector == TX_CALLBACK_FETCH_SINGLE)
      {
        rdcp_chain_callback(TX_CALLBACK_FETCH_SINGLE, false);
      }
      else if (txq[channel].entries[tx_ongoing[channel]].callback_selector == TX_CALLBACK_FETCH_ALL)
      {
        rdcp_chain_callback(TX_CALLBACK_FETCH_ALL, false);
      }
      else if (txq[channel].entries[tx_ongoing[channel]].callback_selector == TX_CALLBACK_PERIODIC868)
      {
        rdcp_chain_callback(TX_CALLBACK_PERIODIC868, false);
      }
      else /* Add more callback options here later */
      {
        // do nothing so far
      }
  
      txq[channel].entries[tx_ongoing[channel]].waiting = false;
      txq[channel].entries[tx_ongoing[channel]].payload_length = 0;
      txq[channel].entries[tx_ongoing[channel]].in_process = false;
      txq[channel].num_entries--;
      retransmission_count[channel] = 0;
      tx_ongoing[channel] = -1;
  
      // When we finished transmitting, others expect the channel to be free
      // and might want to start sending urgent messages. Thus, as we just used
      // the channel for ourselves for some time, give them a chance.
      //int64_t random_delay = my_random_in_range(1000 * CFG.sf_multiplier, 2000 * CFG.sf_multiplier); // history: 10-20 s
      int64_t my_delay = 1 * SECONDS_TO_MILLISECONDS + 100 * (1 + CFG.relay_identifier) * CFG.sf_multiplier;
      snprintf(buf, INFOLEN, "INFO: Rescheduling CHANNEL%d by %d ms due to finished transmission", channel == CHANNEL433 ? 433:868, my_delay);
      serial_writeln(buf);
      rdcp_txqueue_reschedule(channel, 0 - my_delay);
    }
  
    return;
}

bool rdcp_callback_cad(uint8_t channel, bool cad_busy)
{
    bool channel_free = !cad_busy;
    char buf[INFOLEN];
  
    last_tx_activity[channel] = my_millis();
  
    txq[channel].entries[tx_ongoing[channel]].cad_retry += 1;
    uint8_t retry = txq[channel].entries[tx_ongoing[channel]].cad_retry;
  
    snprintf(buf, INFOLEN, "INFO: Send-processing: CAD reports channel %d %s (try %d)", channel == CHANNEL433 ? 433 : 868, channel_free ? "free" : "busy", retry);
    serial_writeln(buf);
  
    if (channel_free)
    {
      rdcp_send_message_force(channel);
      return true;
    }
  
    if (retry == 1)
    {
      if (channel == CHANNEL868)
      {
        radio_start_cad(channel);  
      }
      else 
      {
        start_receive_433();
        txq[channel].entries[tx_ongoing[channel]].in_process = false;
        tx_ongoing[channel] = -1;
        // int64_t random_delay = my_random_in_range(1000 * CFG.sf_multiplier, 2000 * CFG.sf_multiplier); // history: 10-20 s
        int64_t my_delay = 1 * SECONDS_TO_MILLISECONDS + 100 * CFG.relay_identifier * CFG.sf_multiplier;
        snprintf(buf, INFOLEN, "INFO: Rescheduling CHANNEL%d by %" PRId64 " ms due to %d. CAD retry", channel == CHANNEL433 ? 433:868, my_delay, retry);
        serial_writeln(buf);
        rdcp_txqueue_reschedule(channel, 0 - my_delay);
        if (CFEst[channel] < my_millis() + my_delay) CFEst[channel] = my_millis() + my_delay; // Don't re-schedule twice
      }
    }
    else if (retry <= 4)
    {
      radio_start_cad(channel);
    }
    else if (retry == 5)
    {
      if (channel == CHANNEL433) { start_receive_433(); } else { start_receive_868(); }
      txq[channel].entries[tx_ongoing[channel]].in_process = false;
      tx_ongoing[channel] = -1;
      // int64_t random_delay = my_random_in_range(2100 * CFG.sf_multiplier, 2500 * CFG.sf_multiplier);
      int64_t my_delay = 2 * SECONDS_TO_MILLISECONDS + 50 * CFG.relay_identifier * CFG.sf_multiplier;
      snprintf(buf, INFOLEN, "INFO: Rescheduling CHANNEL%d by %" PRId64 " ms due to %d. CAD retry", channel == CHANNEL433 ? 433:868, my_delay, retry);
      serial_writeln(buf);
      rdcp_txqueue_reschedule(channel, 0 - my_delay);
      if (CFEst[channel] < my_millis() + my_delay) CFEst[channel] = my_millis() + my_delay; // Don't re-schedule twice
    }
    else if ((retry >= 6) && (retry <= 9))
    {
      radio_start_cad(channel);
    }
    else if ((retry >= 10) && (retry <= 14))
    {
      if (channel == CHANNEL433) { start_receive_433(); } else { start_receive_868(); }
      txq[channel].entries[tx_ongoing[channel]].in_process = false;
      tx_ongoing[channel] = -1;
      // int64_t random_delay = my_random_in_range(3100 * CFG.sf_multiplier, 3500 * CFG.sf_multiplier); // history: 31-35 s
      int64_t my_delay = 3 * SECONDS_TO_MILLISECONDS + 50 * CFG.relay_identifier * CFG.sf_multiplier;
      snprintf(buf, INFOLEN, "INFO: Rescheduling CHANNEL%d by %" PRId64 " ms due to %d. CAD retry", channel == CHANNEL433 ? 433:868, my_delay, retry);
      serial_writeln(buf);
      rdcp_txqueue_reschedule(channel, 0 - my_delay);
      if (CFEst[channel] < my_millis() + my_delay) CFEst[channel] = my_millis() + my_delay; // Don't re-schedule twice
    }
    else if (retry >= 15)
    {
      snprintf(buf, INFOLEN, "WARNING: CAD retry timeout for TXQ%di %d, force-sending now", 
        channel == CHANNEL433 ? 4 : 8, tx_ongoing[channel]);
      serial_writeln(buf);
      rdcp_send_message_force(channel);
      return true;
    }
  
    return false;
}

/* EOF */