#include "rdcp-incoming.h"
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-entrypoint.h"
#include "rdcp-blockdevice.h"
#include "rdcp-scheduler.h"
#include "rdcp-forward.h"
#include "rdcp-neighbors.h"
#include "rdcp-memory.h"
#include "rdcp-commands.h"
#include "da-crypto.h"
#include "hal.h"
#include "rdcp-callbacks.h"
#include "unishox2.h"

extern lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
extern da_config CFG;
extern runtime_da_data DART;
extern neighbor_table_entry neighbors[MAX_NEIGHBORS];
extern int64_t reboot_requested;
extern bool seqnr_reset_requested;
extern uint16_t new_delivery_receipt_from;
extern rdcp_memory_table mem;
extern bool currently_in_fetch_mode;
bool   oa_reset_seen = false;
extern bool do_not_persist_dupetable;

rdcp_message rdcp_response;
int64_t last_heartbeat_sent = RDCP_TIMESTAMP_ZERO;
int64_t fetch_timeout = RDCP_TIMESTAMP_ZERO;
int64_t last_dasresp_sent = RDCP_TIMESTAMP_ZERO;
bool rtc_active = false;
rtc_entry RTC[MAX_RTC];

#define DASR_ROLLOVER 24
int dasr_counter = DASR_ROLLOVER - 1;

/**
 * Fill the RDCP Header fields for outgoing responses as the are
 * the same for any outgoing response.
 */
void rdcp_prepare_response_header(bool reuse_seqnr)
{
    // Destination, Message Type, Payload Length, Relay123 and the whole Payload
    // must be set before calling this function
    rdcp_response.header.sender = CFG.rdcp_address;
    rdcp_response.header.origin = CFG.rdcp_address;
    if (!reuse_seqnr) rdcp_response.header.sequence_number = get_next_rdcp_sequence_number(CFG.rdcp_address);
    rdcp_response.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(rdcp_response.header.message_type);

    /* Update CRC header field */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &rdcp_response.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++)
        data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rdcp_response.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + rdcp_response.header.rdcp_payload_length);
    rdcp_response.header.checksum = actual_crc;

    return;
}

/**
 * Schedule the prepared response for transmission on the given channel.
 * @param channel Either CHANNEL433 or CHANNEL868
 */
void rdcp_pass_response_to_scheduler(uint8_t channel, bool no_larger_delay=false)
{
    uint8_t data_for_scheduler[INFOLEN];
    memcpy(&data_for_scheduler, &rdcp_response.header, RDCP_HEADER_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++)
        data_for_scheduler[i + RDCP_HEADER_SIZE] = rdcp_response.payload.data[i];

    /*
        Add a short random delay when responding. Otherwise, we might
        be so fast that the recipient has not switched back to receiving
        yet after sending its request.
        We also need to delay in case we received a command via 868 MHz
        and do not know the 433 MHz propagation cycle yet.
    */
    // int64_t random_delay = 0 - my_random_in_range(2000 * CFG.sf_multiplier, 3000 * CFG.sf_multiplier); // history: 2-5 s, last: 9-10 s

    int64_t my_delay = -20 * SECONDS_TO_MILLISECONDS * CFG.sf_multiplier; // base delay of 20 seconds (* sf_multiplier) 
    if (channel == CHANNEL433)
    { // delay minimally based on own relay identifier
      my_delay -= CFG.sf_multiplier * 100 * CFG.relay_identifier;
    }
    else 
    {
      // delay additional 4 base seconds on 868 MHz to give the 433 MHz a headstart
      my_delay -= CFG.sf_multiplier * (4 * SECONDS_TO_MILLISECONDS + 100 * CFG.relay_identifier);
    }

    if (no_larger_delay)
    { // if larger delays have been disabled, use a minimum delay based on relay identifier
        my_delay = -100 * (1 + CFG.relay_identifier);
        if (channel == CHANNEL868DA) my_delay -= 4 * SECONDS_TO_MILLISECONDS; // allow for 433 MHz headstart
    }

    /* Get Roaming Beacons out early as longer delays may defeat their purpose */
    if (rdcp_response.header.message_type == RDCP_MSGTYPE_ROAMINGBEACON)
    {
        my_delay = -100 * (1 + CFG.relay_identifier);
    }

    rdcp_txqueue_add(channel, data_for_scheduler, RDCP_HEADER_SIZE + rdcp_response.header.rdcp_payload_length,
      NOTIMPORTANT, NOFORCEDTX, TX_CALLBACK_NONE, my_delay);

    return;
}

void rdcp_send_roaming_beacon(void)
{
    serial_writeln("INFO: Scheduling CHANNEL868DA Roaming Beacon");
    rdcp_response.header.destination = RDCP_BROADCAST_ADDRESS;
    rdcp_response.header.message_type = RDCP_MSGTYPE_ROAMINGBEACON;
    rdcp_response.header.rdcp_payload_length = COUNT_ZERO;
    rdcp_response.header.sequence_number = RDCP_SEQUENCENR_SPECIAL_ZERO;
    rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_prepare_response_header(true);
    rdcp_pass_response_to_scheduler(CHANNEL868DA);
    return;
}

/**
 * When we received a PING, respond with a PONG.
 */
void rdcp_cmd_send_echo_response(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return; // respond to personal pings only
    rdcp_response.header.destination = rdcp_msg_in.header.origin;
    rdcp_response.header.message_type = RDCP_MSGTYPE_ECHO_RESPONSE;
    rdcp_response.header.rdcp_payload_length = RDCP_PAYLOAD_SIZE_ECHO_RESPONSE;

    /* Respond on the same channel we got the request from unless it was forwarded, set Relays accordingly. */
    if ((current_lora_message.channel == CHANNEL433) || 
        (rdcp_msg_in.header.origin != rdcp_msg_in.header.sender))
    {
        rdcp_response.header.relay1 = (CFG.cirerelays[0] << 4) + 0;
        rdcp_response.header.relay2 = (CFG.cirerelays[1] << 4) + 1;
        rdcp_response.header.relay3 = (CFG.cirerelays[2] << 4) + 2;
        rdcp_prepare_response_header(false);
        rdcp_pass_response_to_scheduler(CHANNEL433);
    }
    else
    {
        rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_prepare_response_header(false);
        rdcp_pass_response_to_scheduler(CHANNEL868DA);
    }

    return;
}

/**
 * Send a DA Status Reponse after receiving a DA Status Request.
 */
void rdcp_cmd_send_da_status_response(bool unsolicited = false)
{
    uint8_t want_reset = 0x00;

    if (!unsolicited)
    {
        if (rdcp_msg_in.header.destination != CFG.rdcp_address) return; // respond to personal status request only
        want_reset = rdcp_msg_in.payload.data[0];
    }

    last_dasresp_sent = my_millis(); // Track when we sent the most recent DA Status Response

    rdcp_response.header.destination  = RDCP_HQ_MULTICAST_ADDRESS;
    rdcp_response.header.message_type = RDCP_MSGTYPE_DA_STATUS_RESPONSE;

    rdcp_response.header.relay1 = (CFG.cirerelays[0] << 4) + 0;
    rdcp_response.header.relay2 = (CFG.cirerelays[1] << 4) + 1;
    rdcp_response.header.relay3 = (CFG.cirerelays[2] << 4) + 2;

    rdcp_response.payload.data[0] = want_reset;    // Counter reset mirrors request
    rdcp_response.payload.data[1] = DART.battery1; // Hopefully the DA has updated this via Serial
    rdcp_response.payload.data[2] = DART.battery2;
    rdcp_response.payload.data[3] = DART.num_rdcp_rx % 256;
    rdcp_response.payload.data[4] = DART.num_rdcp_rx / 256;
    rdcp_response.payload.data[5] = DART.num_rdcp_tx % 256;
    rdcp_response.payload.data[6] = DART.num_rdcp_tx / 256;

    uint16_t num_mgs = 0;
    uint16_t num_das = 0;
    for (int i=0; i < MAX_NEIGHBORS; i++)
    {
        if ((neighbors[i].sender >= RDCP_ADDRESS_MG_LOWERBOUND) && (neighbors[i].sender < RDCP_ADDRESS_MULTICAST_LOWERBOUND))
        {
            if (!neighbors[i].counted)
            {
                num_mgs++;
                if (want_reset) neighbors[i].counted = true;
            }
        }
        if ((neighbors[i].sender >= RDCP_ADDRESS_HQ_LOWERBOUND) && (neighbors[i].sender < RDCP_ADDRESS_MG_LOWERBOUND)) // include HQ devices
        {
            if (!neighbors[i].counted)
            {
                rdcp_response.payload.data[9 + (4*num_das + 0)] = neighbors[i].sender % 256;
                rdcp_response.payload.data[9 + (4*num_das + 1)] = neighbors[i].sender / 256;
                rdcp_response.payload.data[9 + (4*num_das + 2)] = BIAS_RSSI + (int8_t) neighbors[i].rssi;
                rdcp_response.payload.data[9 + (4*num_das + 3)] = BIAS_SNR + (int8_t) neighbors[i].snr;
                num_das++;
                if (want_reset) neighbors[i].counted = true;
            }
        }
    }

    rdcp_response.payload.data[7] = num_mgs % 256;
    rdcp_response.payload.data[8] = num_mgs / 256;

    rdcp_response.header.rdcp_payload_length = 9 + 4 * num_das;

    if (want_reset != 0x00)
    {
        DART.battery1 = 255;
        DART.battery2 = 255;
        DART.num_rdcp_rx = 0;
        DART.num_rdcp_tx = 0;
    }

    rdcp_prepare_response_header(false);
    rdcp_pass_response_to_scheduler(CHANNEL433);

    /* As the HQ might be next to us, we also have to send this on 868 MHz. */
    rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_prepare_response_header(true);
    rdcp_pass_response_to_scheduler(CHANNEL868DA);

    /*
      We use DA Status Requests periodically to align when we send Heartbeats.
      Based on the assumption that DA Status Requests are spread evenly by the HQ,
      this contributes to collision avoidance by lowering the probability that
      multiple DAs send their Heartbeats at roughly the same time.
    */
    if (unsolicited) return; // do not align on self-timed DA Status Requests
    dasr_counter++;
    if (dasr_counter >= DASR_ROLLOVER)
    {
        dasr_counter = 0;
        last_heartbeat_sent = my_millis() - CFG.heartbeat_interval + (30 * SECONDS_TO_MILLISECONDS * CFG.sf_multiplier) + 
            (3 * SECONDS_TO_MILLISECONDS * CFG.relay_identifier * CFG.sf_multiplier);
    }

    return;
}

/**
 * In order to verify the Schnorr signature of an incoming RDCP Message, calculate the
 * hash value for the relevant RDCP Header and RDCP Payload elements.
 * @param m Pointer to an rdcp_message data structure
 * @param payloadprefixlength Number of bytes at the beginning of the RDCP Payload to consider
 * @param hashtarget Pointer where to store the 32-byte hash result
 */
void get_inline_hash(struct rdcp_message *m, uint8_t payloadprefixlength, uint8_t *hashtarget)
{
  /* Prepare the signed data */
  uint8_t data_to_sign[INFOLEN];
  data_to_sign[0] = m->header.origin % 256;
  data_to_sign[1] = m->header.origin / 256;
  data_to_sign[2] = m->header.sequence_number % 256;
  data_to_sign[3] = m->header.sequence_number / 256;
  data_to_sign[4] = m->header.destination % 256;
  data_to_sign[5] = m->header.destination / 256;
  data_to_sign[6] = m->header.message_type;
  data_to_sign[7] = m->header.rdcp_payload_length;
  for (int i=0; i<payloadprefixlength; i++) data_to_sign[8+i] = m->payload.data[i];
  uint8_t data_to_sign_length = 8 + payloadprefixlength;

  /* Get the SHA-256 hash for the data */
  SHA256 h = SHA256();
  h.reset();
  h.update(data_to_sign, data_to_sign_length);
  uint8_t sha[SHABUFSIZE];
  h.finalize(sha, SHABUFSIZE);

  /* Copy result to target buffer */
  for (int i=0; i<SHABUFSIZE; i++) hashtarget[i] = sha[i];

  return;
}

/**
 * Handle an RDCP Device Block Alert message.
 */
void rdcp_cmd_block_device(void)
{
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + RDCP_PAYLOAD_SIZE_INLINE_BLOCKDEVICE)
    {
      serial_writeln("WARNING: Payload size of received RDCP Block Device Alert is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, RDCP_PAYLOAD_SIZE_INLINE_BLOCKDEVICE, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[RDCP_PAYLOAD_SIZE_INLINE_BLOCKDEVICE+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Block Device Alert - ignoring");
      return;
    }

    uint16_t target = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];
    uint16_t duration = rdcp_msg_in.payload.data[2] + 256 * rdcp_msg_in.payload.data[3];

    char info[INFOLEN];
    if (duration == RDCP_DURATION_ZERO)
    {
        snprintf(info, INFOLEN, "INFO: Lifting device block for %04X", target);
        serial_writeln(info);
        rdcp_device_block_remove(target);
    }
    else
    {
        snprintf(info, INFOLEN, "INFO: Blocking device %04X for %d minutes", target, duration);
        serial_writeln(info);
        rdcp_device_block_add(target, duration);
    }

    if (target == CFG.rdcp_address)
    {
        snprintf(info, INFOLEN, "DA_BLOCK %04X %d", target, duration);
        serial_writeln(info);
    }

    return;
}

/**
 * Process a received RDCP Timestamp message.
 */
void rdcp_cmd_timestamp(void)
{
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + RDCP_PAYLOAD_SIZE_INLINE_TIMESTAMP)
    {
      serial_writeln("WARNING: Payload size of received RDCP Timestamp is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, RDCP_PAYLOAD_SIZE_INLINE_TIMESTAMP, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[RDCP_PAYLOAD_SIZE_INLINE_TIMESTAMP+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Timestamp - ignoring");
      return;
    }

    uint8_t year = rdcp_msg_in.payload.data[0];
    uint8_t month = rdcp_msg_in.payload.data[1];
    uint8_t day = rdcp_msg_in.payload.data[2];
    uint8_t hour = rdcp_msg_in.payload.data[3];
    uint8_t minute = rdcp_msg_in.payload.data[4];
    uint8_t status = rdcp_msg_in.payload.data[5];

    char msg[INFOLEN];
    snprintf(msg, INFOLEN, "INFO: Received valid RDCP Timestamp: %02d.%02d.%04d %02d:%02d (Status %d)", day, month, 2025 + year, hour, minute, status);
    serial_writeln(msg);

    snprintf(msg, INFOLEN, "DA_TIME %02d.%02d.%04d %02d:%02d (%d)", day, month, 2025 + year, hour, minute, status);
    serial_writeln(msg);

    /*
        We don't need the timestamp ourselves, only the overall RDCP Infrastructure status.
    */
    if (CFG.infrastructure_status != status)
    { // Infrastructure status change
      if (CFG.infrastructure_status == RDCP_INFRASTRUCTURE_MODE_NONCRISIS)
      { // switch from non-crisis to any crisis mode
        serial_writeln("INFO: Dropping memories based on switch from non-crisis to crisis mode");
        rdcp_memory_forget();
      }
      else 
      { // switch from any crisis mode, forget memories if target mode is non-crisis
        if (status == RDCP_INFRASTRUCTURE_MODE_NONCRISIS) 
        {
            rdcp_memory_forget();
            serial_writeln("INFO: Dropping memories based on switch from crisis to non-crisis mode");
        }
      }
    }
    CFG.infrastructure_status = status;

    return;
}

/**
 * Derive the infrastructure status from a multicast/broadcast OA.
 */
void rdcp_derive_infrastructure_status_from_oa(void)
{
    if ((rdcp_msg_in.header.destination == RDCP_BROADCAST_ADDRESS) ||
       ((rdcp_msg_in.header.destination >= RDCP_ADDRESS_MULTICAST_LOWERBOUND) && (rdcp_msg_in.header.destination <= RDCP_ADDRESS_MULTICAST_UPPERBOUND)))
    {
        /* RDCP v0.4 OAs have a subheader, and thus the OA subtype is the first byte of the RDCP Payload */
        uint8_t oatype = rdcp_msg_in.payload.data[0];
        uint8_t old_infrastructure_mode = CFG.infrastructure_status;
        if (oatype == RDCP_MSGTYPE_OA_SUBTYPE_NONCRISIS)  CFG.infrastructure_status = RDCP_INFRASTRUCTURE_MODE_NONCRISIS;
        if (oatype == RDCP_MSGTYPE_OA_SUBTYPE_CRISIS_TXT) CFG.infrastructure_status = RDCP_INFRASTRUCTURE_MODE_CRISIS;
        if (CFG.infrastructure_status != old_infrastructure_mode)
        {
            if (old_infrastructure_mode == RDCP_INFRASTRUCTURE_MODE_NONCRISIS)
            {
                serial_writeln("INFO: Dropping memories based on switch from non-crisis to crisis mode");
                rdcp_memory_forget();
            }
            else 
            {
                if (CFG.infrastructure_status == RDCP_INFRASTRUCTURE_MODE_NONCRISIS)
                {
                    rdcp_memory_forget();
                    serial_writeln("INFO: Dropping memories based on switch from crisis to non-crisis mode");
                }
            }
        }
    }
    return;
}

/**
 * Process an RDCP Device Reset message.
 */
void rdcp_cmd_device_reset(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return; // respond to personal device resets only

    uint8_t cmd_payload_len = RDCP_PAYLOAD_SIZE_INLINE_DEVICERESET;
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + cmd_payload_len)
    {
      serial_writeln("WARNING: Payload size of received RDCP Device Reset is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, cmd_payload_len, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[cmd_payload_len+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Device Reset - ignoring");
      return;
    }

    uint16_t nonce = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];

    char noncename[NONCENAMESIZE]; snprintf(noncename, NONCENAMESIZE, "rstdev");
    if (!persistence_checkset_nonce(noncename, nonce))
    {
      serial_writeln("WARNING: Invalid nonce received for signed RDCP RESET of DEVICE");
      return;
    }
    else
    {
      serial_writeln("INFO: Performing RESET OF DEVICE");
      serial_writeln("DA_RESETDEVICE");
    }

    /* We can reset all volatile data by simply restarting. Short delay to make sure DA got the message. */
    rdcp_duplicate_table_delete_file();
    rdcp_duplicate_table_delete_all_entries();
    do_not_persist_dupetable = true;
    rdcp_memory_forget();
    delay(5000);
    ESP.restart();

    return;
}

/**
 * Process an RDCP Device Reboot message.
 */
void rdcp_cmd_device_reboot(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return; // respond to personal device reboots only

    uint8_t cmd_payload_len = RDCP_PAYLOAD_SIZE_INLINE_DEVICEREBOOT;
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + cmd_payload_len)
    {
      serial_writeln("WARNING: Payload size of received RDCP Device Reboot is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, cmd_payload_len, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[cmd_payload_len+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Device Reboot - ignoring");
      return;
    }

    uint16_t nonce = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];

    char noncename[NONCENAMESIZE]; snprintf(noncename, NONCENAMESIZE, "rstdev");
    if (!persistence_checkset_nonce(noncename, nonce))
    {
      serial_writeln("WARNING: Invalid nonce received for signed RDCP Reboot");
      return;
    }
    else
    {
      serial_writeln("INFO: Performing reboot");
      serial_writeln("DA_REBOOT");
    }

    delay(5000);
    ESP.restart();

    return;
}

/**
 * Process an RDCP Maintenance message.
 */
void rdcp_cmd_maintenance(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return; // respond to personal device maintenance only

    uint8_t cmd_payload_len = RDCP_PAYLOAD_SIZE_INLINE_MAINTENANCE;
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + cmd_payload_len)
    {
      serial_writeln("WARNING: Payload size of received RDCP Device Maintenance is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, cmd_payload_len, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[cmd_payload_len+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Maintenance - ignoring");
      return;
    }

    uint16_t nonce = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];

    char noncename[NONCENAMESIZE]; snprintf(noncename, NONCENAMESIZE, "rstdev");
    if (!persistence_checkset_nonce(noncename, nonce))
    {
      serial_writeln("WARNING: Invalid nonce received for signed RDCP Maintenance");
      return;
    }
    else
    {
      serial_writeln("INFO: Starting DA Maintenance mode");
      serial_writeln("DA_MAINTENANCE");
      enable_bt();
    }

    return;
}

/**
 * Process an RDCP Infrastructure Reset message.
 */
void rdcp_cmd_infrastructure_reset(void)
{
    uint8_t cmd_payload_len = RDCP_PAYLOAD_SIZE_INLINE_INFRARESET;
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH + cmd_payload_len)
    {
      serial_writeln("WARNING: Payload size of received RDCP Infrastructure Reset is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, cmd_payload_len, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[cmd_payload_len+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP Infrastructure Reset - ignoring");
      return;
    }

    uint16_t nonce = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];

    char noncename[NONCENAMESIZE]; snprintf(noncename, NONCENAMESIZE, "rstinfra");
    if (!persistence_checkset_nonce(noncename, nonce))
    {
      serial_writeln("WARNING: Invalid nonce received for signed RDCP Infrastructure Reset");
      return;
    }
    else
    {
      serial_writeln("INFO: Performing Infrastructure Reset");
      serial_writeln("DA_INFRASTRUCTURE_RESET");
    }

    rdcp_memory_forget();
    reboot_requested = my_millis() + 3 * MINUTES_TO_MILLISECONDS; // Reboot after 3 minutes
    seqnr_reset_requested = true; // Reset sequence numbers on infrastructure reset
    rdcp_duplicate_table_delete_file();
    rdcp_duplicate_table_delete_all_entries();
    do_not_persist_dupetable = true;

    return;
}

/**
 * Process an RDCP Reset of Official Announcements message.
 */
void rdcp_cmd_oa_reset(void)
{
    if (rdcp_msg_in.header.rdcp_payload_length != RDCP_SIGNATURE_LENGTH)
    {
      serial_writeln("WARNING: Payload size of received RDCP OA Reset is invalid - ignoring");
      return;
    }

    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, 0, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[0+i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP OA Reset - ignoring");
      return;
    }

    serial_writeln("DA_OA_RESET");
    rdcp_memory_forget();
    oa_reset_seen = true;

    return;
}

/**
 * Respond to a neighbor's Fetch All New Messages request.
 */
void rdcp_cmd_fetch_all(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return;

    serial_writeln("INFO: Responding to Fetch All New Messages from neighbor");

    /* Other side sends 0x0000 or latest refnr stored there, so increase by 1. */
    uint16_t wanted_min_ref = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1] + 1;

    int first = mem.idx_first;
    int starter = RDCP_INDEX_NONE;
    if (first != RDCP_INDEX_NONE)
    {
        for (int i=0; i < MAX_STORED_MSGS; i++)
        {
            if (mem.entries[(i + first) % MAX_STORED_MSGS].reference_number >= wanted_min_ref)
            {
                starter = i;
                break; // find first only
            }
        }
    }

    /* Sends a delivery receipt on starter == -1 */
    rdcp_chain_starter(TX_CALLBACK_FETCH_ALL, starter, rdcp_msg_in.header.origin, wanted_min_ref);

    return;
}

/**
 * Respond to a neighbor's Fetch (single message) request.
 */
void rdcp_cmd_fetch_one(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return;

    uint16_t wanted_ref = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];

    int first = mem.idx_first;
    int starter = RDCP_INDEX_NONE;
    if (first != RDCP_INDEX_NONE)
    {
        for (int i=0; i < MAX_STORED_MSGS; i++)
        {
            if (mem.entries[(i + first) % MAX_STORED_MSGS].reference_number == wanted_ref)
            {
                starter = i;
                break; // find first only
            }
        }
    }

    /* Sends a delivery receipt on starter == -1 */
    rdcp_chain_starter(TX_CALLBACK_FETCH_SINGLE, starter, rdcp_msg_in.header.origin, wanted_ref);

    return;
}

void rdcp_check_heartbeat(void)
{
    if (CFG.heartbeat_interval == 0) return; // Heartbeat-sending disabled

    if (last_heartbeat_sent == RDCP_TIMESTAMP_ZERO)
    { // Set initial delay for heartbeat messages
      // (intended to evenly spread heartbeat messages after scenario-wide device resets)
      last_heartbeat_sent = (CFG.heartbeat_interval / CFG.scenario_num_relays) * (CFG.relay_identifier + 1);
    }

    int64_t now = my_millis();
    if (last_heartbeat_sent + CFG.heartbeat_interval < now)
    {
        /* Don't even schedule a heartbeat when 433 MHz is currently very busy. */
        if (get_num_txq_entries(CHANNEL433) > 1)
        {
            serial_writeln("WARNING: Postponing heartbeat due to busy 433 MHz channel");
            last_heartbeat_sent += 5 * MINUTES_TO_MILLISECONDS;
            return;
        }

        last_heartbeat_sent = now;
        serial_writeln("INFO: Preparing to send DA Heartbeat to HQ");

        rdcp_response.header.destination = RDCP_HQ_MULTICAST_ADDRESS;
        rdcp_response.header.message_type = RDCP_MSGTYPE_HEARTBEAT;
        rdcp_response.header.relay1 = (CFG.cirerelays[0] << 4) + 0;
        rdcp_response.header.relay2 = (CFG.cirerelays[1] << 4) + 1;
        rdcp_response.header.relay3 = (CFG.cirerelays[2] << 4) + 2;

        uint16_t num_mgs = 0;
        for (int i=0; i < MAX_NEIGHBORS; i++)
        {
            if ((neighbors[i].sender >= RDCP_ADDRESS_MG_LOWERBOUND) && (neighbors[i].sender < RDCP_ADDRESS_MULTICAST_LOWERBOUND))
            {
                if (neighbors[i].timestamp > now - CFG.heartbeat_interval)
                {
                    if (num_mgs <= 91) // max. number of entries according to specs
                    {
                        rdcp_response.payload.data[2 + (2*num_mgs + 0)] = neighbors[i].sender % 256;
                        rdcp_response.payload.data[2 + (2*num_mgs + 1)] = neighbors[i].sender / 256;
                    }
                    num_mgs++;
                }
            }
        }

        rdcp_response.payload.data[0] = num_mgs % 256;
        rdcp_response.payload.data[1] = num_mgs / 256;
        rdcp_response.header.rdcp_payload_length = 2 + 2 * num_mgs;

        rdcp_prepare_response_header(false);
        rdcp_pass_response_to_scheduler(CHANNEL433, true);

        /* As the HQ might be next to us, we also have to send this on 868 MHz. */
        rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;
        rdcp_prepare_response_header(true); // re-use sequence number from 433 MHz channel message
        rdcp_pass_response_to_scheduler(CHANNEL868DA, true);
    }

    return;
}

/**
 * Handle a received RDCP Delivery Receipt.
 */
void rdcp_cmd_delivery_receipt(void)
{
    if (rdcp_msg_in.header.destination != CFG.rdcp_address) return;
    new_delivery_receipt_from = rdcp_msg_in.header.origin;
    char info[INFOLEN];
    snprintf(info, INFOLEN, "INFO: Received delivery receipt from %04X", rdcp_msg_in.header.origin);
    serial_writeln(info);
    snprintf(info, INFOLEN, "DA_DELIVERY %04X", rdcp_msg_in.header.origin);
    serial_writeln(info);
    currently_in_fetch_mode = false;
    fetch_timeout = RDCP_TIMESTAMP_ZERO;
    return;
}

void rdcp_cmd_check_rtc(void)
{
    if (!rtc_active) return;
    bool one_active = false;
    for (int i=0; i<MAX_RTC; i++)
        if (RTC[i].active) one_active = true;
    if (!one_active) rtc_active = false;
    else
    {
        for (int i=0; i<MAX_RTC; i++)
        {
            if ((RTC[i].active) && (my_millis() > RTC[i].alarm))
            {
                RTC[i].active = false;
                String s = String(RTC[i].rtc);
                if (RTC[i].restart) reboot_requested = my_millis() + 5 * RTC[i].restart * MINUTES_TO_MILLISECONDS;
                serial_process_command(s, String("RTC: "), RTC[i].persist != 0 ? true:false);
            }
        }
    }

    return;
}

void rdcp_cmd_rtc(void)
{
    uint8_t sha[SHABUFSIZE];
    get_inline_hash(&rdcp_msg_in, rdcp_msg_in.header.rdcp_payload_length - RDCP_SIGNATURE_LENGTH, sha);
    uint8_t sig[SIGBUFSIZE];
    for (int i=0; i<RDCP_SIGNATURE_LENGTH; i++) sig[i] = rdcp_msg_in.payload.data[rdcp_msg_in.header.rdcp_payload_length - RDCP_SIGNATURE_LENGTH + i];
    bool valid_signature = schnorr_verify_signature(sha, SHABUFSIZE, sig);
    if (!valid_signature)
    {
      serial_writeln("WARNING: Invalid HQ Schnorr signature for RDCP RTC - ignoring");
      return;
    }

    for (int i=0; i<MAX_RTC; i++)
    {
        if (RTC[i].active == false)
        {
            RTC[i].active  = true;
            RTC[i].alarm   = my_millis() + rdcp_msg_in.payload.data[0] * MINUTES_TO_MILLISECONDS;
            RTC[i].restart = rdcp_msg_in.payload.data[1];
            RTC[i].persist = rdcp_msg_in.payload.data[2];
            for (int j=0; j<rdcp_msg_in.header.rdcp_payload_length - RDCP_SIGNATURE_LENGTH - RDCP_PAYLOAD_SIZE_INLINE_RTC; j++)
            {
                RTC[i].rtc[j+0] = rdcp_msg_in.payload.data[j+RDCP_PAYLOAD_SIZE_INLINE_RTC];
                RTC[i].rtc[j+1] = 0;
            }
            break;
        }
    }
    rtc_active = true;
    return;
}

void rdcp_handle_command(void)
{
    uint8_t mt = rdcp_msg_in.header.message_type;

    if (mt == RDCP_MSGTYPE_ECHO_REQUEST) rdcp_cmd_send_echo_response();
    else if (mt == RDCP_MSGTYPE_DA_STATUS_REQUEST) rdcp_cmd_send_da_status_response();
    else if (mt == RDCP_MSGTYPE_DEVICE_BLOCK_ALERT) rdcp_cmd_block_device();
    else if (mt == RDCP_MSGTYPE_TIMESTAMP) rdcp_cmd_timestamp();
    else if (mt == RDCP_MSGTYPE_DEVICE_RESET) rdcp_cmd_device_reset();
    else if (mt == RDCP_MSGTYPE_DEVICE_REBOOT) rdcp_cmd_device_reboot();
    else if (mt == RDCP_MSGTYPE_MAINTENANCE) rdcp_cmd_maintenance();
    else if (mt == RDCP_MSGTYPE_INFRASTRUCTURE_RESET) rdcp_cmd_infrastructure_reset();
    else if (mt == RDCP_MSGTYPE_RESET_ALL_ANNOUNCEMENTS) rdcp_cmd_oa_reset();
    else if (mt == RDCP_MSGTYPE_FETCH_ALL_NEW_MESSAGES) rdcp_cmd_fetch_all();
    else if (mt == RDCP_MSGTYPE_FETCH_MESSAGE) rdcp_cmd_fetch_one();
    else if (mt == RDCP_MSGTYPE_DELIVERY_RECEIPT) rdcp_cmd_delivery_receipt();
    else if (mt == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT) rdcp_derive_infrastructure_status_from_oa();
    else if (mt == RDCP_MSGTYPE_RTC) rdcp_cmd_rtc();

    return;
}

void rdcp_send_cire(uint8_t subtype, uint16_t refnr, char *content)
{
    rdcp_response.header.destination = RDCP_HQ_MULTICAST_ADDRESS;
    rdcp_response.header.message_type = RDCP_MSGTYPE_CITIZEN_REPORT;
    rdcp_response.header.relay1 = (CFG.cirerelays[0] << 4) + 0;
    rdcp_response.header.relay2 = (CFG.cirerelays[1] << 4) + 1;
    rdcp_response.header.relay3 = (CFG.cirerelays[2] << 4) + 2;

    rdcp_response.header.sender = CFG.rdcp_address;
    rdcp_response.header.origin = CFG.rdcp_address;
    rdcp_response.header.sequence_number = get_next_rdcp_sequence_number(CFG.rdcp_address);
    rdcp_response.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(rdcp_response.header.message_type);

    /* Inform the DA about the used Sequence Number */
    char da_info[INFOLEN];
    snprintf(da_info, INFOLEN, "DA_CIRESEQ %04X", rdcp_response.header.sequence_number);
    serial_writeln(da_info);

    /* Prepare the MT-specific header at the beginning of the RDCP Payload */
    rdcp_response.payload.data[0] = subtype;
    rdcp_response.payload.data[1] = refnr % 256;
    rdcp_response.payload.data[2] = refnr / 256;

    /* Convert the text message to its Unishox2 representation */
    char buf[INFOLEN];
    unsigned int len = strlen(content);
    memset(buf, 0, sizeof(buf));
    int c_total = unishox2_compress_simple(content, len, buf);

    /* Fill the RDCP Payload with the Unishox2 content */
    for (int i=0; i < c_total; i++) rdcp_response.payload.data[i+RDCP_PAYLOAD_SIZE_SUBHEADER_CIRE] = buf[i];

    /* RDCP Payload length is subheader length (3) + Unishox2 length + AES-GMC AuthTag size (16) */
    rdcp_response.header.rdcp_payload_length = RDCP_PAYLOAD_SIZE_SUBHEADER_CIRE + c_total + RDCP_AESTAG_SIZE;

    /* AES-GCM encrypt the RDCP Payload */
    uint8_t ciphertext[256];
    uint8_t iv[12];
    uint8_t gcmauthtag[16];
    uint8_t additional_data[8];
    uint8_t additional_data_size = 8;

    memset(iv, 0, sizeof(iv));
    iv[0] = rdcp_response.header.origin % 256;
    iv[1] = rdcp_response.header.origin / 256;
    iv[2] = rdcp_response.header.sequence_number % 256;
    iv[3] = rdcp_response.header.sequence_number / 256;
    iv[4] = rdcp_response.header.destination % 256;
    iv[5] = rdcp_response.header.destination / 256;
    iv[6] = rdcp_response.header.message_type;
    iv[7] = rdcp_response.header.rdcp_payload_length;
    for (int i=0; i<additional_data_size; i++) additional_data[i] = iv[i];

    encrypt_aes256gcm((uint8_t *) &rdcp_response.payload.data, rdcp_response.header.rdcp_payload_length - RDCP_AESTAG_SIZE,
                      additional_data, additional_data_size, CFG.hqsharedsecret, 32, iv, 12,
                      ciphertext, gcmauthtag, RDCP_AESTAG_SIZE);

    /* Copy ciphertext and GCM AuthTag into the RDCP Payload */
    for (int i=0; i<rdcp_response.header.rdcp_payload_length-RDCP_AESTAG_SIZE; i++)
        rdcp_response.payload.data[i] = ciphertext[i];
    for (int i=0; i<RDCP_AESTAG_SIZE; i++)
        rdcp_response.payload.data[rdcp_response.header.rdcp_payload_length-RDCP_AESTAG_SIZE+i] = gcmauthtag[i];

    /* Update CRC header field */
    uint8_t data_for_crc[INFOLEN];
    memcpy(&data_for_crc, &rdcp_response.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++)
        data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rdcp_response.payload.data[i];
    uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + rdcp_response.header.rdcp_payload_length);
    rdcp_response.header.checksum = actual_crc;

    //0 - my_random_in_range(1000 * CFG.sf_multiplier, 2000 * CFG.sf_multiplier);
    int64_t my_delay = 0 - (100 * CFG.sf_multiplier); // send very timely on 433 MHz channel free
    uint8_t data_for_scheduler[INFOLEN];
    memcpy(&data_for_scheduler, &rdcp_response.header, RDCP_HEADER_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++)
        data_for_scheduler[i + RDCP_HEADER_SIZE] = rdcp_response.payload.data[i];

    /* Send on both channels in case we have an HQ in our 868 MHz range */
    /* First, 433 MHz channel. */
    rdcp_txqueue_add(CHANNEL433, data_for_scheduler, RDCP_HEADER_SIZE + rdcp_response.header.rdcp_payload_length,
      IMPORTANT, NOFORCEDTX, TX_CALLBACK_CIRE, my_delay);

    /* Second, 868 MHz channel. Header fields need to be adjusted. */
    rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
    rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

    /* Update CRC header field */
    memcpy(&data_for_crc, &rdcp_response.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++) 
        data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = rdcp_response.payload.data[i];
    actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + rdcp_response.header.rdcp_payload_length);
    rdcp_response.header.checksum = actual_crc;

    // 0 - my_random_in_range(1000 * CFG.sf_multiplier, 2000 * CFG.sf_multiplier);
    my_delay = 0 - (3 * SECONDS_TO_MILLISECONDS * CFG.sf_multiplier); // send after headstart for 433 MHz channel
    memcpy(&data_for_scheduler, &rdcp_response.header, RDCP_HEADER_SIZE);
    for (int i=0; i < rdcp_response.header.rdcp_payload_length; i++) 
        data_for_scheduler[i + RDCP_HEADER_SIZE] = rdcp_response.payload.data[i];

    rdcp_txqueue_add(CHANNEL868DA, data_for_scheduler, RDCP_HEADER_SIZE + rdcp_response.header.rdcp_payload_length,
      IMPORTANT, NOFORCEDTX, TX_CALLBACK_NONE, my_delay);

    return;
}

void rdcp_command_fetch_from_neighbor(void)
{
  if (!CFG.fetch_enabled)
  {
    serial_writeln("INFO: Fetching is disabled, skipping FETCH ALL procedure");
    return;
  }

  serial_writeln("INFO: Preparing to fetch from neighbor (FETCH ALL)");

  uint16_t my_latest = RDCP_OA_REFNR_SPECIAL_ZERO;
  for (int i=0; i < MAX_STORED_MSGS; i++)
  {
    if (mem.entries[i].reference_number > my_latest) my_latest = mem.entries[i].reference_number;
  }

  rdcp_response.header.destination = CFG.neighbor_for_fetch;
  rdcp_response.header.message_type = RDCP_MSGTYPE_FETCH_ALL_NEW_MESSAGES;
  rdcp_response.header.rdcp_payload_length = RDCP_PAYLOAD_SIZE_FANM;
  rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
  rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
  rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

  rdcp_response.payload.data[0] = my_latest % 256;
  rdcp_response.payload.data[1] = my_latest / 256;

  rdcp_prepare_response_header(false);
  rdcp_pass_response_to_scheduler(CHANNEL433, true);

  return;
}

void rdcp_command_fetch_one_from_neighbor(uint16_t refnr)
{
  serial_writeln("INFO: Preparing to fetch a single message from neighbor");

  rdcp_response.header.destination = CFG.neighbor_for_fetch;
  rdcp_response.header.message_type = RDCP_MSGTYPE_FETCH_MESSAGE;
  rdcp_response.header.rdcp_payload_length = RDCP_PAYLOAD_SIZE_FETCHONE;
  rdcp_response.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
  rdcp_response.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
  rdcp_response.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

  rdcp_response.payload.data[0] = refnr % 256;
  rdcp_response.payload.data[1] = refnr / 256;

  rdcp_prepare_response_header(false);
  rdcp_pass_response_to_scheduler(CHANNEL433, true);
  currently_in_fetch_mode = true;

  return;
}

void rdcp_check_fetch_timeout(void)
{
    /* On first run, set timeout only. */
    if (fetch_timeout == RDCP_TIMESTAMP_ZERO)
    {
        fetch_timeout = my_millis() + 10 * MINUTES_TO_MILLISECONDS;
        return;
    }

    /* Not first run, check for timeout. */
    if (my_millis() > fetch_timeout)
    {
        serial_writeln("INFO: Timeout when fetching memories from neighbor");
        serial_writeln("DA_FETCHTIMEOUT");
        currently_in_fetch_mode = false;
        fetch_timeout = RDCP_TIMESTAMP_ZERO;
    }

    return;
}

/* EOF */
