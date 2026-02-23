#ifndef _ROLORAN_DA_LORA
#define _ROLORAN_DA_LORA

#include <Arduino.h> 
#include "rdcp-common.h"
#include "hal.h"

/**
 * Pinout for EBYTE LoRa modules on custom PCB
 */
#define RADIO868DACS    10  
#define RADIO868DADIO1  14
#define RADIO868DARESET 41
#define RADIO868DABUSY  37
#define RADIO868DACLK   12
#define RADIO868DAMISO  13
#define RADIO868DAMOSI  11
#define RADIO868DATXEN  18
#define RADIO868DARXEN   8

#define RADIO868MGCS     6  
#define RADIO868MGDIO1  17
#define RADIO868MGRESET 42
#define RADIO868MGBUSY  36
#define RADIO868MGCLK   12
#define RADIO868MGMISO  13
#define RADIO868MGMOSI  11
#define RADIO868MGTXEN  15
#define RADIO868MGRXEN  16

#define RADIO868LWCS    47  
#define RADIO868LWDIO1   7
#define RADIO868LWRESET 48
#define RADIO868LWBUSY  40
#define RADIO868LWCLK   12
#define RADIO868LWMISO  13
#define RADIO868LWMOSI  11
#define RADIO868LWTXEN   4
#define RADIO868LWRXEN   5

#define RADIO433CS      39
#define RADIO433DIO1     1
#define RADIO433RESET   38
#define RADIO433BUSY     9
#define RADIO433CLK     12
#define RADIO433MISO    13
#define RADIO433MOSI    11
#define RADIO433TXEN    21
#define RADIO433RXEN     2

/**
 * LoRa-specific constants used globally 
 */
#define CHANNEL433      0
#define CHANNEL868DA    1
#define CHANNEL868MG    2
#define CHANNEL868LW    3
#define NUMCHANNELS     4

#define NO_CHANNEL      99

/**
 * Configuration settings for one LoRa channel
 */
struct lora_channel_config {
    float    freq = 0;          /// Frequency; must be set individually in MHz
    float    bw   = 125.0;      /// Bandwidth in kHz
    int      sf   = 7;          /// LoRa spreading factor
    int      cr   = 5;          /// LoRa coding rate (5--8) as per RadioLib
    uint8_t  sw   = 0x12;       /// LoRa sync word (0x12 == private, 0x34 == public)
    int      pw   = 0;          /// TX power in dBm (for the SX126x, consider PAs for regulations!)
    uint16_t pl   = 15;         /// LoRa preamble length
};

/**
 * Configuration for an RDCP DA/Two-Channel Relay
 */
struct da_config {
    lora_channel_config lora[NUMCHANNELS];                        /// Settings for both LoRa channels
    uint16_t rdcp_address                 = 0x02FF;               /// Own RDCP Address
    uint8_t  relay_identifier             = 0x0C;                 /// Own assigned RDCP Relay Identifier
    uint8_t  scenario_num_relays          = 10;                   /// Number of relays in the current RDCP scenario
    uint16_t oarelays[3]                  = {0x0D, 0x0D, 0x0D};   /// Three other relay identifiers for OA relaying
    uint16_t cirerelays[3]                = {0x0D, 0x0D, 0x0D};   /// Three other relay identifiers for CIRE relaying
    bool     ts4allones                   = false;                /// Let everyone relay in the final timeslot
    uint8_t  ts7relay1                    = 0xE0;                 /// Value to set as Relay1 when relaying in Timeslot 7
    uint16_t multicast[5]                 = {0, 0, 0, 0, 0};      /// Own RDCP Multicast Addresses/groups
    uint16_t neighbor_for_fetch           = 0x02FE;               /// Which neighbor (RDCP Address) to fetch all new messages from on start
    char     name[64]                     = {0};                  /// Human-readable device name
    char     hqpubkey[256]                = {0};                  /// HQ's public key to verify Schnorr signatures
    char     mypubkey[256]                = {0};                  /// Own Schnorr public key, if any
    char     myprivkey[256]               = {0};                  /// Own Schnorr private key, if any
    uint8_t  hqsharedsecret[32]           = {0};                  /// Shared secret between this device and HQ
    bool     relay_enabled                = true;                 /// Operate as RDCP Two-Channel Relay 
    bool     ep_enabled                   = true;                 /// Operate as RDCP Entry Point
    bool     forward_enabled              = true;                 /// Forward messages on 868 MHz channel
    bool     status_enabled               = true;                 /// Honor RDCP DA Status Requests
    bool     fetch_enabled                = true;                 /// Honor RDCP Fetch messages from others
    bool     periodic_enabled             = true;                 /// Send old memories periodically on 868 MHz
    bool     send_enabled                 = true;                 /// Enable or disable sending on both channels
    uint8_t  memory_retransmissions       = 0;                    /// Retransmission counter for old memories on Fetch
    int32_t  heartbeat_interval           = 30 * MINUTES_TO_MILLISECONDS;    /// How often to send DA Heartbeats
    uint8_t  infrastructure_status        = RDCP_INFRASTRUCTURE_MODE_CRISIS; /// Current RDCP Infrastructure Status
    int64_t  max_periodic868_age          = 24 * HOURS_TO_MILLISECONDS;      /// Maximum age of periodically retransmitted old memories
    int64_t  periodic_interval            = 30 * MINUTES_TO_MILLISECONDS;    /// How often to send Periodic868 memories
    bool     bt_enabled                   = false;                /// BLE access
    int64_t  beacon_interval[NUMCHANNELS] = {0, 0, 0, 0};         /// Beacon mode intervals
    uint16_t corridor_basetime            = 10;                   /// seconds to keep channel free for ACKs when hearing CIREs, own basetime, 1-2x for other DAs
    uint8_t  sf_multiplier                = 1;                    /// factor for random delays, 1 for SF7
    uint64_t unsolicited_dasrep_timer     = 180 * MINUTES_TO_MILLISECONDS;   /// Send unsolicited DA Status Reponse if no Status Request received
};

#define MAX_LORA_PAYLOAD_SIZE 250

/**
 * Data structure for storing LoRa packets. 
 * Independent of channel. May be simulated or real. 
 */
struct lora_message {
    bool    available      = false;
    uint8_t payload[MAX_LORA_PAYLOAD_SIZE] = {0};
    uint8_t payload_length = 0;
    double  rssi           = 0.0;
    double  snr            = 0.0;
    uint8_t channel        = NO_CHANNEL;
    int64_t timestamp      = RDCP_TIMESTAMP_ZERO;
};
  
/**
 * Initialize the LoRa radios
 */
void setup_lora_hardware(void);

/**
 * Configure the LoRa radios with channel-specific settings
 */
bool setup_radio(void);

/**
 * Periodically handle reception, transmission, CAD, timeouts etc. 
 */
void loop_radio(void);

/**
 * Send a LoRa packet. 
 * @param channel Either CHANNEL433 or CHANNEL868[DA|MG|LW] 
 * @param payload LoRa packet content to send 
 * @param length Length of payload in bytes 
 */
void send_lora_message_binary(int channel, uint8_t *payload, uint8_t length);

/**
 * Start channel activity detection on the given channel. 
 * Results are processed via callbacks. 
 * @param channel Either CHANNEL433 or CHANNEL868[DA|MG|LW] 
 */
void radio_start_cad(uint8_t channel);

/**
 * Start receiving on the 433 MHz channel.
 */
int start_receive_433(void);

/**
 * Start receiving on the 868 MHz DA channel.
 */
int start_receive_868da(void);

/**
 * Start receiving on the 868 MHz MG channel.
 */
int start_receive_868mg(void);

/**
 * Start receiving on the 868 MHz LW channel.
 */
int start_receive_868lw(void);

/**
 * Start receiving on the given channel.
 */
int start_receive(uint8_t channel);

/**
 * Get a random byte from the 868 MHz DA LoRa radio via RadioLib.
 */
uint8_t radio868_random_byte(void);

#endif
/* EOF */