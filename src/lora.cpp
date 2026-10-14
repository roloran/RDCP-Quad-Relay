#include "lora.h"
#include <RadioLib.h>
#include <SPI.h>
#include "serial.h"
#include "Base64ren.h"
#include "hal.h"
#include "rdcp-send.h"
#include "rdcp-common.h"
#include "lora-868-route.h"

#ifdef VSPI
SPIClass vspi = SPIClass(VSPI);
#else 
SPIClass vspi = SPIClass(FSPI);
#endif
SPIClass hspi = SPIClass(HSPI);

// #define _BREADBOARD_MIRROR

#ifdef _BREADBOARD_MIRROR
#define MY_SX1262 SX1268
#define MY_SX1268 SX1262
#else 
#define MY_SX1262 SX1262
#define MY_SX1268 SX1268
#endif

#define SPI_ONE vspi 
#define SPI_TWO vspi

constexpr float RADIO_CURRENT_LIMIT_MA = 140.0;

MY_SX1262 radio868da = new Module(RADIO868DACS, RADIO868DADIO1, RADIO868DARESET, RADIO868DABUSY, SPI_ONE, SPISettings(1000000, MSBFIRST, SPI_MODE0));
MY_SX1262 radio868mg = new Module(RADIO868MGCS, RADIO868MGDIO1, RADIO868MGRESET, RADIO868MGBUSY, SPI_ONE, SPISettings(1000000, MSBFIRST, SPI_MODE0));
MY_SX1262 radio868lw = new Module(RADIO868LWCS, RADIO868LWDIO1, RADIO868LWRESET, RADIO868LWBUSY, SPI_ONE, SPISettings(1000000, MSBFIRST, SPI_MODE0));
MY_SX1268 radio433 = new Module(RADIO433CS, RADIO433DIO1, RADIO433RESET, RADIO433BUSY, SPI_TWO, SPISettings(1000000, MSBFIRST, SPI_MODE0));;

bool hasRadio868da = false;
bool hasRadio868mg = false;
bool hasRadio868lw = false;
bool hasRadio433 = false;

da_config CFG; 
lora_message lorapacket_in_433, lorapacket_in_868da, lorapacket_in_868mg, lorapacket_in_868lw;
lora_message lora_queue_out [NUMCHANNELS];

volatile bool enableInterrupt433    = true;
// These 868 states follow physical PCB radios, not runtime channels.
volatile bool enableInterrupt868da  = true;
volatile bool enableInterrupt868mg  = true;
volatile bool enableInterrupt868lw  = true;
volatile bool hasMsgToSend433       = false;
volatile bool hasMsgToSend868da     = false;
volatile bool hasMsgToSend868mg     = false;
volatile bool hasMsgToSend868lw     = false;
volatile bool msgOnTheWay433        = false;
volatile bool msgOnTheWay868da      = false;
volatile bool msgOnTheWay868mg      = false;
volatile bool msgOnTheWay868lw      = false;
volatile bool transmissionFlag433   = false;
volatile bool transmissionFlag868da = false;
volatile bool transmissionFlag868mg = false;
volatile bool transmissionFlag868lw = false;
int16_t transmissionState433   = 0;
int16_t transmissionState868da = 0;
int16_t transmissionState868mg = 0;
int16_t transmissionState868lw = 0;
int64_t startOfTransmission433   = 0;
int64_t startOfTransmission868da = 0;
int64_t startOfTransmission868mg = 0;
int64_t startOfTransmission868lw = 0;
bool cadMode433            = false;
bool cadMode868da          = false;
bool cadMode868mg          = false;
bool cadMode868lw          = false;

// Keep every 868 channel covered while rotating the DA antenna.
static Lora868Route route868(CHANNEL868DA, CHANNEL868MG, CHANNEL868LW);
static bool rotate868Pending = false;
static bool cad868Pending = false;
static bool send868Pending = false;

SemaphoreHandle_t highlander = NULL;

static int start_receive_868(Radio868 radio);

static MY_SX1262 *get_radio868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return &radio868da;
  }
  if (radio == Radio868::B)
  {
    return &radio868mg;
  }

  return &radio868lw;
}

static const char *get_radio868_name(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return "A";
  }
  if (radio == Radio868::B)
  {
    return "B";
  }

  return "C";
}

static int get_radio868_txen(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return RADIO868DATXEN;
  }
  if (radio == Radio868::B)
  {
    return RADIO868MGTXEN;
  }

  return RADIO868LWTXEN;
}

static int get_radio868_rxen(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return RADIO868DARXEN;
  }
  if (radio == Radio868::B)
  {
    return RADIO868MGRXEN;
  }

  return RADIO868LWRXEN;
}

static bool has_radio868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return hasRadio868da;
  }
  if (radio == Radio868::B)
  {
    return hasRadio868mg;
  }

  return hasRadio868lw;
}

static volatile bool &get_interrupt868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return enableInterrupt868da;
  }
  if (radio == Radio868::B)
  {
    return enableInterrupt868mg;
  }

  return enableInterrupt868lw;
}

static volatile bool &get_send868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return hasMsgToSend868da;
  }
  if (radio == Radio868::B)
  {
    return hasMsgToSend868mg;
  }

  return hasMsgToSend868lw;
}

static volatile bool &get_sending868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return msgOnTheWay868da;
  }
  if (radio == Radio868::B)
  {
    return msgOnTheWay868mg;
  }

  return msgOnTheWay868lw;
}

static volatile bool &get_flag868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return transmissionFlag868da;
  }
  if (radio == Radio868::B)
  {
    return transmissionFlag868mg;
  }

  return transmissionFlag868lw;
}

static int16_t &get_tx_state868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return transmissionState868da;
  }
  if (radio == Radio868::B)
  {
    return transmissionState868mg;
  }

  return transmissionState868lw;
}

static int64_t &get_tx_start868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return startOfTransmission868da;
  }
  if (radio == Radio868::B)
  {
    return startOfTransmission868mg;
  }

  return startOfTransmission868lw;
}

static bool &get_cad_mode868(Radio868 radio)
{
  if (radio == Radio868::A)
  {
    return cadMode868da;
  }
  if (radio == Radio868::B)
  {
    return cadMode868mg;
  }

  return cadMode868lw;
}

static lora_message *get_packet868(uint8_t channel)
{
  if (channel == CHANNEL868DA)
  {
    return &lorapacket_in_868da;
  }
  if (channel == CHANNEL868MG)
  {
    return &lorapacket_in_868mg;
  }

  return &lorapacket_in_868lw;
}

uint8_t radio868_random_byte(void)
{
  return radio868da.randomByte();
}

int start_receive_868da(void)
{
  return start_receive(CHANNEL868DA);
}

int start_receive_868mg(void)
{
  return start_receive(CHANNEL868MG);
}

int start_receive_868lw(void)
{
  return start_receive(CHANNEL868LW);
}

static int start_receive_868(Radio868 radio)
{
  digitalWrite(get_radio868_txen(radio), LOW);
  delay(1);
  digitalWrite(get_radio868_rxen(radio), HIGH);

  return get_radio868(radio)->startReceive();
}

int start_receive_433(void)
{
  digitalWrite(RADIO433TXEN, LOW);
  delay(1);
  digitalWrite(RADIO433RXEN, HIGH);
  return radio433.startReceive();
}

int start_receive(uint8_t channel)
{
  if (channel == CHANNEL433)
  {
    return start_receive_433();
  }

  Radio868 radio = route868.radio(channel);
  if (radio == Radio868::Count)
  {
    return RADIOLIB_ERR_INVALID_FREQUENCY;
  }

  return start_receive_868(radio);
}

static int start_send_868(Radio868 radio, uint8_t channel, const uint8_t* data, size_t len)
{
  digitalWrite(get_radio868_rxen(radio), LOW);
  delay(1);
  digitalWrite(get_radio868_txen(radio), HIGH);
  delay(1);
  if (CFG.send_enabled && (CFG.rdcp_address != RDCP_UNCONFIGURED_DA_ADDRESS))
  {
    int16_t rl_status = get_radio868(radio)->startTransmit(data, len);
    if (rl_status != RADIOLIB_ERR_NONE)
    {
      serial_writeln("WARNING: RadioLib startTransmit() failed, resetting radio");
      reset_radio(channel);
      rl_status = get_radio868(radio)->startTransmit(data, len);
    }
    return rl_status;
  }
  else 
  {
    serial_writeln("INFO: Send 868 disabled");
    return RADIOLIB_ERR_UNSUPPORTED;
  }
}

int16_t start_send_433(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO433RXEN, LOW);
  delay(1);
  digitalWrite(RADIO433TXEN, HIGH);
  delay(1);
  if (CFG.send_enabled && (CFG.rdcp_address != RDCP_UNCONFIGURED_DA_ADDRESS))
  {
    int16_t rl_status = radio433.startTransmit(data, len);
    if (rl_status != RADIOLIB_ERR_NONE)
    {
      serial_writeln("WARNING: RadioLib startTransmit() failed, resetting radio");
      reset_radio(CHANNEL433);
      rl_status = radio433.startTransmit(data, len);
    }
    return rl_status;
  }
  else 
  {
    serial_writeln("INFO: Send 433 disabled");
    return RADIOLIB_ERR_UNSUPPORTED;
  }
}

int16_t start_send(uint8_t channel, const uint8_t* data, size_t len)
{
  if (channel == CHANNEL433)
  {
    return start_send_433(data, len);
  }

  Radio868 radio = route868.radio(channel);
  if (radio == Radio868::Count)
  {
    return RADIOLIB_ERR_INVALID_FREQUENCY;
  }

  return start_send_868(radio, channel, data, len);
}

void setup_lora_hardware(void)
{
  /* Prepare the single SPI interface */
  vspi.begin(RADIO868DACLK, RADIO868DAMISO, RADIO868DAMOSI, RADIO868DACS);

  /* Disable any chip selects on cold boot */
  pinMode(RADIO868DACS, OUTPUT);
  pinMode(RADIO868MGCS, OUTPUT);
  pinMode(RADIO868LWCS, OUTPUT);
  pinMode(RADIO433CS, OUTPUT);
  digitalWrite(RADIO868DACS, HIGH);
  digitalWrite(RADIO868MGCS, HIGH);
  digitalWrite(RADIO868LWCS, HIGH);
  digitalWrite(RADIO433CS, HIGH);

  /* 868 MHz DA radio */
  pinMode(RADIO868DARXEN, OUTPUT);
  pinMode(RADIO868DATXEN, OUTPUT);
  digitalWrite(RADIO868DATXEN, LOW);
  digitalWrite(RADIO868DARXEN, HIGH);
  
  int state = radio868da.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    serial_writeln("INIT: SX1262 (868 MHz DA) hardware initialized successfully.");
    hasRadio868da = true;
    CFG.lora[CHANNEL868DA].freq = 869.525;
  } 
  else 
  {
    serial_writeln("INIT: Failed to initialize SX1262 (868 MHz DA). Error code: " + String(state));
  }

  /* 868 MHz MG radio */
  pinMode(RADIO868MGRXEN, OUTPUT);
  pinMode(RADIO868MGTXEN, OUTPUT);
  digitalWrite(RADIO868MGTXEN, LOW);
  digitalWrite(RADIO868MGRXEN, HIGH);
  
  state = radio868mg.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    serial_writeln("INIT: SX1262 (868 MHz MG) hardware initialized successfully.");
    hasRadio868mg = true;
    CFG.lora[CHANNEL868MG].freq = 869.900;
  } 
  else 
  {
    serial_writeln("INIT: Failed to initialize SX1262 (868 MHz MG). Error code: " + String(state));
  }

  /* 868 MHz LW radio */
  pinMode(RADIO868LWRXEN, OUTPUT);
  pinMode(RADIO868LWTXEN, OUTPUT);
  digitalWrite(RADIO868LWTXEN, LOW);
  digitalWrite(RADIO868LWRXEN, HIGH);
  
  state = radio868lw.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    serial_writeln("INIT: SX1262 (868 MHz LW) hardware initialized successfully.");
    hasRadio868lw = true;
    CFG.lora[CHANNEL868LW].freq = 868.1;
  } 
  else 
  {
    serial_writeln("INIT: Failed to initialize SX1262 (868 MHz LW). Error code: " + String(state));
  }
  
  /* 433 MHz radio */
  /* Not required for single-SPI PCB: hspi.begin(RADIO433CLK, RADIO433MISO, RADIO433MOSI, RADIO433CS); */
  pinMode(RADIO433TXEN, OUTPUT);
  pinMode(RADIO433RXEN, OUTPUT);
  digitalWrite(RADIO433TXEN, LOW);
  digitalWrite(RADIO433RXEN, HIGH);

  state = radio433.begin(); 
  if (state == RADIOLIB_ERR_NONE) 
  {
    serial_writeln("INIT: SX1268 (433 MHz) hardware initialized successfully.");
    hasRadio433 = true;
    CFG.lora[CHANNEL433].freq = 433.175;
  } 
  else 
  {
    serial_writeln("INIT: Failed to initialize SX1268 (433 MHz). Error code: " + String(state));
  }

  return;
}

void reset_radio(int channel)
{
  if ((channel == CHANNEL433) || (channel == ALL_CHANNELS))
  {
    radio433.reset();
    radio433.begin(); 
  }

  for (uint8_t i = 0; i < static_cast<uint8_t>(Radio868::Count); i++)
  {
    Radio868 radio = static_cast<Radio868>(i);
    if ((route868.channel(radio) != channel) && (channel != ALL_CHANNELS))
    {
      continue;
    }

    get_radio868(radio)->reset();
    get_radio868(radio)->begin();
  }

  setup_radio(channel);
  return;
}

void serial_write_incoming_message(int channel)
{
  char info[2*INFOLEN];
  int encodedLength;
  
  if (channel == CHANNEL433)
  {
    encodedLength = Base64ren.encodedLength(lorapacket_in_433.payload_length);
  }
  else if (channel == CHANNEL868DA)
  {
    encodedLength = Base64ren.encodedLength(lorapacket_in_868da.payload_length);
  }
  else if (channel == CHANNEL868MG)
  {
    encodedLength = Base64ren.encodedLength(lorapacket_in_868mg.payload_length);
  }
  else if (channel == CHANNEL868LW)
  {
    encodedLength = Base64ren.encodedLength(lorapacket_in_868lw.payload_length);
  }
  char encodedString[encodedLength + 1];

  if (channel == CHANNEL433)
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_433.payload, lorapacket_in_433.payload_length);
  }
  else if (channel == CHANNEL868DA)
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_868da.payload, lorapacket_in_868da.payload_length);
  }
  else if (channel == CHANNEL868MG)
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_868mg.payload, lorapacket_in_868mg.payload_length);
  }
  else if (channel == CHANNEL868LW)
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_868lw.payload, lorapacket_in_868lw.payload_length);
  }

  snprintf(info, 2*INFOLEN, "RX %s", encodedString);
  serial_writeln(info);
  return;
}

ICACHE_RAM_ATTR
void setFlag433(void)
{
  if (!enableInterrupt433) return;
  transmissionFlag433 = true;
  return;
}

ICACHE_RAM_ATTR
void setFlag868da(void)
{
  if (!enableInterrupt868da) return;
  transmissionFlag868da = true;
  return;
}

ICACHE_RAM_ATTR
void setFlag868mg(void)
{
  if (!enableInterrupt868mg) return;
  transmissionFlag868mg = true;
  return;
}

ICACHE_RAM_ATTR
void setFlag868lw(void)
{
  if (!enableInterrupt868lw) return;
  transmissionFlag868lw = true;
  return;
}

static bool config_radio868(Radio868 radio, uint8_t channel)
{
  MY_SX1262 *device = get_radio868(radio);
  lora_channel_config *config = &CFG.lora[channel];

  if (device->setFrequency(config->freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
  {
    serial_writeln("ERROR: Selected frequency is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setBandwidth(config->bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
  {
    serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setSpreadingFactor(config->sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
  {
    serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setCodingRate(config->cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
  {
    serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setSyncWord(config->sw) != RADIOLIB_ERR_NONE)
  {
    serial_writeln("ERROR: Unable to set LoRa sync word 868!");
    return false;
  }
  if (device->setOutputPower(config->pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
  {
    serial_writeln("ERROR: Selected output power is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setCurrentLimit(RADIO_CURRENT_LIMIT_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
  {
    serial_writeln("ERROR: Selected current limit is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setPreambleLength(config->pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
  {
    serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 868 module!");
    return false;
  }
  if (device->setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
  {
    serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 868 module!");
    return false;
  }

  if (radio == Radio868::A)
  {
    device->setDio1Action(setFlag868da);
  }
  else if (radio == Radio868::B)
  {
    device->setDio1Action(setFlag868mg);
  }
  else
  {
    device->setDio1Action(setFlag868lw);
  }

  return true;
}

static bool setup_radio868(Radio868 radio, uint8_t channel)
{
  if (!config_radio868(radio, channel))
  {
    return false;
  }

  if (xSemaphoreTake(highlander, portMAX_DELAY) != pdTRUE)
  {
    return false;
  }

  int state = start_receive_868(radio);
  xSemaphoreGive(highlander);

  char info[INFOLEN];
  if (state == RADIOLIB_ERR_NONE)
  {
    snprintf(info, INFOLEN, "INIT: LoRa 868 radio %s configured for channel %u.",
      get_radio868_name(radio), channel);
    serial_writeln(info);
    return true;
  }

  snprintf(info, INFOLEN, "ERROR: LoRa 868 radio %s setup failed, code %d",
    get_radio868_name(radio), state);
  serial_writeln(info);
  return false;
}

bool setup_radio(int channel)
{
  if (highlander == NULL) highlander = xSemaphoreCreateBinary();
  assert(highlander);
  xSemaphoreGive(highlander);

  if ((hasRadio433) && ((channel==CHANNEL433) || (channel==ALL_CHANNELS)))
  {
    if (radio433.setFrequency(CFG.lora[CHANNEL433].freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
      serial_writeln("ERROR: Selected frequency is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setBandwidth(CFG.lora[CHANNEL433].bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
      serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setSpreadingFactor(CFG.lora[CHANNEL433].sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
      serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setCodingRate(CFG.lora[CHANNEL433].cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
      serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setSyncWord(CFG.lora[CHANNEL433].sw) != RADIOLIB_ERR_NONE)
    {
      serial_writeln("ERROR: Unable to set LoRa sync word 433!");
      return false;
    }
    if (radio433.setOutputPower(CFG.lora[CHANNEL433].pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    {
      serial_writeln("ERROR: Selected output power is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setCurrentLimit(RADIO_CURRENT_LIMIT_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
    {
      serial_writeln("ERROR: Selected current limit is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setPreambleLength(CFG.lora[CHANNEL433].pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
      serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 433 module!");
      return false;
    }
    if (radio433.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
      serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 433 module!");
      return false;
    }

    radio433.setDio1Action(setFlag433);

    if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
    {
      if (hasRadio433)
      {
        int state = start_receive_433();
        if (state == RADIOLIB_ERR_NONE)
        {
          serial_writeln("INIT: LoRa 433 parameters applied successfully.");
        }
        else
        {
          serial_writeln("ERROR: LoRa 433 parameters setup failed, code " + String(state));
        }
      }
      xSemaphoreGive(highlander);
    }
  }

  for (uint8_t i = 0; i < static_cast<uint8_t>(Radio868::Count); i++)
  {
    Radio868 radio = static_cast<Radio868>(i);
    uint8_t assigned_channel = route868.channel(radio);

    if (!has_radio868(radio))
    {
      continue;
    }
    if ((channel != assigned_channel) && (channel != ALL_CHANNELS))
    {
      continue;
    }
    if (!setup_radio868(radio, assigned_channel))
    {
      return false;
    }
  }
  return true;
}

static void queue_send868(uint8_t channel)
{
  Radio868 radio = route868.radio(channel);
  if (radio == Radio868::Count)
  {
    return;
  }

  get_send868(radio) = true;
}

static void start_cad868(uint8_t channel)
{
  Radio868 radio = route868.radio(channel);
  if (radio == Radio868::Count)
  {
    return;
  }

  get_cad_mode868(radio) = true;
  digitalWrite(get_radio868_txen(radio), LOW);
  delay(1);
  digitalWrite(get_radio868_rxen(radio), HIGH);
  get_radio868(radio)->startChannelScan();
}

static void resume_868_requests(void)
{
  if (cad868Pending)
  {
    cad868Pending = false;
    start_cad868(CHANNEL868DA);
  }

  if (send868Pending)
  {
    send868Pending = false;
    queue_send868(CHANNEL868DA);
  }
}

static void rotate_radio868(void)
{
  if (!rotate868Pending)
  {
    return;
  }
  if (!hasRadio868da || !hasRadio868mg || !hasRadio868lw)
  {
    rotate868Pending = false;
    resume_868_requests();
    return;
  }

  Radio868 current = route868.da_radio();
  Radio868 next = route868.next_da_radio();
  if (get_flag868(current) || get_flag868(next) ||
      get_send868(current) || get_send868(next) ||
      get_sending868(current) || get_sending868(next) ||
      get_cad_mode868(current) || get_cad_mode868(next))
  {
    return;
  }

  get_interrupt868(current) = false;
  get_interrupt868(next) = false;

  if (get_flag868(current) || get_flag868(next))
  {
    get_interrupt868(current) = true;
    get_interrupt868(next) = true;
    return;
  }

  uint8_t displaced_channel = route868.channel(next);
  bool current_ok = config_radio868(current, displaced_channel);
  bool next_ok = current_ok && config_radio868(next, CHANNEL868DA);
  bool receive_ok = false;

  if (current_ok && next_ok)
  {
    int current_state = start_receive_868(current);
    int next_state = start_receive_868(next);
    receive_ok = (current_state == RADIOLIB_ERR_NONE) &&
                 (next_state == RADIOLIB_ERR_NONE);
  }

  if (current_ok && next_ok && receive_ok)
  {
    route868.rotate_da();

    char info[INFOLEN];
    snprintf(info, INFOLEN, "INFO: 868DA moved from radio %s to radio %s",
      get_radio868_name(current), get_radio868_name(next));
    serial_writeln(info);
  }
  else
  {
    // Restore the last complete mapping after a partial reconfiguration.
    config_radio868(current, CHANNEL868DA);
    config_radio868(next, displaced_channel);
    start_receive_868(current);
    start_receive_868(next);
    serial_writeln("ERROR: 868 radio rotation failed; previous mapping restored");
  }

  get_flag868(current) = false;
  get_flag868(next) = false;
  get_interrupt868(current) = true;
  get_interrupt868(next) = true;

  rotate868Pending = false;
  resume_868_requests();
}

static void loop_radio868(Radio868 radio)
{
  uint8_t channel = route868.channel(radio);
  volatile bool &has_message = get_send868(radio);
  volatile bool &is_sending = get_sending868(radio);
  volatile bool &event_flag = get_flag868(radio);
  volatile bool &interrupt_enabled = get_interrupt868(radio);
  int16_t &tx_state = get_tx_state868(radio);
  int64_t &tx_started = get_tx_start868(radio);
  MY_SX1262 *device = get_radio868(radio);
  char info[INFOLEN];

  if (has_message)
  {
    cpu_fast();
    if (is_sending && event_flag)
    {
      interrupt_enabled = false;
      event_flag = false;

      if (tx_state == RADIOLIB_ERR_NONE)
      {
        snprintf(info, INFOLEN, "INFO: LoRa channel %u transmission on radio %s finished",
          channel, get_radio868_name(radio));
        serial_writeln(info);
        snprintf(info, INFOLEN, "INFO: TX%u wallclock time was %" PRId64 " ms",
          channel, my_millis() - tx_started);
        serial_writeln(info);
      }
      else
      {
        snprintf(info, INFOLEN, "ERROR: LoRa channel %u transmission failed, code %d",
          channel, tx_state);
        serial_writeln(info);
      }

      has_message = false;
      is_sending = false;
      start_receive_868(radio);
      interrupt_enabled = true;
      rdcp_callback_txfin(channel);

      if (channel == CHANNEL868DA)
      {
        rotate868Pending = true;
      }
      return;
    }
    if (is_sending)
    {
      return;
    }

    snprintf(info, INFOLEN, "INFO: Transmitting LoRa channel %u on radio %s, length %u bytes",
      channel, get_radio868_name(radio), lora_queue_out[channel].payload_length);
    serial_writeln(info);

    tx_started = my_millis();
    tx_state = start_send_868(radio, channel, lora_queue_out[channel].payload,
      lora_queue_out[channel].payload_length);
    if (tx_state == RADIOLIB_ERR_NONE)
    {
      is_sending = true;
    }
    interrupt_enabled = true;
    return;
  }
  if (!event_flag)
  {
    return;
  }

  cpu_fast();
  interrupt_enabled = false;
  event_flag = false;

  if (get_cad_mode868(radio))
  {
    get_cad_mode868(radio) = false;
    int cad_state = device->getChannelScanResult();
    start_receive_868(radio);
    interrupt_enabled = true;
    rdcp_callback_cad(channel, cad_state == RADIOLIB_LORA_DETECTED);
    return;
  }

  byte payload[300];
  int payload_length = device->getPacketLength();
  if (payload_length > RDCP_MAX_LORA_PAYLOAD_SIZE)
  {
    payload_length = RDCP_MAX_LORA_PAYLOAD_SIZE;
  }
  int state = device->readData(payload, payload_length);

  if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
  {
    if (payload_length > 0)
    {
      lora_message *packet = get_packet868(channel);
      packet->available = true;
      packet->channel = channel;
      packet->rssi = device->getRSSI();
      packet->snr = device->getSNR();
      packet->timestamp = my_millis();
      packet->payload_length = payload_length;
      memcpy(packet->payload, payload, payload_length);

      snprintf(info, INFOLEN, "INFO: LoRa channel %u radio %s received packet",
        channel, get_radio868_name(radio));
      serial_writeln(info);
      snprintf(info, INFOLEN, "RXMETA %u %.2f %.2f %.3f",
        packet->payload_length, packet->rssi, packet->snr, CFG.lora[channel].freq);
      serial_writeln(info);
      serial_write_incoming_message(channel);
    }
    else
    {
      snprintf(info, INFOLEN, "INFO: LoRa channel %u radio %s received empty packet",
        channel, get_radio868_name(radio));
      serial_writeln(info);
    }
  }
  else
  {
    snprintf(info, INFOLEN, "ERROR: LoRa channel %u radio %s receive failed, code %d",
      channel, get_radio868_name(radio), state);
    serial_writeln(info);
  }

  start_receive_868(radio);
  interrupt_enabled = true;
}

void loop_radio(void)
{
  char info[INFOLEN];

  if (!hasRadio433 || !hasRadio868da || !hasRadio868mg)
  {
    serial_writeln("ERROR: Essential LoRa radios not online, refusing operation in loop_radio().");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }

  if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
  {
    if (hasMsgToSend433)
    {
      cpu_fast();
      if (msgOnTheWay433)
      {
        if (transmissionFlag433)
        {
          enableInterrupt433 = false;
          transmissionFlag433 = false;

          if (transmissionState433 == RADIOLIB_ERR_NONE)
          {
            serial_writeln("INFO: LoRa 433 transmission successfully finished!");
            snprintf(info, INFOLEN, "INFO: TX433 wallclock time was %" PRId64 " ms", my_millis() - startOfTransmission433);
            serial_writeln(info);
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 433 transmission failed, code %d", transmissionState433);
            serial_writeln(info);
          }

          hasMsgToSend433 = false;
          msgOnTheWay433 = false;
          start_receive_433();
          enableInterrupt433 = true;
          rdcp_callback_txfin(CHANNEL433);
        }
      }
      else
      {
        snprintf(info, INFOLEN, "INFO: Transmitting LoRa 433 message, length %d bytes", lora_queue_out[CHANNEL433].payload_length);
        serial_writeln(info);

        startOfTransmission433 = my_millis();
        transmissionState433 = start_send_433(lora_queue_out[CHANNEL433].payload, lora_queue_out[CHANNEL433].payload_length);
        if (transmissionState433 == RADIOLIB_ERR_NONE) msgOnTheWay433 = true;
        enableInterrupt433 = true;
      }
    }
    else
    {
      String recv;

      if (transmissionFlag433)
      {
        cpu_fast();
        enableInterrupt433 = false;
        transmissionFlag433 = false;

        if (cadMode433)
        { // CAD, not receiving
          cadMode433 = false;
          int cState = radio433.getChannelScanResult();
          radio433.startReceive(); // immediately go into receive mode, no TXEN/RXEN switch needed
          enableInterrupt433 = true;
          if (cState == RADIOLIB_LORA_DETECTED)
          {
            rdcp_callback_cad(CHANNEL433, true);
          }
          else 
          {
            rdcp_callback_cad(CHANNEL433, false);
          }
        }
        else 
        { // receiving, not CAD
          byte byteArr[300];
          int numBytes = radio433.getPacketLength();
          if (numBytes > 200) numBytes = 200;
          int state = radio433.readData(byteArr, numBytes);

          if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
          {
            if (numBytes > 0)
            {
              serial_writeln("INFO: LoRa 433 Radio received packet.");
              lorapacket_in_433.available = true; 
              lorapacket_in_433.channel = CHANNEL433;
              lorapacket_in_433.rssi = radio433.getRSSI();
              lorapacket_in_433.snr = radio433.getSNR();
              lorapacket_in_433.timestamp = my_millis();
              lorapacket_in_433.payload_length = numBytes;
              for (int i=0; i != numBytes; i++) lorapacket_in_433.payload[i] = byteArr[i];

              snprintf(info, INFOLEN, "RXMETA %d %.2f %.2f %.3f", 
                lorapacket_in_433.payload_length, lorapacket_in_433.rssi, 
                lorapacket_in_433.snr, CFG.lora[CHANNEL433].freq);
              serial_writeln(info);
              serial_write_incoming_message(CHANNEL433);
            }
            else
            {
              serial_writeln("INFO: LoRa 433 Radio received empty packet.");
            }
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 433 packet receiving failed, code %d", state);
            serial_writeln(info);
          }
          start_receive_433();
          enableInterrupt433 = true;
        }        
      }
    }

    loop_radio868(Radio868::A);
    loop_radio868(Radio868::B);
    loop_radio868(Radio868::C);
    rotate_radio868();

    xSemaphoreGive(highlander);
  }
}

void send_lora_message_binary(int channel, uint8_t *payload, uint8_t length)
{
  if (length == 0)
  {
    return;
  }
  if (length > RDCP_MAX_LORA_PAYLOAD_SIZE)
  {
    serial_writeln("ERROR: Cannot send LoRa message, payload length exceeds maximum size");
    return;
  }
  if ((channel < CHANNEL433) || (channel >= NUMCHANNELS))
  {
    serial_writeln("ERROR: Cannot send LoRa message on invalid channel");
    return;
  }

  memcpy(lora_queue_out[channel].payload, payload, length);
  lora_queue_out[channel].payload_length = length;
  if (channel == CHANNEL433)
  {
    hasMsgToSend433 = true;
    return;
  }
  if ((channel == CHANNEL868DA) && rotate868Pending)
  {
    send868Pending = true;
    return;
  }

  queue_send868(channel);
}

void radio_start_cad(uint8_t channel)
{
  if (channel == CHANNEL433)
  {
    cadMode433 = true;
    digitalWrite(RADIO433TXEN, LOW);
    delay(1);
    digitalWrite(RADIO433RXEN, HIGH);
    radio433.startChannelScan();
    return;
  }
  if ((channel == CHANNEL868DA) && rotate868Pending)
  {
    cad868Pending = true;
    return;
  }

  start_cad868(channel);
}

/* EOF */
