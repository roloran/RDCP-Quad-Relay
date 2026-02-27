#include "lora.h"
#include <RadioLib.h>
#include <SPI.h>
#include "serial.h"
#include "Base64ren.h"
#include "hal.h"
#include "rdcp-send.h"
#include "rdcp-common.h"

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

bool enableInterrupt433    = true;
bool enableInterrupt868da  = true;
bool enableInterrupt868mg  = true;
bool enableInterrupt868lw  = true;
bool hasMsgToSend433       = false;
bool hasMsgToSend868da     = false;
bool hasMsgToSend868mg     = false;
bool hasMsgToSend868lw     = false;
bool msgOnTheWay433        = false;
bool msgOnTheWay868da      = false;
bool msgOnTheWay868mg      = false;
bool msgOnTheWay868lw      = false;
bool transmissionFlag433   = false;
bool transmissionFlag868da = false;
bool transmissionFlag868mg = false;
bool transmissionFlag868lw = false;
int transmissionState433   = 0;
int transmissionState868da = 0;
int transmissionState868mg = 0;
int transmissionState868lw = 0;
int64_t startOfTransmission433   = 0;
int64_t startOfTransmission868da = 0;
int64_t startOfTransmission868mg = 0;
int64_t startOfTransmission868lw = 0;
bool cadMode433            = false;
bool cadMode868da          = false;
bool cadMode868mg          = false;
bool cadMode868lw          = false;

SemaphoreHandle_t highlander = NULL;

uint8_t radio868_random_byte(void)
{
  return radio868da.randomByte();
}

int start_receive_868da(void)
{
  digitalWrite(RADIO868DATXEN, LOW);
  delay(1);
  digitalWrite(RADIO868DARXEN, HIGH);
  return radio868da.startReceive();
}

int start_receive_868mg(void)
{
  digitalWrite(RADIO868MGTXEN, LOW);
  delay(1);
  digitalWrite(RADIO868MGRXEN, HIGH);
  return radio868mg.startReceive();
}

int start_receive_868lw(void)
{
  digitalWrite(RADIO868LWTXEN, LOW);
  delay(1);
  digitalWrite(RADIO868LWRXEN, HIGH);
  return radio868lw.startReceive();
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
  if (channel == CHANNEL433) return start_receive_433();
  else if (channel == CHANNEL868DA) return start_receive_868da();
  else if (channel == CHANNEL868MG) return start_receive_868mg();
  else if (channel == CHANNEL868LW) return start_receive_868lw();
  else return -1;
}

bool start_send_868da(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO868DARXEN, LOW);
  delay(1);
  digitalWrite(RADIO868DATXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    return radio868da.startTransmit(data, len);
  }
  else 
  {
    serial_writeln("INFO: Send 868DA disabled");
    return false;
  }
}

bool start_send_868mg(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO868MGRXEN, LOW);
  delay(1);
  digitalWrite(RADIO868MGTXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    return radio868mg.startTransmit(data, len);
  }
  else 
  {
    serial_writeln("INFO: Send 868MG disabled");
    return false;
  }
}

bool start_send_868lw(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO868LWRXEN, LOW);
  delay(1);
  digitalWrite(RADIO868LWTXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    return radio868lw.startTransmit(data, len);
  }
  else 
  {
    serial_writeln("INFO: Send 868LW disabled");
    return false;
  }
}

bool start_send_433(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO433RXEN, LOW);
  delay(1);
  digitalWrite(RADIO433TXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    return radio433.startTransmit(data, len);
  }
  else 
  {
    serial_writeln("INFO: Send 433 disabled");
    return false;
  }
}

int start_send(uint8_t channel, const uint8_t* data, size_t len)
{
  if (channel == CHANNEL433) return start_send_433(data, len);
  else if (channel == CHANNEL868DA) return start_send_868da(data, len);
  else if (channel == CHANNEL868MG) return start_send_868mg(data, len);
  else if (channel == CHANNEL868LW) return start_send_868lw(data, len);
  else return -1;
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
    CFG.lora[CHANNEL868MG].freq = 869.0;
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

bool setup_radio(void)
{
  highlander = xSemaphoreCreateBinary();
  assert(highlander);
  xSemaphoreGive(highlander);

  if (hasRadio433)
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
    if (radio433.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
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

  if (hasRadio868da)
  {
    if (radio868da.setFrequency(CFG.lora[CHANNEL868DA].freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
      serial_writeln("ERROR: Selected frequency is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setBandwidth(CFG.lora[CHANNEL868DA].bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
      serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setSpreadingFactor(CFG.lora[CHANNEL868DA].sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
      serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setCodingRate(CFG.lora[CHANNEL868DA].cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
      serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setSyncWord(CFG.lora[CHANNEL868DA].sw) != RADIOLIB_ERR_NONE)
    {
      serial_writeln("ERROR: Unable to set LoRa sync word 868!");
      return false;
    }
    if (radio868da.setOutputPower(CFG.lora[CHANNEL868DA].pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    {
      serial_writeln("ERROR: Selected output power is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
    {
      serial_writeln("ERROR: Selected current limit is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setPreambleLength(CFG.lora[CHANNEL868DA].pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
      serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868da.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
      serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 868 module!");
      return false;
    }

    radio868da.setDio1Action(setFlag868da);

    if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
    {
      if (hasRadio868da)
      {
        int state = start_receive_868da();
        if (state == RADIOLIB_ERR_NONE)
        {
          serial_writeln("INIT: LoRa 868 DA parameters applied successfully.");
        }
        else
        {
          serial_writeln("ERROR: LoRa 868 DA parameters setup failed, code " + String(state));
        }
      }
      xSemaphoreGive(highlander);
    }
  }

  if (hasRadio868mg)
  {
    if (radio868mg.setFrequency(CFG.lora[CHANNEL868MG].freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
      serial_writeln("ERROR: Selected frequency is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setBandwidth(CFG.lora[CHANNEL868MG].bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
      serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setSpreadingFactor(CFG.lora[CHANNEL868MG].sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
      serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setCodingRate(CFG.lora[CHANNEL868MG].cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
      serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setSyncWord(CFG.lora[CHANNEL868MG].sw) != RADIOLIB_ERR_NONE)
    {
      serial_writeln("ERROR: Unable to set LoRa sync word 868!");
      return false;
    }
    if (radio868mg.setOutputPower(CFG.lora[CHANNEL868MG].pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    {
      serial_writeln("ERROR: Selected output power is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
    {
      serial_writeln("ERROR: Selected current limit is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setPreambleLength(CFG.lora[CHANNEL868MG].pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
      serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868mg.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
      serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 868 module!");
      return false;
    }

    radio868mg.setDio1Action(setFlag868mg);

    if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
    {
      if (hasRadio868mg)
      {
        int state = start_receive_868mg();
        if (state == RADIOLIB_ERR_NONE)
        {
          serial_writeln("INIT: LoRa 868 MG parameters applied successfully.");
        }
        else
        {
          serial_writeln("ERROR: LoRa 868 MG parameters setup failed, code " + String(state));
        }
      }
      xSemaphoreGive(highlander);
    }
  }

  if (hasRadio868lw)
  {
    if (radio868lw.setFrequency(CFG.lora[CHANNEL868LW].freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
      serial_writeln("ERROR: Selected frequency is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setBandwidth(CFG.lora[CHANNEL868LW].bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
      serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setSpreadingFactor(CFG.lora[CHANNEL868LW].sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
      serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setCodingRate(CFG.lora[CHANNEL868LW].cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
      serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setSyncWord(CFG.lora[CHANNEL868LW].sw) != RADIOLIB_ERR_NONE)
    {
      serial_writeln("ERROR: Unable to set LoRa sync word 868!");
      return false;
    }
    if (radio868lw.setOutputPower(CFG.lora[CHANNEL868LW].pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    {
      serial_writeln("ERROR: Selected output power is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
    {
      serial_writeln("ERROR: Selected current limit is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setPreambleLength(CFG.lora[CHANNEL868LW].pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
      serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868lw.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
      serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 868 module!");
      return false;
    }

    radio868lw.setDio1Action(setFlag868lw);

    if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
    {
      if (hasRadio868lw)
      {
        int state = start_receive_868lw();
        if (state == RADIOLIB_ERR_NONE)
        {
          serial_writeln("INIT: LoRa 868 LW parameters applied successfully.");
        }
        else
        {
          serial_writeln("ERROR: LoRa 868 LW parameters setup failed, code " + String(state));
        }
      }
      xSemaphoreGive(highlander);
    }
  }

  return true;
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
        msgOnTheWay433 = true;
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

    if (hasMsgToSend868da)
    {
      cpu_fast();
      if (msgOnTheWay868da)
      {
        if (transmissionFlag868da)
        {
          enableInterrupt868da = false;
          transmissionFlag868da = false;

          if (transmissionState868da == RADIOLIB_ERR_NONE)
          {
            serial_writeln("INFO: LoRa 868 DA transmission successfully finished!");
            snprintf(info, INFOLEN, "INFO: TX868DA wallclock time was %" PRId64 " ms", my_millis() - startOfTransmission868da);
            serial_writeln(info);
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 DA transmission failed, code %d", transmissionState868da);
            serial_writeln(info);
          }

          hasMsgToSend868da = false;
          msgOnTheWay868da = false;
          start_receive_868da();
          enableInterrupt868da = true;
          rdcp_callback_txfin(CHANNEL868DA);
        }
      }
      else
      {
        snprintf(info, INFOLEN, "INFO: Transmitting LoRa 868 DA message, length %d bytes", lora_queue_out[CHANNEL868DA].payload_length);
        serial_writeln(info);

        startOfTransmission868da = my_millis();
        transmissionState868da = start_send_868da(lora_queue_out[CHANNEL868DA].payload, lora_queue_out[CHANNEL868DA].payload_length);
        msgOnTheWay868da = true;
        enableInterrupt868da = true;
      }
    }
    else
    {
      String recv;

      if (transmissionFlag868da)
      {
        cpu_fast();
        enableInterrupt868da = false;
        transmissionFlag868da = false;

        if (cadMode868da)
        {
          cadMode868da = false;
          int cState = radio868da.getChannelScanResult();
          radio868da.startReceive(); // immediately go into receive mode, no TXEN/RXEN switch needed
          enableInterrupt868da = true;
          if (cState == RADIOLIB_LORA_DETECTED)
          {
            rdcp_callback_cad(CHANNEL868DA, true);
          }
          else 
          {
            rdcp_callback_cad(CHANNEL868DA, false);
          }
        }
        else 
        {
          byte byteArr[300];
          int numBytes = radio868da.getPacketLength();
          int state = radio868da.readData(byteArr, numBytes);

          if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
          {
            if (numBytes > 0)
            {
              serial_writeln("INFO: LoRa 868 DA Radio received packet.");
              lorapacket_in_868da.available = true; 
              lorapacket_in_868da.channel = CHANNEL868DA;
              lorapacket_in_868da.rssi = radio868da.getRSSI();
              lorapacket_in_868da.snr = radio868da.getSNR();
              lorapacket_in_868da.timestamp = my_millis();
              lorapacket_in_868da.payload_length = numBytes;
              for (int i=0; i != numBytes; i++) lorapacket_in_868da.payload[i] = byteArr[i];

              snprintf(info, INFOLEN, "RXMETA %d %.2f %.2f %.3f", 
                lorapacket_in_868da.payload_length, lorapacket_in_868da.rssi, 
                lorapacket_in_868da.snr, CFG.lora[CHANNEL868DA].freq);
              serial_writeln(info);
              serial_write_incoming_message(CHANNEL868DA);
            }
            else
            {
              serial_writeln("INFO: LoRa 868 DA Radio received empty packet.");
            }
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 DA packet receiving failed, code %d", state);
            serial_writeln(info);
          }
          start_receive_868da();
          enableInterrupt868da = true;
        }
      }
    }

    if (hasMsgToSend868mg)
    {
      cpu_fast();
      if (msgOnTheWay868mg)
      {
        if (transmissionFlag868mg)
        {
          enableInterrupt868mg = false;
          transmissionFlag868mg = false;

          if (transmissionState868mg == RADIOLIB_ERR_NONE)
          {
            serial_writeln("INFO: LoRa 868 MG transmission successfully finished!");
            snprintf(info, INFOLEN, "INFO: TX868MG wallclock time was %" PRId64 " ms", my_millis() - startOfTransmission868mg);
            serial_writeln(info);
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 MG transmission failed, code %d", transmissionState868mg);
            serial_writeln(info);
          }

          hasMsgToSend868mg = false;
          msgOnTheWay868mg = false;
          start_receive_868mg();
          enableInterrupt868mg = true;
          rdcp_callback_txfin(CHANNEL868MG);
        }
      }
      else
      {
        snprintf(info, INFOLEN, "INFO: Transmitting LoRa 868 MG message, length %d bytes", lora_queue_out[CHANNEL868MG].payload_length);
        serial_writeln(info);

        startOfTransmission868mg = my_millis();
        transmissionState868mg = start_send_868mg(lora_queue_out[CHANNEL868MG].payload, lora_queue_out[CHANNEL868MG].payload_length);
        msgOnTheWay868mg = true;
        enableInterrupt868mg = true;
      }
    }
    else
    {
      String recv;

      if (transmissionFlag868mg)
      {
        cpu_fast();
        enableInterrupt868mg = false;
        transmissionFlag868mg = false;

        if (cadMode868mg)
        {
          cadMode868mg = false;
          int cState = radio868mg.getChannelScanResult();
          radio868mg.startReceive(); // immediately go into receive mode, no TXEN/RXEN switch needed
          enableInterrupt868mg = true;
          if (cState == RADIOLIB_LORA_DETECTED)
          {
            rdcp_callback_cad(CHANNEL868MG, true);
          }
          else 
          {
            rdcp_callback_cad(CHANNEL868MG, false);
          }
        }
        else 
        {
          byte byteArr[300];
          int numBytes = radio868mg.getPacketLength();
          int state = radio868mg.readData(byteArr, numBytes);

          if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
          {
            if (numBytes > 0)
            {
              serial_writeln("INFO: LoRa 868 MG Radio received packet.");
              lorapacket_in_868mg.available = true; 
              lorapacket_in_868mg.channel = CHANNEL868MG;
              lorapacket_in_868mg.rssi = radio868mg.getRSSI();
              lorapacket_in_868mg.snr = radio868mg.getSNR();
              lorapacket_in_868mg.timestamp = my_millis();
              lorapacket_in_868mg.payload_length = numBytes;
              for (int i=0; i != numBytes; i++) lorapacket_in_868mg.payload[i] = byteArr[i];

              snprintf(info, INFOLEN, "RXMETA %d %.2f %.2f %.3f", 
                lorapacket_in_868mg.payload_length, lorapacket_in_868mg.rssi, 
                lorapacket_in_868mg.snr, CFG.lora[CHANNEL868MG].freq);
              serial_writeln(info);
              serial_write_incoming_message(CHANNEL868MG);
            }
            else
            {
              serial_writeln("INFO: LoRa 868 MG Radio received empty packet.");
            }
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 MG packet receiving failed, code %d", state);
            serial_writeln(info);
          }
          start_receive_868mg();
          enableInterrupt868mg = true;
        }
      }
    }

    if (hasMsgToSend868lw)
    {
      cpu_fast();
      if (msgOnTheWay868lw)
      {
        if (transmissionFlag868lw)
        {
          enableInterrupt868lw = false;
          transmissionFlag868lw = false;

          if (transmissionState868lw == RADIOLIB_ERR_NONE)
          {
            serial_writeln("INFO: LoRa 868 LW transmission successfully finished!");
            snprintf(info, INFOLEN, "INFO: TX868LW wallclock time was %" PRId64 " ms", my_millis() - startOfTransmission868lw);
            serial_writeln(info);
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 LW transmission failed, code %d", transmissionState868lw);
            serial_writeln(info);
          }

          hasMsgToSend868lw = false;
          msgOnTheWay868lw = false;
          start_receive_868lw();
          enableInterrupt868lw = true;
          rdcp_callback_txfin(CHANNEL868LW);
        }
      }
      else
      {
        snprintf(info, INFOLEN, "INFO: Transmitting LoRa 868 LW message, length %d bytes", lora_queue_out[CHANNEL868LW].payload_length);
        serial_writeln(info);

        startOfTransmission868lw = my_millis();
        transmissionState868lw = start_send_868lw(lora_queue_out[CHANNEL868LW].payload, lora_queue_out[CHANNEL868LW].payload_length);
        msgOnTheWay868lw = true;
        enableInterrupt868lw = true;
      }
    }
    else
    {
      String recv;

      if (transmissionFlag868lw)
      {
        cpu_fast();
        enableInterrupt868lw = false;
        transmissionFlag868lw = false;

        if (cadMode868lw)
        {
          cadMode868lw = false;
          int cState = radio868lw.getChannelScanResult();
          radio868lw.startReceive(); // immediately go into receive mode, no TXEN/RXEN switch needed
          enableInterrupt868lw = true;
          if (cState == RADIOLIB_LORA_DETECTED)
          {
            rdcp_callback_cad(CHANNEL868LW, true);
          }
          else 
          {
            rdcp_callback_cad(CHANNEL868LW, false);
          }
        }
        else 
        {
          byte byteArr[300];
          int numBytes = radio868lw.getPacketLength();
          int state = radio868lw.readData(byteArr, numBytes);

          if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
          {
            if (numBytes > 0)
            {
              serial_writeln("INFO: LoRa 868 LW Radio received packet.");
              lorapacket_in_868lw.available = true; 
              lorapacket_in_868lw.channel = CHANNEL868LW;
              lorapacket_in_868lw.rssi = radio868lw.getRSSI();
              lorapacket_in_868lw.snr = radio868lw.getSNR();
              lorapacket_in_868lw.timestamp = my_millis();
              lorapacket_in_868lw.payload_length = numBytes;
              for (int i=0; i != numBytes; i++) lorapacket_in_868lw.payload[i] = byteArr[i];

              snprintf(info, INFOLEN, "RXMETA %d %.2f %.2f %.3f", 
                lorapacket_in_868lw.payload_length, lorapacket_in_868lw.rssi, 
                lorapacket_in_868lw.snr, CFG.lora[CHANNEL868LW].freq);
              serial_writeln(info);
              serial_write_incoming_message(CHANNEL868LW);
            }
            else
            {
              serial_writeln("INFO: LoRa 868 LW Radio received empty packet.");
            }
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 LW packet receiving failed, code %d", state);
            serial_writeln(info);
          }
          start_receive_868lw();
          enableInterrupt868lw = true;
        }
      }
    }

    xSemaphoreGive(highlander);
  }
}

void send_lora_message_binary(int channel, uint8_t *payload, uint8_t length)
{
  if (length == 0) return;

  for (int i=0; i != length; i++) { lora_queue_out[channel].payload[i] = payload[i]; }
  lora_queue_out[channel].payload_length = length;
  if (channel == CHANNEL433)
  {
    hasMsgToSend433 = true;
  }
  else if (channel == CHANNEL868DA) 
  {
    hasMsgToSend868da = true;
  }
  else if (channel == CHANNEL868MG) 
  {
    hasMsgToSend868mg = true;
  }
  else if (channel == CHANNEL868LW) 
  {
    hasMsgToSend868lw = true;
  }
  return;
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
  }
  else if (channel == CHANNEL868DA)
  {
    cadMode868da = true;
    digitalWrite(RADIO868DATXEN, LOW);
    delay(1);
    digitalWrite(RADIO868DARXEN, HIGH);
    radio868da.startChannelScan();
  }
  else if (channel == CHANNEL868MG)
  {
    cadMode868mg = true;
    digitalWrite(RADIO868MGTXEN, LOW);
    delay(1);
    digitalWrite(RADIO868MGRXEN, HIGH);
    radio868mg.startChannelScan();
  }
  else if (channel == CHANNEL868LW)
  {
    cadMode868lw = true;
    digitalWrite(RADIO868LWTXEN, LOW);
    delay(1);
    digitalWrite(RADIO868LWRXEN, HIGH);
    radio868lw.startChannelScan();
  }
  return;
}

/* EOF */