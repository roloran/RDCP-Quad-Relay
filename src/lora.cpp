#include "lora.h"
#include <RadioLib.h>
#include <SPI.h>
#include "serial.h"
#include "Base64ren.h"
#include "hal.h"
#include "rdcp-send.h"
#include "rdcp-common.h"

SPIClass vspi = SPIClass(VSPI);
SPIClass hspi = SPIClass(HSPI);

SX1262 radio868 = new Module(RADIO868CS, RADIO868DIO1, RADIO868RESET, RADIO868BUSY, vspi, SPISettings(1000000, MSBFIRST, SPI_MODE0));
SX1268 radio433 = new Module(RADIO433CS, RADIO433DIO1, RADIO433RESET, RADIO433BUSY, hspi, SPISettings(1000000, MSBFIRST, SPI_MODE0));;

bool hasRadio868 = false;
bool hasRadio433 = false;

da_config CFG; 
lora_message lorapacket_in_433, lorapacket_in_868;
lora_message lora_queue_out [NUMCHANNELS];

bool enableInterrupt433  = true;
bool enableInterrupt868  = true;
bool hasMsgToSend433     = false;
bool hasMsgToSend868     = false;
bool msgOnTheWay433      = false;
bool msgOnTheWay868      = false;
bool transmissionFlag433 = false;
bool transmissionFlag868 = false;
int transmissionState433 = 0;
int transmissionState868 = 0;
int64_t startOfTransmission433 = 0;
int64_t startOfTransmission868 = 0;
bool cadMode433          = false;
bool cadMode868          = false;

SemaphoreHandle_t highlander = NULL;

uint8_t radio868_random_byte(void)
{
  return radio868.randomByte();
}

int start_receive_868(void)
{
  digitalWrite(RADIO868TXEN, LOW);
  delay(1);
  digitalWrite(RADIO868RXEN, HIGH);
  return radio868.startReceive();
}

int send_now_868(String s)
{
  digitalWrite(RADIO868RXEN, LOW);
  delay(1);
  digitalWrite(RADIO868TXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    radio868.transmit(s);
  }
  else 
  {
    serial_writeln("INFO: Send 868 disabled");
  }
  transmissionFlag868 = false;
  return start_receive_868();
}

bool start_send_868(const uint8_t* data, size_t len)
{
  digitalWrite(RADIO868RXEN, LOW);
  delay(1);
  digitalWrite(RADIO868TXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    return radio868.startTransmit(data, len);
  }
  else 
  {
    serial_writeln("INFO: Send 868 disabled");
    return false;
  }
}

int start_receive_433(void)
{
  digitalWrite(RADIO433TXEN, LOW);
  delay(1);
  digitalWrite(RADIO433RXEN, HIGH);
  return radio433.startReceive();
}

int send_now_433(String s)
{
  digitalWrite(RADIO433RXEN, LOW);
  delay(1);
  digitalWrite(RADIO433TXEN, HIGH);
  delay(1);
  if (CFG.send_enabled)
  {
    radio433.transmit(s);
  }
  else 
  {
    serial_writeln("INFO: Send 433 disabled");
  }
  transmissionFlag433 = false;
  return start_receive_433();
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

void setup_lora_hardware(void)
{
    /* 868 MHz radio */
    vspi.begin(RADIO868CLK, RADIO868MISO, RADIO868MOSI, RADIO868CS);
    pinMode(RADIO868RXEN, OUTPUT);
    pinMode(RADIO868TXEN, OUTPUT);
    digitalWrite(RADIO868TXEN, LOW);
    digitalWrite(RADIO868RXEN, HIGH);
  
    int state = radio868.begin();
    if (state == RADIOLIB_ERR_NONE)
    {
      serial_writeln("INIT: SX1262 (868 MHz) hardware initialized successfully.");
      hasRadio868 = true;
      CFG.lora[CHANNEL868].freq = 868.2;
    } 
    else 
    {
      serial_writeln("INIT: Failed to initialize SX1262 (868 MHz). Error code: " + String(state));
    }
  
    /* 433 MHz radio */
    hspi.begin(RADIO433CLK, RADIO433MISO, RADIO433MOSI, RADIO433CS);
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
  else 
  {
    encodedLength = Base64ren.encodedLength(lorapacket_in_868.payload_length);
  }
  char encodedString[encodedLength + 1];

  if (channel == CHANNEL433)
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_433.payload, lorapacket_in_433.payload_length);
  }
  else
  {
    Base64ren.encode(encodedString, (char *) lorapacket_in_868.payload, lorapacket_in_868.payload_length);
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
void setFlag868(void)
{
  if (!enableInterrupt868) return;
  transmissionFlag868 = true;
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

  if (hasRadio868)
  {
    if (radio868.setFrequency(CFG.lora[CHANNEL868].freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
      serial_writeln("ERROR: Selected frequency is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setBandwidth(CFG.lora[CHANNEL868].bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
      serial_writeln("ERROR: Selected bandwidth is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setSpreadingFactor(CFG.lora[CHANNEL868].sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
      serial_writeln("ERROR: Selected spreading factor is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setCodingRate(CFG.lora[CHANNEL868].cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
      serial_writeln("ERROR: Selected coding rate is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setSyncWord(CFG.lora[CHANNEL868].sw) != RADIOLIB_ERR_NONE)
    {
      serial_writeln("ERROR: Unable to set LoRa sync word 868!");
      return false;
    }
    if (radio868.setOutputPower(CFG.lora[CHANNEL868].pw) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    {
      serial_writeln("ERROR: Selected output power is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
    {
      serial_writeln("ERROR: Selected current limit is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setPreambleLength(CFG.lora[CHANNEL868].pl) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
      serial_writeln("ERROR: Selected preamble length is invalid for this LoRa 868 module!");
      return false;
    }
    if (radio868.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
      serial_writeln("ERROR: Selected CRC mode is invalid for this LoRa 868 module!");
      return false;
    }

    radio868.setDio1Action(setFlag868);

    if (xSemaphoreTake(highlander, portMAX_DELAY) == pdTRUE)
    {
      if (hasRadio868)
      {
        int state = start_receive_868();
        if (state == RADIOLIB_ERR_NONE)
        {
          serial_writeln("INIT: LoRa 868 parameters applied successfully.");
        }
        else
        {
          serial_writeln("ERROR: LoRa 868 parameters setup failed, code " + String(state));
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

  if (!hasRadio433 || !hasRadio868)
  {
    serial_writeln("ERROR: LoRa radios not online, refusing operation in loop_radio().");
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

    if (hasMsgToSend868)
    {
      cpu_fast();
      if (msgOnTheWay868)
      {
        if (transmissionFlag868)
        {
          enableInterrupt868 = false;
          transmissionFlag868 = false;

          if (transmissionState868 == RADIOLIB_ERR_NONE)
          {
            serial_writeln("INFO: LoRa 868 transmission successfully finished!");
            snprintf(info, INFOLEN, "INFO: TX868 wallclock time was %" PRId64 " ms", my_millis() - startOfTransmission868);
            serial_writeln(info);
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 transmission failed, code %d", transmissionState868);
            serial_writeln(info);
          }

          hasMsgToSend868 = false;
          msgOnTheWay868 = false;
          start_receive_868();
          enableInterrupt868 = true;
          rdcp_callback_txfin(CHANNEL868);
        }
      }
      else
      {
        snprintf(info, INFOLEN, "INFO: Transmitting LoRa 868 message, length %d bytes", lora_queue_out[CHANNEL868].payload_length);
        serial_writeln(info);

        startOfTransmission868 = my_millis();
        transmissionState868 = start_send_868(lora_queue_out[CHANNEL868].payload, lora_queue_out[CHANNEL868].payload_length);
        msgOnTheWay868 = true;
        enableInterrupt868 = true;
      }
    }
    else
    {
      String recv;

      if (transmissionFlag868)
      {
        cpu_fast();
        enableInterrupt868 = false;
        transmissionFlag868 = false;

        if (cadMode868)
        {
          cadMode868 = false;
          int cState = radio868.getChannelScanResult();
          radio868.startReceive(); // immediately go into receive mode, no TXEN/RXEN switch needed
          enableInterrupt868 = true;
          if (cState == RADIOLIB_LORA_DETECTED)
          {
            rdcp_callback_cad(CHANNEL868, true);
          }
          else 
          {
            rdcp_callback_cad(CHANNEL868, false);
          }
        }
        else 
        {
          byte byteArr[300];
          int numBytes = radio868.getPacketLength();
          int state = radio868.readData(byteArr, numBytes);

          if ((state == RADIOLIB_ERR_NONE) || (state == RADIOLIB_ERR_CRC_MISMATCH))
          {
            if (numBytes > 0)
            {
              serial_writeln("INFO: LoRa 868 Radio received packet.");
              lorapacket_in_868.available = true; 
              lorapacket_in_868.channel = CHANNEL868;
              lorapacket_in_868.rssi = radio868.getRSSI();
              lorapacket_in_868.snr = radio868.getSNR();
              lorapacket_in_868.timestamp = my_millis();
              lorapacket_in_868.payload_length = numBytes;
              for (int i=0; i != numBytes; i++) lorapacket_in_868.payload[i] = byteArr[i];

              snprintf(info, INFOLEN, "RXMETA %d %.2f %.2f %.3f", 
                lorapacket_in_868.payload_length, lorapacket_in_868.rssi, 
                lorapacket_in_868.snr, CFG.lora[CHANNEL868].freq);
              serial_writeln(info);
              serial_write_incoming_message(CHANNEL868);
            }
            else
            {
              serial_writeln("INFO: LoRa 868 Radio received empty packet.");
            }
          }
          else
          {
            snprintf(info, INFOLEN, "ERROR: LoRa 868 packet receiving failed, code %d", state);
            serial_writeln(info);
          }
          start_receive_868();
          enableInterrupt868 = true;
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
  else 
  {
    hasMsgToSend868 = true;
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
  else 
  {
    cadMode868 = true;
    digitalWrite(RADIO868TXEN, LOW);
    delay(1);
    digitalWrite(RADIO868RXEN, HIGH);
    radio868.startChannelScan();
  }
  return;
}

/* EOF */