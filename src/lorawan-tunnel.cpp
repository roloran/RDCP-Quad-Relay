#include <Arduino.h>
#include "lorawan-tunnel.h"
#include "lora.h"
#include "serial.h"
#include "rdcp-common.h"
#include "persistence.h"
#include "rdcp-scheduler.h"

extern lora_message current_lora_message;
extern da_config CFG;

tunnel_entry tunnel_entries[MAX_TUNNEL_ENTRIES];
char lwt_info[INFOLEN];

void rdcpv04_tunnel(uint8_t* data, uint8_t len, uint8_t tunneltype)
{
  if (len == 0)
  {
    serial_writeln("WARNING: Refusing to tunnel empty payload");
    return;
  }

  rdcp_message prepared_rdcpv04_message;

  prepared_rdcpv04_message.header.destination = RDCP_HQ_MULTICAST_ADDRESS;
  prepared_rdcpv04_message.header.message_type = RDCP_MSGTYPE_TUNNEL;
  prepared_rdcpv04_message.header.rdcp_payload_length = 2 + len; 

  prepared_rdcpv04_message.payload.data[0] = 0; // Backport to RDCP v0.5 forces a single transmission of TUNNEL messages
  prepared_rdcpv04_message.payload.data[1] = tunneltype;
  for (int i=COUNT_ZERO; i<len; i++) prepared_rdcpv04_message.payload.data[i+2] = data[i];

  prepared_rdcpv04_message.header.relay1 = (CFG.cirerelays[0] << 4) + 0;
  prepared_rdcpv04_message.header.relay2 = (CFG.cirerelays[1] << 4) + 1;
  prepared_rdcpv04_message.header.relay3 = (CFG.cirerelays[2] << 4) + 2;

  prepared_rdcpv04_message.header.sender = CFG.rdcp_address;
  prepared_rdcpv04_message.header.origin = CFG.rdcp_address;
  prepared_rdcpv04_message.header.sequence_number = get_next_rdcp_sequence_number(CFG.rdcp_address);
  prepared_rdcpv04_message.header.counter = rdcp_get_default_retransmission_counter_for_messagetype(prepared_rdcpv04_message.header.message_type);

  /* Update CRC header field */
  uint8_t data_for_crc[INFOLEN];
  memcpy(&data_for_crc, &prepared_rdcpv04_message.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
  for (int i=0; i < prepared_rdcpv04_message.header.rdcp_payload_length; i++)
       data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = prepared_rdcpv04_message.payload.data[i];
  uint16_t actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + prepared_rdcpv04_message.header.rdcp_payload_length);
  prepared_rdcpv04_message.header.checksum = actual_crc;

  uint8_t data_for_scheduler[INFOLEN];
  memcpy(&data_for_scheduler, &prepared_rdcpv04_message.header, RDCP_HEADER_SIZE);
  for (int i=0; i < prepared_rdcpv04_message.header.rdcp_payload_length; i++)
       data_for_scheduler[i + RDCP_HEADER_SIZE] = prepared_rdcpv04_message.payload.data[i];

  int64_t my_delay = -100 * (1 + CFG.relay_identifier);

  serial_writeln("INFO: Scheduling TUNNEL Message on 433 MHz");
  rdcp_txqueue_add(CHANNEL433, data_for_scheduler, RDCP_HEADER_SIZE + prepared_rdcpv04_message.header.rdcp_payload_length,
                   NOTIMPORTANT, NOFORCEDTX, TX_CALLBACK_NONE, my_delay);

  serial_writeln("INFO: Scheduling TUNNEL Message on 868DA MHz");

  prepared_rdcpv04_message.header.relay1 = RDCP_HEADER_RELAY_MAGIC_NONE;
  prepared_rdcpv04_message.header.relay2 = RDCP_HEADER_RELAY_MAGIC_NONE;
  prepared_rdcpv04_message.header.relay3 = RDCP_HEADER_RELAY_MAGIC_NONE;

  /* Update CRC header field */
  memcpy(&data_for_crc, &prepared_rdcpv04_message.header, RDCP_HEADER_SIZE - RDCP_CRC_SIZE);
  for (int i=0; i < prepared_rdcpv04_message.header.rdcp_payload_length; i++)
       data_for_crc[i + RDCP_HEADER_SIZE - RDCP_CRC_SIZE] = prepared_rdcpv04_message.payload.data[i];
  actual_crc = crc16(data_for_crc, RDCP_HEADER_SIZE - RDCP_CRC_SIZE + prepared_rdcpv04_message.header.rdcp_payload_length);
  prepared_rdcpv04_message.header.checksum = actual_crc;

  memcpy(&data_for_scheduler, &prepared_rdcpv04_message.header, RDCP_HEADER_SIZE);
  for (int i=0; i < prepared_rdcpv04_message.header.rdcp_payload_length; i++)
       data_for_scheduler[i + RDCP_HEADER_SIZE] = prepared_rdcpv04_message.payload.data[i];

  my_delay = 0 - (3 * SECONDS_TO_MILLISECONDS * CFG.sf_multiplier);
  rdcp_txqueue_add(CHANNEL868DA, data_for_scheduler, RDCP_HEADER_SIZE + prepared_rdcpv04_message.header.rdcp_payload_length,
                   NOTIMPORTANT, NOFORCEDTX, TX_CALLBACK_NONE, my_delay);

  return;
}

void lorawan_tunnel_device_add(uint32_t devaddr)
{
  bool success = false;
  
  for (int i=0; i<MAX_TUNNEL_ENTRIES; i++)
  {
    if ((tunnel_entries[i].devaddr == 0) || (tunnel_entries[i].devaddr == devaddr))
    {
      tunnel_entries[i].devaddr = devaddr;
      tunnel_entries[i].last_tunneled = 0;
      success = true;
      break;
    }
  }

  if (success)
  {
    snprintf(lwt_info, INFOLEN, "INFO: Device %08X added to list of tunneled LoRaWAN devices", devaddr);
  }
  else 
  {
    snprintf(lwt_info, INFOLEN, "WARNING: Cannot add more LoRaWAN devices to tunnel list");
  }
  serial_writeln(lwt_info);

  return;
}

void lorawan_tunnel_device_remove(uint32_t devaddr)
{
  bool success = false;
  
  for (int i=0; i<MAX_TUNNEL_ENTRIES; i++)
  {
    if (tunnel_entries[i].devaddr == devaddr)
    {
      tunnel_entries[i].devaddr = 0;
      tunnel_entries[i].last_tunneled = 0;
      success = true;
    }
  }

  if (success)
  {
    snprintf(lwt_info, INFOLEN, "INFO: Device %08X removed from list of tunneled LoRaWAN devices", devaddr);
  }
  else 
  {
    snprintf(lwt_info, INFOLEN, "WARNING: Cannot remove this LoRaWAN device from tunnel list (not found)");
  }
  serial_writeln(lwt_info);

  return;
}

void lorawan_tunnel_device_clear(void)
{
  for (int i=0; i<MAX_TUNNEL_ENTRIES; i++)
  {
    tunnel_entries[i].devaddr = 0;
    tunnel_entries[i].last_tunneled = 0;
  }

  serial_writeln("INFO: List of LoRaWAN devices for tunneling has been cleared");

  return;
}

/*
 * Read a `uint32` in little endian from the first four bytes of `buf` regardless of target architecture
 */
uint32_t read_uint32_le(const uint8_t *buf)
{
  return ((uint32_t)buf[0])       |
         ((uint32_t)buf[1] << 8)  |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

/*
lorawan_tunnel_incoming() is called from rdcp-incoming/rdcp_handle_incoming_lora_message()
whenever we received a LoRa packet on CHANNEL868LW.
We have to analyze `current_lora_message` to determine whether it should be tunneled to
our HQ using a RDCP_MSGTYPE_TUNNEL RDCP Message.
*/
void lorawan_tunnel_incoming(void)
{
  uint32_t devaddr = read_uint32_le(&current_lora_message.payload[1]);

  int64_t last_tunneled = -1;
  int i = 0;

  for (i=0; i<MAX_TUNNEL_ENTRIES; i++)
  {
    if (tunnel_entries[i].devaddr == devaddr)
    {
      last_tunneled = tunnel_entries[i].last_tunneled;
      break;
    }
  }

  snprintf(lwt_info, INFOLEN, "INFO: LoRaWAN packet by device %08X, size: %d bytes, last seen: %" PRId64, 
           devaddr, current_lora_message.payload_length, last_tunneled);
  serial_writeln(lwt_info);

  if (last_tunneled == -1) return; // not in list
  int64_t now = my_millis();
  if (now - last_tunneled < MIN_TUNNEL_INTERVAL) return; // too often
  if (current_lora_message.payload_length > 182)
  {
    serial_writeln("WARNING: LoRa packet too large to be tunneled, skipping");
    return;
  }

  tunnel_entries[i].last_tunneled = now;
  rdcpv04_tunnel(current_lora_message.payload, current_lora_message.payload_length, 0x00); // tunnel payload type LoRaWAN

  return;
}