#ifndef _LORAWAN_TUNNEL
#define _LORAWAN_TUNNEL

#include <Arduino.h>

/**
 * Handle incoming LoRaWAN packets.
 */
void lorawan_tunnel_incoming(void);

/*
 * Add a LoRaWAN device for tunneling.
 */
void lorawan_tunnel_device_add(uint32_t devaddr);

/*
 * Remove a LoRaWAN device from tunneling.
 */
void lorawan_tunnel_device_remove(uint32_t devaddr);

/*
 * Remove all LoRaWAN devices from tunneling.
 */
void lorawan_tunnel_device_clear(void);

struct tunnel_entry
{
  uint32_t devaddr = 0;
  int64_t  last_tunneled = 0;
};

#define MAX_TUNNEL_ENTRIES 32
#define MIN_TUNNEL_INTERVAL 30 * 60 * 1000

void lorawan_tunnel_device_list(void);

#endif 

/* EOF */