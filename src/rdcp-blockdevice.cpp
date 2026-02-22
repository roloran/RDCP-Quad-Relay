#include "rdcp-incoming.h"
#include "lora.h"
#include "serial.h"
#include "persistence.h"
#include "rdcp-common.h"
#include "rdcp-relay.h"
#include "rdcp-entrypoint.h"
#include "rdcp-blockdevice.h"
#include "hal.h"

device_block_entry BLOCKED[MAX_DEVICE_BLOCKS];

bool rdcp_relay_allowed_for_device(uint16_t origin)
{
    for (int i=0; i<MAX_DEVICE_BLOCKS; i++)
    {
        if (BLOCKED[i].address == origin)
        {
            int64_t now = my_millis();
            if (now < (BLOCKED[i].timestamp + MINUTES_TO_MILLISECONDS * BLOCKED[i].duration)) 
                return false;
        }
    }

    return true;
}

void rdcp_device_block_add(uint16_t target, uint16_t duration)
{
    int idx_found = RDCP_INDEX_NONE;
    int idx_free = RDCP_INDEX_NONE;

    for (int i=0; i<MAX_DEVICE_BLOCKS; i++)
    {
        if (BLOCKED[i].address == target) idx_found = i;
        if ((BLOCKED[i].address == RDCP_ADDRESS_SPECIAL_ZERO) && (idx_free == RDCP_INDEX_NONE)) idx_free = i;
    }

    if ((idx_found == RDCP_INDEX_NONE) && (idx_free == RDCP_INDEX_NONE))
    {
        serial_writeln("WARNING: Device Block entry table overflow, cannot block more devices");
        return;
    }
    else 
    {
        if (idx_found == RDCP_INDEX_NONE) idx_found = idx_free;
        BLOCKED[idx_found].address     = target;
        BLOCKED[idx_found].duration    = duration;
        BLOCKED[idx_found].timestamp   = my_millis();
    }

    return;
}

void rdcp_device_block_remove(uint16_t target)
{
    for (int i=0; i<MAX_DEVICE_BLOCKS; i++)
    {
        if (BLOCKED[i].address == target) BLOCKED[i].address = RDCP_ADDRESS_SPECIAL_ZERO;
    } 
    return;
}

void rdcp_device_block_clear(void)
{
    for (int i=0; i<MAX_DEVICE_BLOCKS; i++) BLOCKED[i].address = RDCP_ADDRESS_SPECIAL_ZERO;
    return;
}

/* EOF */