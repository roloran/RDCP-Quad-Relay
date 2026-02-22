#include "rdcp-neighbors.h"
#include "serial.h"
#include "hal.h"

neighbor_table_entry neighbors[MAX_NEIGHBORS];

void rdcp_neighbor_register_rx(uint8_t channel, uint16_t sender, double rssi, double snr, int64_t timestamp, bool heartbeat, bool explicit_refnr, uint16_t latest_refnr, uint16_t roamingrec)
{
    uint16_t used_sender = sender;

    /* Map neighboring Relays to the old BBK range on their 868 MHz channel */
    if ((channel == CHANNEL868) &&
        (sender >= RDCP_ADDRESS_DA_LOWERBOUND) && (sender <= RDCP_ADDRESS_MG_LOWERBOUND))
    {
        used_sender -= 0x0100;
    }

    int index = RDCP_INDEX_NONE;
    for (int i=0; i<MAX_NEIGHBORS; i++)
    {
        if ((neighbors[i].sender == RDCP_ADDRESS_SPECIAL_ZERO) || (neighbors[i].sender == used_sender))
        {
            index = i;
            break;
        } 
    }

    if (index == RDCP_INDEX_NONE)
    {
        serial_writeln("WARNING: Neighbor table overflow - increase size!");
        return;
    }

    neighbors[index].channel   = channel;
    neighbors[index].sender    = used_sender;
    neighbors[index].rssi      = rssi;
    neighbors[index].snr       = snr;
    neighbors[index].timestamp = timestamp;
    neighbors[index].heartbeat = heartbeat;
    neighbors[index].counted   = false;
    neighbors[index].explicit_refnr   = explicit_refnr;
    neighbors[index].latest_refnr     = latest_refnr;
    neighbors[index].roamingrec       = roamingrec;

    return;
}

void rdcp_neighbor_dump(void)
{
    char info[INFOLEN];
    int64_t now = my_millis();
    serial_writeln("INFO: Begin of neighbor table dump");
    for (int i=0; i<MAX_NEIGHBORS; i++)
    {
        if (neighbors[i].sender != RDCP_ADDRESS_SPECIAL_ZERO)
        {
            int mytime = (int) ((now - neighbors[i].timestamp) / (MINUTES_TO_MILLISECONDS));
            snprintf(info, INFOLEN, "INFO: Neighbor %d,%04X,%d,%.0f,%.0f,%c,%c,%04X,%04X,%dm",
                     i, neighbors[i].sender, neighbors[i].channel == CHANNEL433 ? 433:868,
                     neighbors[i].rssi, neighbors[i].snr, 
                     neighbors[i].heartbeat ? 'H':'-', 
                     neighbors[i].explicit_refnr ? 'X':'-',
                     neighbors[i].latest_refnr,
                     neighbors[i].roamingrec,                     
                     mytime);
            serial_writeln(info);
        }
    }
    serial_writeln("INFO: End of neighbor table dump");
    return;
}

/* EOF */