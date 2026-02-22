#include "rdcp-memory.h"
#include "serial.h"
#include "hal.h"
#ifdef ROLORAN_USE_FFAT
#include "FFat.h"
#else
#include <LittleFS.h>
#endif

rdcp_memory_table mem;
extern lora_message current_lora_message;
extern rdcp_message rdcp_msg_in;
runtime_da_data DART;

#define FILENAME_MEMORIES "/memory.da"

void rdcp_memory_remember(void)
{
    int index = RDCP_INDEX_NONE;

    /* Check for duplicates. Only store unique memories. */
    bool is_dupe = false;
    uint16_t this_refnr = RDCP_OA_REFNR_SPECIAL_ZERO;
    if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT)
    {
        this_refnr = rdcp_msg_in.payload.data[1] + 256 * rdcp_msg_in.payload.data[2];
    }
    else if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE)
    {
        this_refnr = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];
    }
    uint16_t this_size = RDCP_HEADER_SIZE + rdcp_msg_in.header.rdcp_payload_length;
    for (int i=0; i < MAX_STORED_MSGS; i++)
    {
        if (
            (mem.entries[i].reference_number == this_refnr) &&
            (mem.entries[i].payload_length == this_size)
        ) is_dupe = true;
    }
    if (is_dupe)
    {
        serial_writeln("INFO: Ignoring duplicated memory");
        return; // Don't store duplicates.
    }

    /* Proceed to store the memory. */
    if (mem.idx_first == RDCP_INDEX_NONE)
    { // no messages stored yet
        index = 0;
        mem.idx_first = index;
    }
    else 
    { // look for free slots
        int free_slot = RDCP_INDEX_NONE;
        for (int i=0; i<MAX_STORED_MSGS; i++)
        {
            if (mem.entries[i].slot_used == false)
            {
                free_slot = i;
                break;
            }
        }

        if (free_slot != RDCP_INDEX_NONE)
        { // free slot found
            index = free_slot;
        }
        else 
        { // no free slot, need to re-use oldest 
            index = mem.idx_first; // re-use this slot
            mem.idx_first = (mem.idx_first + 1) % MAX_STORED_MSGS; // cyclic next one is oldest now
        }
    }

    mem.entries[index].slot_used = true;
    mem.entries[index].payload_length = current_lora_message.payload_length;

    for (int i=0; i<current_lora_message.payload_length; i++)
        mem.entries[index].payload[i] = current_lora_message.payload[i];

    uint16_t refnr = RDCP_OA_REFNR_SPECIAL_ZERO;
    if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_OFFICIAL_ANNOUNCEMENT)
    {
        refnr = rdcp_msg_in.payload.data[1] + 256 * rdcp_msg_in.payload.data[2];
    }
    else if (rdcp_msg_in.header.message_type == RDCP_MSGTYPE_SIGNATURE)
    {
        refnr = rdcp_msg_in.payload.data[0] + 256 * rdcp_msg_in.payload.data[1];
    }
    mem.entries[index].reference_number = refnr;
    mem.entries[index].timestamp_added = my_millis();

    return;
}

void rdcp_memory_forget(void)
{
    for (int i=0; i<MAX_STORED_MSGS; i++)
    {
        mem.entries[i].slot_used = false;
        mem.entries[i].timestamp_added = RDCP_TIMESTAMP_ZERO;
        mem.entries[i].reference_number = RDCP_OA_REFNR_SPECIAL_ZERO;
        mem.entries[i].payload_length = 0;
        mem.entries[i].used_in_fetch_all = false;
        mem.entries[i].used_in_fetch_single = false;
        mem.entries[i].used_in_periodic868 = false;
    }
    mem.idx_first = RDCP_INDEX_NONE;
    return;
}

void rdcp_memory_dump(void)
{
    char info[INFOLEN];
    snprintf(info, INFOLEN, "INFO: Begin of memorized RDCP Messages dump, idx_first = %d", mem.idx_first);
    serial_writeln(info);
    for (int i=0; i<MAX_STORED_MSGS; i++)
    {
        if (mem.entries[i].slot_used)
        {
            snprintf(info, INFOLEN, "INFO: Memory %02d,%03d,%04X,", i, mem.entries[i].payload_length, mem.entries[i].reference_number);
            serial_write(info);
            serial_write_base64((char *)mem.entries[i].payload, mem.entries[i].payload_length, true);
        }
    }
    serial_writeln("INFO: End of memorized RDCP Messages dump");
    return;
}

void rdcp_memory_persist(void)
{
    serial_writeln("INFO: Persisting memories");
#ifdef ROLORAN_USE_FFAT
    FFat.remove(FILENAME_MEMORIES);
    File f = FFat.open(FILENAME_MEMORIES, FILE_WRITE);
#else
    LittleFS.remove(FILENAME_MEMORIES);
    File f = LittleFS.open(FILENAME_MEMORIES, FILE_WRITE);
#endif
    if (!f) return;
    f.write((uint8_t *) &mem, sizeof(mem));
    f.close();
    return;
}

void rdcp_memory_restore(void)
{
    serial_writeln("INFO: Restoring memories");
#ifdef ROLORAN_USE_FFAT
    File f = FFat.open(FILENAME_MEMORIES, FILE_READ);
#else
    File f = LittleFS.open(FILENAME_MEMORIES, FILE_READ);
#endif
    if (!f) return;
    f.read((uint8_t *) &mem, sizeof(mem));
    f.close();
    rdcp_memory_dump();
    return;
}

/* EOF */