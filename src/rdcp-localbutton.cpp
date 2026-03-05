#include <Arduino.h> 
#include "rdcp-localbutton.h"
#include "rdcp-commands.h"
#include "serial.h"
#include "hal.h"

bool local_button_initialized = false;
uint64_t last_button_press = 0;

void local_button_check(void)
{
    if (!local_button_initialized)
    {
        // TODO 
        local_button_initialized = true;
    }

    int64_t now = my_millis();
    if (now < last_button_press + 5 * SECONDS_TO_MILLISECONDS) return; // debounce

    // TODO: Check for button press
    if (false)
    {
        last_button_press = now;
        serial_writeln("DA_LOCALBUTTON");
        rdcp_cmd_send_da_status_response(true);
    }

    return;
}