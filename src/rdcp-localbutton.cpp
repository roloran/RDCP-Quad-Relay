#include <Arduino.h> 
#include "rdcp-localbutton.h"
#include "rdcp-commands.h"
#include "serial.h"
#include "hal.h"

bool local_button_initialized = false;
uint64_t last_button_press = 0;

#define LOCAL_BUTTON_PIN GPIO_NUM_35

void local_button_check(void)
{
    int64_t now = my_millis();

    if (!local_button_initialized)
    {
        pinMode(LOCAL_BUTTON_PIN, INPUT_PULLUP);
        last_button_press = now;
        local_button_initialized = true;
    }

    if (now < last_button_press + 5 * SECONDS_TO_MILLISECONDS) return; // debounce

    if (digitalRead(LOCAL_BUTTON_PIN) == LOW)
    {
        last_button_press = now;
        serial_writeln("INFO: Local Button has been pressed!");
        serial_writeln("DA_LOCALBUTTON");
        rdcp_cmd_send_da_status_response(true);
    }

    return;
}