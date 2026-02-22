#include "hal.h"
#include "serial.h"
#include "lora.h"

extern da_config CFG;

int64_t my_millis(void)
{
    return (int64_t) esp_timer_get_time() / MILLISECONDS_TO_MICROSECONDS;
}

void cpu_fast(void)
{
    setCpuFrequencyMhz(240);
    return;
}

void cpu_slow(void)
{
    if (!CFG.bt_enabled) setCpuFrequencyMhz(80);
    return;
}

int64_t my_random_in_range(uint32_t r_min, uint32_t r_max)
{
    static bool srand_called = false;

    if (!srand_called)
    {
        srand_called = true;
        uint32_t my_seed = radio868_random_byte() + 
                           (radio868_random_byte() << 8) +   
                           (radio868_random_byte() << 16) +   
                           (radio868_random_byte() << 24);
        srand(my_seed);
    }

    int64_t result = rand() % (r_max - r_min + 1) + r_min;
    return result;
}

/* EOF */