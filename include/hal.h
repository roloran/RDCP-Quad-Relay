#ifndef _ROLORAN_HAL_H 
#define _ROLORAN_HAL_H

#include <Arduino.h> 

#define SECONDS_TO_MILLISECONDS      1000
#define MILLISECONDS_TO_MICROSECONDS 1000
#define MINUTES_TO_SECONDS           60 
#define MINUTES_TO_MILLISECONDS      60000
#define HOURS_TO_SECONDS             3600
#define HOURS_TO_MILLISECONDS        3600000

/**
 * @return Number of milliseconds since device start as int64_t
 */
int64_t my_millis(void);

/**
 * Set maximum CPU frequency. 
 */
void cpu_fast(void);

/**
 * Set power-conserving CPU frequency.
 */
void cpu_slow(void);

/**
 * Generate a random number between r_min and r_max.
 */
int64_t my_random_in_range(uint32_t r_min, uint32_t r_max);

#endif 
/* EOF */