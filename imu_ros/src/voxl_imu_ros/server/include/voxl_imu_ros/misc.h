
#ifndef MISC_H
#define MISC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int64_t my_time_monotonic_ns(void);
int64_t my_time_realtime_ns(void);
void my_nanosleep(uint64_t ns);
int my_loop_sleep(double rate_hz, int64_t* next_time);

#ifdef __cplusplus
}
#endif

#endif // MISC_H
