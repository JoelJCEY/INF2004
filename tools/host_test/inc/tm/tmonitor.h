#ifndef HOST_TM_H
#define HOST_TM_H
#include <stdio.h>
#define tm_printf(fmt, ...) printf((const char*)(fmt), ##__VA_ARGS__)
#ifndef app_log
#define app_log(fmt, ...) printf((const char*)(fmt), ##__VA_ARGS__)
#endif
#endif
