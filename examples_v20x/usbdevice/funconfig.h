#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

// Though this should be on by default we can extra force it on.
#define FUNCONF_USE_DEBUGPRINTF 1
#define FUNCONF_DEBUGPRINTF_TIMEOUT (1<<31) // Wait for a very very long time.
#define FUNCONF_USE_UARTPRINTF  0
#define FUNCONF_USE_HSE 1
#define FUNCONF_SYSTICK_USE_HCLK 1
#define FUNCONF_ENABLE_HPE 0

#endif

