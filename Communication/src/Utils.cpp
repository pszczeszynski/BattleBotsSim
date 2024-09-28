#include "Utils.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include <Arduino.h>

#define MAX_DOWNSAMPLED_PRINTS 20
#define MAX_PRINT_LENGTH 256

#define PRINT_RATE_US 1000000 // 1s

static char formats[MAX_DOWNSAMPLED_PRINTS][MAX_PRINT_LENGTH];
static uint32_t last_printed_times[MAX_DOWNSAMPLED_PRINTS + 1] = {0};
static int used_prints = 0;

void DownsampledPrintf(const char *format, ...)
{
    for(int i = 0; i < used_prints; i++)
    {
        if (!strcmp(format, formats[i]))
        {
            if (last_printed_times[i] + PRINT_RATE_US < micros())
            {
                last_printed_times[i] = micros();
                char buffer[256];  // or smaller or static &c.
                va_list args;
                va_start(args, format);
                vsprintf(buffer, format, args);
                va_end(args);
                Serial.print(buffer);
            }
            return;
        }
    }
    if(used_prints == MAX_DOWNSAMPLED_PRINTS)
    {
        if (last_printed_times[MAX_DOWNSAMPLED_PRINTS] + PRINT_RATE_US < micros())
        {
            Serial.println("max number of downsampled prints exceeded");
            last_printed_times[MAX_DOWNSAMPLED_PRINTS] = micros();
        }
        return;
    }
    strcpy(formats[used_prints], format);
    last_printed_times[used_prints] = micros();
    used_prints++;
    char buffer[256];  // or smaller or static &c.
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    Serial.print(buffer);
}