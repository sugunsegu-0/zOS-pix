
#ifndef IOX_BINDING_C_SLEEP_FOR_H
#define IOX_BINDING_C_SLEEP_FOR_H

#ifdef _WIN32
#include <windows.h>

void sleep_for(uint32_t milliseconds)
{
    Sleep((uint64_t)milliseconds);
}

#else
#include <unistd.h>

void sleep_for(uint32_t milliseconds)
{
    usleep(milliseconds * 1000U);
}
#endif

#endif
