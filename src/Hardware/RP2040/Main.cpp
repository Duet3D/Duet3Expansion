#include <CoreIO.h>

#if RP2040

// SystemCoreClock is needed by FreeRTOS
uint32_t SystemCoreClock = 125000000;

extern "C" int main()
{
    AppMain();
}

#endif

// End
