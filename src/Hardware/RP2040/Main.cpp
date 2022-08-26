#include <CoreIO.h>

#if RP2040

// SystemCoreClock is needed by FreeRTOS
uint32_t SystemCoreClock = 125000000;		//TODO is this correct?

#if 0
RP2040 rp2040;
extern "C" {
    volatile bool __otherCoreIdled = false;
    int __holdUpPendSV = 0;
};
#endif

extern "C" int main()
{
#if 0 //F_CPU != 125000000
    set_sys_clock_khz(F_CPU / 1000, true);
#endif

#if 0
    mutex_init(&_pioMutex);

    rp2040.begin();

#ifndef NO_USB
#ifdef USE_TINYUSB
    TinyUSB_Device_Init(0);

#else
    __USBStart();

#ifndef DISABLE_USB_SERIAL

    if (!__isFreeRTOS) {
        // Enable serial port for reset/upload always
        Serial.begin(115200);
    }
#endif
#endif
#endif

#if defined DEBUG_RP2040_PORT
    if (!__isFreeRTOS) {
        DEBUG_RP2040_PORT.begin(115200);
    }
#endif

#ifndef NO_USB
    if (!__isFreeRTOS) {
        if (setup1 || loop1) {
            rp2040.fifo.begin(2);
        } else {
            rp2040.fifo.begin(1);
        }
        rp2040.fifo.registerCore();
    }
#endif

    if (!__isFreeRTOS) {
        if (setup1 || loop1) {
            delay(1); // Needed to make Picoprobe upload start 2nd core
            multicore_launch_core1(main1);
        }
        setup();
        while (true) {
            loop();
            __loop();
        }
    } else {
        rp2040.fifo.begin(2);
        startFreeRTOS();
    }

#endif

    AppMain();
}

#endif

// End
