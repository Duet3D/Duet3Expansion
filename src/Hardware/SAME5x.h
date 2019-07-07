/*
 * SAME5x.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_H_
#define SRC_HARDWARE_SAME5X_H_

#include <cstdint>

extern uint32_t SystemCoreClock;		// in system_same51.c

// Definitions for the SAME5x that I can't find in the Microchip files
static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t usec){
    /*
     * Based on Paul Stoffregen's implementation
     * for Teensy 3.0 (http://www.pjrc.com/)
     */
    if (usec == 0) return;
    uint32_t n = usec * (SystemCoreClock / 3000000);
    asm volatile(
        "L_%=_delayMicroseconds:"       "\n\t"
        "subs   %0, #1"                 "\n\t"
        "bne    L_%=_delayMicroseconds" "\n"
        : "+r" (n) :
    );
}

// Mapping from pin numbers to EXINT numbers
constexpr uint8_t Nx = 0xFF;											// out-of-range number to indicate no EXINT available
static constexpr uint8_t PinToExint[] =
{
	// Port A
	 0,  1,  2,  3,  4,  5,  6,  7, Nx,  9, 10, 11, 12, 13, 14, 15,		// PA8 is NMI
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, Nx, 11, Nx, Nx, 14, 15,		// PA26,28,29 pins not present
	// Port B
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 12, 13, 14, 15, 14, 15,		// unusual numbering on PB26-29
	// Port C
	 0,  1,  2,  3,  4,  5,  6,  9, Nx, Nx, 10, 11, 12, 13, 14, 15,		// PC8,9 pins not present
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, Nx, 14, 15,		// PC29 pin not present
	// Port D
	 0,  1, Nx, Nx, Nx, Nx, Nx, Nx, 3,  4,  5,  6,  7,  Nx, Nx, Nx,
	Nx, Nx, Nx, Nx, 10, 11, Nx, Nx, Nx, Nx, Nx, Nx, Nx, Nx, Nx, Nx
};

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
#endif

static_assert(ARRAY_SIZE(PinToExint) == 128);

#endif /* SRC_HARDWARE_SAME5X_H_ */
