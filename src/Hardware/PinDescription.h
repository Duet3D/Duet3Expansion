/*
 * PinDescription.h
 *
 *  Created on: 10 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_PINDESCRIPTION_H_
#define SRC_HARDWARE_SAME5X_PINDESCRIPTION_H_

#include <CoreIO.h>

#if 0	// this project doesn't yet include PinCapability in the pin tables

// Enum to represent allowed types of pin access
// We don't have a separate bit for servo, because Duet PWM-capable ports can be used for servos if they are on the Duet main board
enum class PinCapability: uint8_t
{
	// Individual capabilities
	none = 0u,
	read = 1u,				// digital read
	ain = 2u,				// analog read
	write = 4u,				// digital write
	pwm = 8u,				// PWM write
	npDma = 16u,			// Neopixel output using DMA e.g. using SPI MOSI

	// Combinations
	ainr = 1u|2u,
	rw = 1u|4u,
	wpwm = 4u|8u,
	rwpwm = 1u|4u|8u,
	ainrw = 1u|u2|4u,
	ainrwpwm = 1u|2u|4u|8u
};

constexpr inline PinCapability operator|(PinCapability a, PinCapability b) noexcept
{
	return (PinCapability)((uint8_t)a | (uint8_t)b);
}

#endif

// The pin description says what functions are available on each pin, filtered to avoid allocating the same function to more than one pin..
// It is a struct not a class so that it can be direct initialised in read-only memory.
struct PinDescription : public PinDescriptionBase
{
#if 0
	PinCapability cap;
#endif
	const char* pinNames;

#if 0
	PinCapability GetCapability() const noexcept { return cap; }
#endif
	const char* GetNames() const noexcept { return pinNames; }
};

#endif /* SRC_HARDWARE_SAME5X_PINDESCRIPTION_H_ */
