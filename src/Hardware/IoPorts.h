/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include "RepRapFirmware.h"

// Enumeration to describe what we want to do with a logical pin
enum class PinAccess : int
{
	read,
	write0,
	write1,
	pwm,
	servo
};

// Pin mode enumeration. Would ideally be a C++ scoped enum, but we need to use it from C library functions.
enum PinMode
{
	PIN_MODE_NOT_CONFIGURED = -1,	// used in Platform class to record that the mode for a pin has not been set yet
	INPUT = 0,						// pin is a digital input
	INPUT_PULLUP,					// pin is a digital input with pullup enabled
	INPUT_PULLDOWN,					// pin is a digital input with pulldown enabled
	OUTPUT_LOW,						// pin is an output with initial state LOW
	OUTPUT_HIGH,					// pin is an output with initial state HIGH
	AIN,							// pin is an analog input, digital input buffer is disabled if possible
	SPECIAL,						// pin is used for the special function defined for it in the variant.cpp file
	OUTPUT_PWM_LOW,					// PWM output mode, initially low
	OUTPUT_PWM_HIGH,				// PWM output mode, initially high
	OUTPUT_LOW_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
	OUTPUT_HIGH_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
	OUTPUT_PWM_OPEN_DRAIN			// used in SX1509B expansion driver to put the pin in PWM output mode
};

// Logical pins used for general output, servos, CCN and laser control
typedef uint16_t LogicalPin;				// Type used to represent logical pin numbers
constexpr LogicalPin NoLogicalPin = 0xFFFFu;

// Class to represent a port
class IoPort
{
public:
	IoPort();
	void Clear();
	bool Set(LogicalPin lp, PinAccess access, bool pInvert);

	LogicalPin GetLogicalPin() const { return logicalPin; }
	LogicalPin GetLogicalPin(bool& pInvert) const { pInvert = invert; return logicalPin; }
	void WriteDigital(bool high) const { if (pin != NoPin) { WriteDigital(pin, (invert) ? !high : high); } }

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

protected:
	LogicalPin logicalPin;
	Pin pin;
	bool invert;
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();
	void SetFrequency(float freq);
	float GetFrequency() const { return (float)frequency; }
	void WriteAnalog(float pwm) const;

private:
	uint16_t frequency;
};

void pinMode(Pin pin, PinMode mode);

inline bool digitalRead(Pin p)
{
	return gpio_get_pin_level(p);
}

inline void digitalWrite(Pin p, bool high)
{
	gpio_set_pin_level(p, high);

}

inline void fastDigitalWriteHigh(Pin p)
{
	gpio_set_pin_level(p, true);
}

inline void fastDigitalWriteLow(Pin p)
{
	gpio_set_pin_level(p, false);
}

#endif /* SRC_PORT_H_ */
