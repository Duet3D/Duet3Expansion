/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include "RepRapFirmware.h"
#include "Interrupts.h"
#include "AnalogIn.h"

// Enumeration to describe what we want to do with a pin
enum class PinAccess : int
{
	read,
	readWithPullup,
	readAnalog,
	write0,
	write1,
	pwm,
	servo
};

enum class PinUsedBy : uint8_t
{
	unused = 0,
	heater,
	fan,
	endstop,
	zprobe,
	tacho,
	spindle,
	laser,
	gpio,
	filamentMonitor,
	temporaryInput,
	sensor
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

// Class to represent a port
class IoPort
{
public:
	IoPort();
	bool SetMode(PinAccess access);
	void Release();
	void AppendDetails(const StringRef& str);

//	Pin GetPin() const { return pin; }

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) { return Allocate(pinName, reply, neededFor, access); }

	void AppendPinName(const StringRef& str) const;
	bool IsValid() const { return pin != NoPin; }
	bool GetInvert() const;
	void SetInvert(bool pInvert);
	void ToggleInvert(bool pInvert);

	void WriteDigital(bool high) const;
	bool Read() const;
	uint16_t ReadAnalog() const { return AnalogIn::ReadChannel(PinTable[pin].adc); }
	bool AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const;
	void DetachInterrupt() const;

	// Initialise static data
	static void Init();

	static void AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[]);

	// Look up a pin name in the pins table
	static bool LookupPinName(const char*pn, Pin& pin, bool& hardwareInverted);

	// Find the ADC channel associated with a pin
	static AdcInput PinToAdcInput(Pin pin) { return (pin < ARRAY_SIZE(PinTable)) ? PinTable[pin].adc : AdcInput::none; }

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static const char* TranslatePinAccess(PinAccess access);

	Pin pin;
	uint8_t hardwareInvert : 1,								// true if the hardware includes inversion
			totalInvert : 1,								// true if the value should be inverted when reading/writing the pin
			isSharedInput : 1;								// true if we are using this pin as a shared input

	static PinUsedBy portUsedBy[NumPins];					// the list of what each logical port is used by
	static int8_t logicalPinModes[NumPins];					// what mode each logical pin is set to - would ideally be class PinMode not int8_t

	static constexpr const char *NoPinName = "nil";
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();

	void SetFrequency(float freq);
//	float GetFrequency() const { return (float)frequency; }
	void WriteAnalog(float pwm) const;

private:
	PwmFrequency frequency;
};

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
