/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include <RepRapFirmware.h>
#include <Interrupts.h>
#include <AnalogIn.h>

// Enumeration to describe what we want to do with a pin
enum class PinAccess : int
{
	read,
	readWithPullup_InternalUseOnly,
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
	gpin,
	gpout,
	filamentMonitor,
	temporaryInput,
	sensor
};

// Class to represent a port
class IoPort
{
public:
	IoPort();
	bool SetMode(PinAccess access);
	void Release();
	void AppendDetails(const StringRef& str) const;

	Pin GetPin() const { return pin; }

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) { return Allocate(pinName, reply, neededFor, access); }

	void AppendPinName(const StringRef& str) const;
	bool IsValid() const { return pin != NoPin; }
	bool GetInvert() const;
	void SetInvert(bool pInvert);
	void ToggleInvert(bool pInvert);
	bool UseAlternateConfig() const { return alternateConfig; }

	void WriteDigital(bool high) const;
	bool Read() const;
	uint16_t ReadAnalog() const;

	bool AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const;
	void DetachInterrupt() const;
	bool SetAnalogCallback(AnalogInCallbackFunction fn, CallbackParameter cbp, uint32_t ticksPerCall);

	// Initialise static data
	static void Init();

	static void AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[]);

	// Look up a pin name in the pins table
	static bool LookupPinName(const char*pn, Pin& pin, bool& hardwareInverted, bool& pullupAlways);

	// Find the ADC channel associated with a pin
	static AdcInput PinToAdcInput(Pin pin, bool useAlternateAdc);

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode) noexcept { pinMode(p, mode); }

	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

#if SAMC21
	// Set high driver strength on an output pin
	static void SetHighDriveStrength(Pin p);
#endif

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static const char* TranslatePinAccess(PinAccess access);

	Pin pin;
	uint8_t hardwareInvert : 1,								// true if the hardware includes inversion
			totalInvert : 1,								// true if the value should be inverted when reading/writing the pin
			isSharedInput : 1,								// true if we are using this pin as a shared input
			alternateConfig : 1;							// true if we are using the alternate configuration of this pin, e.g. SDADC instyead of ADC

	static PinUsedBy portUsedBy[NumPins];					// the list of what each logical port is used by
	static int8_t logicalPinModes[NumPins];					// what mode each logical pin is set to - would ideally be class PinMode not int8_t

	static constexpr const char *NoPinName = "nil";
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();

	void AppendDetails(const StringRef& str) const;			// hides the version in IoPort
	void SetFrequency(PwmFrequency freq) { frequency = freq; }
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
