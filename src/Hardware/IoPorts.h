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
	sensor,
	led,
	sdCard
};

// Class to represent a port
class IoPort
{
public:
	IoPort() noexcept;
	~IoPort() { Release(); }

	bool SetMode(PinAccess access) noexcept;
	void Release() noexcept;
	void AppendBasicDetails(const StringRef& str) const noexcept;

	Pin GetPin() const noexcept { return pin; }

	// Functions normally used to assign a pin, using the pin name
	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]) noexcept;
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept { return Allocate(pinName, reply, neededFor, access); }

	void AppendPinName(const StringRef& str, bool includeBoardAddress = true) const noexcept;
	bool IsValid() const noexcept { return pin != NoPin; }
	bool IsRealPort() const noexcept { return pin < NumRealPins; }
	bool GetInvert() const noexcept;
	void SetInvert(bool pInvert) noexcept;
	void ToggleInvert(bool pInvert) noexcept;
	bool GetTotalInvert() const noexcept { return totalInvert; }
	bool UseAlternateConfig() const noexcept { return alternateConfig; }

	void WriteDigital(bool high) const noexcept;

	// Warning: for speed when bit-banging Neopixels, FastDigitalWriteHigh and FastDigitalWriteLow do not take account of pin inversion!
	void FastDigitalWriteLow() const noexcept pre(IsValid()) { fastDigitalWriteLow(pin); }
	void FastDigitalWriteHigh() const noexcept pre(IsValid()) { fastDigitalWriteHigh(pin); }

	bool ReadDigital() const noexcept;
	uint32_t ReadAnalog() const noexcept;

#if SUPPORT_LDC1612
	bool IsLdc1612() const noexcept { return IsValid() && PinTable[pin].adc == AdcInput::ldc1612; }
#endif

	bool AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const noexcept;
	void DetachInterrupt() const noexcept;
	bool SetAnalogCallback(AnalogInCallbackFunction fn, CallbackParameter cbp, uint32_t ticksPerCall) noexcept;
	void ClearAnalogCallback() noexcept;

	// Initialise static data
	static void Init() noexcept;

	static void AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[]) noexcept;

	// Look up a pin name in the pins table
	static bool LookupPinName(const char*pn, Pin& returnedPin, bool& hardwareInverted, bool& pullupAlways) noexcept;

	// Find the ADC channel associated with a pin
	static AdcInput PinToAdcInput(Pin p, bool useAlternateAdc) noexcept;

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode) noexcept { pinMode(p, mode); }

	static bool ReadPin(Pin p) noexcept;
	static void WriteDigital(Pin p, bool high) noexcept;
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency) noexcept;

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept;

	static const char* TranslatePinAccess(PinAccess access) noexcept;

	Pin pin;
	uint8_t hardwareInvert : 1,								// true if the hardware includes inversion
			totalInvert : 1,								// true if the value should be inverted when reading/writing the pin
			isSharedInput : 1,								// true if we are using this pin as a shared input
			alternateConfig : 1;							// true if we are using the alternate configuration of this pin, e.g. SDADC instead of ADC

	static PinUsedBy portUsedBy[NumPins];					// the list of what each logical port is used by
	static int8_t logicalPinModes[NumPins];					// what mode each logical pin is set to - would ideally be class PinMode not int8_t

	static constexpr const char *NoPinName = "nil";
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();

	void AppendFullDetails(const StringRef& str) const noexcept;
	void AppendFrequency(const StringRef& str) const noexcept;		// append the frequency if the port is valid
	void SetFrequency(PwmFrequency freq) noexcept { frequency = freq; }
	void WriteAnalog(float pwm) const noexcept;

private:
	PwmFrequency frequency;
};

#endif /* SRC_PORT_H_ */
