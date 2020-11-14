/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"
#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Interrupts.h>
#include <CAN/CanInterface.h>

#ifdef ATEIO
# include <Hardware/ATEIO/ExtendedAnalog.h>
#endif

// Members of class IoPort

PinUsedBy IoPort::portUsedBy[NumPins];
int8_t IoPort::logicalPinModes[NumPins];	// what mode each logical pin is set to - would ideally be class PinMode not int8_t

/*static*/ void IoPort::Init()
{
	for (PinUsedBy& p : portUsedBy)
	{
		p = PinUsedBy::unused;
	}
	for (int8_t& p : logicalPinModes)
	{
		p = PIN_MODE_NOT_CONFIGURED;
	}
}

IoPort::IoPort() : pin(NoPin), hardwareInvert(false), totalInvert(false), isSharedInput(false)
{
}

// Set the specified pin mode returning true if successful
bool IoPort::SetMode(PinAccess access)
{
	if (!IsValid())
	{
		return false;
	}

#ifdef ATEIO
	// Check for extended analog pins
	if (pin >= NumPhysicalPins)
	{
		return access == PinAccess::readAnalog;
	}
#endif

	// Check that the pin mode has been defined suitably
	PinMode desiredMode;
	switch (access)
	{
	case PinAccess::write0:
		desiredMode = (totalInvert) ? OUTPUT_HIGH : OUTPUT_LOW;
		break;

	case PinAccess::write1:
		desiredMode = (totalInvert) ? OUTPUT_LOW : OUTPUT_HIGH;
		break;

	case PinAccess::pwm:
	case PinAccess::servo:
		desiredMode = (totalInvert) ? OUTPUT_PWM_HIGH : OUTPUT_PWM_LOW;
		break;

	case PinAccess::readAnalog:
		desiredMode = AIN;
		break;

	case PinAccess::readWithPullup_InternalUseOnly:
		desiredMode = INPUT_PULLUP;
		break;

	case PinAccess::read:
	default:
		desiredMode = INPUT;
		break;
	}

	if (logicalPinModes[pin] != (int8_t)desiredMode)
	{
		const AnalogChannelNumber chan = PinToAdcChannel(pin);
		if (chan != NO_ADC)
		{
			if (access == PinAccess::readAnalog)
			{
				IoPort::SetPinMode(pin, AIN);		// SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
				AnalogInEnableChannel(chan, true);
				logicalPinModes[pin] = (int8_t)desiredMode;
				return true;
			}
			else
			{
				AnalogInEnableChannel(chan, false);
			}
		}
		else if (access == PinAccess::readAnalog)
		{
			return false;
		}
		IoPort::SetPinMode(pin, desiredMode);
		logicalPinModes[pin] = (int8_t)desiredMode;
	}
	return true;
}

void IoPort::Release()
{
	if (IsValid() && !isSharedInput)
	{
		detachInterrupt(pin);
		portUsedBy[pin] = PinUsedBy::unused;
		logicalPinModes[pin] = PIN_MODE_NOT_CONFIGURED;
	}
	pin = NoPin;
	hardwareInvert = totalInvert = false;
}

void IoPort::WriteDigital(bool high) const
{
	if (IsValid())
	{
		WriteDigital(pin, (totalInvert) ? !high : high);
	}
}

bool IoPort::ReadDigital() const
{
	if (IsValid())
	{
		const bool b = ReadPin(pin);
		return (totalInvert) ? !b : b;
	}
	return false;
}

uint16_t IoPort::ReadAnalog() const
{
	if (IsValid())
	{
		const AdcInput chan = PinTable[pin].adc;
		if (chan != AdcInput::none)
		{
			const uint16_t val =
#ifdef ATEIO
			(pin >= NumPhysicalPins)
				? ExtendedAnalog::AnalogIn(GetInputNumber(chan)) :
#endif
				AnalogIn::ReadChannel(chan);
			return (totalInvert) ? ((1u << AnalogIn::AdcBits) - 1) - val : val;
		}
	}
	return 0;
}

// Attach an interrupt to the pin. Nor permitted if we allocated the pin in shared input mode.
bool IoPort::AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const
{
	return IsValid() && !isSharedInput && attachInterrupt(pin, callback, mode, param);
}

void IoPort::DetachInterrupt() const
{
	if (IsValid() && !isSharedInput)
	{
		detachInterrupt(pin);
	}
}

bool IoPort::SetAnalogCallback(AnalogInCallbackFunction fn, CallbackParameter cbp, uint32_t ticksPerCall)
{
	return AnalogIn::SetCallback(PinToAdcChannel(pin), fn, cbp, ticksPerCall, false);
}

// Try to assign ports, returning the number of ports successfully assigned
/*static*/ size_t IoPort::AssignPorts(const char* pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort* const ports[], const PinAccess access[])
{
	// Release any existing assignments
	for (size_t i = 0; i < numPorts; ++i)
	{
		ports[i]->Release();
	}

	// Parse the string into individual port names
	size_t index = 0;
	for (size_t i = 0; i < numPorts; ++i)
	{
		// Get the next port name
		String<StringLength50> pn;
		char c;
		while ((c = pinNames[index]) != 0 && c != '+')
		{
			pn.cat(c);
			++index;
		}

		// Try to allocate the port
		if (!ports[i]->Allocate(pn.c_str(), reply, neededFor, access[i]))
		{
			for (size_t j = 0; j < i; ++j)
			{
				ports[j]->Release();
			}
			return 0;
		}

		if (c != '+')
		{
			return i + 1;
		}
		++index;					// skip the "+"
	}
	return numPorts;
}

// Allocate the specified logical pin, returning true if successful
bool IoPort::Allocate(const char *pn, const StringRef& reply, PinUsedBy neededFor, PinAccess access)
{
	Release();

	bool inverted = false;
	alternateConfig = false;
	for (;;)
	{
		if (*pn == '!')
		{
			inverted = !inverted;
		}
		else if (*pn == '^')
		{
			if (access == PinAccess::read)
			{
				access = PinAccess::readWithPullup_InternalUseOnly;
			}
		}
		else if (*pn == '*')
		{
			alternateConfig = true;
		}
		else
		{
			break;
		}
		++pn;
	}

	const char *const fullPinName = pn;			// the full pin name less the inversion and pullup flags

	Pin lp;
	bool hwInvert, pullupAlways;
	if (!LookupPinName(pn, lp, hwInvert, pullupAlways))
	{
		reply.printf("Unknown pin name '%s'", fullPinName);
		return false;
	}

	if (lp != NoPin)					// if not assigning "nil"
	{
		bool doSetMode = true;
		if (portUsedBy[lp] == PinUsedBy::unused || (portUsedBy[lp] == PinUsedBy::temporaryInput && neededFor != PinUsedBy::temporaryInput))
		{
			portUsedBy[lp] = neededFor;
		}
		else
		{
			const PinMode pm = (PinMode)logicalPinModes[lp];
			if (   neededFor != PinUsedBy::temporaryInput
				|| (pm != INPUT && pm != INPUT_PULLUP)
			   )
			{
				reply.printf("Pin %u.%s is not free", CanInterface::GetCanAddress(), fullPinName);
				return false;
			}
			doSetMode = false;
		}
		pin = lp;
		hardwareInvert = hwInvert;
		isSharedInput = (neededFor == PinUsedBy::temporaryInput);
		SetInvert(inverted);

		if (pullupAlways && access == PinAccess::read)
		{
			access = PinAccess::readWithPullup_InternalUseOnly;
		}
		if (doSetMode && !SetMode(access))
		{
			reply.printf("Pin '%s' does not support mode %s", fullPinName, TranslatePinAccess(access));
			Release();
			return false;
		}
	}

	return true;
}

bool IoPort::GetInvert() const
{
	return (hardwareInvert) ? !totalInvert : totalInvert;
}

void IoPort::SetInvert(bool pInvert)
{
	totalInvert = (hardwareInvert) ? !pInvert : pInvert;
}

void IoPort::ToggleInvert(bool pInvert)
{
	if (pInvert)
	{
		totalInvert = !totalInvert;
	}
}

void IoPort::AppendDetails(const StringRef& str) const
{
	if (IsValid())
	{
		str.catf(" pin ");
		AppendPinName(str);
		if (logicalPinModes[pin] == INPUT_PULLUP)
		{
			str.cat(", pullup enabled");
		}
		else if (logicalPinModes[pin] == INPUT)
		{
			str.cat(", pullup disabled");
		}
	}
	else
	{
		str.cat(" has no pin");
	}
}

// Append the names of the pin to a string, picking only those that have the correct hardware invert status
void IoPort::AppendPinName(const StringRef& str) const
{
	if (IsValid())
	{
		if (GetInvert())
		{
			str.cat('!');
		}
		const size_t insertPoint = str.strlen();
		const char *pn = PinTable[pin].pinNames;
		unsigned int numPrinted = 0;
		do
		{
			bool inverted = (*pn == '!');
			if (inverted)
			{
				++pn;
			}
			if (hardwareInvert)
			{
				inverted = !inverted;
			}
			if (inverted)
			{
				// skip this one
				while (*pn != 0 && *pn != ',')
				{
					++pn;
				}
			}
			else
			{
				// Include this one
				if (numPrinted == 0)
				{
					str.catf("%u.", CanInterface::GetCanAddress());
				}
				else
				{
					str.cat(',');
				}
				++numPrinted;
				while (*pn != 0 && *pn != ',')
				{
					str.cat(*pn);
					++pn;
				}
			}

		} while (*pn++ == ',');

		if (numPrinted > 1)
		{
			str.Insert(insertPoint, '(');
			str.cat(')');
		}
	}
	else
	{
		str.cat(NoPinName);
	}
}

/*static*/ void IoPort::AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[])
{
	for (size_t i = 0; i < numPorts; ++i)
	{
		if (ports[i]->IsValid())
		{
			if (i != 0)
			{
				str.cat('+');
			}
			ports[i]->AppendPinName(str);
		}
		else
		{
			if (i == 0)
			{
				str.cat("nil");
			}
			break;
		}
	}
}

// Function to look up a pin name pass back the corresponding index into the pin table
// On this platform, the mapping from pin names to pins is fixed, so this is a simple lookup
/*static*/ bool IoPort::LookupPinName(const char*pn, Pin& pin, bool& hardwareInverted, bool& pullupAlways)
{
	if (StringEqualsIgnoreCase(pn, NoPinName))
	{
		pin = NoPin;
		hardwareInverted = false;
		return true;
	}

	for (size_t lp = 0; lp < ARRAY_SIZE(PinTable); ++lp)
	{
		const char *q = PinTable[lp].pinNames;
		while (*q != 0)
		{
			// Try the next alias in the list of names for this pin
			bool hwInverted = false, pullAlways = false;
			for (;;)
			{
				if (*q == '!')
				{
					hwInverted = true;
				}
				else if (*q == '^')
				{
					pullAlways = true;
				}
				else
				{
					break;
				}
				++q;
			}
			const char *p = pn;
			while (*q != ',' && *q != 0 && *p == *q)
			{
				++p;
				++q;
			}
			if (*p == 0 && (*q == 0 || *q == ','))
			{
				// Found a match
				pin = (Pin)lp;
				hardwareInverted = hwInverted;
				pullupAlways = pullAlways;
				return true;
			}

			// Skip to the start of the next alias
			while (*q != 0 && *q != ',')
			{
				++q;
			}
			if (*q == ',')
			{
				++q;
			}
		}
	}
	return false;
}

/*static*/ const char* IoPort::TranslatePinAccess(PinAccess access)
{
	switch (access)
	{
	case PinAccess::read:			return "digital read";
	case PinAccess::readWithPullup_InternalUseOnly:	return "digital read (pullup resistor enabled)";
	case PinAccess::readAnalog:		return "analog read";
	case PinAccess::write0:			return "write (initially low)";
	case PinAccess::write1:			return "write (initially high)";
	case PinAccess::pwm:			return "write PWM";
	case PinAccess::servo:			return "servo write";
	default:						return "[unknown]";
	}
}

// Low level pin access functions

// Find the ADC channel associated with a pin
/*static*/ AdcInput IoPort::PinToAdcInput(Pin pin, bool useAlternateAdc)
{
	return (pin >= ARRAY_SIZE(PinTable)) ? AdcInput::none
#if SAMC21
			: (useAlternateAdc) ? PinTable[pin].sdadc
#endif
				: PinTable[pin].adc;
}

/*static*/ bool IoPort::ReadPin(Pin pin)
{
	return (pin != NoPin) && digitalRead(pin);
}

/*static*/ void IoPort::WriteDigital(Pin pin, bool high)
{
	if (pin != NoPin)
	{
		digitalWrite(pin, high);
	}
}

/*static*/ void IoPort::WriteAnalog(Pin p, float pwm, uint16_t frequency)
{
	AnalogOut::Write(p, pwm, frequency);
}

#if SAMC21

// Set high driver strength on an output pin
/*static*/ void IoPort::SetHighDriveStrength(Pin p)
{
	PORT->Group[GpioPortNumber(p)].PINCFG[GpioPinNumber(p)].reg |= PORT_PINCFG_DRVSTR;
}

#endif

// Members of class PwmPort
PwmPort::PwmPort()
{
	frequency = DefaultPinWritePwmFreq;
}

void PwmPort::AppendDetails(const StringRef& str) const
{
	IoPort::AppendDetails(str);
	if (IsValid())
	{
		str.catf(" frequency %uHz", frequency);
	}
}

void PwmPort::WriteAnalog(float pwm) const
{
	if (pin != NoPin)
	{
		IoPort::WriteAnalog(pin, ((totalInvert) ? 1.0 - pwm : pwm), frequency);
	}
}

// End
