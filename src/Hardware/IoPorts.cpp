/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"
#include <Hardware/AnalogIn.h>
#include <Hardware/AnalogOut.h>
#include <Hardware/Interrupts.h>

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
	case PinAccess::readWithPullup:
		desiredMode = INPUT_PULLUP;
		break;
	case PinAccess::read:
	default:
		desiredMode = INPUT;
		break;
	}

	if (logicalPinModes[pin] != (int8_t)desiredMode)
	{
		if (access == PinAccess::readAnalog && !AnalogIn::IsChannelEnabled(pin))
		{
			AnalogIn::EnableChannel(pin, nullptr, CallbackParameter());
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
		::DetachInterrupt(pin);
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

bool IoPort::Read() const
{
	if (IsValid())
	{
		const bool b = ReadPin(pin);
		return (totalInvert) ? !b : b;
	}
	return false;
}

// Attach an interrupt to the pin. Nor permitted if we allocated the pin in shared input mode.
bool IoPort::AttachInterrupt(StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param) const
{
	return IsValid() && !isSharedInput && ::AttachInterrupt(pin, callback, mode, param);
}

void IoPort::DetachInterrupt() const
{
	if (IsValid() && !isSharedInput)
	{
		::DetachInterrupt(pin);
	}
}

// Allocate the specified logical pin, returning true if successful
bool IoPort::Allocate(const char *pn, const StringRef& reply, PinUsedBy neededFor, PinAccess access)
{
	Release();

	bool inverted = false;
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
				access = PinAccess::readWithPullup;
			}
		}
		else
		{
			break;
		}
		++pn;
	}

	const char *const fullPinName = pn;			// the full pin name less the inversion and pullup flags

	Pin lp;
	bool hwInvert;
	if (!LookupPinName(pn, lp, hwInvert))
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
				reply.printf("Pin '%s' is not free", fullPinName);
				return false;
			}
			doSetMode = false;
		}
		pin = lp;
		hardwareInvert = hwInvert;
		isSharedInput = (neededFor == PinUsedBy::temporaryInput);
		SetInvert(inverted);

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

// Function to look up a pin name pass back the corresponding index into the pin table
// On this platform, the mapping from pin names to pins is fixed, so this is a simple lookup
/*static*/ bool IoPort::LookupPinName(const char*pn, Pin& pin, bool& hardwareInverted)
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
			const char *p = pn;
			bool hwInverted = (*q == '!');
			if (hwInverted)
			{
				++q;
			}
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
	case PinAccess::readWithPullup:	return "digital read (pullup resistor enabled)";
	case PinAccess::readAnalog:		return "analog read";
	case PinAccess::write0:			return "write (initially low)";
	case PinAccess::write1:			return "write (initially high)";
	case PinAccess::pwm:			return "write PWM";
	case PinAccess::servo:			return "servo write";
	default:						return "[unknown]";
	}
}

// Low level pin access functions

/*static*/ void IoPort::SetPinMode(Pin pin, PinMode mode)
{
	switch (mode)
	{
	case INPUT:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		break;

	case INPUT_PULLUP:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
		break;

	case INPUT_PULLDOWN:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
		break;

	case OUTPUT_LOW:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(pin, false);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case OUTPUT_HIGH:
		gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(pin, true);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case AIN:
		// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);		// disable the data input buffer
		gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_B);		// ADC is always on peripheral B
		break;

	default:
		break;
	}
}

/*static*/ bool IoPort::ReadPin(Pin pin)
{
	return digitalRead(pin);
}

/*static*/ void IoPort::WriteDigital(Pin pin, bool high)
{
	digitalWrite(pin, high);
}

/*static*/ void IoPort::WriteAnalog(Pin p, float pwm, uint16_t frequency)
{
	AnalogOut::Write(p, pwm, frequency);
}

// Members of class PwmPort
PwmPort::PwmPort()
{
	frequency = DefaultPinWritePwmFreq;
}

void PwmPort::SetFrequency(float freq)
{
	frequency = (uint16_t)constrain<float>(freq, 1.0, 65535);
}

void PwmPort::WriteAnalog(float pwm) const
{
	if (pin != NoPin)
	{
		IoPort::WriteAnalog(pin, ((totalInvert) ? 1.0 - pwm : pwm), frequency);
	}
}

// End
