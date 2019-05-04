/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"

// members of class IoPort
IoPort::IoPort()
{
	Clear();
}

void IoPort::Clear()
{
	logicalPin = NoLogicalPin;
	pin = NoPin;
	invert = false;
}

#if 0
bool IoPort::Set(LogicalPin lp, PinAccess access, bool pInvert)
{
	const bool ret = reprap.GetPlatform().GetFirmwarePin(lp, access, pin, invert);
	if (ret)
	{
		if (pInvert)
		{
			invert = !invert;
		}
	}
	else
	{
		Clear();
	}
	return ret;
}
#endif

/*static*/ void IoPort::SetPinMode(Pin pin, PinMode mode)
{
	pinMode(pin, mode);
}

/*static*/ bool IoPort::ReadPin(Pin pin)
{
	return digitalRead(pin);
}

/*static*/ void IoPort::WriteDigital(Pin pin, bool high)
{
	digitalWrite(pin, high);
}

#if 0
/*static*/ void IoPort::WriteAnalog(Pin pin, float pwm, uint16_t freq)
{
	AnalogOut(pin, pwm, freq);
}
#endif

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
		IoPort::WriteAnalog(pin, ((invert) ? 1.0 - pwm : pwm), frequency);
	}
}

void pinMode(Pin pin, PinMode mode)
{
	switch (mode)
	{
	case INPUT:
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		break;

	case INPUT_PULLUP:
		// The direction must be set before the pullup, otherwise setting the pullup doesn't work
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
		break;

	case INPUT_PULLDOWN:
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
		break;

	case OUTPUT_LOW:
		gpio_set_pin_level(pin, false);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case OUTPUT_HIGH:
		gpio_set_pin_level(pin, true);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case AIN:
		gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);
		break;

	default:
		break;
	}
}

// End
