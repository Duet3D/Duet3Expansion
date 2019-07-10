/*
 * PinInterrupts.cpp
 *
 *  Created on: 6 Jul 2019
 *      Author: David
 */

#include <Hardware/Interrupts.h>
#include "SAME5x.h"

struct InterruptCallback
{
	StandardCallbackFunction func;
	CallbackParameter param;

	InterruptCallback() : func(nullptr) { }
};

// On the SAME5x we have 16 external interrupts shared between multiple pins. Only one of those pins may be programmed to generate an interrupt.
// Therefore we will have a clash if we try to attach an interrupt to two pins that use the aame EXINT.
static InterruptCallback exintCallbacks[16];

// Record of which pin is using each EXINT
static Pin pinUsingExint[] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };
static_assert(ARRAY_SIZE(pinUsingExint) == 16);

void InitialisePinChangeInterrupts()
{
	if (!hri_eic_is_syncing(EIC, EIC_SYNCBUSY_SWRST)) {
		if (hri_eic_get_CTRLA_reg(EIC, EIC_CTRLA_ENABLE)) {
			hri_eic_clear_CTRLA_ENABLE_bit(EIC);
			hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_ENABLE);
		}
		hri_eic_write_CTRLA_reg(EIC, EIC_CTRLA_SWRST);
	}
	hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_SWRST);

	hri_eic_write_CTRLA_CKSEL_bit(EIC, 0);				// clocked by GCLK

	// Leave NMI disabled (hri_eic_write_NMICTRL_reg)
	// Leave event control disabled (hri_eic_write_EVCTRL_reg)

	hri_eic_write_ASYNCH_reg(EIC, 0);					// all channels synchronous (needed to have debouncing or filtering)
	hri_eic_write_DEBOUNCEN_reg(EIC, 0);				// debouncing disabled

#if 0
	hri_eic_write_DPRESCALER_reg(
	    EIC,
	    (EIC_DPRESCALER_PRESCALER0(CONF_EIC_DPRESCALER0)) | (CONF_EIC_STATES0 << EIC_DPRESCALER_STATES0_Pos)
	        | (EIC_DPRESCALER_PRESCALER1(CONF_EIC_DPRESCALER1)) | (CONF_EIC_STATES1 << EIC_DPRESCALER_STATES1_Pos)
	        | CONF_EIC_TICKON << EIC_DPRESCALER_TICKON_Pos | 0);
#endif

	hri_eic_write_CONFIG_reg(EIC, 0, 0);
	hri_eic_write_CONFIG_reg(EIC, 1, 0);

	hri_eic_set_CTRLA_ENABLE_bit(EIC);
}

// Attach an interrupt to the specified pin returning true if successful
bool AttachInterrupt(uint32_t pin, StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param)
{
	if (pin >= ARRAY_SIZE(PinTable))
	{
		return false;			// pin number out of range
	}

	const unsigned int exintNumber = PinTable[pin].exintNumber;
	if (exintNumber >= ARRAY_SIZE(pinUsingExint))
	{
		return false;			// no EXINT available on this pin (only occurs for PA8 which is NMI)
	}

	if (pinUsingExint[exintNumber] != pin)
	{
		if (pinUsingExint[exintNumber] != NoPin)
		{
			return false;		// this EXINT number is already used by another pin
		}
		pinUsingExint[exintNumber] = pin;
	}

	exintCallbacks[exintNumber].func = callback;
	exintCallbacks[exintNumber].param = param;

	// Configure the interrupt mode
	uint32_t modeWord;
	switch (mode)
	{
	case InterruptMode::low:		modeWord = EIC_CONFIG_SENSE0_LOW_Val  | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::high:		modeWord = EIC_CONFIG_SENSE0_HIGH_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::falling:	modeWord = EIC_CONFIG_SENSE0_FALL_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::rising:		modeWord = EIC_CONFIG_SENSE0_RISE_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::change:		modeWord = EIC_CONFIG_SENSE0_BOTH_Val | EIC_CONFIG_FILTEN0; break;
	default:						modeWord = EIC_CONFIG_SENSE0_NONE_Val; break;
	}

	// Switch the pin into EIC mode
	gpio_set_pin_function(pin, ((pin & 31) << 16) | GPIO_PIN_FUNCTION_A);		// EIC is always on peripheral A

	const unsigned int shift = (exintNumber & 7u) << 2u;
	const uint32_t mask = ~(0x0000000F << shift);
	if (exintNumber < 8)
	{
		EIC->CONFIG[0].reg = (EIC->CONFIG[0].reg & mask) | (modeWord << shift);
	}
	else
	{
		EIC->CONFIG[1].reg = (EIC->CONFIG[1].reg & mask) | (modeWord << shift);
	}

	// Enable interrupt
	hri_eic_set_INTEN_reg(EIC, 1ul << exintNumber);
	NVIC_EnableIRQ((IRQn)(EIC_0_IRQn + exintNumber));

	return true;
}

void DetachInterrupt(Pin pin)
{
	if (pin <= ARRAY_SIZE(PinTable))
	{
		const unsigned int exintNumber = PinTable[pin].exintNumber;
		if (exintNumber < ARRAY_SIZE(pinUsingExint) && pinUsingExint[exintNumber] == pin)
		{
			const unsigned int shift = (exintNumber & 7u) << 2u;
			const uint32_t mask = ~(0x0000000F << shift);
			if (exintNumber < 8)
			{
				EIC->CONFIG[0].reg &= mask;
			}
			else
			{
				EIC->CONFIG[1].reg &= mask;
			}

			// Disable the interrupt
			hri_eic_clear_INTEN_reg(EIC, 1ul << exintNumber);

			// Switch the pin out of EIC mode
			gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_OFF);

			exintCallbacks[exintNumber].func = nullptr;
			pinUsingExint[exintNumber] = NoPin;
		}
	}
}

// Common EXINT handler
static inline void CommonExintHandler(size_t exintNumber)
{
	const InterruptCallback& cb = exintCallbacks[exintNumber];
	if (cb.func != nullptr)
	{
		cb.func(cb.param);
	}
}

extern "C" void EIC_0_Handler(void)
{
	CommonExintHandler(0);
}

extern "C" void EIC_1_Handler(void)
{
	CommonExintHandler(1);
}

extern "C" void EIC_2_Handler(void)
{
	CommonExintHandler(2);
}

extern "C" void EIC_3_Handler(void)
{
	CommonExintHandler(3);
}

extern "C" void EIC_4_Handler(void)
{
	CommonExintHandler(4);
}

extern "C" void EIC_5_Handler(void)
{
	CommonExintHandler(5);
}

extern "C" void EIC_6_Handler(void)
{
	CommonExintHandler(6);
}

extern "C" void EIC_7_Handler(void)
{
	CommonExintHandler(7);
}

extern "C" void EIC_8_Handler(void)
{
	CommonExintHandler(8);
}

extern "C" void EIC_9_Handler(void)
{
	CommonExintHandler(9);
}

extern "C" void EIC_10_Handler(void)
{
	CommonExintHandler(10);
}

extern "C" void EIC_11_Handler(void)
{
	CommonExintHandler(11);
}

extern "C" void EIC_12_Handler(void)
{
	CommonExintHandler(12);
}

extern "C" void EIC_13_Handler(void)
{
	CommonExintHandler(13);
}

extern "C" void EIC_14_Handler(void)
{
	CommonExintHandler(14);
}

extern "C" void EIC_15_Handler(void)
{
	CommonExintHandler(15);
}

// End
