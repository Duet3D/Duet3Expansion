/*
 * AnalogOut.cpp
 *
 *  Created on: 9 Jul 2019
 *      Author: David
 */

#include "AnalogOut.h"
#include "IoPorts.h"

#include "hri_tc_e51.h"
#include "hri_tcc_e51.h"
#include "hri_mclk_e51.h"

namespace AnalogOut
{

	// Convert a float in 0..1 to unsigned integer in 0..N
	static inline uint32_t ConvertRange(float f, uint32_t top)
	pre(0.0 <= ulValue; ulValue <= 1.0)
	post(result <= top)
	{
		return lrintf(f * (float)(top + 1));
	}

	// Choose the most appropriate prescaler for the PWM frequency we want.
	// Some TCs share a clock selection, so we always use GCLK1 as the clock
	// 'counterBits' is either 16 or 8
	static uint32_t ChoosePrescaler(uint16_t freq, unsigned int counterBits, uint16_t& top)
	{
		static const unsigned int PrescalerShifts[] = { 0, 1, 2, 3, 4, 6, 8, 10 };		// available prescalers are 1 2 4 8 16 64 256 1024
		for (uint32_t i = 0; i < ARRAY_SIZE(PrescalerShifts); ++i)
		{
			if ((SystemPeripheralClock >> (PrescalerShifts[i] + counterBits)) <= (uint32_t)freq)
			{
				top = (uint16_t)((SystemPeripheralClock >> PrescalerShifts[i])/(uint32_t)freq - 1);
				return i;
			}
		}
		top = (1ul << counterBits) - 1;
		return ARRAY_SIZE(PrescalerShifts) - 1;
	}

	static void EnableTcClock(unsigned int device)
	{
		static constexpr uint8_t TcClockIDs[] = { TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID };

		hri_gclk_write_PCHCTRL_reg(GCLK, TcClockIDs[device], GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

		switch (device)
		{
		case 0:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0; break;
		case 1:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1; break;
		case 2:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2; break;
		case 3:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3; break;
		case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
		case 5:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5; break;
		}
	}

	// Write PWM to the specified TC device. 'output' may be 0 or 1.
	static bool AnalogWriteTc(Pin pin, unsigned int device, unsigned int output, float val, PwmFrequency freq)
	{
		static Tc* const TcDevices[] = { TC0, TC1, TC2, TC3, TC4, TC5 };		// TC6 and TC7 are reserved for the step clock
		static uint16_t tcFreq[ARRAY_SIZE(TcDevices)] = { 0 };
		static uint16_t tcTop[ARRAY_SIZE(TcDevices)] = { 0 };

		if (device < ARRAY_SIZE(TcDevices))
		{
			if (freq == 0)
			{
				tcFreq[device] = freq;
				return false;
			}

			Tc * const tcdev = TcDevices[device];
			if (freq != tcFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, 16, tcTop[device]);
				if (output == 0)
				{
					// We need to use CC0 for the compare output, so we can't use it to define TOP. We will get a lower frequency than requested.
					// TODO see if we can use 8-bit mode instead
					tcTop[device] = 0xFFFF;
				}

				const uint16_t cc = ConvertRange(val, (uint32_t)tcTop[device]);

				if (tcFreq[device] == 0)
				{
					EnableTcClock(device);

					// Initialise the TC
					hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
					hri_tc_set_CTRLA_SWRST_bit(tcdev);
					hri_tc_set_CTRLA_PRESCALER_bf(tcdev, prescaler);
					hri_tc_set_CTRLA_MODE_bf(tcdev, TC_CTRLA_MODE_COUNT16_Val);
					if (output == 0)
					{
						hri_tc_write_WAVE_WAVEGEN_bf(tcdev, TC_WAVE_WAVEGEN_NPWM_Val);
					}
					else
					{
						hri_tc_write_WAVE_WAVEGEN_bf(tcdev, TC_WAVE_WAVEGEN_MPWM_Val);
						hri_tccount16_write_CC_CC_bf(tcdev, 0, tcTop[device]);
						hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, 0, tcTop[device]);
					}
					hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, output, cc);
					hri_tccount16_write_CC_CC_bf(tcdev, output, cc);

					hri_tc_set_CTRLA_ENABLE_bit(tcdev);
					gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_E);			// TCs are all on peripheral select E
				}
				else
				{
					hri_tc_set_CTRLA_PRESCALER_bf(tcdev, prescaler);
					if (output != 0)
					{
						hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, 0, tcTop[device]);
					}
					hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, output, cc);
				}
				tcFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				const uint16_t cc = ConvertRange(val, (uint32_t)tcTop[device]);
				hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, output, cc);
			}
			return true;
		}
		return false;
	}

	static void EnableTccClock(unsigned int device)
	{
		static constexpr uint8_t TccClockIDs[] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID };

		hri_gclk_write_PCHCTRL_reg(GCLK, TccClockIDs[device], GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

		switch (device)
		{
		case 0:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0; break;
		case 1:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1; break;
		case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2; break;
		case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC3; break;
		case 4:	MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4; break;
		}
	}

	// Write PWM to the specified TCC device. 'output' may be 0..5.
	static bool AnalogWriteTcc(Pin pin, unsigned int device, unsigned int output, unsigned int peri, float val, PwmFrequency freq)
	{
		static Tcc* const TccDevices[] = { TCC0, TCC1, TCC2, TCC3, TCC4 };
		static uint16_t tccFreq[ARRAY_SIZE(TccDevices)] = { 0 };
		static uint16_t tccTop[ARRAY_SIZE(TccDevices)] = { 0 };

		if (device < ARRAY_SIZE(TccDevices))
		{
			if (freq == 0)
			{
				tccFreq[device] = freq;
				return false;
			}

			Tcc * const tccdev = TccDevices[device];
			if (freq != tccFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, 16, tccTop[device]);
				const uint16_t cc = ConvertRange(val, (uint32_t)tccTop[device]);

				if (tccFreq[device] == 0)
				{
					EnableTccClock(device);

					// Initialise the TCC
					hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
					hri_tcc_set_CTRLA_SWRST_bit(tccdev);
					hri_tcc_set_CTRLA_PRESCALER_bf(tccdev, prescaler);
					hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM_Val);
					hri_tcc_write_PERBUF_PERBUF_bf(tccdev, tccTop[device]);
					hri_tcc_write_PER_PER_bf(tccdev, tccTop[device]);
					hri_tcc_write_CCBUF_CCBUF_bf(tccdev, output, cc);
					hri_tcc_write_CC_CC_bf(tccdev, output, cc);

					hri_tcc_set_CTRLA_ENABLE_bit(tccdev);
					hri_tcc_wait_for_sync(tccdev, TCC_SYNCBUSY_MASK);
					hri_tcc_write_COUNT_reg(tccdev, 0);							// if we don't do this then there may be a 5 second delay before PWM starts
					gpio_set_pin_function(pin, peri);
				}
				else
				{
					hri_tcc_set_CTRLA_PRESCALER_bf(tccdev, prescaler);
					hri_tcc_write_PERBUF_PERBUF_bf(tccdev, tccTop[device]);
					hri_tcc_write_PER_PER_bf(tccdev, tccTop[device]);
					hri_tcc_write_CCBUF_CCBUF_bf(tccdev, output, cc);
					hri_tcc_write_CC_CC_bf(tccdev, output, cc);
					hri_tcc_write_COUNT_reg(tccdev, 0);
				}
				tccFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				const uint16_t cc = ConvertRange(val, (uint32_t)tccTop[device]);
				hri_tcc_write_CCBUF_CCBUF_bf(tccdev, output, cc);
			}
			return true;
		}
		return false;
	}
}

// Initialise this module
void AnalogOut::Init()
{
	// Nothing to do yet
}

// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float val, PwmFrequency freq)
{
	if (pin >= ARRAY_SIZE(PinTable) || std::isnan(val))
	{
		return;
	}

	val = constrain<float>(val, 0.0, 1.0);

	{
		const TcOutput tc = PinTable[pin].tc;
		if (tc != TcOutput::none)
		{
			if (AnalogWriteTc(pin, GetDeviceNumber(tc), GetOutputNumber(tc), val, freq))
			{
				return;
			}
		}
	}

	{
		const TccOutput tcc = PinTable[pin].tcc;
		if (tcc != TccOutput::none)
		{
			if (AnalogWriteTcc(pin, GetDeviceNumber(tcc), GetOutputNumber(tcc), GetPeriNumber(tcc), val, freq))
			{
				return;
			}
		}
	}

	// Fall back to digital write
	IoPort::SetPinMode(pin, (val < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
