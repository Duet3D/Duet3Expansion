/*
 * InductiveHeaterPort.cpp
 *
 *  Created on: 29 Dec 2025
 *      Author: David
 */

#include "InductiveHeaterPort.h"

#if SUPPORT_INDUCTIVE_HEATER

#include <Platform/Platform.h>
#include <Timers.h>

#if SAME5x
# include <hri_tc_e54.h>
# include <hri_tcc_e54.h>
# include <hri_mclk_e54.h>
#elif SAMC21
# include <hri_tc_c21.h>
# include <hri_tcc_c21.h>
# include <hri_mclk_c21.h>
#else
# error Unsupported processor
#endif

InductiveHeaterPort::InductiveHeaterPort()
{
	// Nothing to do here
}

void InductiveHeaterPort::Init() noexcept
{
	// Set up the oscillator TCC to generate a square wave at the coil resonant frequency
	EnableTccClock(InductiveHeaterOscTccDeviceNumber, GclkNum120MHz);	// use the 120MHz GCLK to get the best frequency setting resolution
	const uint32_t oscPrescaler = 0;									// 16-or 24-bit TCC with a 120MHz clock and prescaler 1 gives us frequencies from 1.8kHz upwards
	oscTimerPeriod = 120'000'000/OscResonantFrequency;
	const uint32_t oscMarkCount = (uint32_t)((float)oscTimerPeriod * OscMarkSpaceRatio);

	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccosc);
	hri_tcc_set_CTRLA_SWRST_bit(tccosc);
	tccosc->CTRLA.bit.PRESCALER = oscPrescaler;
	tccosc->CTRLA.bit.RESOLUTION = 0;
	hri_tcc_write_WAVE_WAVEGEN_bf(tccosc, TCC_WAVE_WAVEGEN_NPWM_Val);
	tccosc->PERBUF.bit.PERBUF = oscTimerPeriod - 1;
	tccosc->PER.bit.PER = oscTimerPeriod - 1;
	tccosc->CCBUF[InductiveHeaterOscTccOutputNumber].bit.CCBUF = oscMarkCount - 1;
	tccosc->CC[InductiveHeaterOscTccOutputNumber].bit.CC = oscMarkCount - 1;

	hri_tcc_set_CTRLA_ENABLE_bit(tccosc);

	// Set up the PWM TCC to generate the PWM.
	// We sync it to the oscillator TCC to avoid short pulses at the start or end of a PWM period.
	// To do this we use the same GCLK and prescaler as the oscillator TCC and just multiple the period by a suitable integer.
	// Then we always set the counter match value to a multiple of the oscillator period. Because of this, we need a 24-bit TCC.
	EnableTccClock(InductiveHeaterPwmTccDeviceNumber, GclkNum120MHz);
	const uint32_t pwmPrescaler = 0;
	pwmTimerPeriod = oscTimerPeriod * PwmFrequencyDivisor;

	volatile Tcc *const tccpwm = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccpwm);
	hri_tcc_set_CTRLA_SWRST_bit(tccpwm);

	tccpwm->CTRLA.bit.PRESCALER = pwmPrescaler;
	tccpwm->CTRLA.bit.RESOLUTION = 0;
	hri_tcc_write_WAVE_WAVEGEN_bf(tccpwm, TCC_WAVE_WAVEGEN_NPWM);

	tccpwm->PERBUF.bit.PERBUF = pwmTimerPeriod - 1;
	tccpwm->PER.bit.PER = pwmTimerPeriod - 1;
	tccpwm->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = 0;
	tccpwm->CC[InductiveHeaterPwmTccOutputNumber].bit.CC = 0;

	hri_tcc_set_CTRLA_ENABLE_bit(tccpwm);

	// Retrigger them both to synchronise them.
	{
		AtomicCriticalSectionLocker lock;
		tccpwm->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
		tccosc->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
	}

	// Set up the CCL to gate them together
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;								// enable the CCL APB clock
	CCL->CTRL.reg = 0;														// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

	// Currently we don't need to provide a clock to the CCL because we don't use input events, a filter, edge detection or sequential logic.
	// If we do start using any of these then we need to provide the CCL with a clock, max 100MHz.

	// Set up CCL0 to just copy TCC0 WO[0] to the output so that we can use it as an input to CCL3
	constexpr uint32_t Lut0RegValue =  CCL_LUTCTRL_TRUTH(1u << 1)						// just copy INSEL0
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC output 0
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_MASK_Val)	// INSEL1 not used
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue;
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue | CCL_LUTCTRL_ENABLE;

	// Set up CCL3 to AND the TCC3 WO[1] and the output from CCL0 together
	constexpr uint32_t Lut3RegValue = CCL_LUTCTRL_TRUTH(1u << 3)						// AND of INSEL0 and INSEL11
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC3.0 output
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_LINK_Val)	// INSEL1 from CCL1
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue;
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue | CCL_LUTCTRL_ENABLE;
	CCL->CTRL.reg = CCL_CTRL_ENABLE;										// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

	// Finally, set the FET drive pin to be the CCL3 output
	SetPinFunction(InductiveHeaterCCLOutPin, InductiveHeaterCCLOutPinPeriphMode);
}

// Set the PWM value in the range 0..1
void InductiveHeaterPort::SetPwm(float pwm) noexcept
{
	const uint32_t ccIdeal = (uint32_t)(pwm * (float)pwmTimerPeriod);
	const uint32_t cc = ccIdeal - (ccIdeal % oscTimerPeriod);
	volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = cc;
}

#endif

// End
