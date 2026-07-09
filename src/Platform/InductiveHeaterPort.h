/*
 * InductiveHeaterPort.h
 *
 *  Created on: 29 Dec 2025
 *      Author: David
 */

#ifndef SRC_PLATFORM_INDUCTIVEHEATERPORT_H_
#define SRC_PLATFORM_INDUCTIVEHEATERPORT_H_

#include <RepRapFirmware.h>

#if SUPPORT_INDUCTIVE_HEATER

class InductiveHeaterPort
{
public:
	InductiveHeaterPort() noexcept;
	void Init() noexcept;
	void SetPwm(float pwm) noexcept;									// pwm is a fraction in [0,1]

private:
	static constexpr uint32_t OscResonantFrequency = 120'000;			// the coil resonant frequency is higher than this but the coil is actually driven like a flyback converter
	static constexpr uint32_t PwmFrequencyDivisor = 512;				// high enough for good resolution, low enough for fast response (PWM frequency = 120000/512 = 234Hz)
	static constexpr float OscMarkSpaceRatio = 0.4;						// the mark/space ratio for the heater FET drive

	uint32_t oscTimerPeriod;
	uint32_t pwmTimerPeriod;
};

#endif

#endif /* SRC_PLATFORM_INDUCTIVEHEATERPORT_H_ */
