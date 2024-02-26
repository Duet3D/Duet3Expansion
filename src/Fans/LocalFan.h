/*
 * LocalFan.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_LOCALFAN_H_
#define SRC_FANS_LOCALFAN_H_

#include "Fan.h"

class LocalFan : public Fan
{
public:
	LocalFan(unsigned int fanNum) noexcept;
	~LocalFan();

	bool Check(bool checkSensors) noexcept override;					// update the fan PWM returning true if it is a thermostatic fan that is on
	bool IsEnabled() const noexcept override { return port.IsValid(); }
	void SetPwmFrequency(PwmFrequency freq) noexcept override { port.SetFrequency(freq); }
	void SetTachoPulsesPerRev(float ppr) noexcept override;
	int32_t GetRPM() noexcept override;
	void ReportPortDetails(const StringRef& str) const noexcept override;

	bool AssignPorts(const char *pinNames, const StringRef& reply) noexcept;

	void Interrupt() noexcept;

protected:
	void Refresh(bool checkSensors) noexcept override;
	bool UpdateFanConfiguration(const StringRef& reply) noexcept override;

private:
	void SetHardwarePwm(float pwmVal) noexcept;

	PwmPort port;											// port used to control the fan
	IoPort tachoPort;										// port used to read the tacho

	// Variables used to read the tacho
	static constexpr uint32_t TachoMaxInterruptCount = 16;	// number of fan interrupts that we average over
	uint32_t tachoMultiplier;								// what we divide the tacho interrupt interval into to get the RPM
	uint32_t tachoInterruptCount;							// accessed only in ISR, so no need to declare it volatile
	volatile uint32_t tachoLastResetTime;					// time (in step clocks) at which we last reset the interrupt count, accessed inside and outside ISR
	volatile uint32_t tachoInterval;						// written by ISR, read outside the ISR

	uint32_t blipStartTime;
	bool blipping;
};

#endif /* SRC_FANS_LOCALFAN_H_ */
