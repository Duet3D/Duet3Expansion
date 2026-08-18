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

#include <Platform/InductiveHeaterCalibrationParameters.h>
#include <RTOSIface/RTOSIface.h>

// Note that this class relies on static variables, so we should not create more than one instance of it!
class InductiveHeaterPort
{
public:
	InductiveHeaterPort() noexcept;
	void Init() noexcept;
	void SetPwm(float pwm) noexcept;													// pwm is a fraction in [0,1]
	GCodeResult Calibrate(bool start, const StringRef& reply) noexcept;					// start calibrating the heater or check whether calibration has completed
	bool IsCalibrated() const noexcept { return isCalibrated; }
	bool HasFaulted() const noexcept { return hasFaulted; }
	void ClearFault() noexcept;

	static void TurnOff() noexcept;														// this is called by the ISR - NOTE FOR_GENERAL USE
	static bool hasFaulted;																// set if we have detected over-voltage and turned the heater off NOT FOR GENERAL USE OUTSIDE THIS MODULE

private:
	// Constants to define the oscillator parameters
	static constexpr uint32_t OscClockFrequencyMHz = 120;								// the frequency at which the oscillator timer is clocked
	static constexpr uint32_t PwmOnCountOff = 0x00FFFFFF;								// value to pass to SetupOscillator to ensure heater is off

	// The following values are expressed in microseconds converted to clocks. Typical values after calibration are:
	// firstOnTime 290 (2.42us), mainOnTime 450 (3.75us), offTime 437 (3.64us)
	static constexpr uint32_t OscMinOnTime = (uint32_t)(1.5 * OscClockFrequencyMHz);
	static constexpr uint32_t OscMaxFirstOnTime = (uint32_t)(3.5 * OscClockFrequencyMHz);
	static constexpr uint32_t OscDefaultMainOnTime = (uint32_t)(2.5 * OscClockFrequencyMHz);
	static constexpr uint32_t OscMaxMainOnTime = (uint32_t)(5.5 * OscClockFrequencyMHz);
	static constexpr uint32_t OscMinOffTime = (uint32_t)(3.0 * OscClockFrequencyMHz);	// the flyback time is usually about 3.5us so set a minimum of 3.0us
	static constexpr uint32_t OscMaxOffTime = (uint32_t)(4.6 * OscClockFrequencyMHz);	// long enough to always contain a full half cycle, not so long that the drain voltage goes backup to +24V
	static constexpr uint32_t OscOnTimeStep = 2;
	static constexpr uint32_t OnTimeBackoff = 16;										// how much we back off the mosfet on-time to allow a margin before overvoltage detection

	static constexpr uint32_t PwmFrequencyDivisor = 512;								// high enough for good resolution, low enough for fast response (PWM frequency = 120000/512 = 234Hz)

	// Calibration task
	static constexpr unsigned int CalibrationTaskStackWords = 300;						// calibration task stack size
	static Task<CalibrationTaskStackWords> *_ecv_null calibrationTask;

	[[noreturn]] static void CalibrationTaskEntry(void *pv) noexcept;
	[[noreturn]] void CalibrationTaskFunc() noexcept;									// function executed by the calibration task

	void SetupOscillator(uint32_t pwmOnCount = PwmOnCountOff) noexcept;					// set up the timing parameters from firstCycleLength, laterCycleLength and offTime; pwmOnCount defaults to heater off
	void CalibrateHeater() noexcept;
	void SetBurst(uint32_t burstLength) noexcept;

	// Calibration variables
	InductiveHeaterCalibrationParameters calibrationParams;
	uint32_t pwmTimerPeriod;
	uint32_t lastCc = 0;
	const char *volatile calibrationFailedReason = "calibration not run";
	bool isCalibrated = false;															// set if calibration has been run or valid parameters have been read from NVM
};

#endif

#endif /* SRC_PLATFORM_INDUCTIVEHEATERPORT_H_ */
