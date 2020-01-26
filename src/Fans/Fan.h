/*
 * Fan.h
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#ifndef SRC_FAN_H_
#define SRC_FAN_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"
#include "GCodes/GCodeResult.h"

class GCodeBuffer;
class CanMessageFanParameters;

class Fan
{
public:
	Fan(unsigned int fanNum);

	virtual bool Check() = 0;								// update the fan PWM returning true if it is a thermostatic fan that is on
	virtual void SetPwmFrequency(PwmFrequency freq) = 0;
	virtual bool IsEnabled() const = 0;
	virtual int32_t GetRPM() = 0;
	virtual void ReportPortDetails(const StringRef& str) const = 0;
	virtual ~Fan() { }

	unsigned int GetNumber() const { return fanNumber; }

	// Set or report the parameters for this fan
	// If 'mCode' is an M-code used to set parameters (which should only ever be 106 or 107)
	// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
	// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
	// If no relevant parameters are found, print the existing ones to 'reply' and return false.
	GCodeResult Configure(const CanMessageFanParameters& msg, const StringRef& reply);
	bool IsConfigured() const { return isConfigured && IsEnabled(); }

	float GetConfiguredPwm() const { return val; }			// returns the configured PWM. Actual PWM may be different, e.g. due to blipping or for thermostatic fans.

	void SetPwm(float speed);
	bool HasMonitoredSensors() const { return !sensorsMonitored.IsEmpty(); }

protected:
	virtual void Refresh() = 0;
	virtual bool UpdateFanConfiguration(const StringRef& reply) = 0;

	unsigned int fanNumber;

	// Variables that control the fan
	float val;
	float lastVal;
	float minVal;
	float maxVal;
	float triggerTemperatures[2];
	uint32_t blipTime;										// in milliseconds
	SensorsBitmap sensorsMonitored;

	bool isConfigured;
};

#endif /* SRC_FAN_H_ */
