/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"

#include "CanMessageFormats.h"

Fan::Fan(unsigned int fanNum) noexcept
	: fanNumber(fanNum),
	  val(0.0), lastVal(0.0),
	  minVal(DefaultMinFanPwm),
	  maxVal(1.0),										// 100% maximum fan speed
	  blipTime(DefaultFanBlipTime)
{
	triggerTemperatures[0] = triggerTemperatures[1] = DefaultHotEndFanTemperature;
}

// Set the parameters for this fan
GCodeResult Fan::Configure(const CanMessageFanParameters& msg, const StringRef& reply) noexcept
{
	triggerTemperatures[0] = msg.triggerTemperatures[0];
	triggerTemperatures[1] = msg.triggerTemperatures[1];
	blipTime = msg.blipTime;
	val = msg.val;
	minVal = msg.minVal;
	maxVal = msg.maxVal;
	sensorsMonitored.SetFromRaw(msg.sensorsMonitored);
	(void)UpdateFanConfiguration(reply);
	return GCodeResult::ok;
}

// Set the PWM. 'speed' is in the interval 0.0..1.0.
void Fan::SetPwm(float speed) noexcept
{
	val = speed;
	Refresh(true);
}

// End
