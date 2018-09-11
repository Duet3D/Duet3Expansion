/*
 * ZLeadscrewKinematics.cpp
 *
 *  Created on: 8 Jul 2017
 *      Author: David
 */

#include "ZLeadscrewKinematics.h"

const float M3ScrewPitch = 0.5;

ZLeadscrewKinematics::ZLeadscrewKinematics(KinematicsType k)
	: Kinematics(k, -1.0, 0.0, true), numLeadscrews(0), correctionFactor(1.0), maxCorrection(1.0), screwPitch(M3ScrewPitch)
{
}

ZLeadscrewKinematics::ZLeadscrewKinematics(KinematicsType k, float segsPerSecond, float minSegLength, bool doUseRawG0)
	: Kinematics(k, segsPerSecond, minSegLength, doUseRawG0), numLeadscrews(0), correctionFactor(1.0), maxCorrection(1.0), screwPitch(M3ScrewPitch)
{
}

// End
