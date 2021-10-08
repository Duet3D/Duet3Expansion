
#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

# include "AS5047D.h"
# include "AbsoluteEncoder.h"
# include "RelativeEncoder.h"

# if SUPPORT_TMC2160
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support closed loop with the specified hardware
# endif

static int32_t tuneCounter;							// A counter for tuning tasks to use
static constexpr uint32_t PHASE_STEP_DISTANCE = 8;	// The distance of one small step, essentially a speed control.

/*
 * The following sections, delimited by comments, implement different tuning moves.
 * The comment gives the name of the move, and a description for how it is implemented
 * for both relative and absolute encoders.
 * Each function should perform one 'iteration' of the tuning move, and return true if
 * the iteration was it's last. It will also be supplied with an argument representing
 * if this is it's first iteration.
 *
 * At the bottom of the file, ClosedLoop::PerformTune() is implemented to take advantage
 * of these function
 */


/*
 * Basic tuning
 * ------------------
 *
 *  - Increase the step phase from 0 to 4096
 *  - If the raw encoder reading has increased, reverse_polarity = false
 *  -   Otherwise, reverse_polarity = true
 *
 */

static bool BasicTuning(bool firstIteration) noexcept
{
	static int32_t startEncoderReading;
	static float forwardCountsPerStep;
	static uint16_t initialStepPhase;

	if (firstIteration) {
		tuneCounter = 0;
		ClosedLoop::SetForwardPolarity();
		initialStepPhase = ClosedLoop::desiredStepPhase;
	}

	//TODO rewrite this as a state machine
	//TODO remove accesses to desiredStepPhase, call a function in ClosedLoop to adjust it and set the current instead
	if (tuneCounter == 0 && ClosedLoop::desiredStepPhase != 0) {
		// Go back to phase 0. Note that ClosedLoop::desiredStepPhase in this context is the actual current step phase.
		ClosedLoop::desiredStepPhase -= min<uint16_t>(ClosedLoop::desiredStepPhase, PHASE_STEP_DISTANCE);
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
	} else if (tuneCounter < 4096) {
		if (tuneCounter == 0) {
			startEncoderReading = ClosedLoop::encoder->GetReading();
		}
		// Calculate the current desired step phase, and move the motor
		ClosedLoop::desiredStepPhase = tuneCounter;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
		tuneCounter += PHASE_STEP_DISTANCE;
	} else if (tuneCounter < 8192) {
		if (tuneCounter == 4096) {
			const int32_t reading = ClosedLoop::encoder->GetReading();
			forwardCountsPerStep = (float)(reading - startEncoderReading) * 0.25;
			startEncoderReading = reading;
		}
		// Calculate the current desired step phase, and move the motor
		ClosedLoop::desiredStepPhase = 8192 - tuneCounter;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
		tuneCounter += PHASE_STEP_DISTANCE;
	} else {
		if (tuneCounter == 8192) {
			// We are finished, calculate the correct polarity
			const int32_t reading = ClosedLoop::encoder->GetReading();
			const float reverseCountsPerStep = (float)(startEncoderReading - reading) * 0.25;
			ClosedLoop::SetBasicTuningResults(forwardCountsPerStep, reverseCountsPerStep, reading);
			++tuneCounter;		// so that we only do the above once
		}
		if (ClosedLoop::desiredStepPhase < initialStepPhase) {
			// Go forward to the original phase
			const uint16_t distanceToGo = initialStepPhase - ClosedLoop::desiredStepPhase;
			ClosedLoop::desiredStepPhase += min<uint16_t>(distanceToGo, PHASE_STEP_DISTANCE);
			ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
		} else {
			ClosedLoop::ResetStepPosition(initialStepPhase);
			return true;
		}
	}

	return false;
}


/*
 * Magnetic encoder calibration
 * ----------------------------
 *
 * Absolute:
 * 	- Move to a number of 'target positions'
 * 	- At each target position, record the current encoder reading
 * 	- Store this reading in the encoder LUT
 */

static bool EncoderCalibration(bool firstIteration) noexcept
{
	static int targetPosition;
	static int positionCounter;

	if (ClosedLoop::encoder->GetPositioningType() == EncoderPositioningType::relative)
	{
		return true;			// we don't do this tuning for relative encoders
	}

	AS5047D* absoluteEncoder = (AS5047D*) ClosedLoop::encoder;
	if (firstIteration) {
		absoluteEncoder->ClearLUT();
		targetPosition = 0;
		positionCounter = 0;
	}

	if (ClosedLoop::rawEncoderReading < targetPosition) {
		positionCounter += 1;
	} else if (ClosedLoop::rawEncoderReading > targetPosition) {
		positionCounter -= 1;
	} else {
		const float realWorldPos = absoluteEncoder->GetMaxValue() * positionCounter / (1024 * (360.0 / ClosedLoop::PulsePerStepToExternalUnits(ClosedLoop::encoderPulsePerStep, EncoderType::AS5047)));
		absoluteEncoder->StoreLUTValueForPosition(ClosedLoop::rawEncoderReading, realWorldPos);
		targetPosition += absoluteEncoder->GetLUTResolution();
	}

	if ((unsigned int) targetPosition >= absoluteEncoder->GetMaxValue()) {
		// We are finished
		absoluteEncoder->StoreLUT();
		return true;
	}

	ClosedLoop::desiredStepPhase = (positionCounter > 0 ? 0 : 4096) + positionCounter % 4096;
	ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
	return false;
}


/*
 * Continuous Phase Increase
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

static bool ContinuousPhaseIncrease(bool firstIteration) noexcept
{
	return true;
}


/*
 * Step
 * -------------
 *
 * Absolute:
 * Relative:
 *  - Increase the target motor steps by 4
 *
 */

static bool Step(bool firstIteration) noexcept
{
	ClosedLoop::targetMotorSteps = ClosedLoop::targetMotorSteps + 4;
	return true;
}


/*
 * Ziegler Nichols Manoeuvre
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

// TODO: Implement ziegler-Nichols move
#if true
static bool ZieglerNichols(bool firstIteration) noexcept
{
	return true;
}

#else
static bool ZieglerNichols(bool firstIteration) noexcept
{

	// We will need to restore these afterwards...
	const float prevKp = Kp;
	const float prevKi = Ki;
	const float prevKd = Kd;

	// Reset the PID controller
	Ki = 0;
	Kd = 0;
	Kp = 0;
	PIDITerm = 0;

	ultimateGain = 0;		// Reset the ultimate gain value
	int direction = 1;		// Which direction are we moving in

	float lowerBound = 0;
	float upperBound = 10000;

	while (upperBound - lowerBound > 100) {

		Kp = lowerBound + (upperBound - lowerBound) / 2;

		targetMotorSteps = currentMotorSteps + (direction * 10);

		// Flip the direction
		direction = -direction;

		unsigned int initialRiseTime = 0;		// The time it takes to initially meet the target

		float peakError = 0;			// The peak of the current oscillation
		float prevPeakError = 0;		// The peak of the previous oscillation
		unsigned int prevTimestamp = 0;			// The previous time of oscillation

		unsigned int oscillationCount = 0;		// The number of oscillations that have occurred

		float ewmaDecayFraction = 0;	// An EWMA of the decay fraction of oscillations
		float ewmaOscillationPeriod = 0;// An EWMA of the oscillation period

		// Run up to a maximum of 4096
		for (unsigned int time=0; time<16384; time++) {
			TaskBase::Take(10);		// TODO: Use delayuntil here? And run at PID frequency

			ControlMotorCurrents();

			float currentPosition = direction * currentMotorSteps;
			float targetPosition = direction * targetMotorSteps;
			float error = abs(currentPosition - targetPosition);

			// Search for the initial rise time
			if (initialRiseTime == 0) {
				if (currentPosition > targetPosition) {
					initialRiseTime = time;
				} else {
					continue;
				}
			}

			// Wait another two initial rise times for oscillations to occur
			if (time < 3 * initialRiseTime) {continue;}

			// We're now in the prime time for oscillations - check if they are actually happening:

			// Record data if we are above the target
			if (currentPosition > targetPosition) {
				peakError = max<float>(peakError, error);
				continue;
			}
			// Process data if we have just crossed the target
			float decayFraction;
			if (peakError > 0) {
				if (prevPeakError > 0) {
					decayFraction = peakError / prevPeakError;
					ewmaDecayFraction =
							ewmaDecayFraction == 0
							? decayFraction
							: 0.7 * ewmaDecayFraction + 0.3 * decayFraction;
					if (oscillationCount > 5) {
						ewmaOscillationPeriod =
								ewmaOscillationPeriod == 0
								? (time - prevTimestamp)
								: 0.3 * ewmaOscillationPeriod + 0.7 * (time - prevTimestamp);
					}
				}
				oscillationCount++;
				prevPeakError = peakError;
				peakError = 0;
				prevTimestamp = time;
			}

			PIDPTerm = ewmaOscillationPeriod;
			PIDDTerm = (time - prevTimestamp);

			// Wait for at least 5 oscillations
			if (oscillationCount < 5) {
				continue;
			}

			// Check that the next 5 oscillations all keep the average decay fraction above 98%
			if (ewmaDecayFraction < 0.98) {
				// No oscillations, this is the new lower bound.
				lowerBound = Kp;
				break;
			}
			if (oscillationCount >= 10) {
				// Oscillations found! This is the new upper bound.
				upperBound = Kp;
				oscillationPeriod = ewmaOscillationPeriod;
				break;
			}

			// If we time out of this loop, assume no oscillations
			if (time == 16383) {
				lowerBound = Kp;
			}

		}
	}

	ultimateGain = upperBound;
	Kp = prevKp;
	Ki = prevKi;
	Kd = prevKd;

	tuning &= ~ZIEGLER_NICHOLS_MANOEUVRE;
}
#endif


/*
 * ClosedLoop::PerformTune()
 * -------------------------
 *
 * Makes use of the above tuning functions.
 *
 */

// This is called from every iteration of the closed loop control loop if tuning is enabled
void ClosedLoop::PerformTune() noexcept
{
	static bool newTuningMove = true;						// indicates if a tuning move has just finished

	// Check we are in direct drive mode and we have an encoder
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct || encoder == nullptr ) {
		tuningError |= TUNE_ERR_SYSTEM_ERROR;
		tuning = 0;
		return;
	}

	// Run one iteration of the one, highest priority, tuning move
	if (tuning & BASIC_TUNING_MANOEUVRE) {
		if (encoder->GetPositioningType() == EncoderPositioningType::absolute && (tuning & ENCODER_CALIBRATION_MANOEUVRE)) {
			((AS5047D*)encoder)->ClearLUT();				//TODO this assumes that any absolute encoder is a AS5047D. Make ClearLUT a virtual method?
		}
		newTuningMove = BasicTuning(newTuningMove);
		if (newTuningMove) {
			tuningError &= ~TUNE_ERR_NOT_DONE_BASIC;
			tuning &= ~BASIC_TUNING_MANOEUVRE;				// we can do encoder calibration after basic tuning
		}
	} else if (tuning & ENCODER_CALIBRATION_MANOEUVRE) {
		newTuningMove = EncoderCalibration(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else if (tuning & CONTINUOUS_PHASE_INCREASE_MANOEUVRE) {
		newTuningMove = ContinuousPhaseIncrease(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else if (tuning & STEP_MANOEUVRE) {
		newTuningMove = Step(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {
		newTuningMove = ZieglerNichols(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else {
		tuning = 0;
		newTuningMove = true;								// ready for next time
	}
}

#endif	// #if SUPPORT_CLOSED_LOOP
