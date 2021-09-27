
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
 * Polarity Detection
 * ------------------
 *
 * Relative:
 * Absolute:
 *  - Increase the step phase from 0 to 4096
 *  - If the raw encoder reading has increased, reverse_polarity = false
 *  -   Otherwise, reverse_polarity = true
 *
 */

static int32_t startEncoderReading;

static bool PolarityDetection(bool firstIteration) noexcept
{
	if (firstIteration) {
		tuneCounter = 0;
		ClosedLoop::reversePolarity = false;
		startEncoderReading = ClosedLoop::encoder->GetReading();
	}

	if (tuneCounter < 4096) {
		// Calculate the current desired step phase, and move the motor
		ClosedLoop::desiredStepPhase = tuneCounter % 4096;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
	} else {
		// We are finished, calculate the correct polarity
		ClosedLoop::reversePolarity = ClosedLoop::encoder->GetReading() <= startEncoderReading;
		return true;
	}

	tuneCounter += PHASE_STEP_DISTANCE;
	return false;
}


/*
 * Polarity Detection
 * ------------------
 *
 * Absolute:
 * 	- Move to a number of 'target positions'
 * 	- At each target position, record the current encoder reading
 * 	- Store this reading in the encoder LUT
 *
 * Relative:
 *  - Decrease step phase from 4096 to 0
 *  - Wait for 1/4 of a cycle to allow motor position to settle
 *  - Set the current position as the zero position
 *  TODO: A better procedure might be to record the offset as we
 *        move from 4096 to 0, then set the zero position as the
 *        average of this offset.
 *
 */

static AS5047D* absoluteEncoder;
int targetPosition;
int positionCounter;

static bool ZeroingAbsolute(bool firstIteration) noexcept
{
	if (firstIteration) {
		absoluteEncoder = (AS5047D*) ClosedLoop::encoder;
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

static bool ZeroingRelative(bool firstIteration) noexcept
{
	if (firstIteration) {
		// This is the first run
		tuneCounter = 4096 + 1024;	// 4096 phase movement + 1024 delay
	}

	if (tuneCounter > 1024) {
		// Until we reach 1024, move
		// (When < 1024 but >0 do nothing to allow motor to settle)
		ClosedLoop::desiredStepPhase = tuneCounter - 1024;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
	} else if (tuneCounter < 0) {
		// We are done, calculate where the motor has moved to
		int32_t zeroPosition = ClosedLoop::encoder->GetReading();

		// Set this as the new zero position
		((RelativeEncoder*) ClosedLoop::encoder)->SetOffset(-zeroPosition);
		ClosedLoop::targetMotorSteps = ClosedLoop::targetMotorSteps - (zeroPosition / ClosedLoop::encoderPulsePerStep);

		// Return that we are done
		return true;
	}

	// Decrement the counter and return that we haven't finished
	tuneCounter -= PHASE_STEP_DISTANCE;
	return false;
}


/*
 * Polarity Check
 * ------------------
 *
 * Absolute:
 * Relative:
 *  - Move from position 0 to 4096
 *  - At each position, compare the current position to the intended position
 *  - If the error exceeds maxAllowedError, maxAllowedDeviations number of times, fail
 *  - Otherwise, succeed
 *
 */

static size_t numberOfDeviations;
static constexpr uint32_t maxAllowedError = 409;	// ~= 10% of 4095
static constexpr size_t maxAllowedDeviations = 10;	// Allow up to 10 errors


static bool PolarityCheck(bool firstIteration) noexcept
{
	if (firstIteration) {
		tuneCounter = 0;
		numberOfDeviations = 0;
	}

	if (tuneCounter < 4096) {
		// Calculate where the motor has moved to
		int16_t distance1 = ClosedLoop::stepPhase - ClosedLoop::desiredStepPhase;
		int16_t distance2 = 4095 - max(ClosedLoop::stepPhase, ClosedLoop::desiredStepPhase) + min(ClosedLoop::stepPhase, ClosedLoop::desiredStepPhase);

		// Check the error in the movement
		numberOfDeviations += (abs(distance1) > maxAllowedError && abs(distance2) > maxAllowedError);

		// Move the motor for the next iteration
		ClosedLoop::desiredStepPhase = tuneCounter;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1);
	} else {
		// We are finished - check the number of deviations (Allow a small number of deviations)
		if (numberOfDeviations > maxAllowedDeviations) {
			ClosedLoop::tuningError |= ClosedLoop::TUNE_ERR_INCORRECT_POLARITY;
		} else {
			ClosedLoop::tuningError &= ~ClosedLoop::TUNE_ERR_INCORRECT_POLARITY;
		}
		return true;
	}

	tuneCounter += PHASE_STEP_DISTANCE;
	return false;
}


/*
 * Control Check
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

static bool ControlCheck(bool firstIteration) noexcept
{
	return true;
}


/*
 * Encoder Steps Check
 * -------------
 *
 * Absolute:
 * Relative:
 *  - TODO!
 *
 */

static bool EncoderStepsCheck(bool firstIteration) noexcept
{
	return true;
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
	ClosedLoop::targetMotorSteps = ClosedLoop::currentMotorSteps + 4;
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

static bool newTuningMove = true;				// Indicates if a tuning move has just finished

void ClosedLoop::PerformTune() noexcept
{
	if (!tuning) {return;}

	// Check we are in direct drive mode
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct) {
		tuningError |= TUNE_ERR_SYSTEM_ERROR;
		tuning = 0;
		return;
	}

	// Run one iteration of the one, highest priority, tuning move
	if (tuning & POLARITY_DETECTION_MANOEUVRE) {
		// If we are soon to do a zeroing move, and we are using an abs encoder, clear the encoder LUT
		if (encoder != nullptr
				&& encoder->GetPositioningType() == EncoderPositioningType::absolute
				&& (tuning & ZEROING_MANOEUVRE)) {
			((AS5047D*) encoder)->ClearLUT();
		}
		newTuningMove = PolarityDetection(newTuningMove);
		if (newTuningMove) {
			tuningError &= ~TUNE_ERR_NOT_FOUND_POLARITY;
			tuning &= ~POLARITY_DETECTION_MANOEUVRE;
		}
	} else if (tuning & ZEROING_MANOEUVRE) {
		if (encoder == nullptr) {
			tuningError |= TUNE_ERR_SYSTEM_ERROR;
			newTuningMove = true;
		} else if (encoder->GetPositioningType() == EncoderPositioningType::absolute) {
			newTuningMove = ZeroingAbsolute(newTuningMove);
		} else {
			newTuningMove = ZeroingRelative(newTuningMove);
		}
		if (newTuningMove) {
			tuningError &= ~TUNE_ERR_NOT_ZEROED;
			tuning &= ~ZEROING_MANOEUVRE;
		}
	} else if (tuning & POLARITY_CHECK) {
		newTuningMove = PolarityCheck(newTuningMove);
		if (newTuningMove) {
			tuningError &= ~TUNE_ERR_NOT_CHECKED_POLARITY;
			tuning &= ~POLARITY_CHECK;
		}
	} else if (tuning & CONTROL_CHECK) {
		newTuningMove = ControlCheck(newTuningMove);
		if (newTuningMove) {
			tuning &= ~CONTROL_CHECK;
			tuningError &= ~TUNE_ERR_NOT_CHECKED_CONTROL;
		}
	} else if (tuning & ENCODER_STEPS_CHECK) {
		newTuningMove = EncoderStepsCheck(newTuningMove);
		if (newTuningMove) {
			tuning &= ~ENCODER_STEPS_CHECK;
			tuningError &= ~TUNE_ERR_NOT_CHECKED_ENCODER_STEPS;
		}
	} else if (tuning & CONTINUOUS_PHASE_INCREASE_MANOEUVRE) {
		newTuningMove = ContinuousPhaseIncrease(newTuningMove);
		if (newTuningMove) {
			tuning &= ~CONTINUOUS_PHASE_INCREASE_MANOEUVRE;
		}
	} else if (tuning & STEP_MANOEUVRE) {
		newTuningMove = Step(newTuningMove);
		if (newTuningMove) {
			tuning &= ~STEP_MANOEUVRE;
		}
	} else if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {
		newTuningMove = ZieglerNichols(newTuningMove);
		if (newTuningMove) {
			tuning &= ~ZIEGLER_NICHOLS_MANOEUVRE;
		}
	}

}

#endif	// #if SUPPORT_CLOSED_LOOP
