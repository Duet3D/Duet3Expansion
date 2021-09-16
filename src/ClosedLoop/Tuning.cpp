
#include "ClosedLoop.h"

# include "AS5047D.h"
# include "AbsoluteEncoder.h"
# include "RelativeEncoder.h"

# if SUPPORT_TMC2160
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support closed loop with the specified hardware
# endif

#if SUPPORT_CLOSED_LOOP

void ClosedLoop::PerformTune() noexcept
{
	if (!tuning) {return;}

	constexpr unsigned int stepPhaseDistance = 8;	// Must be a power of 2 < 4096. Represents how much the phase will change each time the motor needs to move.

	// Check we are in direct drive mode
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct) {
		tuningError |= TUNE_ERR_SYSTEM_ERROR;
		tuning = 0;
		return;
	}

	// Do a polarity detection manoeuvre for quadrature encoders
	if (tuning & POLARITY_DETECTION_MANOEUVRE) {

		if (newTuningMove) {
			// This is the first run
			reversePolarity = false;
			tuningVar1 = encoder->GetReading();		// startEncoderReading
			tuneCounter = 0;
			newTuningMove = false;
		}

		if (tuneCounter < 4096) {
			// Calculate the current desired step phase
			desiredStepPhase = tuneCounter % 4096;

			// Move the motor
			SetMotorPhase(desiredStepPhase, 1);
		} else {
			// We are finished

			// Calculate the correct polarity
			if (encoder->GetReading() > tuningVar1) {
				reversePolarity = false;
			} else {
				reversePolarity = true;
			}

			// Set the appropriate flags
			tuningError &= ~TUNE_ERR_NOT_FOUND_POLARITY;
			tuning &= ~POLARITY_DETECTION_MANOEUVRE;
			newTuningMove = true;
		}

		tuneCounter += stepPhaseDistance;
		return;		// If we have done this tuning move, we don't want to do any others
	}

	// Do a zeroing manoeuvre for quadrature encoders
	if ((tuning & ZEROING_MANOEUVRE)
			&& (encoder != nullptr
					&& (encoder->GetType() == EncoderType::rotaryQuadrature
					||  encoder->GetType() == EncoderType::linearQuadrature))) {

		if (newTuningMove) {
			// This is the first run
			tuneCounter = 4096 + 1024;	// 4096 phase movement + 1024 delay
			newTuningMove = false;
		}

		if (tuneCounter > 1024) {
			// We are still moving
			tuneCounter -= stepPhaseDistance;
			desiredStepPhase = tuneCounter - 1024;
			SetMotorPhase(desiredStepPhase, 1);
		} else if (tuneCounter > 0) {
			tuneCounter -= stepPhaseDistance;
		} else {
			// We are done
			// Calculate where the motor has moved to
			int32_t zeroPosition = encoder->GetReading();

			// Set this as the new zero position
			((RelativeEncoder*) encoder)->SetOffset(-zeroPosition);
			targetMotorSteps = targetMotorSteps - (zeroPosition / encoderPulsePerStep);

			// Set the appropriate flags
			newTuningMove = true;
			tuning &= ~ZEROING_MANOEUVRE;
			tuningError &= ~TUNE_ERR_NOT_ZEROED;
		}

		return;		// If we have done this tuning move, we don't want to do any others
	}

	// Do a zeroing manoeuvre for AS5047D encoders
	if ((tuning & ZEROING_MANOEUVRE)
			&& (encoder != nullptr && encoder->GetType() == EncoderType::AS5047)) {

		AS5047D* absoluteEncoder = (AS5047D*) encoder;

		if (newTuningMove) {
			// This is the first run
			absoluteEncoder->ClearLUT();
			tuningVar3 = 0;						// Target position
			tuningVar4 = 0;						// Position counter
			newTuningMove = false;
		}

		if (rawEncoderReading < tuningVar3) {
			tuningVar4 += 1;
		} else if (rawEncoderReading > tuningVar3) {
			tuningVar4 -= 1;
		} else {
			const float realWorldPos = absoluteEncoder->GetMaxValue() * tuningVar4 / (1024 * (360.0 / PulsePerStepToExternalUnits(encoderPulsePerStep, EncoderType::AS5047)));
			absoluteEncoder->StoreLUTValueForPosition(rawEncoderReading, realWorldPos);
			tuningVar3 += absoluteEncoder->GetLUTResolution();
		}

		if (tuningVar3 >= absoluteEncoder->GetMaxValue()) {
			// We are finished
			absoluteEncoder->StoreLUT();
			tuning &= ~ZEROING_MANOEUVRE;
			tuningError &= ~TUNE_ERR_NOT_ZEROED;
			newTuningMove = true;
		}

		desiredStepPhase = (tuningVar4 > 0 ? 0 : 4096) + (int)tuningVar4 % 4096;
		SetMotorPhase(desiredStepPhase, 1);
	}

	// Handle zeroing manoeuvre for other encoder types
	if (tuning & POLARITY_DETECTION_MANOEUVRE) {
		// TODO
	}

	// Do a polarity check manoeuvre
	if (tuning & POLARITY_CHECK) {
		// We are going to step through a full phase, and check that the error never exceeds max_err

		if (newTuningMove) {
			// This is the first run
			tuningVar1 = 409;	// max_err - allow up to ~10% error
			tuningVar2 = 0;		// deviations
			tuneCounter = 0;
			newTuningMove = false;
		}

		if (tuneCounter < 4096) {
			// Calculate where the motor has moved to
			int16_t distance1 = stepPhase - desiredStepPhase;
			int16_t distance2 = 4095 - max(stepPhase, desiredStepPhase) + min(stepPhase, desiredStepPhase);

			// Check the error in the movement
			if (abs(distance1) > tuningVar1 && abs(distance2) > tuningVar1) {
				tuningVar2++;
			}

			// Move the motor for the next iteration
			desiredStepPhase = tuneCounter;
			SetMotorPhase(desiredStepPhase, 1);
		} else {
			// We are finished - check the number of deviations (Allow a small number of deviations)
			if (tuningVar2 > 10) {
				tuningError |= TUNE_ERR_INCORRECT_POLARITY;
			} else {
				tuningError &= ~TUNE_ERR_INCORRECT_POLARITY;
			}
			tuning &= ~POLARITY_CHECK;
			tuningError &= ~TUNE_ERR_NOT_CHECKED_POLARITY;
			newTuningMove = true;
		}

		tuneCounter += stepPhaseDistance;
		return;		// If we have done this tuning move, we don't want to do any others
	}

	// Do a polarity detection manoeuvre
	if (tuning & CONTROL_CHECK) {
		// TODO
		tuning &= ~CONTROL_CHECK;
		tuningError &= ~TUNE_ERR_NOT_CHECKED_CONTROL;
	}

	if (tuning & ENCODER_STEPS_CHECK) {
		// TODO
		tuning &= ~ENCODER_STEPS_CHECK;
		tuningError &= ~TUNE_ERR_NOT_CHECKED_ENCODER_STEPS;
	}

	// Do a continuous phase increase manoeuvre
	if (tuning & CONTINUOUS_PHASE_INCREASE_MANOEUVRE) {

		if (newTuningMove) {
			// This is the first run
			tuneCounter = 0;
			newTuningMove = false;
		}

		// Increase the desired step phase
		desiredStepPhase = tuneCounter;
		SetMotorPhase(desiredStepPhase, 1);
		tuneCounter += stepPhaseDistance;

		if (tuneCounter >= 4096) {
			//tuning &= ~CONTINUOUS_PHASE_INCREASE_MANOEUVRE;
			newTuningMove = true;
			return;
		}

	}

	// Do a step manoeuvre
	if (tuning & STEP_MANOEUVRE) {
		targetMotorSteps = currentMotorSteps + 4;
		tuning &= ~STEP_MANOEUVRE;
		newTuningMove = true;
		return;
	}

	// TODO: Implement Ziegler-Nichols manoeuvre
#if true
	// Do a Ziegler-Nichols manoeuvre
	if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {
		tuning &= ~ZIEGLER_NICHOLS_MANOEUVRE;
	}
#else
	// Do a ziegler-nichols tuning manoeuvre
	if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {

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

}

#endif
