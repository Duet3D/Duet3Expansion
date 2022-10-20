
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
 *  - Increase the step phase by a little over 4096 counts and back again
 *  - ignore the points near the start position
 *  - feed the remaining (phase, encoder reading) points into a linear regression algorithm, separately on the forward and the reverse movements
 *  - the linear regression gives us the encoder offset and the counts per step
 *  - pass these figures to the ClosedLoop module. It will check the counts per step, set the forward/reverse encoder polarity flag, and set the zero position
 *
 *  Notes on linear regression:
 *  From https://en.wikipedia.org/wiki/Simple_linear_regression the formula to fit a straight line y = mx + c to a set of N (x, y) points is:
 *   m = sigma(i=0..(N-1): (xi - xm) * (yi - ym)) / sigma(i=0..(N-1): (xi - xm)^2)
 *   c = ym - m * xm
 *  where xi is the ith x, yi is the ith y, xm is the mean x, ym is the mean y
 *  In our case the x values are the motor phase values selected, which are spaced uniformly, so that xi = x0 + p*i
 *  So xm = (x0 + (x0 + p*(N-1)))/2 = x0 + p*(N-1)/2
 *  and (xi - xm) = x0 + p*i - x0 - p*(N-1)/2 = p*(i - (N-1)/2)
 *
 *  Expand the numerator in the equation for m to:
 *   sigma(i=0..(N-1): yi*(xi - xm)) - ym*sigma(i=0..(N-1): (xi - xm))
 *  Simplify this to:
 *   sigma(i=0..N-1): yi*p*(i - (N-1)/2)) - ym*sigma(i=0..(N-1): p*(i - (N-1)/2))
 *  and further to:
 *   p * sigma(i=0..N-1): yi*(i - (N-1)/2)) - ym*p*(N * (N-1)/2 - N * (N-1)/2)
 *  and even further to:
 *   p * (sigma(i=0..N-1): yi*(i - (N-1)/2))
 *  We can accumulate the first term as we take readings, and we can accumulate the sum of the y values so we can calculate ym at the end.
 *
 *  The denominator in the equation for m expands to:
 *   sigma(i=0..(N-1): p^2*(i - (N-1)/2)^2)
 *  Expand this to:
 *   p^2 * sigma(i=0..(N-1): i^2) - sigma(i=0..(N-1): i * (N-1)) + sigma(i=0..(N-1): ((N-1)/2)^2)
 *  Simplify to:
 *   p^2 * sigma(i=0..(N-1): i^2) - (N-1) * sigma(i=0..(N-1): i) + N * ((N-1)/2)^2
 *  Using sigma(i=0..(N-1): i) = N * (N-1)/2, sigma(i=0..(N-1): i^2) = (N * (N-1) * (2*N - 1))/6 we get:
 *   p^2 * ((N * (N-1) * (2*N - 1))/6 - (N-1)*N * (N-1)/2 + N * ((N-1)/2)^2)
 *  which simplifies to:
 *   p^2*(N^3-N)/12
 *  So numerator/denominator is:
 *   (sigma(i=0..N-1): yi*(i - (N-1)/2))) / (p*(N^3-N)/12)
 */

static bool BasicTuning(bool firstIteration) noexcept
{
	enum class BasicTuningState { forwardInitial = 0, forwards, reverseInitial, reverse };

	static BasicTuningState state;									// state machine control
	static uint16_t initialStepPhase;								// the step phase we started at
	static unsigned int stepCounter;								// a counter to use within a state
	static uint32_t initialCount;
	static float regressionAccumulator;
	static float readingAccumulator;
	static float forwardSlope;
	static float forwardOrigin;
	static float forwardXmean;

	constexpr unsigned int NumDummySteps = 8;						// how many steps to take before we start collecting data
	constexpr uint16_t PhaseIncrement = 8;							// how much to increment the phase by on each step, must be a factor of 4096
	static_assert(4096 % PhaseIncrement == 0);
	constexpr unsigned int NumSamples = 4096/PhaseIncrement;		// the number of samples we take to d the linear regression
	constexpr float HalfNumSamplesMinusOne = (float)(NumSamples - 1) * 0.5;
	constexpr float Denominator = (float)PhaseIncrement * (fcube((float)NumSamples) - (float)NumSamples)/12.0;

	if (firstIteration)
	{
		state = BasicTuningState::forwardInitial;
		stepCounter = 0;
		ClosedLoop::encoder->SetBackwards(false);
		ClosedLoop::encoder->ClearFullRevs();
	}

	switch (state)
	{
	case BasicTuningState::forwardInitial:
		// In this state we move forwards a few microsteps to allow the motor to settle down
		ClosedLoop::desiredStepPhase += PhaseIncrement;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1.0);
		++stepCounter;
		if (stepCounter == NumDummySteps)
		{
			regressionAccumulator = readingAccumulator = 0.0;
			stepCounter = 0;
			initialStepPhase = ClosedLoop::desiredStepPhase;
			state = BasicTuningState::forwards;
		}
		break;

	case BasicTuningState::forwards:
		// Collect data and move forwards, until we have moved 4 full steps
		{
			int32_t reading = ClosedLoop::encoder->GetCurrentCount();
			if (stepCounter == 0)
			{
				initialCount = reading;
			}
			reading -= initialCount;
			readingAccumulator += (float)reading;
			regressionAccumulator += (float)reading * ((float)stepCounter - HalfNumSamplesMinusOne);
		}

		++stepCounter;
		if (stepCounter == NumSamples)
		{
			// Save the accumulated data
			forwardSlope = regressionAccumulator / Denominator;											// the average encoder counts per phase position
			forwardXmean = (float)initialStepPhase + (float)PhaseIncrement * HalfNumSamplesMinusOne;	// the average phase
			const float forwardYmean = readingAccumulator/NumSamples + (float)initialCount;				// the average count
			forwardOrigin = forwardYmean - forwardSlope * forwardXmean;									// the encoder reading at zero phase

			stepCounter = 0;
			state = BasicTuningState::reverseInitial;
		}
		else
		{
			ClosedLoop::desiredStepPhase += PhaseIncrement;
			ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1.0);
		}
		break;

	case BasicTuningState::reverseInitial:
		// In this state we move backwards a few microsteps to allow the motor to settle down
		ClosedLoop::desiredStepPhase -= PhaseIncrement;
		ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1.0);
		++stepCounter;
		if (stepCounter == NumDummySteps)
		{
			regressionAccumulator = readingAccumulator = 0.0;
			stepCounter = 0;
			initialStepPhase = ClosedLoop::desiredStepPhase;
			state = BasicTuningState::reverse;
		}
		break;

	case BasicTuningState::reverse:
		// Collect data and move backwards, until we have moved 4 full steps
		{
			int32_t reading = ClosedLoop::encoder->GetCurrentCount();
			if (stepCounter == 0)
			{
				initialCount = reading;
			}
			reading -= initialCount;
			readingAccumulator += (float)reading;
			regressionAccumulator += (float)reading * ((float)stepCounter - HalfNumSamplesMinusOne);
		}

		++stepCounter;
		if (stepCounter == NumSamples)
		{
			// Save the accumulated data
			const float reverseSlope = regressionAccumulator / (-Denominator);			// negate the denominator because the phase increment was negative this time
			const float reverseXmean = (float)initialStepPhase - (float)PhaseIncrement * HalfNumSamplesMinusOne;
			const float reverseYmean = readingAccumulator/NumSamples + (float)initialCount;
			const float reverseOrigin = reverseYmean - reverseSlope * reverseXmean;
			ClosedLoop::FinishedBasicTuning(forwardSlope, reverseSlope, forwardOrigin, reverseOrigin, forwardXmean, reverseXmean);
			return true;																// finished tuning
		}
		else
		{
			ClosedLoop::desiredStepPhase -= PhaseIncrement;
			ClosedLoop::SetMotorPhase(ClosedLoop::desiredStepPhase, 1.0);
		}
		break;
	}

	return false;
}


/*
 * Magnetic encoder calibration
 * ----------------------------
 *
 * Absolute:
 * 	- Move forwards somewhat (to counter any backlash) and then to the next full step position (to give (hopefully) consistent results)
 * 	- Move forwards at a constant rate. At each position, take the current encoder reading and update the Fourier coefficients
 * 	- Store the Fourier coefficients in the encoder LUT
 */

static bool EncoderCalibration(bool firstIteration) noexcept
{
	enum class EncoderCalibrationState { setup = 0, forwards, backwards };

	static EncoderCalibrationState state = EncoderCalibrationState::setup;
	static uint32_t positionsPerRev;			// this gets set to 1024 * the number of full steps per revolution, i.e. 204800 or 409600
	static uint32_t positionsTillStart;			// the position we advance to before we start tuning proper
	static uint32_t positionIncrement;			// how much we increase the phase position by on each movement
	static uint32_t positionCounter;			// how many positions we have moved
	static uint32_t virtualStartPosition;		// the virtual motor phase position we started at, calculated from the initial encoder reading

	if (!ClosedLoop::encoder->IsAbsolute())
	{
		return true;							// we don't do this tuning for relative encoders
	}

	AbsoluteEncoder* const absoluteEncoder = (AbsoluteEncoder*)ClosedLoop::encoder;
	const uint32_t maxValue = absoluteEncoder->GetMaxValue();								// get the maximum encoder reading plus one, we'll need it several times
	const uint32_t currentPosition = ClosedLoop::currentMotorPhase;

	if (firstIteration)
	{
		// Set up some variables
		state = EncoderCalibrationState::setup;
		absoluteEncoder->ClearLUT();
		positionsPerRev = absoluteEncoder->GetStepsPerRev() * 1024u;
		const float positionsPerEncoderCount = (float)positionsPerRev/(float)maxValue;

		// Decide how many phase positions to advance at a time. If we always advance by just one phase position, calibration can take a long time.
		// To speed things up, advance several phase positions at a time, but by less than one expected encoder count.
		// Keep the advance a power of 2 phase positions so that it is a factor of positionsPerRev
		// The AS5047 and TLI5012B both have 14-bit precision, so positionsPerEncoderCount is 12.5 for a 1.8deg motor and 25 for a 0.9deg motor
		positionIncrement = (positionsPerEncoderCount >= 16.0) ? 8
							: (positionsPerEncoderCount >= 8.0) ? 4
								: (positionsPerEncoderCount >= 4.0) ? 2
									: 1;

		// To counter any backlash, start by advancing a bit. Then advance to the next position which is a multiple of 4 full steps so that the phase position is zero.
		positionsTillStart = 4096 - currentPosition;
		if (positionsTillStart < 256)
		{
			positionsTillStart += 4096;
		}
	}

	const uint32_t currentReading = absoluteEncoder->GetRawAngle();		// get the current position in 0..(maxValue - 1)

	switch (state)
	{
	case EncoderCalibrationState::setup:
		// Advancing to a suitable full step position
		if (positionsTillStart != 0)
		{
			const uint32_t phaseChange = (positionsTillStart % positionIncrement) + positionIncrement;
			positionsTillStart -= phaseChange;
			ClosedLoop::SetMotorPhase(currentPosition + phaseChange, 1.0);
			return false;
		}

		// Calculate the approximate offset
		virtualStartPosition = ((uint64_t)currentReading * positionsPerRev)/maxValue;			// get the position we expect for this encoder reading to use as a reference
		positionCounter = 0;
		state = EncoderCalibrationState::forwards;
		// no break

	case EncoderCalibrationState::forwards:
		// Advancing slowly and recording positions
		if (positionCounter < positionsPerRev)
		{
			// Record the current data point
			// To avoid rounding error (caused by subtracting large values from each other repeatedly) in calculating the Fourier components, just calculate them from the error
			const uint32_t position = (positionCounter + virtualStartPosition) % positionsPerRev;
			const float expectedReading = ((float)position * (float)absoluteEncoder->GetMaxValue())/(float)positionsPerRev;
			float error = (float)currentReading - expectedReading;

			// Allow for wrap around, e.g. expected reading = 16383, actual reading = 0 or vice versa
			if (error > (float)maxValue/2) { error -= (float)maxValue; }
			else if (error < -(float)maxValue/2) { error += (float)maxValue; }

			const float angle = (TwoPi * position)/positionsPerRev;
			absoluteEncoder->RecordDataPoint(angle, error);
		}

		// Move to the next position. After a complete revolution we continue another 256 positions without recording data, ready for the reverse pass.
		ClosedLoop::SetMotorPhase(currentPosition + positionIncrement, 1.0);
		positionCounter += positionIncrement;
		if (positionCounter == positionsPerRev + 256)
		{
			state = EncoderCalibrationState::backwards;
		}
		break;

	case EncoderCalibrationState::backwards:
		if (positionCounter < positionsPerRev)
		{
			// Record the current data point
			// To avoid rounding error (caused by subtracting large values from each other repeatedly) in calculating the Fourier components, just calculate them from the error
			{
				const uint32_t position = (positionCounter + virtualStartPosition) % positionsPerRev;
				const float expectedReading = ((float)position * (float)absoluteEncoder->GetMaxValue())/(float)positionsPerRev;
				float error = (float)currentReading - expectedReading;

				// Allow for wrap around, e.g. expected reading = 16383, actual reading = 0 or vice versa
				if (error > (float)maxValue/2) { error -= (float)maxValue; }
				else if (error < -(float)maxValue/2) { error += (float)maxValue; }

				const float angle = (TwoPi * position)/positionsPerRev;
				absoluteEncoder->RecordDataPoint(angle, error);
			}

			// Retreating slowly and recording positions
			if (positionCounter == 0)
			{
				// We are finished
				absoluteEncoder->StoreLUT(virtualStartPosition, (2 * positionsPerRev)/positionIncrement);
				ClosedLoop::FinishedEncoderCalibration();			// set target position to current position
				return true;
			}
		}

		// Move to the next position
		ClosedLoop::SetMotorPhase(currentPosition - positionIncrement, 1.0);
		positionCounter -= positionIncrement;
		break;
	}
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

#if 0	// not implemented

static bool ContinuousPhaseIncrease(bool firstIteration) noexcept
{
	return true;
}

#endif

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
	ClosedLoop::AdjustTargetMotorSteps(4.0);
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

#if 1
// TODO: Implement ziegler-Nichols move

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

// This is called from the closed loop control loop every (1/tuningStepsPerSecond) seconds if tuning is enabled, currently every 0.5ms
void ClosedLoop::PerformTune() noexcept
{
	static bool newTuningMove = true;						// indicates if a tuning move has just finished

	// Check we are in direct drive mode and we have an encoder
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct || encoder == nullptr)
	{
		tuningError |= TUNE_ERR_SYSTEM_ERROR;
		tuning = 0;
		return;
	}

	// Run one iteration of the one, highest priority, tuning move
	if (tuning & BASIC_TUNING_MANOEUVRE)
	{
		if (encoder->IsAbsolute() && (tuning & ENCODER_CALIBRATION_MANOEUVRE))
		{
			((AbsoluteEncoder*)encoder)->ClearLUT();
		}
		newTuningMove = BasicTuning(newTuningMove);
		if (newTuningMove)
		{
			tuning &= ~BASIC_TUNING_MANOEUVRE;				// we can do encoder calibration after basic tuning
		}
	}
	else if (tuning & ENCODER_CALIBRATION_MANOEUVRE)
	{
		if (tuningError & (TUNE_ERR_TOO_MUCH_MOTION | TUNE_ERR_TOO_LITTLE_MOTION | TUNE_ERR_INCONSISTENT_MOTION | TUNE_ERR_NOT_DONE_BASIC))
		{
			// Basic tuning failed, so don't attempt encoder calibration because it may not complete
			tuning = 0;
		}
		else
		{
			newTuningMove = EncoderCalibration(newTuningMove);
			if (newTuningMove)
			{
				tuning = BASIC_TUNING_MANOEUVRE;			// run basic tuning again
			}
		}
	}
	else if (tuning & STEP_MANOEUVRE)
	{
		newTuningMove = Step(newTuningMove);
		if (newTuningMove)
		{
			tuning = 0;
		}
#if 0	// not implemented
	} else if (tuning & CONTINUOUS_PHASE_INCREASE_MANOEUVRE) {
		newTuningMove = ContinuousPhaseIncrease(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
	} else if (tuning & ZIEGLER_NICHOLS_MANOEUVRE) {
		newTuningMove = ZieglerNichols(newTuningMove);
		if (newTuningMove) {
			tuning = 0;
		}
#endif
	}
	else
	{
		tuning = 0;
		newTuningMove = true;								// ready for next time
	}
}

#endif	// #if SUPPORT_CLOSED_LOOP
