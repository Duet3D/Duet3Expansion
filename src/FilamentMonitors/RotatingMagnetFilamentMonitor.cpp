/*
 * RotatingMagnetFilamentMonitor.cpp
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#include "RotatingMagnetFilamentMonitor.h"

#if SUPPORT_DRIVERS

#include "Platform.h"
#include "Movement/Move.h"
#include <CanMessageFormats.h>
#include <CanMessageGenericParser.h>

// Unless we set the option to compare filament on all type of move, we reject readings if the last retract or reprime move wasn't completed
// well before the start bit was received. This is because those moves have high accelerations and decelerations, so the measurement delay
// is more likely to cause errors. This constant sets the delay required after a retract or reprime move before we accept the measurement.
const int32_t SyncDelayMillis = 10;

RotatingMagnetFilamentMonitor::RotatingMagnetFilamentMonitor(unsigned int extruder, unsigned int monitorType) noexcept
	: Duet3DFilamentMonitor(extruder, monitorType),
	  mmPerRev(DefaultMmPerRev),
	  minMovementAllowed(DefaultMinMovementAllowed), maxMovementAllowed(DefaultMaxMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), comparisonEnabled(false), checkNonPrintingMoves(false)
{
	switchOpenMask = (monitorType == 4) ? TypeMagnetV1SwitchOpenMask : 0;
	Init();
}

void RotatingMagnetFilamentMonitor::Init() noexcept
{
	dataReceived = false;
	sensorValue = lastKnownPosition = 0;
	parityErrorCount = framingErrorCount = overrunErrorCount = polarityErrorCount = overdueCount = 0;
	lastMeasurementTime = 0;
	lastErrorCode = 0;
	version = 1;
	magnitude = 0;
	agc = 0;
	backwards = false;
	sensorError = false;
	InitReceiveBuffer();
	Reset();
}

void RotatingMagnetFilamentMonitor::Reset() noexcept
{
	extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	magneticMonitorState = MagneticMonitorState::idle;
	haveStartBitData = false;
	synced = false;							// force a resync
}

bool RotatingMagnetFilamentMonitor::HaveCalibrationData() const noexcept
{
	return magneticMonitorState != MagneticMonitorState::calibrating && totalExtrusionCommanded > 10.0;
}

float RotatingMagnetFilamentMonitor::MeasuredSensitivity() const noexcept
{
	return totalExtrusionCommanded/totalMovementMeasured;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult RotatingMagnetFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::change, seen);
	if (Succeeded(rslt))
	{
		if (seen)
		{
			Init();				// Init() resets dataReceived and version, so only do it if the port has been configured
		}

		if (parser.GetFloatParam('L', mmPerRev))
		{
			seen = true;
		}
		if (parser.GetFloatParam('E', minimumExtrusionCheckLength))
		{
			seen = true;
		}

		uint16_t minMax[2];
		size_t numValues = 2;
		if (parser.GetUint16ArrayParam('R', numValues, minMax))
		{
			if (numValues > 0)
			{
				seen = true;
				minMovementAllowed = (float)minMax[0] * 0.01;
			}
			if (numValues > 1)
			{
				maxMovementAllowed = (float)minMax[1] * 0.01;
			}
		}

		uint16_t temp;
		if (parser.GetUintParam('S', temp))
		{
			seen = true;
			comparisonEnabled = (temp > 0);
		}

		if (parser.GetUintParam('A', temp))
		{
			seen = true;
			checkNonPrintingMoves = (temp > 0);
		}

		if (!seen)
		{
			reply.printf("Duet3D rotating magnet filament monitor v%u%s on pin ", version, (switchOpenMask != 0) ? " with switch" : "");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, sensitivity %.2fmm/rev, allow %ld%% to %ld%%, check %s moves every %.1fmm, ",
						(comparisonEnabled) ? "enabled" : "disabled",
						(double)mmPerRev,
						ConvertToPercent(minMovementAllowed),
						ConvertToPercent(maxMovementAllowed),
						(checkNonPrintingMoves) ? "all" : "printing",
						(double)minimumExtrusionCheckLength);

			if (!dataReceived)
			{
				reply.cat("no data received");
			}
			else
			{
				reply.catf("version %u, ", version);
				if (switchOpenMask != 0)
				{
					reply.cat(((sensorValue & switchOpenMask) != 0) ? "no filament, " : "filament present, ");
				}
				if (version >= 3)
				{
					reply.catf("mag %u agc %u, ", magnitude, agc);
				}
				if (sensorError)
				{
					reply.cat("error: ");
					// Translate the common error codes to text
					switch (lastErrorCode)
					{
					case 6:
						reply.cat("no magnet detected");
						break;
					case 7:
						reply.cat("magnet too weak");
						break;
					case 8:
						reply.cat("magnet too strong");
						break;
					default:
						reply.catf("%u", lastErrorCode);
						break;
					}
				}
				else if (HaveCalibrationData())
				{
					const float measuredMmPerRev = MeasuredSensitivity();
					reply.catf("measured sensitivity %.2fmm/rev, min %ld%% max %ld%% over %.1fmm\n",
						(double)measuredMmPerRev,
						ConvertToPercent(minMovementRatio * measuredMmPerRev),
						ConvertToPercent(maxMovementRatio * measuredMmPerRev),
						(double)totalExtrusionCommanded);
				}
				else
				{
					reply.cat("no calibration data");
				}
			}
		}
	}
	return rslt;
}

// Deal with any received data
void RotatingMagnetFilamentMonitor::HandleIncomingData() noexcept
{
	uint16_t val;
	PollResult res;
	while ((res = PollReceiveBuffer(val)) != PollResult::incomplete)
	{
		// We have either received a report or there has been a framing error
		bool receivedPositionReport = false;
		if (res == PollResult::complete)
		{
			// We have a completed a position report. Check the parity.
			uint8_t data8 = (uint8_t)((val >> 8) ^ val);
			data8 ^= (data8 >> 4);
			data8 ^= (data8 >> 2);
			data8 ^= (data8 >> 1);

			// Version 1 sensor:
			//  Data word:			0S00 00pp pppppppp		S = switch open, pppppppppp = 10-bit filament position
			//  Error word:			1000 0000 00000000
			//
			// Version 2 sensor
			//  Data word:			P00S 10pp pppppppp		S = switch open, ppppppppppp = 10-bit filament position
			//  Error word:			P010 0000 0000eeee		eeee = error code
			//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
			//
			// Version 3 firmware:
			//  Data word:			P00S 10pp pppppppp		S = switch open, ppppppppppp = 10-bit filament position
			//  Error word:			P010 0000 0000eeee		eeee = error code
			//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
			//  Magnitude word		P110 0010 mmmmmmmm		mmmmmmmm = highest 8 bits of magnitude (cf. brightness of laser sensor)
			//  AGC word			P110 0011 aaaaaaaa		aaaaaaaa = AGC setting (cf. shutter of laser sensor)
			if (version == 1)
			{
				if ((data8 & 1) == 0 && (val & 0x7F00) == 0x6000 && (val & 0x00FF) >= 2)
				{
					// Received a version word with the correct parity, so must be version 2 or later
					version = val & 0x00FF;
					if (switchOpenMask != 0)
					{
						switchOpenMask = TypeMagnetV2SwitchOpenMask;
					}
				}
				else if (val == TypeMagnetV1ErrorMask)
				{
					sensorError = true;
					lastErrorCode = 0;
				}
				else if ((val & 0xBC00) == 0)
				{
					receivedPositionReport = true;
					sensorError = false;
					dataReceived = true;
				}
			}
			else if ((data8 & 1) != 0)
			{
				++parityErrorCount;
			}
			else
			{
				switch (val & TypeMagnetV2MessageTypeMask)
				{
				case TypeMagnetV2MessageTypePosition:
					receivedPositionReport = true;
					sensorError = false;
					dataReceived = true;
					break;

				case TypeMagnetV2MessageTypeError:
					lastErrorCode = val & 0x00FF;
					sensorError = (lastErrorCode != 0);
					dataReceived = true;
					break;

				case TypeMagnetV2MessageTypeInfo:
					switch (val & TypeMagnetV2InfoTypeMask)
					{
					case TypeMagnetV2InfoTypeVersion:
						version = val & 0x00FF;
						dataReceived = true;
						break;

					case TypeMagnetV3InfoTypeMagnitude:
						magnitude = val & 0x00FF;
						break;

					case TypeMagnetV3InfoTypeAgc:
						agc = val & 0x00FF;
						break;

					default:
						break;
					}
					break;

				default:
					break;
				}
			}
		}
		else
		{
			// A receive error occurred. Any start bit data we stored is wrong.
			++framingErrorCount;
		}

		if (receivedPositionReport)
		{
			// We have a completed a position report
			lastKnownPosition = sensorValue & TypeMagnetAngleMask;
			const uint16_t angleChange = (val - sensorValue) & TypeMagnetAngleMask;			// angle change in range 0..1023
			const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
			movementMeasuredSinceLastSync += (float)movement/1024;
			sensorValue = val;
			lastMeasurementTime = millis();

			if (haveStartBitData)					// if we have a synchronised value for the amount of extrusion commanded
			{
				if (synced)
				{
					if (   checkNonPrintingMoves
						|| (wasPrintingAtStartBit && (int32_t)(lastSyncTime - moveInstance->ExtruderPrintingSince()) >= SyncDelayMillis)
					   )
					{
						// We can use this measurement
						extrusionCommandedThisSegment += extrusionCommandedAtCandidateStartBit;
						movementMeasuredThisSegment += movementMeasuredSinceLastSync;
					}
				}
				lastSyncTime = candidateStartBitTime;
				extrusionCommandedSinceLastSync -= extrusionCommandedAtCandidateStartBit;
				movementMeasuredSinceLastSync = 0.0;
				synced = checkNonPrintingMoves || wasPrintingAtStartBit;
			}
		}
		haveStartBitData = false;
	}
}

// Call the following at intervals to check the status. This is only called when printing is in progress.
// 'filamentConsumed' is the net amount of extrusion commanded since the last call to this function.
// 'hadNonPrintingMove' is true if filamentConsumed includes extruder movement from non-printing moves.
// 'fromIsr' is true if this measurement was taken at the end of the ISR because a potential start bit was seen
FilamentSensorStatus RotatingMagnetFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	// 1. Update the extrusion commanded and whether we have had an extruding but non-printing move
	extrusionCommandedSinceLastSync += filamentConsumed;

	// 2. If this call passes values synced to the start bit, save the data for the next completed measurement.
	if (fromIsr && IsWaitingForStartBit())
	{
		extrusionCommandedAtCandidateStartBit = extrusionCommandedSinceLastSync;
		wasPrintingAtStartBit = isPrinting;
		candidateStartBitTime = isrMillis;
		haveStartBitData = true;
	}

	// 3. Process the receive buffer and update everything if we have received anything or had a receive error
	HandleIncomingData();

	// 4. Decide whether it is time to do a comparison, and return the status
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (sensorError)
	{
		ret = FilamentSensorStatus::sensorError;
	}
	else if ((sensorValue & switchOpenMask) != 0)
	{
		ret = FilamentSensorStatus::noFilament;
	}
	else if (extrusionCommandedThisSegment >= minimumExtrusionCheckLength)
	{
		ret = CheckFilament(extrusionCommandedThisSegment, movementMeasuredThisSegment, false);
		extrusionCommandedThisSegment = movementMeasuredThisSegment = 0.0;
	}
	else if (   extrusionCommandedThisSegment + extrusionCommandedSinceLastSync >= minimumExtrusionCheckLength * 3
			 && millis() - lastMeasurementTime > 500
			 && !IsReceiving()
			)
	{
		// A sync is overdue
		ret = CheckFilament(extrusionCommandedThisSegment + extrusionCommandedSinceLastSync, movementMeasuredThisSegment + movementMeasuredSinceLastSync, true);
		extrusionCommandedThisSegment = extrusionCommandedSinceLastSync = movementMeasuredThisSegment = movementMeasuredSinceLastSync = 0.0;
	}

	return (comparisonEnabled) ? ret : FilamentSensorStatus::ok;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus RotatingMagnetFilamentMonitor::CheckFilament(float amountCommanded, float amountMeasured, bool overdue) noexcept
{
	if (!dataReceived)
	{
		return FilamentSensorStatus::noDataReceived;
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;

	switch (magneticMonitorState)
	{
	case MagneticMonitorState::idle:
		magneticMonitorState = MagneticMonitorState::calibrating;
		totalExtrusionCommanded = amountCommanded;
		totalMovementMeasured = amountMeasured;
		break;

	case MagneticMonitorState::calibrating:
		totalExtrusionCommanded += amountCommanded;
		totalMovementMeasured += amountMeasured;
		if (totalExtrusionCommanded >= 10.0)
		{
			backwards = (totalMovementMeasured < 0.0);
			if (backwards)
			{
				totalMovementMeasured = -totalMovementMeasured;
			}
			float ratio = totalMovementMeasured/totalExtrusionCommanded;
			minMovementRatio = maxMovementRatio = ratio;

			if (comparisonEnabled)
			{
				ratio *= mmPerRev;
				if (ratio < minMovementAllowed)
				{
					ret = FilamentSensorStatus::tooLittleMovement;
				}
				else if (ratio > maxMovementAllowed)
				{
					ret = FilamentSensorStatus::tooMuchMovement;
				}
			}
			magneticMonitorState = MagneticMonitorState::comparing;
		}
		break;

	case MagneticMonitorState::comparing:
		{
			totalExtrusionCommanded += amountCommanded;
			if (backwards)
			{
				amountMeasured = -amountMeasured;
			}
			totalMovementMeasured += amountMeasured;
			float ratio = amountMeasured/amountCommanded;
			if (ratio > maxMovementRatio)
			{
				maxMovementRatio = ratio;
			}
			else if (ratio < minMovementRatio)
			{
				minMovementRatio = ratio;
			}

			if (comparisonEnabled)
			{
				ratio *= mmPerRev;
				if (ratio < minMovementAllowed)
				{
					ret = FilamentSensorStatus::tooLittleMovement;
				}
				else if (ratio > maxMovementAllowed)
				{
					ret = FilamentSensorStatus::tooMuchMovement;
				}
			}
		}
		break;
	}

	return ret;
}

// Clear the measurement state. Called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus RotatingMagnetFilamentMonitor::Clear() noexcept
{
	Reset();											// call this first so that haveStartBitData and synced are false when we call HandleIncomingData
	HandleIncomingData();								// to keep the diagnostics up to date

	return (!comparisonEnabled) ? FilamentSensorStatus::ok
			: (!dataReceived) ? FilamentSensorStatus::noDataReceived
				: (sensorError) ? FilamentSensorStatus::sensorError
					: ((sensorValue & switchOpenMask) != 0) ? FilamentSensorStatus::noFilament
						: FilamentSensorStatus::ok;
}

// Print diagnostic info for this sensor
void RotatingMagnetFilamentMonitor::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Driver %u: ", GetDriver());
	if (dataReceived)
	{
		reply.catf("pos %.2f", (double)(float)(sensorValue * (360.0/1024.0)));
	}
	else
	{
		reply.cat("no data received");
	}
	reply.catf(", errs: frame %" PRIu32 " parity %" PRIu32 " ovrun %" PRIu32 " pol %" PRIu32 " ovdue %" PRIu32,
				framingErrorCount, parityErrorCount, overrunErrorCount, polarityErrorCount, overdueCount);
}

#endif	// SUPPORT_DRIVERS

// End
