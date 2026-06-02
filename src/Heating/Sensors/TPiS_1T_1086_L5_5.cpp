/*
 * TPiS_1T_1086_L5_5.cpp
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#include "TPiS_1T_1086_L5_5.h"

#if SUPPORT_TPiS_1T_1086_L5_5

#include <CanMessageGenericParser.h>
#include <Platform/Platform.h>

#if SUPPORT_LP5817
# include <Platform/LedStatusControl.h>
#endif

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TPiS_1T_1086_L5_5::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_1T_1086_L5_5(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor TPiS_AmbientTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_AmbientTemperatureSensor(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor TPiS_EnvironmentTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_EnvironmentTemperatureSensor(sensorNum); } );

TPiS_1T_1086_L5_5::TPiS_1T_1086_L5_5(unsigned int sensorNum) noexcept
	: I2CTemperatureSensor(sensorNum, TypeName, TPiS_I2CChannel, TPiS_I2CAddress),
	  r25(ThermistorR25[envThermistorAdcFilterChannel]),
	  beta(ThermistorBeta[envThermistorAdcFilterChannel]),
	  shC(ThermistorShC[envThermistorAdcFilterChannel]),
	  seriesR(ThermistorSeriesR[envThermistorAdcFilterChannel])
{
}

// Configure this temperature sensor
GCodeResult TPiS_1T_1086_L5_5::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool changed = false;
	if (parser.HasParameter('P'))
	{
		changed = true;

		// Configure the environment thermistor
		SetPinMode(TempSensePins[envThermistorAdcFilterChannel], PinMode::AIN);
		Platform::InitThermistorFilter(TempSensePins[envThermistorAdcFilterChannel], envThermistorAdcFilterChannel, false);

		// Initialise the sensor
		bool ok;
		// Set the slave address of the sensor. After power up it only responds to the General Call address.
		{
			device.SetAddress(0);									// general call address
			const uint8_t txBuffer[] = { 0x04 };					// command to load address from EPROM
			ok = device.Transfer(txBuffer, nullptr, sizeof(txBuffer), 0, TPis_I2C_timeout, true);
		}

		// Set up to read the EPROM
		if (ok)
		{
			delay(10);
			device.SetAddress(TPiS_I2CAddress);						// default address read from EPROM
			const uint8_t txBuffer[] = { 0x1F, 0x80 };				// command to enable read from EPROM
			ok = device.Transfer(txBuffer, nullptr, sizeof(txBuffer), 0, TPis_I2C_timeout, false);
		}

		// Read the whole EPROM
		uint8_t epromBuffer[32];
		if (ok)
		{
			const uint8_t txBuffer[] = { 0x20 };
			ok = device.Transfer(txBuffer, epromBuffer, sizeof(txBuffer), sizeof(epromBuffer), TPis_I2C_timeout, false);
			const uint8_t txBuffer2[] = { 0x1F, 0x00 };				// command to disable read from EPROM
			(void)device.Transfer(txBuffer2, nullptr, sizeof(txBuffer2), 0, TPis_I2C_timeout, true);
		}

		if (ok)
		{
			// Check the EPROM
			uint16_t calculatedChecksum = epromBuffer[0];
			for (size_t i = 3; i < 32; ++i)
			{
				calculatedChecksum += epromBuffer[i];
			}
			const uint16_t storedChecksum = ((uint16_t)epromBuffer[1] << 8) | (uint16_t)epromBuffer[2];
			if (epromBuffer[0] == 3 && epromBuffer[31] == (TPiS_I2CAddress | 0x80) && calculatedChecksum == storedChecksum)
			{
				// Get needed data from EPROM contents
				TempCalibPtat25 = ((uint16_t)epromBuffer[10] << 8) | epromBuffer[11];				// ambient sensor output at 25C
				TempCalibM = ((uint16_t)epromBuffer[12] << 8) | epromBuffer[13];					// slope of ambient sensor output per degC times 100
				TempCalibUo = (((uint32_t)epromBuffer[14] << 8) | epromBuffer[15]) + 32768UL;		// sensor output when object temperature = ambient temperature
				TempCalibUout1 = (((uint32_t)epromBuffer[16] << 8) | epromBuffer[17]) * 2;			// sensor output when object temperature is tempCalibTobj1, ambient is 25C and object is black
				TempCalibTobj1 = epromBuffer[18];													// object temperature in degC at which the output is TempCalibUout1

				// Derived parameters
				recipTempCalibM = 100.0/(float)TempCalibM;
				TempCalibK = (powf(ConvertDegCToDegK(TempCalibTobj1), RadiationExponent) - powf(ConvertDegCToDegK(25.0), RadiationExponent)) / (float)(int32_t)(TempCalibUout1 - TempCalibUo);

				// Set the starting temperatures
				ambientTemperatureDegK = objectTemperatureDegK = ambientTemperatureLast = objectTemperatureLast = ConvertDegCToDegK(25.0);
			}
			else
			{
				reply.printf("Error in thermopile sensor EPROM: %02x %02x %04x %04x", epromBuffer[0], epromBuffer[31], calculatedChecksum, storedChecksum);
#if SUPPORT_LP5817
				Platform::GetStatusLedControl()->SetErrorOrTransient(LedStatusCode::irSensorFail);
#endif
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Failed to communicate with thermopile sensor");
#if SUPPORT_LP5817
			Platform::GetStatusLedControl()->SetErrorOrTransient(LedStatusCode::irSensorFail);
#endif
			return GCodeResult::error;
		}
	}

	// Get the environment thermistor parameters. We don't currently allow the series resistance to be configured because we don't expect it to change.
	if (parser.GetFloatParam('B', beta))
	{
		shC = 0.0;						// if user changes B and doesn't define C, assume C=0
		changed = true;
	}
	changed = parser.GetFloatParam('C', shC) || changed;
	changed = parser.GetFloatParam('T', r25) || changed;

	if (changed)
	{
		shB = 1.0/beta;
		const float lnR25 = logf(r25);
		shA = 1.0/(ConvertDegCToDegK(25.0)) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", environment thermistor: T:%.1f B:%.1f C:%.2e R:%.1f", (double)r25, (double)beta, (double)shC, (double)seriesR);
	}
#if SUPPORT_LP5817
		Platform::GetStatusLedControl()->ClearError(LedStatusCode::irSensorFail);
#endif
	return GCodeResult::ok;
}

void TPiS_1T_1086_L5_5::Poll() noexcept
{
	// Read the environment thermistor first
	const volatile ThermistorAveragingFilter * const tempFilter = Platform::GetAdcFilter(envThermistorAdcFilterChannel);
	if (tempFilter->IsValid())
	{
		const int32_t averagedTempReading = tempFilter->GetSum()/(tempFilter->NumAveraged() >> AdcOversampleBits);
		if (OversampledAdcRange <= averagedTempReading)
		{
			SetResult(ABS_ZERO, TemperatureError::openCircuit);
		}
		else
		{
			const float denom = (float)(OversampledAdcRange - averagedTempReading) - 0.5;
			float resistance = seriesR * ((float)averagedTempReading + 0.5)/denom;
			const float logResistance = logf(resistance);
			const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
			const float tempDegK = (recipT > 0.0) ? (1.0/recipT) : ConvertDegCToDegK(BadErrorTemperature);

			if (tempDegK < AuxTempReadingMin)
			{
				// Thermistor is probably disconnected
				environmentTemperatureDegK = 0.0;
				thermistorResult = TemperatureError::readingTooLow;
			}
			else if (tempDegK > AuxTempReadingMax)
			{
				environmentTemperatureDegK = tempDegK;
				thermistorResult = TemperatureError::readingTooHigh;
			}
			else
			{
				environmentTemperatureDegK = tempDegK;
				thermistorResult = TemperatureError::ok;
			}
		}
	}
	else
	{
		environmentTemperatureDegK = ConvertDegCToDegK(BadErrorTemperature);
		thermistorResult = TemperatureError::notReady;
	}

	// Read the object and ambient temperatures from the sensor
	const uint8_t txBuffer[] = { 0x01 };
	uint8_t rxBuffer[4];
	TemperatureError rslt = TemperatureError::ioError;
	if (device.Transfer(txBuffer, rxBuffer, sizeof(txBuffer), sizeof(rxBuffer), true))
	{
		const uint32_t rawTAmb = ((uint32_t)(rxBuffer[2] & 0x7F)) << 8 | (uint32_t)rxBuffer[3];
		const uint32_t rawTpObject = ((uint32_t)rxBuffer[0] << 9) | ((uint32_t)rxBuffer[1] << 1) | ((uint32_t)rxBuffer[2] >> 7);

		// 1. Calculate the ambient temperature reported by the sensor in degK. See datasheet section 8.4.
		ambientTemperatureDegK = ConvertDegCToDegK(25.0) + (float)((int32_t)(rawTAmb - TempCalibPtat25)) * recipTempCalibM;
		const float ambientRadiance = powf(ambientTemperatureDegK, RadiationExponent);

		// 2. Calculate the detected radiance
		const float detectedRadiance = (float)(int32_t)(rawTpObject - TempCalibUo) * TempCalibK;

		// 3. Decide what nozzle environment temperature to use and calculate its radiance
		const float auxTemperatureDegK = (thermistorResult == TemperatureError::ok) ? environmentTemperatureDegK : (7.0 * ambientTemperatureLast + objectTemperatureLast) * 0.125;
		const float auxRadiance = powf(auxTemperatureDegK, RadiationExponent);

		// The remaining detected radiance must be due to the object temperature
		const float receivedObjectRadiance = detectedRadiance - auxRadiance * AuxFovAndEmissivityCorrection + ambientRadiance * (AuxFovAndEmissivityCorrection + ObjectFovAndEmissivityCorrection);

		// Check that the received object radiance is non-negative to avoid a math error
		if (receivedObjectRadiance < 0.0)
		{
			objectTemperatureDegK = ConvertDegCToDegK(BadErrorTemperature);
			rslt = TemperatureError::overOrUnderVoltage;
		}
		else
		{
			// The actual object radiance will be greater because of the limited FOV and because the object is not completely black
			const float actualObjectRadiance = receivedObjectRadiance * (1.0/ObjectFovAndEmissivityCorrection);

			// Calculate the object temperature. See datasheet section 8.5. We assume that LOOKUP# = 2.
			objectTemperatureDegK = powf(actualObjectRadiance, 1.0/RadiationExponent);

			objectTemperatureLast = objectTemperatureDegK;
			ambientTemperatureLast = ambientTemperatureDegK;

			// Do some error checking. Only display one error, otherwise we will get debug messages sent too rapidly to process.
			if (objectTemperatureDegK > NozzleTempReadingMax)
			{
				if (GetLastResult() != TemperatureError::readingTooHigh && Platform::Debug(Module::Heat))
				{
					debugPrintf("ErrTempMax [%.1f]\n", (double)ConvertDegKToDegC(objectTemperatureDegK));
				}
				rslt = TemperatureError::readingTooHigh;
			}
			else if (objectTemperatureDegK < NozzleTempReadingMin)
			{
				if (GetLastResult() != TemperatureError::readingTooLow && Platform::Debug(Module::Heat))
				{
					debugPrintf("ErrTempMin [%.1f]\n", (double)ConvertDegKToDegC(objectTemperatureDegK));
				}
				rslt = TemperatureError::readingTooLow;
			}
			else if (ambientTemperatureDegK > AmbientTempReadingMax)
			{
				if (GetLastResult() != TemperatureError::ambientReadingTooHigh && Platform::Debug(Module::Heat))
				{
					debugPrintf("ErrTempAmbMax [%.1f]\n", (double)ConvertDegKToDegC(ambientTemperatureDegK));
				}
				rslt = TemperatureError::ambientReadingTooHigh;
			}
			else if (ambientTemperatureDegK < AmbientTempReadingMin)
			{
				if (GetLastResult() != TemperatureError::ambientReadingTooLow && Platform::Debug(Module::Heat))
				{
					debugPrintf("ErrTempAmbMin [%.1f]\n", (double)ConvertDegKToDegC(ambientTemperatureDegK));
				}
				rslt = TemperatureError::ambientReadingTooLow;
			}
			else
			{
				rslt = TemperatureError:: ok;
			}
		}

#if 0	//DEBUG
		{
			static uint8_t count=0;
			++count;
			if (count == 0)
			{
				debugPrintf("PTAT25=%u M=%u U0=%lu Uout1=%lu Tobj1=%u 1/M=%.4f 1/K=%.3f", TempCalibPtat25, TempCalibM, TempCalibUo, TempCalibUout1, TempCalibTobj1, (double)recipTempCalibM, (double)recipCorrectedK);
				debugPrintf(" %lu %lu %.1f %.1f\n", rawTAmb, rawTpObject, (double)ambientTemperatureDegK, (double)objectTemperatureDegK);
			}
		}
#endif
	}

	if (rslt == TemperatureError::ok)
	{
		SetResult(ConvertDegKToDegC(objectTemperatureDegK), rslt);
	}
	else
	{
		SetResult(rslt);
	}
}

TemperatureError TPiS_1T_1086_L5_5::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	const auto result = TemperatureSensor::GetLatestTemperature(t);
	switch (outputNumber)
	{
	case 1:
		t = ConvertDegKToDegC(ambientTemperatureDegK);
		break;

	case 2:
		t = ConvertDegKToDegC(environmentTemperatureDegK);
		return thermistorResult;

	default:
		t = BadErrorTemperature;
		return TemperatureError::invalidOutputNumber;
	}
	return result;
}

// TPiS_AmbientTemperatureSensor members
TPiS_AmbientTemperatureSensor::TPiS_AmbientTemperatureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, TypeName, false)
{
}

TPiS_AmbientTemperatureSensor::~TPiS_AmbientTemperatureSensor() noexcept
{
}

// TPiS_EnvironmentTemperatureSensor members
TPiS_EnvironmentTemperatureSensor::TPiS_EnvironmentTemperatureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, TypeName, false)
{
}

TPiS_EnvironmentTemperatureSensor::~TPiS_EnvironmentTemperatureSensor() noexcept
{
}

#endif

// End

