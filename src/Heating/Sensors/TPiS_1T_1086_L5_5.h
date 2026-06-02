/*
 * TPiS_1T_1086_L5_5.h
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_
#define SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_

#include "RepRapFirmware.h"

#if SUPPORT_TPiS_1T_1086_L5_5

#include "I2CTemperatureSensor.h"
#include "AdditionalOutputSensor.h"

class CanMessageGenericParser;

class TPiS_1T_1086_L5_5 : public I2CTemperatureSensor
{
public:
	TPiS_1T_1086_L5_5(unsigned int sensorNum) noexcept;

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;

	static constexpr const char *TypeName = "thermopile_tpis.object";

	const uint8_t GetNumAdditionalOutputs() const noexcept override { return 2; }
	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;

	void Poll() override;

private:
	static SensorTypeDescriptor typeDescriptor;

	static constexpr uint32_t TPis_I2C_timeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	// Constants that we adjust to get accurate readings
	static constexpr float RadiationExponent = 4.20;					// theoretically this should be 4.0 but the sensor manufacturer recommends 4.2
	static constexpr float ObjectFovAndEmissivityCorrection = 0.478;	// calibration factor for reduced emissivity and/or reduced FOV coverage of the object
	static constexpr float AuxFovAndEmissivityCorrection = 0.50;		// calibration factor for reduced emissivity and/or reduced FOV coverage of the environment
	static_assert(ObjectFovAndEmissivityCorrection + AuxFovAndEmissivityCorrection < 1.0);		// the total can't exceed 1.0 and will only be 1.0 if object and environment are both completely black

	// Calibration constants read from EPROM
	uint16_t TempCalibPtat25;
	uint16_t TempCalibM;
	uint32_t TempCalibUo;
	uint32_t TempCalibUout1;
	uint8_t TempCalibTobj1;

	// Calibration constants calculated from the above
	float TempCalibK;
	float recipTempCalibM;

	// Latest raw readings from the sensor
	float ambientTemperatureDegK;
	float objectTemperatureDegK;
	float ambientTemperatureLast, objectTemperatureLast;

	// Parameters for the thermistor that measures the surround temperature
	float r25, beta, shC, seriesR;										// configured parameters
	float shA, shB;														// derived parameters
	float environmentTemperatureDegK = ConvertDegCToDegK(BadErrorTemperature);
	TemperatureError thermistorResult = TemperatureError::notReady;

	static constexpr unsigned int AdcOversampleBits = 2;				// we use 2-bit oversampling
	static constexpr int32_t OversampledAdcRange = 1u << (AnalogIn::AdcBits + AdcOversampleBits);	// The readings we pass in should be in range 0..(AdcRange - 1)

	// Reading limits we apply to detect bad readings
	static constexpr float AmbientTempReadingMin = ConvertDegCToDegK(5.0);
	static constexpr float AmbientTempReadingMax = ConvertDegCToDegK(95.0);
	static constexpr float NozzleTempReadingMin = ConvertDegCToDegK(0.0);
	static constexpr float NozzleTempReadingMax = ConvertDegCToDegK(350.0);
	static constexpr float AuxTempReadingMin = ConvertDegCToDegK(0.0);
	static constexpr float AuxTempReadingMax = ConvertDegCToDegK(110.0);
};

// This class represents the TPiS_1T_1086_L5_5 ambient temperature sensor
class TPiS_AmbientTemperatureSensor : public AdditionalOutputSensor
{
public:
	TPiS_AmbientTemperatureSensor(unsigned int sensorNum) noexcept;
	~TPiS_AmbientTemperatureSensor() noexcept;

	static constexpr const char *TypeName = "thermopile_tpis.ambient";

private:
	static SensorTypeDescriptor typeDescriptor;
};

// This class represents the environment temperature sensor
class TPiS_EnvironmentTemperatureSensor : public AdditionalOutputSensor
{
public:
	TPiS_EnvironmentTemperatureSensor(unsigned int sensorNum) noexcept;
	~TPiS_EnvironmentTemperatureSensor() noexcept;

	static constexpr const char *TypeName = "thermopile_tpis.environment";

private:
	static SensorTypeDescriptor typeDescriptor;
};

#endif

#endif /* SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_ */
