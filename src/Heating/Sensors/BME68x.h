/*
 * BME68x.h
 *
 *  Created on: 9 Mar 2026
 *      Author: Christian
 *
 *  Support for BME680/BME688 environmental sensors (temperature, pressure, humidity, gas resistance)
 */

#ifndef SRC_HEATING_SENSORS_BME68X_H_
#define SRC_HEATING_SENSORS_BME68X_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_BME68X

#include "AdditionalOutputSensor.h"
#include "bme68x_defs.h"

class BME68xTemperatureSensor : public SpiTemperatureSensor
{
public:
	BME68xTemperatureSensor(unsigned int sensorNum) noexcept;

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;

	const uint8_t GetNumAdditionalOutputs() const noexcept override { return 3; }
	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;
	void Poll() noexcept override;

	static constexpr const char *TypeName = "bme68x";

private:
	static SensorTypeDescriptor typeDescriptor;

	static constexpr size_t MaxRegistersToRead = BME68X_LEN_COEFF_ALL;

	TemperatureError bme68x_init() noexcept;
	TemperatureError bme68x_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) noexcept;
	TemperatureError bme68x_set_reg(uint8_t reg_addr, uint8_t reg_data) noexcept;
	TemperatureError bme68x_soft_reset() noexcept;
	TemperatureError get_calib_data() noexcept;
	TemperatureError set_mem_page(uint8_t reg_addr) noexcept;
	TemperatureError configure_sensor() noexcept;
	TemperatureError set_gas_config(uint16_t targetTemp, uint16_t duration) noexcept;
	TemperatureError start_forced_measurement() noexcept;
	TemperatureError bme68x_get_sensor_data() noexcept;
	void parse_gas_calib_data(const uint8_t *coeff) noexcept;
	void parse_calib_data(const uint8_t *coeff) noexcept;
	void parse_calib_data_690(const uint8_t *coeff) noexcept;
	void parse_field_data(const uint8_t *reg_data) noexcept;
	float compensate_temperature(uint32_t temp_adc) noexcept;
	float compensate_pressure(uint32_t pres_adc) const noexcept;
	float compensate_humidity(uint16_t hum_adc) const noexcept;
	float compensate_temperature_690(uint32_t temp_adc) const noexcept;
	float compensate_pressure_690(uint32_t pres_adc, float tLin) const noexcept;
	float compensate_humidity_690(uint16_t hum_adc, float tLin) const noexcept;
	float calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range) const noexcept;
	float calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range) const noexcept;
	uint8_t calc_res_heat(uint16_t temp) const noexcept;
	static uint8_t calc_gas_wait(uint16_t dur) noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;

	bool IsBme690() const noexcept { return variant_id == BME690_VARIANT_GAS_HIGH; }
	bool IsHighGasVariant() const noexcept { return variant_id != BME68X_VARIANT_GAS_LOW; }

	bme68x_calib_data calib;
	bme690_calib_data calib690;
	uint32_t variant_id;
	uint8_t mem_page;
	float compTemperature;			/*< Compensated temperature */
	float compPressure;				/*< Compensated pressure in hPa */
	float compHumidity;				/*< Compensated humidity in %RH */
	float compGasResistance;		/*< Compensated gas resistance in Ohms */
	bool gasValid;					/*< Whether the last gas reading was valid */
};

// This class represents a BME68x pressure sensor
class BME68xPressureSensor : public AdditionalOutputSensor
{
public:
	BME68xPressureSensor(unsigned int sensorNum) noexcept;
	~BME68xPressureSensor() noexcept;

	static constexpr const char *TypeName = "bme68xpressure";

private:
	static SensorTypeDescriptor typeDescriptor;
};

// This class represents a BME68x humidity sensor
class BME68xHumiditySensor : public AdditionalOutputSensor
{
public:
	BME68xHumiditySensor(unsigned int sensorNum) noexcept;
	~BME68xHumiditySensor() noexcept;

	static constexpr const char *TypeName = "bme68xhumidity";

private:
	static SensorTypeDescriptor typeDescriptor;
};

// This class represents a BME68x gas resistance sensor
class BME68xGasResistanceSensor : public AdditionalOutputSensor
{
public:
	BME68xGasResistanceSensor(unsigned int sensorNum) noexcept;
	~BME68xGasResistanceSensor() noexcept;

	static constexpr const char *TypeName = "bme68xgas";

private:
	static SensorTypeDescriptor typeDescriptor;
};

#endif	// SUPPORT_BME68X

#endif /* SRC_HEATING_SENSORS_BME68X_H_ */
