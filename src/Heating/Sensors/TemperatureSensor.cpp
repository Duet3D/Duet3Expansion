#include "TemperatureSensor.h"
#include "Thermistor.h"
#include "ThermocoupleSensor31855.h"
#include "ThermocoupleSensor31856.h"
#include "RtdSensor31865.h"
#include "CurrentLoopTemperatureSensor.h"
#include "LinearAnalogSensor.h"
#include "BME280.h"
#include "CanMessageGenericParser.h"

#if HAS_CPU_TEMP_SENSOR
#include "CpuTemperatureSensor.h"
#endif

#if SUPPORT_DHT_SENSOR
#include "DhtSensor.h"
#endif

#if HAS_SMART_DRIVERS
#include "TmcDriverTemperatureSensor.h"
#endif

#include "CAN/CanInterface.h"

// Constructor
TemperatureSensor::TemperatureSensor(unsigned int sensorNum, const char *t)
	: next(nullptr), sensorNumber(sensorNum), sensorType(t), whenLastRead(0), lastResult(TemperatureError::notReady), lastRealError(TemperatureError::ok) {}

// Virtual destructor
TemperatureSensor::~TemperatureSensor()
{
}

// Return the latest temperature reading
TemperatureError TemperatureSensor::GetLatestTemperature(float& t) noexcept
{
	// We must read whenLastRead *before* we call millis(). Otherwise, a task switch to the heater task could occur after we call millis and before we read whenLastRead,
	// so that when we read whenLastRead its value is greater than the result from millis().
	const uint32_t wlr = whenLastRead;
	if (millis() - wlr > GetTemperatureReadingTimeout())
	{
		lastTemperature = BadErrorTemperature;
		lastResult = TemperatureError::timeout;
#ifdef DEBUG
		debugPrintf("temp timeout on sensor %u\n", sensorNumber);
#endif
	}
	t = lastTemperature;
	return lastResult;
}

// Return the value of an additional output. Default implementation for sensors with no additional output.
TemperatureError TemperatureSensor::GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept
{
	t = BadErrorTemperature;
	return TemperatureError::invalidOutputNumber;
}

// Configure the reading adjustment parameters
void TemperatureSensor::ConfigureCommonParameters(const CanMessageGenericParser& parser, bool& seen) noexcept
{
	if (parser.GetFloatParam('U', offsetAdjustment)) { seen = true; }
	if (parser.GetFloatParam('V', slopeAdjustment)) { seen = true; }
}
// Default implementation of Configure, for sensors that have no configurable parameters
GCodeResult TemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply)
{
	bool seen = parser.HasParameter('Y');
	ConfigureCommonParameters(parser, seen);
	if (!seen)
	{
		// No parameters were provided, so report the current configuration
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

void TemperatureSensor::CopyBasicDetails(const StringRef& reply) const noexcept
{
	reply.printf("type %s", sensorType);
	AppendPinDetails(reply);
	reply.catf(", last error %s, ", ((TemperatureError)lastRealError).ToString());
	if (slopeAdjustment != 0.0 || offsetAdjustment != 0.0)
	{
		reply.catf("offset adjustment %.1f, slope adjustment %.3f, adjusted reading %.1f", (double)offsetAdjustment, (double)slopeAdjustment, (double)lastTemperature);
	}
	else
	{
		reply.catf("reading %.1f", (double)lastTemperature);
	}
}

void TemperatureSensor::SetResult(float t, TemperatureError rslt) noexcept
{
	lastResult = rslt;
	lastTemperature = (t * (1.0 + slopeAdjustment)) + offsetAdjustment;
	whenLastRead = millis();
	if (rslt != TemperatureError::ok)
	{
		lastRealError = rslt;
	}
}

// This version is used for unsuccessful readings only
void TemperatureSensor::SetResult(TemperatureError rslt)
{
	lastResult = rslt;
	lastRealError = rslt;
	lastTemperature = BadErrorTemperature;
	whenLastRead = millis();
}

// Get the expansion board address. Overridden for remote sensors.
CanAddress TemperatureSensor::GetBoardAddress() const
{
	return CanInterface::GetCanAddress();
}

// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
void TemperatureSensor::UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report)
{
	// Nothing to do here. This function is overridden in class RemoteSensor.
}

// Factory method
TemperatureSensor *TemperatureSensor::Create(unsigned int sensorNum, const char *typeName, const StringRef& reply)
{
	TemperatureSensor *ts;

	// We now always support linear analog sensors to make it easier to test analog inputs on the ATE IOMB
	if (ReducedStringEquals(typeName, LinearAnalogSensor::TypeName))
	{
		ts = new LinearAnalogSensor(sensorNum);
	}
#if SUPPORT_THERMISTORS
	else if (ReducedStringEquals(typeName, Thermistor::TypeNameThermistor))
	{
		ts = new Thermistor(sensorNum, false);
	}
	else if (ReducedStringEquals(typeName, Thermistor::TypeNamePT1000))
	{
		ts = new Thermistor(sensorNum, true);
	}
#endif
#if SUPPORT_SPI_SENSORS
	else if (ReducedStringEquals(typeName, ThermocoupleSensor31855::TypeName))
	{
		ts = new ThermocoupleSensor31855(sensorNum);
	}
	else if (ReducedStringEquals(typeName, ThermocoupleSensor31856::TypeName))
	{
		ts = new ThermocoupleSensor31856(sensorNum);
	}
	else if (ReducedStringEquals(typeName, RtdSensor31865::TypeName))
	{
		ts = new RtdSensor31865(sensorNum);
	}
	else if (ReducedStringEquals(typeName, CurrentLoopTemperatureSensor::TypeName))
	{
		ts = new CurrentLoopTemperatureSensor(sensorNum);
	}
#endif
#if SUPPORT_DHT_SENSOR
	else if (ReducedStringEquals(typeName, DhtTemperatureSensor::TypeNameDht21))
	{
		ts = new DhtTemperatureSensor(sensorNum, DhtSensorType::Dht21);
	}
	else if (ReducedStringEquals(typeName, DhtTemperatureSensor::TypeNameDht22))
	{
		ts = new DhtTemperatureSensor(sensorNum, DhtSensorType::Dht22);
	}
	else if (ReducedStringEquals(typeName, DhtHumiditySensor::TypeName))
	{
		ts = new DhtHumiditySensor(sensorNum);
	}
#endif
#if SUPPORT_BME280
	else if (ReducedStringEquals(typeName, BME280TemperatureSensor::TypeName))
	{
		ts = new BME280TemperatureSensor(sensorNum);
	}
	else if (ReducedStringEquals(typeName, BME280PressureSensor::TypeName))
	{
		ts = new BME280PressureSensor(sensorNum);
	}
	else if (ReducedStringEquals(typeName, BME280HumiditySensor::TypeName))
	{
		ts = new BME280HumiditySensor(sensorNum);
	}
#endif
#if HAS_CPU_TEMP_SENSOR
	else if (ReducedStringEquals(typeName, CpuTemperatureSensor::TypeName))
	{
		ts = new CpuTemperatureSensor(sensorNum);
	}
#endif
#if HAS_SMART_DRIVERS
	else if (ReducedStringEquals(typeName, TmcDriverTemperatureSensor::TypeName))
	{
		ts = new TmcDriverTemperatureSensor(sensorNum);
	}
#endif
	else
	{
		ts = nullptr;
		reply.printf("Unknown sensor type name \"%s\"", typeName);
	}

	return ts;
}

// Shared function used by two derived classes
//	 pt100rtd list of resistances v temperature
//
//	DIN 43760 / IEC 751 resistance values (ohms) were multiplied by 100 and
//	converted to 16 bit unsigned integers with no loss of accuracy.
//
//	Examples:
//	1852 represents 18.52 ohms and corresponds to a temperature of -200C.
//	10000 ==> 100.00 ohms @   0C
//	13851 ==> 138.51 ohms @ 100C

const float CelsiusMin = -100.0;					// starting temperature of the temp table below
const float CelsiusInterval = 10.0;

static const uint16_t tempTable[] =
{
	6026,  6430,  6833,  7233,  7633,  8031,  8427,  8822,  9216,  9609,
	10000, 10390, 10779, 11167, 11554, 11940, 12324, 12708, 13090, 13471,
	13851, 14229, 14607, 14983, 15358, 15733, 16105, 16477, 16848, 17217,
	17586, 17953, 18319, 18684, 19047, 19410, 19771, 20131, 20490, 20848,
	21205, 21561, 21915, 22268, 22621, 22972, 23321, 23670, 24018, 24364,
	24709, 25053, 25396, 25738, 26078, 26418, 26756, 27093, 27429, 27764,
	28098, 28430, 28762, 29092, 29421, 29749, 30075, 30401, 30725, 31048,
	31371, 31692, 32012, 32330, 32648, 32964, 33279, 33593, 33906, 34218,
	34528, 34838, 35146, 35453, 35759, 36064, 36367, 36670, 36971, 37271,
	37570, 37868, 38165, 38460, 38755, 39048
};

const size_t NumTempTableEntries = sizeof(tempTable)/sizeof(tempTable[0]);

/*static*/ TemperatureError TemperatureSensor::GetPT100Temperature(float& t, uint16_t ohmsx100)
{

	// Formally-verified binary search routine, adapted from one of the eCv examples
	size_t low = 0u, high = NumTempTableEntries;
	while (high > low)
	keep(low <= high; high <= NumTempTableEntries)
	keep(low == 0u || tempTable[low - 1u] < ohmsx100)
	keep(high == NumTempTableEntries || ohmsx100 <= tempTable[high])
	decrease(high - low)
	{
		size_t mid = (high - low)/2u + low;			// get the mid point, avoiding arithmetic overflow
		if (ohmsx100 <= tempTable[mid])
		{
			high = mid;
		}
		else
		{
			low = mid + 1u;
		}
	}
	_ecv_assert(low <= NumTempTableEntries);
	_ecv_assert(low == 0 || tempTable[low - 1] < ohmsx100);
	_ecv_assert(low == NumTempTableEntries || ohmsx100 <= tempTable[low]);

	if (low == 0)									// if off the bottom of the table
	{
		t = BadErrorTemperature;
		return TemperatureError::shortCircuit;
	}

	if (low >= NumTempTableEntries)					// if off the top of the table
	{
		t = BadErrorTemperature;
		return TemperatureError::openCircuit;
	}

	const float temperatureFraction = (float)(ohmsx100 - tempTable[low - 1])/(float)(tempTable[low] - tempTable[low - 1]);
	t = CelsiusInterval * (low - 1 + temperatureFraction) + CelsiusMin;

	//debugPrintf("raw %f low %u temp %f\n", ohmsx100, low, t);
	return TemperatureError::ok;
}

// End
