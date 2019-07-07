#include "TemperatureSensor.h"
#include "Thermistor.h"
#include "ThermocoupleSensor31855.h"
#include "ThermocoupleSensor31856.h"
#include "RtdSensor31865.h"
#include "CurrentLoopTemperatureSensor.h"
#include "LinearAnalogSensor.h"

#if HAS_CPU_TEMP_SENSOR
#include "CpuTemperatureSensor.h"
#endif

#if SUPPORT_DHT_SENSOR
#include "DhtSensor.h"
#endif

#if HAS_SMART_DRIVERS
#include "TmcDriverTemperatureSensor.h"
#endif

// Constructor
TemperatureSensor::TemperatureSensor(unsigned int chan) : sensorChannel(chan), lastError(TemperatureError::success) {}

// Virtual destructor
TemperatureSensor::~TemperatureSensor()
{
}

// Default config function for sensors with nothing to configure
GCodeResult TemperatureSensor::Configure(unsigned int heater, const CanMessageM305& msg, const StringRef& reply)
{
	return GCodeResult::ok;
}

// Try to get a temperature reading
TemperatureError TemperatureSensor::GetTemperature(float& t)
{
	const TemperatureError rslt = TryGetTemperature(t);
	if (rslt != TemperatureError::success)
	{
		lastError = rslt;
	}
	return rslt;
}

// Factory method
TemperatureSensor *TemperatureSensor::Create(unsigned int channel)
{
	TemperatureSensor *ts = nullptr;
	if (channel < NumThermistorInputs)
	{
		ts = new Thermistor(channel, false);
	}
	else if (FirstLinearAnalogChannel <= channel && channel < FirstLinearAnalogChannel + NumThermistorInputs)
	{
		ts = new LinearAnalogSensor(channel);
	}
	else if (FirstPT1000Channel <= channel && channel < FirstPT1000Channel + NumThermistorInputs)
	{
		ts = new Thermistor(channel, true);
	}
#if SUPPORT_SPI_SENSORS
	else if (FirstMax31855ThermocoupleChannel <= channel && channel < FirstMax31855ThermocoupleChannel + MaxSpiTempSensors)
	{
		ts = new ThermocoupleSensor31855(channel);
	}
	else if (FirstMax31856ThermocoupleChannel <= channel && channel < FirstMax31856ThermocoupleChannel + MaxSpiTempSensors)
	{
		ts = new ThermocoupleSensor31856(channel);
	}
	else if (FirstRtdChannel <= channel && channel < FirstRtdChannel + MaxSpiTempSensors)
	{
		ts = new RtdSensor31865(channel);
	}
	else if (FirstLinearAdcChannel <= channel && channel < FirstLinearAdcChannel + MaxSpiTempSensors)
	{
		ts = new CurrentLoopTemperatureSensor(channel);
	}
#endif
#if SUPPORT_DHT_SENSOR
	else if (FirstDhtTemperatureChannel <= channel && channel < FirstDhtTemperatureChannel + MaxSpiTempSensors)
	{
		ts = new DhtTemperatureSensor(channel);
	}
	else if (FirstDhtHumidityChannel <= channel && channel < FirstDhtHumidityChannel + MaxSpiTempSensors)
	{
		ts = new DhtHumiditySensor(channel);
	}
#endif
#if HAS_CPU_TEMP_SENSOR
	else if (channel == CpuTemperatureSenseChannel)
	{
		ts = new CpuTemperatureSensor(channel);
	}
#endif
#if HAS_SMART_DRIVERS
	else if (channel >= FirstTmcDriversSenseChannel && channel < FirstTmcDriversSenseChannel + NumTmcDriversSenseChannels)
	{
		ts = new TmcDriverTemperatureSensor(channel);
	}
#endif

	if (ts != nullptr)
	{
		ts->Init();
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
	assert(low <= NumTempTableEntries);
	assert(low == 0 || tempTable[low - 1] < ohmsx100);
	assert(low == NumTempTableEntries || ohmsx100 <= tempTable[low]);

	if (low == 0)									// if off the bottom of the table
	{
		return TemperatureError::shortCircuit;
	}

	if (low >= NumTempTableEntries)					// if off the top of the table
	{
		return TemperatureError::openCircuit;
	}

	const float temperatureFraction = (float)(ohmsx100 - tempTable[low - 1])/(float)(tempTable[low] - tempTable[low - 1]);
	t = CelsiusInterval * (low - 1 + temperatureFraction) + CelsiusMin;

	//debugPrintf("raw %f low %u temp %f\n", ohmsx100, low, t);
	return TemperatureError::success;
}

// End
