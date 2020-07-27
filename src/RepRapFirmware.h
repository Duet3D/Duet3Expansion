/*
 * RepRapFirmwareASF4.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_REPRAPFIRMWARE_H_
#define SRC_REPRAPFIRMWARE_H_

#include <CoreIO.h>
//#include <cmath>
//#include <cinttypes>
//#include <climits>		// for CHAR_BIT

typedef double floatc_t;

#include "Configuration.h"
#include <General/String.h>
#include <General/StringFunctions.h>
#include <General/Bitmap.h>
#include <General/SimpleMath.h>
#include "MessageType.h"

// Warn of what's to come, so we can use pointers to classes without including the entire header files
class Move;
class DDA;
class DriveMovement;
class Kinematics;
class TemperatureSensor;
class FilamentMonitor;

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#define DEBUG_HERE do { } while (false)
//#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

#define RAMFUNC __attribute__((section(".ramfunc")))

// Classes to facilitate range-based for loops that iterate from 0 up to just below a limit
template<class T> class SimpleRangeIterator
{
public:
	SimpleRangeIterator(T value_) : val(value_) {}
    bool operator != (SimpleRangeIterator<T> const& other) const { return val != other.val;     }
    T const& operator*() const { return val; }
    SimpleRangeIterator& operator++() { ++val; return *this; }

private:
    T val;
};

template<class T> class SimpleRange
{
public:
	SimpleRange(T limit) : _end(limit) {}
	SimpleRangeIterator<T> begin() const { return SimpleRangeIterator<T>(0); }
	SimpleRangeIterator<T> end() const { return SimpleRangeIterator<T>(_end); 	}

private:
	const T _end;
};

// Macro to create a SimpleRange from an array
#define ARRAY_INDICES(_arr) (SimpleRange<size_t>(ARRAY_SIZE(_arr)))

// A simple milliseconds timer class
class MillisTimer
{
public:
	MillisTimer() { running = false; }
	void Start();
	void Stop() { running = false; }
	bool Check(uint32_t timeoutMillis) const;
	bool CheckAndStop(uint32_t timeoutMillis);
	bool IsRunning() const { return running; }

private:
	uint32_t whenStarted;
	bool running;
};

constexpr size_t ScratchStringLength = 220;							// standard length of a scratch string, enough to print delta parameters to
constexpr size_t ShortScratchStringLength = 50;

constexpr size_t XYZ_AXES = 3;										// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2;				// The indices of the Cartesian axes in drive arrays
constexpr size_t U_AXIS = 3;										// The assumed index of the U motor when aligning a rotary axis

// Common conversion factors
constexpr unsigned int MinutesToSeconds = 60;
constexpr float SecondsToMinutes = 1.0/(float)MinutesToSeconds;
constexpr unsigned int SecondsToMillis = 1000.0;
constexpr float MillisToSeconds = 1.0/(float)SecondsToMillis;
constexpr float InchToMm = 25.4;
constexpr float Pi = 3.141592653589793;
constexpr float TwoPi = 3.141592653589793 * 2;
constexpr float DegreesToRadians = 3.141592653589793/180.0;
constexpr float RadiansToDegrees = 180.0/3.141592653589793;

#define DEGREE_SYMBOL	"\xC2\xB0"									// degree-symbol encoding in UTF8

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
#endif

// Macro to give us the highest valid index into an array i.e. one less than the size
#define ARRAY_UPB(_x)	(ARRAY_SIZE(_x) - 1)

// TEMPORARY stuff
//TODO move these to the right place

extern Move *moveInstance;

namespace RepRap
{
	void Init();
	void Spin();
	void Tick();
}

// Module numbers and names, used for diagnostics and debug
enum Module : uint8_t
{
	modulePlatform = 0,
	moduleNetwork = 1,
	moduleWebserver = 2,
	moduleGcodes = 3,
	moduleMove = 4,
	moduleHeat = 5,
	moduleDda = 6,
	moduleRoland = 7,
	moduleScanner = 8,
	modulePrintMonitor = 9,
	moduleStorage = 10,
	modulePortControl = 11,
	moduleDuetExpansion = 12,
	moduleFilamentSensors = 13,
	moduleWiFi = 14,
	moduleDisplay = 15,
	numModules = 16,				// make this one greater than the last module number
	noModule = 16
};

extern const char * const moduleName[];

#include "Config/BoardDef.h"

typedef Bitmap<uint32_t> AxesBitmap;				// Type of a bitmap representing a set of axes
typedef Bitmap<uint32_t> DriversBitmap;				// Type of a bitmap representing a set of driver numbers
typedef Bitmap<uint32_t> FansBitmap;				// Type of a bitmap representing a set of fan numbers
typedef Bitmap<uint64_t> SensorsBitmap;				// Type of a bitmap representing sensors

static_assert(MaxFans <= FansBitmap::MaxBits());
static_assert(MaxSensors <= SensorsBitmap::MaxBits());

// Task priorities
namespace TaskPriority
{
	static constexpr int SpinPriority = 1;							// priority for tasks that rarely block
	static constexpr int HeatPriority = 2;
	static constexpr int DhtPriority = 2;
	static constexpr int TmcPriority = 2;
	static constexpr int AinPriority = 2;
	static constexpr int HeightFollowingPriority = 2;
	static constexpr int DueXPriority = 3;
	static constexpr int LaserPriority = 3;
	static constexpr int CanSenderPriority = 3;
	static constexpr int CanReceiverPriority = 3;
	static constexpr int CanAsyncSenderPriority = 4;
}

#endif /* SRC_REPRAPFIRMWARE_H_ */
