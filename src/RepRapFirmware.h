/*
 * RepRapFirmwareASF4.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_REPRAPFIRMWARE_H_
#define SRC_REPRAPFIRMWARE_H_

#include <CoreIO.h>

// Define floating point type to use for calculations where we would like high precision in matrix calculations
#if SAME70
typedef double floatc_t;						// type of matrix element used for calibration
#else
// We are more memory-constrained on the other processors and they don't support double precision
typedef float floatc_t;							// type of matrix element used for calibration
#endif

#include <Config/BoardDef.h>
#include <Config/Configuration.h>
#include <General/String.h>
#include <General/StringFunctions.h>
#include <General/Bitmap.h>
#include <General/SimpleMath.h>
#include <GCodeResult.h>

// Warn of what's to come, so we can use pointers to classes without including the entire header files
#if SUPPORT_DRIVERS
class Move;
class DDA;
class DriveMovement;
class Kinematics;
#endif
#if SUPPORT_CLOSED_LOOP
class ClosedLoop;
#endif


class TemperatureSensor;
class FilamentMonitor;

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) noexcept __attribute__ ((format (printf, 1, 2)));
extern "C" void debugVprintf(const char *fmt, va_list vargs) noexcept;

#define DEBUG_HERE do { } while (false)
//#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

#define RAMFUNC __attribute__((section(".ramfunc")))

#define SPEED_CRITICAL	__attribute__((optimize("O2")))

// Classes to facilitate range-based for loops that iterate from 0 up to just below a limit
template<class T> class SimpleRangeIterator
{
public:
	SimpleRangeIterator(T value_) noexcept : val(value_) {}
    bool operator != (SimpleRangeIterator<T> const& other) const noexcept { return val != other.val;     }
    T const& operator*() const noexcept { return val; }
    SimpleRangeIterator& operator++() noexcept { ++val; return *this; }

private:
    T val;
};

template<class T> class SimpleRange
{
public:
	SimpleRange(T limit) noexcept : _end(limit) {}
	SimpleRangeIterator<T> begin() const noexcept { return SimpleRangeIterator<T>(0); }
	SimpleRangeIterator<T> end() const noexcept { return SimpleRangeIterator<T>(_end); 	}

private:
	const T _end;
};

// Macro to create a SimpleRange from an array
#define ARRAY_INDICES(_arr) (SimpleRange<size_t>(ARRAY_SIZE(_arr)))

// Function to delete an object and clear the pointer. Safe to call even if the pointer is already null.
template <typename T> void DeleteObject(T*& ptr) noexcept
{
	T *null p2 = nullptr;
	std::swap(ptr, p2);
	delete p2;
}

// Function to delete an array of objects and clear the pointer. Safe to call even if the pointer is already null.
template <typename T> void DeleteArray(T*& ptr) noexcept
{
	T *null p2 = nullptr;
	std::swap(ptr, p2);
	delete[] p2;
}

// Function to make a pointer point to a new object and delete the existing object, if any. T2 must be the same as T or derived from it.
template <typename T, typename T2> void ReplaceObject(T*& ptr, T2* pNew) noexcept
{
	T *null p2 = static_cast<T *null>(pNew);
	std::swap(ptr, p2);
	delete p2;
}

constexpr const char* NoPinName = "nil";

constexpr size_t XYZ_AXES = 3;										// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2;				// The indices of the Cartesian axes in drive arrays

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

constexpr uint32_t UpdateBootloaderMagicValue = 0x0b00140ad;		// magic number that we write to the user area word 3 to indicate that the bootloader is to be updated
constexpr size_t UpdateBootloaderMagicWordIndex = 9;				// which word in the user area we write the value to

#if RP2040
constexpr uint32_t UpdateFirmwareMagicValue = 0x0b00240ae;			// magic number that we write to the watchdog scratch word 0 to indicate that a firmware update is needed
constexpr size_t UpdateFirmwareMagicWordIndex = 0;					// which word in the watchdog scratch words we write the value to
#endif

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
#endif

// Macro to give us the highest valid index into an array i.e. one less than the size
#define ARRAY_UPB(_x)	(ARRAY_SIZE(_x) - 1)

#if SUPPORT_DRIVERS
extern Move *moveInstance;
#endif

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

#endif /* SRC_REPRAPFIRMWARE_H_ */
