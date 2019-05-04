/*
 * RepRapFirmwareASF4.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_REPRAPFIRMWARE_H_
#define SRC_REPRAPFIRMWARE_H_

#include "atmel_start.h"
#include "ecv.h"
#undef value			// needed because we include <optional>

#include <cmath>
#include <cinttypes>
#include <climits>		// for CHAR_BIT

#ifdef __SAME51N19A__
# define SAME51		1
#endif

typedef uint16_t PwmFrequency;		// type used to represent a PWM frequency. 0 sometimes means "default".
typedef double floatc_t;

#include "Configuration.h"
#include "General/StringRef.h"
#include "MessageType.h"

// Warn of what's to come, so we can use pointers to classes without including the entire header files
class Move;
class DDA;
class DriveMovement;
class Kinematics;
class Heat;
class PID;
class TemperatureSensor;
class OutputBuffer;
class OutputStack;
//class GCodeBuffer;
//class GCodeQueue;
//class FilamentMonitor;
class Logger;

#if SUPPORT_IOBITS
class PortControl;
#endif

#if SUPPORT_12864_LCD
class Display;
#endif

// These three are implemented in Tasks.cpp
void delay(uint32_t ms);
uint32_t millis();
uint64_t millis64();

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#define DEBUG_HERE do { } while (false)
//#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

// Functions to change the base priority, to shut out interrupts up to a priority level

// Get the base priority and shut out interrupts lower than or equal to a specified priority
inline uint32_t ChangeBasePriority(uint32_t prio)
{
	const uint32_t oldPrio = __get_BASEPRI();
	__set_BASEPRI_MAX(prio << (8 - __NVIC_PRIO_BITS));
	return oldPrio;
}

// Restore the base priority following a call to ChangeBasePriority
inline void RestoreBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio);
}

// Set the base priority when we are not interested in the existing value i.e. definitely in non-interrupt code
inline void SetBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio << (8 - __NVIC_PRIO_BITS));
}

// Helper functions to work on bitmaps of various lengths.
// The primary purpose of these is to allow us to switch between 16, 32 and 64-bit bitmaps.

// Convert an unsigned integer to a bit in a bitmap
template<typename BitmapType> inline constexpr BitmapType MakeBitmap(unsigned int n)
{
	return (BitmapType)1u << n;
}

// Make a bitmap with the lowest n bits set
template<typename BitmapType> inline constexpr BitmapType LowestNBits(unsigned int n)
{
	return ((BitmapType)1u << n) - 1;
}

// Check if a particular bit is set in a bitmap
template<typename BitmapType> inline constexpr bool IsBitSet(BitmapType b, unsigned int n)
{
	return (b & ((BitmapType)1u << n)) != 0;
}

// Set a bit in a bitmap
template<typename BitmapType> inline void SetBit(BitmapType &b, unsigned int n)
{
	b |= ((BitmapType)1u << n);
}

// Clear a bit in a bitmap
template<typename BitmapType> inline void ClearBit(BitmapType &b, unsigned int n)
{
	b &= ~((BitmapType)1u << n);
}

// Convert an array of longs to a bit map with overflow checking
template<typename BitmapType> BitmapType UnsignedArrayToBitMap(const uint32_t *arr, size_t numEntries)
{
	BitmapType res = 0;
	for (size_t i = 0; i < numEntries; ++i)
	{
		const uint32_t f = arr[i];
		if (f < sizeof(BitmapType) * CHAR_BIT)
		{
			SetBit(res, f);
		}
	}
	return res;
}

template<class X> inline constexpr X min(X _a, X _b)
{
	return (_a < _b) ? _a : _b;
}

template<class X> inline constexpr X max(X _a, X _b)
{
	return (_a > _b) ? _a : _b;
}

// Specialisations for float and double to handle NaNs properly
template<> inline constexpr float min(float _a, float _b)
{
	return (std::isnan(_a) || _a < _b) ? _a : _b;
}

template<> inline constexpr float max(float _a, float _b)
{
	return (std::isnan(_a) || _a > _b) ? _a : _b;
}

template<> inline constexpr double min(double _a, double _b)
{
	return (std::isnan(_a) || _a < _b) ? _a : _b;
}

template<> inline constexpr double max(double _a, double _b)
{
	return (std::isnan(_a) || _a > _b) ? _a : _b;
}

inline constexpr float fsquare(float arg)
{
	return arg * arg;
}

inline constexpr double dsquare(double arg)
{
	return arg * arg;
}

inline constexpr uint64_t isquare64(int32_t arg)
{
	return (uint64_t)((int64_t)arg * arg);
}

inline constexpr uint64_t isquare64(uint32_t arg)
{
	return (uint64_t)arg * arg;
}

inline void swap(float& a, float& b)
{
	float temp = a;
	a = b;
	b = temp;
}

// Note that constrain<float> will return NaN for a NaN input because of the way we define min<float> and max<float>
template<class T> inline constexpr T constrain(T val, T vmin, T vmax)
{
	return max<T>(min<T>(val, vmax), vmin);
}

constexpr size_t ScratchStringLength = 220;							// standard length of a scratch string, enough to print delta parameters to
constexpr size_t ShortScratchStringLength = 50;

constexpr size_t XYZ_AXES = 3;										// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays
constexpr size_t CoreXYU_AXES = 5;									// The number of axes in a CoreXYU machine (there is a hidden V axis)
constexpr size_t CoreXYUV_AXES = 5;									// The number of axes in a CoreXYUV machine
constexpr size_t U_AXIS = 3, V_AXIS = 4;							// The indices of the U and V motors in a CoreXYU machine (needed by Platform)

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

// Standard callback function type
union CallbackParameter
{
	uint32_t u32;
	int32_t i32;
	void *vp;

	CallbackParameter() { u32 = 0; }
	CallbackParameter(unsigned int p) { u32 = p; }
	CallbackParameter(uint32_t p) { u32 = p; }
	CallbackParameter(int p) { i32 = p; }
	CallbackParameter(int32_t p) { i32 = p; }
	CallbackParameter(void *p) { vp = p; }
};

typedef void (*StandardCallbackFunction)(CallbackParameter);

// Atomic section locker, alternative to InterruptCriticalSectionLocker (may be faster)
class AtomicCriticalSectionLocker
{
public:
	AtomicCriticalSectionLocker() : basePrio(__get_PRIMASK())
	{
		__disable_irq();
		__DMB();
	}

	~AtomicCriticalSectionLocker()
	{
		__DMB();
		__set_PRIMASK(basePrio);
	}

private:
	uint32_t basePrio;
};

// Return true if we are in any interrupt service routine
static inline bool inInterrupt()
{
	return (__get_IPSR() & 0x01FF) != 0;
}

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
}

#ifdef __SAME51N19A__
# define SAME51		1
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

typedef uint8_t Pin;
constexpr Pin NoPin = 0xFF;

typedef uint16_t PwmFrequency;
typedef uint32_t AxesBitmap;				// Type of a bitmap representing a set of axes
typedef uint32_t DriversBitmap;				// Type of a bitmap representing a set of driver numbers
typedef uint32_t FansBitmap;				// Type of a bitmap representing a set of fan numbers

#define PortAPin(_n)	(GPIO(GPIO_PORTA, (_n)))
#define PortBPin(_n)	(GPIO(GPIO_PORTB, (_n)))
#define PortCPin(_n)	(GPIO(GPIO_PORTC, (_n)))
#define PortDPin(_n)	(GPIO(GPIO_PORTD, (_n)))

typedef uint8_t DmaChannel;

#include "Pins.h"

#endif /* SRC_REPRAPFIRMWARE_H_ */
