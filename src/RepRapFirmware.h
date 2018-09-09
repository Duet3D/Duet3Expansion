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

#include <cmath>
#include <cinttypes>
#include "General/StringRef.h"

// These three are implemented in Tasks.cpp
void delay(uint32_t ms);
uint32_t millis();
uint64_t millis64();

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

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
#endif

// Macro to give us the highest valid index into an array i.e. one less than the size
#define ARRAY_UPB(_x)	(ARRAY_SIZE(_x) - 1)

// TEMPORARY stuff
//TODO move these to the right place

#ifdef __SAME51N19A__
# define SAME51		1
#endif

typedef uint8_t Pin;
constexpr Pin NoPin = 0xFF;
typedef uint16_t PwmFrequency;

#define PORTA_PIN(_n)	(GPIO(GPIO_PORTA, (_n)))
#define PORTB_PIN(_n)	(GPIO(GPIO_PORTB, (_n)))
#define PORTC_PIN(_n)	(GPIO(GPIO_PORTC, (_n)))
#define PORTD_PIN(_n)	(GPIO(GPIO_PORTD, (_n)))

typedef uint8_t DmaChannel;

#include "Pins.h"

#endif /* SRC_REPRAPFIRMWARE_H_ */
