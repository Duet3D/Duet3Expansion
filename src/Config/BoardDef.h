/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

#include <Duet3Common.h>								// this file is in the CANlib project because both main and expansion boards need it
#include <RRF3Common.h>

#if defined(EXP3HC)
# include "EXP3HC.h"
#elif defined(TOOL1LC)
# include "TOOL1LC.h"
#elif defined(TOOL1RR)
# include "TOOL1RR.h"
#elif defined(EXP1XD)
# include "EXP1XD.h"
#elif defined(EXP1HCL)
# include "EXP1HCL.h"
#elif defined(SAMMYC21)
# include "SAMMYC21.h"
#elif defined(ATECM)
# include "ATECM.h"
#elif defined(ATEIO)
# include "ATEIO.h"
#elif defined(RPI_PICO)
# include "RPi_Pico.h"
#elif defined(M23CL)
# include "M23CL.h"
#elif defined(SZP)
# include "SZP.h"
#elif defined(F3PTB)
# include "F3PTB.h"
#else
# error Board type not defined
#endif

// Default board features
#ifndef DIFFERENTIAL_STEPPER_OUTPUTS
# define DIFFERENTIAL_STEPPER_OUTPUTS	0
#endif

#ifndef USE_TC_FOR_STEP
# define USE_TC_FOR_STEP				0
#endif

#ifndef SUPPORT_CLOSED_LOOP
# define SUPPORT_CLOSED_LOOP			0
#endif

#ifndef SUPPORT_BRAKE_PWM
# define SUPPORT_BRAKE_PWM				0
#endif

#ifndef DEDICATED_STEP_TIMER
# define DEDICATED_STEP_TIMER			0
#endif

#if !SUPPORT_DRIVERS
# define HAS_SMART_DRIVERS				0
# define SUPPORT_TMC22xx				0
# define SUPPORT_TMC51xx				0
# define SUPPORT_TMC2160				0
# define SUPPORT_SLOW_DRIVERS			0
constexpr size_t NumDrivers = 0;
#endif

#if !defined(SUPPORT_BME280)
# define SUPPORT_BME280					(SUPPORT_SPI_SENSORS)
#endif

#if !defined(SUPPORT_LIS3DH)
# define SUPPORT_LIS3DH					0
#endif

#if !defined(USE_SERIAL_DEBUG)
# define USE_SERIAL_DEBUG				0
#endif

#ifndef SUPPORT_LED_STRIPS
# define SUPPORT_LED_STRIPS				1
#endif

#ifndef SUPPORT_DMA_NEOPIXEL
# define SUPPORT_DMA_NEOPIXEL			0
#endif

#ifndef SUPPORT_PIO_NEOPIXEL
# define SUPPORT_PIO_NEOPIXEL			(SUPPORT_LED_STRIPS && RP2040)
#endif

#ifndef SUPPORT_LDC1612
# define SUPPORT_LDC1612				0
#endif

#ifndef SUPPORT_AS5601
# define SUPPORT_AS5601					0
#endif

#ifndef SUPPORT_TCA6408A
# define SUPPORT_TCA6408A				0
#endif

#ifndef USE_SPICAN
# define USE_SPICAN						0
#endif

#ifndef BOARD_USES_UF2_BINARY
# define BOARD_USES_UF2_BINARY			0
#endif

#endif /* SRC_CONFIG_BOARDDEF_H_ */
