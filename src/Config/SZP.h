/*
 * SZP.h
 *
 *  Created on: 19 Jul 2023
 *      Author: David
 */

#ifndef SRC_CONFIG_SZP_H_
#define SRC_CONFIG_SZP_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"SZP"
#define BOOTLOADER_NAME		"SAMC21"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define USE_SERIAL_DEBUG		defined(DEBUG)

// Drivers configuration
#define SUPPORT_DRIVERS			0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240			0

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		1
#define SUPPORT_LIS3DH			1
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			1
#define SUPPORT_LDC1612			1

#define USE_MPU					0
#define USE_CACHE				0

#define DIAG_SERCOM_NUMBER		0		// which SERCOM device we use for debugging output

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 1;
constexpr float DefaultThermistorSeriesR = 2200.0;
// Thermistor is a 10K Murata NCU15XH103J6SRC. B25/50 = 3380, B25/80 = 3428, B25/85 = 3434, B25/100 = 3455
// From this we deduce R25 = 10000, R50 = 4160.1, R80 = 1668.5, R85 = 1452.2, R100 = 973.8
// The following Beta and C values use the 25, 50 and 65C values
constexpr float DefaultThermistorR25_SZP = 10000;
constexpr float DefaultThermistorBeta_SZP = 3425.0;
constexpr float DefaultThermistorC_SZP = 1.68e-7;

constexpr Pin BoardTypePin = PortAPin(5);

constexpr Pin VinMonitorPin = PortAPin(10);
constexpr float VinDividerRatio = (10.0 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 2.5;				// we use a 2.5V voltage reference

constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(7) };

// Pins for future analog mode
constexpr Pin AnalogModeSelectPin = PortAPin(27);
constexpr Pin AnalogModeOutputPin = PortAPin(2);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

#if SUPPORT_I2C_SENSORS

// I2C using pins PA16,17
constexpr uint8_t I2CSercomNumber = 1;
constexpr Pin I2CSDAPin = PortAPin(16);
constexpr GpioPinFunction I2CSDAPinPeriphMode = GpioPinFunction::C;
constexpr Pin I2CSCLPin = PortAPin(17);
constexpr GpioPinFunction I2CSCLPinPeriphMode = GpioPinFunction::C;
#define I2C_HANDLER		SERCOM1_Handler

#endif

#if SUPPORT_LIS3DH
# define ACCELEROMETER_USES_SPI			(0)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr bool Lis3dhAddressLsb = true;
constexpr Pin Lis3dhInt1Pin = PortAPin(19);
#endif

#if SUPPORT_LDC1612
constexpr uint16_t LDC1612_I2CAddress = 0x2A;				// pin 4 is tied low on this board
constexpr Pin LDC1612ClockGenPin = PortAPin(23);
constexpr Pin LDC1612InterruptPin = PortAPin(22);
#endif

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::tcc2_0E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom0d,	0,	"pa00"			},	// PA00 test pad
	{ TcOutput::none,	TccOutput::tcc2_1E,	AdcInput::none,		AdcInput::none,		SercomIo::sercom0d,	SercomIo::none,		1,	"pa01"			},	// PA01 test pad
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA02 DAC out for analog mode
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 VREFA for ADC, tied to +2.5V
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 VREFB for SDADC, tied to +2.5V
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 board type (analog)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA06 SDADC INN0, tied to Vssa
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::sdadc_0,	SercomIo::none,		SercomIo::none,		Nx,	"temp0,coiltemp" },	// PA07 Temp0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::sercom2d,	Nx,	nullptr			},	// PA08 UART Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::sercom2d,	SercomIo::none,		9,	nullptr			},	// PA09 UART Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA10 5V monitor
	{ TcOutput::none,	TccOutput::tcc1_1E,	AdcInput::adc0_11,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	"pa11"			},	// PA11 test pad
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA12 unused (not on SAMC21E)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 unused (not on SAMC21E)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 SCL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA18 CAN reset jumper
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	nullptr			},	// PA19 accelerometer interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA20 unused (not on SAMC21E)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA21 unused (not on SAMC21E)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr			},	// PA22 LDC interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA23 GCLK to LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA27 analog mode select pad
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"pa28"			},	// PA28 test pad
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk, also LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio, also LED1

	// Port B
	// We don't use any Port B pins so that we could switch to a smaller version of the SAMC21

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.lis3dh"	},	// LIS3DH sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::ldc1612,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.ldc1612"	},	// LDC1612 sensor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32;				// 32 pins on port A (some missing)
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612;

static_assert(NumPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		1
constexpr IRQn Serial0_IRQn = SERCOM4_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanAdc0Rx = 2;
constexpr DmaChannel DmacChanSdadcRx = 3;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

// DMA priorities, higher is better. 0 to 3 are available.
constexpr DmaPriority DmacPrioAdcRx = 2;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C = 2;
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts

#endif /* SRC_CONFIG_SZP_H_ */
