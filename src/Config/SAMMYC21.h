/*
 * SAMMYC21.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CONFIG_SAMMYC21_H_
#define SRC_CONFIG_SAMMYC21_H_

#include <Hardware/PinDescription.h>
#include <SPI/SpiParameters.h>
#include <I2C/I2cParameters.h>
#include <UART/UartParameters.h>

#define BOARD_TYPE_NAME		"SAMMYC21"
#define BOOTLOADER_NAME		"SAMMYC21"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1
#define USE_SERIAL_DEBUG		1

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		0
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	1
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_INPUT_SHAPING	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240_SPI		0

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;				// two channels used
constexpr DmaChannel DmacChanSspiTx = 4;
constexpr DmaChannel DmacChanSspiRx = 5;

constexpr unsigned int NumDmaChannelsUsed = 6;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

// DMA priorities, higher is better. 0 to 3 are available.
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioSspiTx = 0;
constexpr DmaPriority DmacPrioSspiRx = 2;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C = 2;
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts

// Stepper drivers
constexpr size_t NumDrivers = 1;

#define USE_CCL		0			// USE_CCL also requires DIFFERENTIAL_STEPPER_OUTPUTS

#if USE_CCL

PortGroup * const StepPio = &(PORT->Group[1]);		// the PIO that all the step pins are on
constexpr Pin EnablePins[NumDrivers] = { PortAPin(9) };
constexpr Pin StepPins[NumDrivers] = { PortBPin(10) };
constexpr Pin InvertedStepPins[NumDrivers] = { PortAPin(11) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(10) };

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_ENABLE		0		// 1 = active high, 0 = active low

#else

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on
constexpr Pin EnablePins[NumDrivers] = { PortAPin(9) };
constexpr Pin StepPins[NumDrivers] = { PortAPin(11) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(10) };

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_ENABLE		0		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_LIS3DH			1
#define SUPPORT_LDC1612			1
#define SUPPORT_SDADC			0

#define NUM_I2C_CHANNELS		1
#define NUM_SHARED_SPI			1

#define USE_MPU					0
#define USE_CACHE				0

#define DIAG_SERCOM_NUMBER		5		// which SERCOM device we use for debugging output

constexpr unsigned int CANInstanceNumber = 0;
constexpr bool UseLaterCanPins = true;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 2200.0;

constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(2), PortAPin(3) };

constexpr Pin CanStandbyPin = PortAPin(27);

constexpr Pin ButtonPins[] = { PortBPin(9) };

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(28) };
constexpr bool LedActiveHigh = true;

#if SUPPORT_SPI_SENSORS

// Shared SPI using pins PA16,17,18. If changing this, also change the available pins in the pin table.
constexpr SpiParameters SharedSpiParams =
{
	.sercomNumber = 1,
	.mosiPin = PortAPin(16),
	.misoPin = PortAPin(18),
	.sclkPin = PortAPin(17),
	.pinFunction = GpioPinFunction::C,
	.dataInPad = 2,
	.dataOutPad = 0,
	.dmaChanTx = DmacChanSspiTx,
	.dmaChanRx = DmacChanSspiRx,
	.dmaPrioTx = DmacPrioSspiTx,
	.dmaPrioRx = DmacPrioSspiRx,
};

#endif

#if NUM_I2C_CHANNELS != 0

// I2C using pins PA22,23. If changing this, also change the available pins in the pin table.
constexpr I2cParameters I2C0Params =
{
	.sercomNumber = 3,
	.sclPin = PortAPin(23),
	.sdaPin = PortAPin(22),
	.pinFunction = GpioPinFunction::C,
	.irqPriority = NvicPriorityI2C,
	.rxDmaChannel = NoDmaChannel
};

#endif

#if SUPPORT_LIS3DH
# define ACCELEROMETER_USES_SPI			(0)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr unsigned int Lis_I2CChannel = 0;
constexpr Pin Lis3dhInt1Pin = PortAPin(13);
#endif

#if SUPPORT_LDC1612
constexpr unsigned int LDC1612_I2CChannel = 0;
constexpr uint16_t LDC1612_I2CAddress = 0x2B;				// pin 4 is tied high on the Grove board
constexpr Pin LDC1612InterruptPin = PortAPin(21);
#endif

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA00 not on board
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA01 not on board
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"temp0"		},	// PA02 thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"temp1"		},	// PA03 thermistor
	{ TcOutput::none,	TccOutput::tcc0_0E,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"pa04"		},	// PA04
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"pa05"		},	// PA05
	{ TcOutput::none,	TccOutput::tcc1_0E,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"pa06"		},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"pa07"		},	// PA07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa08"		},	// PA08
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA09 driver EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA10 driver DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA11 driver STEP
	{ TcOutput::none,	TccOutput::tcc2_0E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	"pa12"	 	},	// PA12
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	nullptr		},	// PA13 accelerometer INT1
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	"pa13"		},	// PA13
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA15 crystal
#if SUPPORT_SPI_SENSORS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	Nx,	nullptr		},	// PA16 SPI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		Nx,	nullptr		},	// PA17 SPI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr	 	},	// PA18 SPI
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	0,	"pa16"		},	// PA16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		1,	"pa17"		},	// PA17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"pa18"	 	},	// PA18
#endif
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	"pa19"		},	// PA19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa20"		},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa21"		},	// PA21 LDC1612 INT
#if NUM_I2C_CHANNELS != 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA22 I2C
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA23 I2C
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa22"		},	// PA22 (has TC0.0 on that pin but can't control the frequency well)
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa23"		},	// PA23
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa24"		},	// PA24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa25"		},	// PA25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA27 CAN transceiver standby
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA28 LED 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB02 USB interface (SERCOM5 pad 0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB03 USB interface (SERCOM5 pad 1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"pb08"		},	// PB08
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"!^button0"	},	// PB09 button recognised by bootloader
	// PB22/23 are used for CAN0, PB10/11 for CAN1

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,	Nx,	"i2c.lis3dh"		},	// LIS3DH sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::ldc1612,	AdcInput::none,		SercomIo::none,		SercomIo::none,	Nx,	"i2c.ldc1612"		},	// LDC1612 sensor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 10;			// 32 pins on port A (some missing), only PB08 and PB09 are brought out on this board
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612;

static_assert(NumPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// Available UART ports
#define NUM_ASYNC_PORTS		1

// SERCOM5 is connected to the USB port through the serial-to-USB converter
constexpr UartParameters Serial0Params =
{
	.sercomNumber = 5,
	.rxPin = PortBPin(3),
	.txPin = PortBPin(2),
	.pinFunction = GpioPinFunction::D,
	.dataInPad = 1,
	.dataOutPad = 0,
	.numRxSlots = 32,
	.numTxSlots = 128
};

#endif /* SRC_CONFIG_SAMMYC21_H_ */
