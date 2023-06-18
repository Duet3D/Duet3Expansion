/*
 * TOOL1ED_v04.h
 *
 *  Created on: 7 Nov 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXP1XD_H_
#define SRC_CONFIG_EXP1XD_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"EXP1XD"
#define BOOTLOADER_NAME		"SAMC21"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		0
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	1
#define SUPPORT_DELTA_MOVEMENT	0
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_BRAKE_PWM		1
#define DIFFERENTIAL_STEPPER_OUTPUTS	1
#define USE_TC_FOR_STEP			1

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0

constexpr size_t NumDrivers = 1;

#if DIFFERENTIAL_STEPPER_OUTPUTS

PortGroup * const StepPio = &(PORT->Group[1]);						// the PIO that all the step pins are on

constexpr Pin StepPins[NumDrivers] = { PortBPin(10) };
constexpr Pin InvertedStepPins[NumDrivers] = { PortAPin(11) };		// driven automatically by the CCL

constexpr Pin DirectionPins[NumDrivers] = { PortAPin(12) };
constexpr Pin InvertedDirectionPins[NumDrivers] = { PortAPin(10) };

constexpr Pin EnablePins[NumDrivers] = { PortAPin(3) };
constexpr Pin InvertedEnablePins[NumDrivers] = { PortBPin(11) };

#else

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on

constexpr Pin EnablePins[NumDrivers] = { PortAPin(3) };
constexpr Pin StepPins[NumDrivers] = { PortAPin(11) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(10) };

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_ENABLE		0		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			0

#define USE_MPU					0
#define USE_CACHE				0

#define DIAG_SERCOM_NUMBER		4		// which SERCOM device we use for debugging output

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 1;
constexpr float DefaultThermistorSeriesR = 2200.0;

constexpr Pin BoardTypePin = PortAPin(5);

constexpr Pin VinMonitorPin = PortAPin(8);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 5.0;				// we use the 5V supply as the voltage reference

constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(6) };

constexpr Pin JumperPin = PortAPin(27);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(19), PortAPin(18) };
constexpr bool LedActiveHigh = true;

// Table of pin functions that we are allowed to use
// Sercom0 is also available on pins PA0/PA1 but TX and RX are the wrong way round
// Sercom0 on PA16/PA17 is left in the pin table in case we decide to add a connector for thie UART in future
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	"io1.in"		},	// PA00
	{ TcOutput::none,	TccOutput::tcc2_1E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io0.out"		},	// PA01
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"io0.in"		},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "ate.d0.ena.p"	},	// PA03 driver EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 board type (analog)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"temp0"			},	// PA06
	{ TcOutput::none,	TccOutput::tcc1_1E,	AdcInput::adc0_7,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA08 VinMon
	{ TcOutput::none,	TccOutput::tcc0_1E,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"			},	// PA09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir.m"	},	// PA10 driver DIR-
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA11 driver STEP- driven via CCL from PB10
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir.p" 	},	// PA12 driver DIR+
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	Nx,	nullptr			},	// PA16 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		Nx,	nullptr			},	// PA17 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA18 LED1
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA19 LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA20 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA21 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io2.out2"		},	// PA22 io2.out on older boards
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io2.out"		},	// PA23 io2 out
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.jumper0"	},	// PA27 jumper0 (for resetting address etc.)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"io2.in"		},	// PA28
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB02 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB08 unused
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB09 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.step"	},	// PB10 driver0 step+
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.en.m"	},	// PB11 driver0 en-

	// The chip also has PB22 and PB23 but they are unused
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 32 + 12;			// 32 pins on port A (some missing) and first 12 pins of port B
static_assert(NumPins == NumRealPins);					// no virtual pins

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

#if USE_TC_FOR_STEP
TcCount16 * const StepGenTc = &(TC1->COUNT16);
constexpr unsigned int StepGenTcNumber = 1;
#endif

// Available UART ports
#define NUM_SERIAL_PORTS		1
constexpr IRQn Serial0_IRQn = SERCOM4_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts

#endif /* SRC_CONFIG_EXP1XD_H_ */
