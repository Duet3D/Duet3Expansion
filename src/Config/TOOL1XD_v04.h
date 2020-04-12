/*
 * TOOL1ED_v04.h
 *
 *  Created on: 7 Nov 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_TOOL1ED_V04_H_
#define SRC_CONFIG_TOOL1ED_V04_H_

#include "RepRapFirmware.h"
#include "Hardware/Peripherals.h"

constexpr const char* BoardTypeName = "TOOL1XD";

// General features
#define HAS_VREF_MONITOR		1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1

// Drivers configuration
#define HAS_SMART_DRIVERS		0
#define HAS_STALL_DETECT		0

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			0		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_ENABLE		0		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0

constexpr size_t NumDrivers = 1;

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on
constexpr Pin EnablePins[NumDrivers] = { PortBPin(2) };
constexpr Pin StepPins[NumDrivers] = { PortAPin(27) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(28) };

#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	1
#define SUPPORT_DELTA_MOVEMENT	1
#define USE_EVEN_STEPS			1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_DHT_SENSOR		0

#define USE_MPU					0
#define USE_CACHE				0

#define DIAG_SERCOM_NUMBER		4		// which SERCOM device we use for debugging output

constexpr size_t MaxAxes = 3;			//TEMP we won't need this

constexpr size_t NumOutputPorts = 3;
constexpr size_t NumIoPorts = 2;
constexpr size_t NumTachoInputs = 2;
constexpr size_t NumThermistorInputs = 3;
constexpr size_t NumAddressBits = 4;
constexpr size_t NumBoardTypeBits = 3;

constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / 3) * 4700.0/((DefaultThermistorSeriesR / 3) + 4700.0);
																			// there are 3 temperature sensing channels and a 4K7 load resistor
constexpr Pin BoardTypePin = PortAPin(5);

constexpr Pin VinMonitorPin = PortAPin(8);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 5.0;				// we use the 5V supply as the voltage reference

constexpr Pin VrefPin = PortAPin(7);
constexpr Pin VssaPin = PortAPin(6);

constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(9), PortAPin(3), PortAPin(2) };
constexpr Pin TachoInputPins[NumTachoInputs] = { PortAPin(13), PortBPin(10) };			// tachos are on output connectors 1-2
constexpr Pin IoInPins[NumIoPorts] = { PortAPin(9), PortAPin(21) };
constexpr Pin IoOutPins[NumIoPorts] = { PortAPin(12), NoPin };

constexpr size_t NumButtons = 2;
constexpr Pin ButtonPins[NumButtons] = { PortBPin(22), PortBPin(23) };

// Table of pin functions that we are allowed to use
constexpr uint8_t Nx = 0xFF;	// this means no EXINT usable

constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA00 LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA01 LED1
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "temp2"		},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "temp1"		},	// PA03
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA04 VREFB for SDADC
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA05 board type (analog)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA06 VssaMon (SDADC INN0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::sdadc_0,	SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA07 Vrefmon (SDADC INP0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA08 VinMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"io0.in"	},	// PA09 Z probe in
	{ TcOutput::none,	TccOutput::tcc0_2F,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"		},	// PA10
	{ TcOutput::none,	TccOutput::tcc1_1E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"		},	// PA11
	{ TcOutput::none,	TccOutput::tcc2_0E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	"io0.out" 	},	// PA12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	"out1.tach"	},	// PA13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	Nx,	nullptr		},	// PA16 spare, sercom1 pad 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		Nx,	nullptr		},	// PA17 spare, sercom1 pad 1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 	},	// PA18 spare, sercom1 pad 2
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA19 spare, sercom1 pad 3, also potential PWM output
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA20 drivers USART Rx (sercom 3.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"io1.in"	},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA22 drivers USART Tx (sercom 3.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA23 drivers USART SCLK (not needed for TMC22xx)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA24 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA25 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA27 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA28 driver0 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB02 driver enn
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	nullptr		},	// PB03 driver0 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB08 VssaMon (SDADC INN1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	AdcInput::sdadc_1,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"		},	// PB09 (SDADC INP1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"out2.tach"	},	// PB10
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out2"		},	// PB11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB12 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"button0"	},	// PB22 button0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"button1"	},	// PB23 button1
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static_assert(NumPins == 32 + 24);		// 32 pins on port A (some missing), 24 on port B

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// Diagnostic LED
constexpr Pin DiagLedPin = PortAPin(0);
constexpr Pin DiagLed1Pin = PortAPin(1);

// Available UART ports
constexpr IRQn Serial0_IRQn = SERCOM4_IRQn;

// DMA channel assignments
constexpr DmaChannel TmcTxDmaChannel = 0;
constexpr DmaChannel TmcRxDmaChannel = 1;
constexpr DmaChannel Adc0RxDmaChannel = 2;
constexpr DmaChannel SdAdcRxDmaChannel = 3;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

constexpr uint8_t TmcTxDmaPriority = 0;
constexpr uint8_t TmcRxDmaPriority = 3;
constexpr uint8_t AdcRxDmaPriority = 2;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls.
const uint32_t NvicPriorityStep = 1;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityUart = 2;					// serial driver makes RTOS calls
const uint32_t NvicPriorityPins = 2;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 3;
const uint32_t NvicPriorityDmac = 3;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_TOOL1ED_V04_H_ */
