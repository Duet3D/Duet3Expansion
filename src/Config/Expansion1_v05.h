/*
 * Expansion1_v05.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V05_H_
#define SRC_CONFIG_EXPANSION1_V05_H_

#include "RepRapFirmware.h"
#include "Hardware/SAME5X.h"

const size_t MaxHeaters = 6;
const size_t MaxExtraHeaterProtections = 6;

#define HAS_SMART_DRIVERS	1
#define HAS_VREF_MONITOR	1
#define HAS_CPU_TEMP_SENSOR	1

#define SUPPORT_TMC51xx		1
#define SUPPORT_DHT_SENSOR	1
#define SUPPORT_SPI_SENSORS	0

constexpr size_t NumDrivers = 3;
constexpr size_t MaxSmartDrivers = 3;

constexpr size_t MaxAxes = 3;			//TEMP we won't need this

constexpr size_t NumOutputPorts = 9;
constexpr size_t NumIoPorts = 6;
constexpr size_t NumSpiSlaves = 4;
constexpr size_t MaxSpiTempSensors = NumSpiSlaves;
constexpr size_t NumTachoInputs = 3;
constexpr size_t NumThermistorInputs = 3;
constexpr size_t NumAddressBits = 4;
constexpr size_t NumBoardTypeBits = 3;

constexpr float DefaultThermistorSeriesR = 2200.0;

constexpr Pin GlobalTmc51xxEnablePin = PortBPin(23);
constexpr Pin GlobalTmc51xxCSPin = PortBPin(22);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM0;

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortAPin(25), PortAPin(27), PortAPin(1) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(21), PortCPin(28), PortAPin(0) };

constexpr Pin OutPins[NumOutputPorts] = { PortAPin(18), PortAPin(19), PortAPin(24), PortBPin(8), PortAPin(4), PortAPin(8), PortAPin(10), PortBPin(10), PortAPin(12) };

constexpr Pin BoardTypePins[NumBoardTypeBits] = { PortBPin(18), PortCPin(19), PortCPin(16) };

constexpr Pin VinMonitorPin = PortAPin(11);
constexpr Pin VRefPin = PortBPin(4);
constexpr int VRefADC = 1;
constexpr Pin VssaPin = PortBPin(6);
constexpr int VssaADC = 1;

constexpr Pin BoardAddressPins[4] = { PortCPin(11), PortCPin(12), PortCPin(13), PortCPin(14) };
constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(3), PortBPin(1), PortBPin(0) };
constexpr Pin TachoInputPins[NumTachoInputs] = { PortAPin(13), PortBPin(20), PortBPin(21) };			// tachos are on output connectors 3-5
constexpr Pin IoInPins[NumIoPorts] = { PortAPin(3), PortCPin(3), PortCPin(1), PortAPin(2), PortCPin(2), PortCPin(0) };
constexpr Pin IoOutPins[NumIoPorts] = { PortAPin(5), PortBPin(7), PortAPin(20), PortAPin(6), PortBPin(9), PortBPin(5) };
constexpr Pin SpiCSPins[NumSpiSlaves] = { PortBPin(11), PortAPin(9), PortAPin(7), PortBPin(14) };

// Table of pin functions that we are allowed to use
constexpr uint8_t Nx = 0xFF;

constexpr PinDescription PinTable[] =
{
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	nullptr		},	// PA00
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	nullptr		},	// PA01
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		2,	"io3.in "	},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		3,	"io0.in"	},	// PA03
	{ TcOutput::tc0_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"out4"		},	// PA04
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"io0.out"	},	// PA05
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"io3.out"	},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"spi.cs2"	},	// PA07
	{ TcOutput::none,	TccOutput::tcc0_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out5"		},	// PA08 (EXINT is NMI)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"spi.cs1"	},	// PA09
	{ TcOutput::tc1_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"out6"		},	// PA10
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		11,	nullptr		},	// PA11
	{ TcOutput::tc2_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	"out8"		},	// PA12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	"tacho6"	},	// PA13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PA14
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PA15
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0, nullptr		},	// PA16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	nullptr		},	// PA17
	{ TcOutput::none,	TccOutput::tcc1_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"out0"		},	// PA18
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	"out1"		},	// PA19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"io2out"	},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	nullptr		},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr		},	// PA22
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	nullptr		},	// PA23
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"out2"		},	// PA24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	nullptr		},	// PA25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA26 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr		},	// PA27
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA28 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA29 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PA30
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PA31

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_12,	SercomIo::none,		SercomIo::none,		0,	"therm2"	},	// PB00
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_13,	SercomIo::none,		SercomIo::none,		1,	"therm1"	},	// PB01
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	nullptr		},	// PB02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_15,	SercomIo::none,		SercomIo::none,		3,	"therm0"	},	// PB03
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_6,	SercomIo::none,		SercomIo::none,		4,	nullptr		},	// PB04
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		5,	"io5.out"	},	// PB05
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		6,	nullptr		},	// PB06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"io1.out"	},	// PB07
	{ TcOutput::tc4_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"out3"		},	// PB08
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"io4.out"	},	// PB09
	{ TcOutput::tc5_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"out7"		},	// PB10
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	"spi.cs0"	},	// PB11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	nullptr		},	// PB12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	nullptr		},	// PB13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	"spi1.cs3"	},	// PB14
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PB15
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	nullptr		},	// PB16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	nullptr		},	// PB17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	nullptr		},	// PB18
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	nullptr		},	// PB19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"tach7"		},	// PB20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"tach8"		},	// PB21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr		},	// PB22
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	nullptr		},	// PB23
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	nullptr		},	// PB24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	nullptr		},	// PB25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	nullptr		},	// PB26 (unusual EXINT numbering)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	nullptr		},	// PB27 (unusual EXINT numbering)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PB28 (unusual EXINT numbering)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PB29 (unusual EXINT numbering)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PB30
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PB31

	// Port C
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_10,	SercomIo::none,		SercomIo::none,		0,	"io4.in"	},	// PC00
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_11,	SercomIo::none,		SercomIo::none,		1,	"io2.in"	},	// PC01
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_4,	SercomIo::none,		SercomIo::none,		2,	"io3.in"	},	// PC02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_5,	SercomIo::none,		SercomIo::none,		3,	"io1.in"	},	// PC03
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	nullptr		},	// PC04
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	nullptr		},	// PC05
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr		},	// PC06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	nullptr		},	// PC07 (unusual EXINT numbering)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC08 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC09 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	nullptr		},	// PC10
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr		},	// PC11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	nullptr		},	// PC12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	nullptr		},	// PC13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PC14
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PC15
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	nullptr		},	// PC16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	nullptr		},	// PC17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	nullptr		},	// PC18
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	nullptr		},	// PC19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	nullptr		},	// PC20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	nullptr		},	// PC21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr		},	// PC22
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	nullptr		},	// PC23
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	nullptr		},	// PC24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	nullptr		},	// PC25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	nullptr		},	// PC26
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr		},	// PC27
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	nullptr		},	// PC28
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC29 (not present on chip)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PC30
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	nullptr		},	// PC31
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static_assert(NumPins == 96);

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC6->COUNT32);
constexpr IRQn StepTcIRQn = TC6_IRQn;
constexpr unsigned int StepTcClockId = TC6_GCLK_ID;
#define STEP_TC_HANDLER		TC6_Handler

// Diagnostic LED
constexpr Pin DiagLedPin = PortCPin(10);

// Available UART ports
constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;			// tacho 7/8 pins
constexpr IRQn Serial1_IRQn = SERCOM5_0_IRQn;			// this one is not really available

// DMA channel assignments. Channels 0-3 have individual interrupt vectors, channels 4-31 share an interrupt vector.
constexpr DmaChannel TmcTxDmaChannel = 0;
constexpr DmaChannel TmcRxDmaChannel = 1;
constexpr DmaChannel Adc0TxDmaChannel = 2;
// Next channel is used by ADC0 for receive
constexpr DmaChannel Adc1TxDmaChannel = 4;
// Next channel is used by ADC1 for receive

constexpr unsigned int NumDmaChannelsUsed = 10;			// must be at least the number of channels used, may be larger. Max 32 on the SAME51.

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const uint32_t NvicPriorityUart = 1;
const uint32_t NvicPriorityStep = 2;					// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityPins = 3;					// priority for GPIO pin interrupts
const uint32_t NvicPriorityCan = 4;
const uint32_t NvicPriorityDmac = 5;					// priority for DMA complete interrupts

#endif /* SRC_CONFIG_EXPANSION1_V05_H_ */
