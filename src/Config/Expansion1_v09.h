/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V09_H_
#define SRC_CONFIG_EXPANSION1_V09_H_

#include "RepRapFirmware.h"
#include "Hardware/SAME5X.h"

const size_t MaxHeaters = 6;
const size_t MaxExtraHeaterProtections = 6;

#define HAS_SMART_DRIVERS	1
#define HAS_STALL_DETECT	1
#define HAS_VREF_MONITOR	1
#define HAS_12V_MONITOR		1
#define HAS_CPU_TEMP_SENSOR	1

#define SUPPORT_TMC51xx		1
#define SUPPORT_DHT_SENSOR	0	//TEMP!!!
#define SUPPORT_SPI_SENSORS	1

constexpr size_t NumDrivers = 3;
constexpr size_t MaxSmartDrivers = 3;

constexpr size_t MaxAxes = 3;			//TEMP we won't need this

constexpr size_t NumOutputPorts = 9;
constexpr size_t NumIoPorts = 6;
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
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(23), PortCPin(28), PortAPin(0) };

constexpr Pin OutPins[NumOutputPorts] = { PortAPin(18), PortAPin(19), PortAPin(24), PortBPin(8), PortAPin(4), PortAPin(8), PortAPin(10), PortBPin(10), PortAPin(12) };

constexpr Pin BoardTypePins[NumBoardTypeBits] = { PortBPin(18), PortCPin(19), PortCPin(16) };

constexpr Pin VinMonitorPin = PortAPin(10);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr Pin V12MonitorPin = PortBPin(5);
constexpr float V12DividerRatio = (60.4 + 4.7)/4.7;
constexpr Pin VrefPin = PortBPin(4);
constexpr Pin VssaPin = PortBPin(6);

constexpr Pin BoardAddressPins[4] = { PortCPin(11), PortCPin(12), PortCPin(14), PortCPin(15) };
constexpr Pin TempSensePins[NumThermistorInputs] = { PortCPin(3), PortBPin(8), PortBPin(7) };
constexpr Pin TachoInputPins[NumTachoInputs] = { PortAPin(13), PortBPin(19), PortCPin(21) };			// tachos are on output connectors 6-8
constexpr Pin IoInPins[NumIoPorts] = { PortAPin(2), PortCPin(2), PortCPin(0), PortAPin(3), PortCPin(3), PortCPin(1) };
constexpr Pin IoOutPins[NumIoPorts] = { PortAPin(16), PortBPin(16), PortBPin(20), PortAPin(5), PortBPin(0), PortAPin(20) };

// Shared SPI
Sercom * const SERCOM_SSPI = SERCOM6;

// Table of pin functions that we are allowed to use
constexpr uint8_t Nx = 0xFF;

constexpr PinDescription PinTable[] =
{
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA00 driver2 DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA01 driver2 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr		},	// PA02 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr		},	// PA03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io3.out"	},	// PA04
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out4"		},	// PA05
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"io0.in"	},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"spi.cs2"	},	// PA07
	{ TcOutput::none,	TccOutput::tcc0_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out5"		},	// PA08
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	nullptr		},	// PA09 driver0 diag0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA10 VINmon
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out6"		},	// PA11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	"out3.tach" },	// PA12
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out8"		},	// PA13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	Nx,	"io0.out,uart0.tx" },	// PA16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		Nx,	"uart0.rx"	},	// PA17
	{ TcOutput::none,	TccOutput::tcc1_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0" 		},	// PA18
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"		},	// PA19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io5.out"	},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA21 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA22 driver0 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA23 driver0 dir
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out2"		},	// PA24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA25 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA27 driver1 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB00 driver2 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB01 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB02 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_6,	SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB04 VrefMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB05 12VMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB06 VssaMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_9,	SercomIo::none,		SercomIo::none,		Nx,	"temp2"		},	// PB07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_0,	SercomIo::none,		SercomIo::none,		Nx,	"temp1"		},	// PB08
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out3"		},	// PB09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"spi.cs0"	},	// PB10
	{ TcOutput::tc5_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out7"		},	// PB11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB12 CANtx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB13 CANrx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"spi.cs3"	},	// PB14 don't allow DHT11 on this pin, no EXINT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	"spi.cs1"	},	// PB15
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom5c,	Nx,	"io1.out,uart1.tx"	},	// PB16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom5c,	SercomIo::none,		Nx,	"uart1.rx"	},	// PB17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB18 board type 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	"out4.tach"	},	// PB19
#if 1
	// Temporarily don't allow PB20 to be an OUT pin, so that we can use it to connect a PanelDue as a debug console
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom3c,	Nx,	"uart2.tx" },	// PB20
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom3c,	Nx,	"io2.out,uart2.tx" },	// PB20
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom3c,	SercomIo::none,		Nx,	"uart2.rx"	},	// PB21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB22 drivers CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB23 drivers ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB24 spi0 mosi
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB25 spi0 clock
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB26 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB27 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB28 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB29 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	nullptr		},	// PB30 driver2 diag0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	"io4.out"	},	// PB31

	// Port C
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	"io2.in"	},	// PC00 IO2 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	"io5.in"	},	// PC01 IO5 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"io1.in"	},	// PC02 IO1 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_5,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"		},	// PC03 thermistor 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC04 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC05 was SPI1 clock
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC06 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC07 was SPI1 miso
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC08 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC09 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC10 diag LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC11 board ID 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC12 board ID 1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC13 was SPI1 mosi
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC14 board ID 2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC15 board ID 3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC16 was board type 2, now SPI1_MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC17 was unused, now SPI1_CLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC18 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC19 was board type 1, now SPI1_MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"io3.in"	},	// PC20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"out5.tach"	},	// PC21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC22 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC23 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io4.in"	},	// PC24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC25 spi0 mosi
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC26 driver1 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr		},	// PC27 driver1 diag0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC28 driver1 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC30 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PC31 not on 100-pin chip
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
constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;
constexpr IRQn Serial1_IRQn = SERCOM5_0_IRQn;

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

#endif /* SRC_CONFIG_EXPANSION1_V09_H_ */
