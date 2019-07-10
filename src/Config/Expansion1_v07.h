/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V07_H_
#define SRC_CONFIG_EXPANSION1_V07_H_

#include "RepRapFirmware.h"
#include "Hardware/SAME5X.h"

const size_t MaxHeaters = 6;
const size_t MaxExtraHeaterProtections = 6;

#define HAS_SMART_DRIVERS	1
#define HAS_VREF_MONITOR	1
#define HAS_CPU_TEMP_SENSOR	1

#define SUPPORT_TMC51xx		1
#define SUPPORT_DHT_SENSOR	1
#define SUPPORT_SPI_SENSORS	1

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

constexpr Pin BoardAddressPins[4] = { PortCPin(11), PortCPin(12), PortCPin(14), PortCPin(15) };
constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(9), PortBPin(9), PortBPin(7) };
constexpr Pin TachoInputPins[NumTachoInputs] = { PortAPin(13), PortBPin(19), PortCPin(21) };			// tachos are on output connectors 6-8
constexpr Pin IoInPins[NumIoPorts] = { PortAPin(2), PortCPin(2), PortCPin(0), PortAPin(3), PortCPin(3), PortCPin(1) };
constexpr Pin IoOutPins[NumIoPorts] = { PortAPin(16), PortBPin(16), PortBPin(20), PortAPin(5), PortBPin(0), PortAPin(20) };
constexpr Pin SpiCSPins[NumSpiSlaves] = { PortBPin(11), PortAPin(9), PortBPin(15), PortBPin(14) };

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	// Port A
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA00
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA01
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA02
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA03
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA04
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA05
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA06
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA07
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA08
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA09
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA10
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA11
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA12
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA13
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA14
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA15
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA16
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA17
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA18
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA19
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA20
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA21
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA22
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA23
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA24
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA25
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA26
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA27
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA28
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA29
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA30
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PA31

	// Port B
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB00
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB01
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB02
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB03
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB04
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB05
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB06
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB07
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB08
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB09
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB10
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB11
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB12
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB13
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB14
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB15
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB16
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB17
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB18
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB19
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB20
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB21
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB22
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB23
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB24
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB25
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB26
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB27
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB28
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB29
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB30
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PB31

	// Port C
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC00
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC01
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC02
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC03
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC04
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC05
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC06
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC07
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC08
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC09
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC10
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC11
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC12
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC13
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC14
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC15
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC16
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC17
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC18
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC19
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC20
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC21
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC22
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC23
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC24
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC25
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC26
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC27
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC28
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC29
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC30
	{ TcOutput::none,		TccOutput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none },	// PC31
};

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

#endif /* SRC_CONFIG_EXPANSION1_V07_H_ */
