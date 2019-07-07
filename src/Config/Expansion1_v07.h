/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXPANSION1_V07_H_
#define SRC_CONFIG_EXPANSION1_V07_H_

#include "RepRapFirmware.h"

const size_t MaxHeaters = 6;
const size_t MaxExtraHeaterProtections = 6;

#define HAS_SMART_DRIVERS	1
#define HAS_VREF_MONITOR	1
#define HAS_CPU_TEMP_SENSOR	1

#define SUPPORT_TMC51xx		1
#define SUPPORT_DHT_SENSOR	1
#define SUPPORT_SPI_SENSORS	0	//TEMP!!!

constexpr size_t NumDrivers = 3;
constexpr size_t MaxSmartDrivers = 3;

constexpr size_t MaxAxes = 3;			//TEMP we won't need this
constexpr size_t MaxExtruders = 3;		//TEMP we won't need this

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
constexpr int TempSenseADCs[NumThermistorInputs] = { 1, 1, 1 };
constexpr Pin TachoInputPins[NumTachoInputs] = { PortAPin(13), PortBPin(19), PortCPin(21) };			// tachos are on output connectors 6-8
constexpr Pin IoInPins[NumIoPorts] = { PortAPin(2), PortCPin(2), PortCPin(0), PortAPin(3), PortCPin(3), PortCPin(1) };
constexpr Pin IoOutPins[NumIoPorts] = { PortAPin(16), PortBPin(16), PortBPin(20), PortAPin(5), PortBPin(0), PortAPin(20) };
constexpr Pin SpiCSPins[NumSpiSlaves] = { PortBPin(11), PortAPin(9), PortBPin(15), PortBPin(14) };

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
