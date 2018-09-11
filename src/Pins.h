/*
 * Pins.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_PINS_H_
#define SRC_PINS_H_

#define SUPPORT_TMC51xx		1
#define HAS_SMART_DRIVERS	1

constexpr size_t DRIVES = 3;
constexpr size_t MaxSmartDrivers = 3;

constexpr size_t MaxAxes = 3;			//TEMP we won't need this
constexpr size_t MaxExtruders = 3;		//TEMP we won't need this

constexpr Pin GlobalTmc51xxEnablePin = PORTB_PIN(23);
constexpr Pin GlobalTmc51xxCSPin = PORTB_PIN(22);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM0;

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are one
constexpr Pin StepPins[DRIVES] = { PORTA_PIN(25), PORTA_PIN(27), PORTA_PIN(1) };
constexpr Pin DirectionPins[DRIVES] = { PORTA_PIN(21), PORTC_PIN(28), PORTA_PIN(0) };

constexpr Pin VinMonitorPin = PORTA_PIN(11);

constexpr Pin BoardAddressPins[4] = { PORTC_PIN(11), PORTC_PIN(12), PORTC_PIN(13), PORTC_PIN(14) };

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC6->COUNT32);
constexpr IRQn StepTcIRQn = TC6_IRQn;
constexpr unsigned int StepTcClockId = TC6_GCLK_ID;
#define STEP_TC_HANDLER		TC6_Handler

// Diagnostic LED
constexpr Pin DiagLedPin = PORTC_PIN(10);

// DMA channel assignments. Channels 0-3 have individual interrupt vectors, channels 4-31 share an interrupt vector.
constexpr DmaChannel TmcTxDmaChannel = 0;
constexpr DmaChannel TmcRxDmaChannel = 1;
constexpr DmaChannel Adc0TxDmaChannel = 2;
// Next channel is used by ADC0 for receive
constexpr DmaChannel Adc1TxDmaChannel = 4;
// Next channel is used by ADC1 for receive

constexpr unsigned int NumDmaChannelsUsed = 10;			// must be at least the number of channels used, may be larger. Max 32 on the SAME51.

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const uint32_t NvicPriorityPins = 3;					// priority for GPIO pin interrupts - filament sensors must be higher than step
const uint32_t NvicPriorityDmac = 4;					// priority for DMA complete interrupts
const uint32_t NvicPriorityStep = 5;					// step interrupt is next highest, it can preempt most other interrupts

#endif /* SRC_PINS_H_ */
