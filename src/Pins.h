/*
 * Pins.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_PINS_H_
#define SRC_PINS_H_

#define SUPPORT_TMC51xx		1

const size_t MaxSmartDrivers = 3;

constexpr Pin GlobalTmc51xxEnablePin = PORTB_PIN(23);
constexpr Pin GlobalTmc51xxCSPin = PORTB_PIN(22);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM0;

constexpr Pin VinMonitorPin = PORTA_PIN(11);

// Timer/counter used to generate step pulses and other sub-milisecond timings
TcCount32 * const StepTc = &(TC6->COUNT32);
constexpr IRQn StepTcIRQn = TC6_IRQn;
constexpr unsigned int StepTcClockId = TC6_GCLK_ID;

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

// PWM frequencies
//TODO move to Configuration.h ?
constexpr PwmFrequency SlowHeaterPwmFreq = 10;			// slow PWM frequency for bed and chamber heaters, compatible with DC/AC SSRs
constexpr PwmFrequency NormalHeaterPwmFreq = 250;		// normal PWM frequency used for hot ends
constexpr PwmFrequency DefaultFanPwmFreq = 250;			// increase to 25kHz using M106 command to meet Intel 4-wire PWM fan specification
constexpr PwmFrequency DefaultPinWritePwmFreq = 500;	// default PWM frequency for M42 pin writes and extrusion ancillary PWM


#endif /* SRC_PINS_H_ */
