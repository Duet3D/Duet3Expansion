/*
 * RPI_PICO.h
 *
 *  Created on: 24 Aug 2022
 *      Author: David
 */

#ifndef SRC_CONFIG_RPI_PICO_H_
#define SRC_CONFIG_RPI_PICO_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"RPI_PICO"
#define BOOTLOADER_NAME		"RPI_PICO"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1

// Drivers configuration
#define SUPPORT_DRIVERS			0	// temporary!

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		0
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	1
#define SUPPORT_DELTA_MOVEMENT	0
#define USE_EVEN_STEPS			1

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0

constexpr size_t NumDrivers = 1;

constexpr Pin EnablePins[NumDrivers] = { PortAPin(9) };
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
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0

#define USE_MPU					0
#define USE_CACHE				0

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
constexpr uint8_t SspiSercomNumber = 1;
constexpr uint32_t SspiDataInPad = 2;
constexpr Pin SSPIMosiPin = PortAPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPISclkPin = PortAPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPIMisoPin = PortAPin(18);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;

#endif

#if SUPPORT_I2C_SENSORS

// I2C using pins PA22,23. If changing this, also change the available pins in the pin table.
constexpr uint8_t I2CSercomNumber = 3;
constexpr Pin I2CSDAPin = PortAPin(22);
constexpr GpioPinFunction I2CSDAPinPeriphMode = GpioPinFunction::C;
constexpr Pin I2CSCLPin = PortAPin(23);
constexpr GpioPinFunction I2CSCLPinPeriphMode = GpioPinFunction::C;
#define I2C_HANDLER		SERCOM3_Handler

#endif

#if SUPPORT_LIS3DH
constexpr Pin Lis3dhInt1Pin = PortAPin(13);
#endif

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	PWM					ADC				PinName
	// Port A
	{ PwmOutput::none,	AdcInput::none,		"gpio0"		},	// GPIO00
	{ PwmOutput::none,	AdcInput::none,		"gpio1"		},	// GPIO01
	{ PwmOutput::none,	AdcInput::adc0_0,	"gpio2"		},	// GPIO02
	{ PwmOutput::none,	AdcInput::adc0_1,	"gpio3"		},	// GPIO03
	{ PwmOutput::none,	AdcInput::adc0_4,	"gpio4"		},	// GPIO04
	{ PwmOutput::none,	AdcInput::adc0_5,	"gpio5"		},	// GPIO05
	{ PwmOutput::none,	AdcInput::adc0_6,	"gpio6"		},	// GPIO06
	{ PwmOutput::none,	AdcInput::adc0_7,	"gpio7"		},	// GPIO07
	{ PwmOutput::none,	AdcInput::adc0_8,	"gpio8"		},	// GPIO08
	{ PwmOutput::none,	AdcInput::adc0_9,	"gpio9"		},	// GPIO09
	{ PwmOutput::none,	AdcInput::none,		"gpio10"	},	// GPIO10
	{ PwmOutput::none,	AdcInput::none,		"gpio11"	},	// GPIO11
	{ PwmOutput::none,	AdcInput::none,		"gpio12" 	},	// GPIO12
	{ PwmOutput::none,	AdcInput::none,		"gpio13"	},	// GPIO13
	{ PwmOutput::none,	AdcInput::none,		"gpio14"	},	// GPIO14
	{ PwmOutput::none,	AdcInput::none,		"gpio15"	},	// GPIO15
	{ PwmOutput::none,	AdcInput::none,		"gpio16"	},	// GPIO16
	{ PwmOutput::none,	AdcInput::none,		"gpio17"	},	// GPIO17
	{ PwmOutput::none,	AdcInput::none,		"gpio18" 	},	// GPIO18
	{ PwmOutput::none,	AdcInput::none,		"gpio19"	},	// GPIO19
	{ PwmOutput::none,	AdcInput::none,		"gpio20"	},	// GPIO20
	{ PwmOutput::none,	AdcInput::none,		"gpio21"	},	// GPIO21
	{ PwmOutput::none,	AdcInput::none,		"gpio22"	},	// GPIO22
	{ PwmOutput::none,	AdcInput::none,		"gpio23"	},	// GPIO23
	{ PwmOutput::none,	AdcInput::none,		"gpio24"	},	// GPIO24
	{ PwmOutput::none,	AdcInput::none,		"gpio25"	},	// GPIO25
	{ PwmOutput::none,	AdcInput::none,		"gpio26"	},	// GPIO26
	{ PwmOutput::none,	AdcInput::none,		"gpio27"	},	// GPIO27
	{ PwmOutput::none,	AdcInput::none,		"gpio28"	},	// GPIO28
	{ PwmOutput::none,	AdcInput::none,		"gpio29"	},	// GPIO29
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static_assert(NumPins == 30);		// 30 GPIO pins on RP2040

// Timer/counter used to generate step pulses and other sub-millisecond timings
//TODO

// Available UART ports
#define NUM_SERIAL_PORTS		1
//constexpr IRQn Serial0_IRQn = SERCOM5_IRQn;

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
const NvicPriority NvicPriorityI2C = 4;

#endif /* SRC_CONFIG_RPI_PICO_H_ */
