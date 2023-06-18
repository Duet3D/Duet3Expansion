/*
 * ATEIO.h
 *
 *  Created on: 7 Oct 2020
 *      Author: David
 */

#ifndef SRC_CONFIG_ATEIO_H_
#define SRC_CONFIG_ATEIO_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"ATEIO"
#define BOOTLOADER_NAME		"SAMC21"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1

#define SUPPORT_DRIVERS			0
#define SUPPORT_THERMISTORS		0
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			1

#define USE_MPU					0
#define USE_CACHE				0

constexpr size_t MaxPortsPerHeater = 1;

constexpr bool UseAlternateCanPins = true;

constexpr Pin BoardTypePins[] = { PortAPin(5), PortAPin(4) };
constexpr Pin ButtonPins[] = { PortAPin(18) };

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

// Shared SPI
constexpr uint8_t SspiSercomNumber = 1;
constexpr uint32_t SspiDataInPad = 3;
constexpr Pin SSPIMosiPin = PortAPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPISclkPin = PortAPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPIMisoPin = PortAPin(19);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;

constexpr Pin ExtendedAdcCsPin = PortAPin(18);

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!rsel1"	},	// PA00 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!rsel10"	},	// PA01 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "aingnd"	},	// PA02 EUT gnd
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "ainvssa"	},	// PA03 EUT Vssa
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA04 board type 1
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA05 board type 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain2"		},	// PA06 Ain2
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain3"		},	// PA07 Ain3
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain4"		},	// PA08 Ain4
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain5"		},	// PA09 Ain5
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain6,cs6"	},	// PA10 Ain6 or current sense
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain7"		},	// PA11 Ain7
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din6" 		},	// PA12 Din6
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA13 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::sercom1c,	Nx,	nullptr		},	// PA16 SPI1 MOSI (sercom1 pad 0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::sercom1c,	SercomIo::none,		Nx,	nullptr		},	// PA17 SPI1 SCK (sercom1 pad 1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr	 	},	// PA18 SPI1 CS0 (sercom1 pad 2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA19 SPI1 MISO (sercom1 pad 3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din5"		},	// PA20 Din5
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din4"		},	// PA21 Din4
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din3"		},	// PA22 Din3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din2"		},	// PA23 Din2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din1"		},	// PA24 Din1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din0"		},	// PA25 Din0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!tsel0"	},	// PA27 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!tsel1"	},	// PA28 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA30 swclk and LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PA31 swdio and LED

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!rsel0"	},	// PB02 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"!rsel100"	},	// PB03 relay
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain0"		},	// PB08 Ain0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ain1"		},	// PB09 Ain1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"din7"		},	// PB10 Din7
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB11 unused
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
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB22 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PB23 CAN0 Rx

	// Extended ADC. We can re-use the ADC0 input channel IDs because the code checks whether the pin is a physical pin.
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"load0"		},	// extended ADC channel 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"load1"		},	// extended ADC channel 1
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"loadspare0"},	// extended ADC channel 2
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"loadspare1"},	// extended ADC channel 3
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"load2"		},	// extended ADC channel 4
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"load3"		},	// extended ADC channel 5
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"voutlc"	},	// extended ADC channel 6
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"load4"		},	// extended ADC channel 7
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 32 + 24;		// 32 pins on port A (some missing), 24 on port B
static_assert(NumPins == NumRealPins + 8);			// 8 virtual pins for extended ADC inputs

static inline constexpr bool IsExtendedAnalogPin(Pin p) noexcept
{
	return p >= NumRealPins && p < NumRealPins + 8;
}

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		0

// DMA channel assignments
constexpr DmaChannel DmacChanAdc0Rx = 0;
constexpr DmaChannel DmacChanSdadcRx = 1;

constexpr unsigned int NumDmaChannelsUsed = 2;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

constexpr DmaPriority DmacPrioAdcRx = 2;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts

#endif /* SRC_CONFIG_ATEIO_H_ */
