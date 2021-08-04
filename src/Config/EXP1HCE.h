/*
 * EXP1CL.h
 *
 *  Created on: 22 May 2020
 *      Author: David
 */

#ifndef SRC_CONFIG_EXP1CL_H_
#define SRC_CONFIG_EXP1CL_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"EXP1HCE"
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
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	1
#define USE_EVEN_STEPS			1

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			1
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_CLOSED_LOOP		1
#define SUPPORT_CAN_LOGGING		1

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;
constexpr float MaxTmc5160Current = 4500.0;			// The maximum current we allow the TMC5160/5161 drivers to be set to.
													// TODO: MaxTmc5160Current could be 6300, but not when holding.

constexpr Pin GlobalTmc51xxEnablePin = PortBPin(2);
constexpr Pin GlobalTmc51xxCSPin = PortAPin(1);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM4;
constexpr uint8_t SERCOM_TMC51xx_NUMBER = 4;

constexpr Pin TMC51xxMosiPin = PortAPin(12);
constexpr GpioPinFunction TMC51xxMosiPinPeriphMode = GpioPinFunction::D;
constexpr Pin TMC51xxSclkPin = PortAPin(13);
constexpr GpioPinFunction TMC51xxSclkPinPeriphMode = GpioPinFunction::D;
constexpr Pin TMC51xxMisoPin = PortBPin(10);
constexpr GpioPinFunction TMC51xxMisoPinPeriphMode = GpioPinFunction::D;

PortGroup * const StepPio = &(PORT->Group[1]);		// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortBPin(3) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(23) };
constexpr Pin DiagPins[NumDrivers] = { PortAPin(28) };

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			0
#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				0

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 1;
constexpr float DefaultThermistorSeriesR = 2200.0;

constexpr Pin BoardTypePin = PortAPin(5);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

constexpr Pin VinMonitorPin = PortAPin(2);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 5.0;				// we use the 5V supply as the voltage reference

constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(7) };

// Encoder and quadrature decoder interface
constexpr Pin QuadratureResetPin = PortAPin(0);
constexpr Pin EncoderCsPin = PortAPin(18);

// Shared SPI (used for interface to encoders, not for temperature sensors)
constexpr uint8_t EncoderSspiSercomNumber = 1;
constexpr uint32_t EncoderSspiDataInPad = 3;
constexpr Pin EncoderMosiPin = PortAPin(16);
constexpr Pin QuadratureErrorOutPin = EncoderMosiPin;
constexpr GpioPinFunction EncoderMosiPinPeriphMode = GpioPinFunction::C;

constexpr Pin EncoderSclkPin = PortAPin(17);
constexpr Pin QuadratureCountUpPin = EncoderSclkPin;
constexpr GpioPinFunction EncoderSclkPinPeriphMode = GpioPinFunction::C;

constexpr Pin EncoderMisoPin = PortAPin(19);
constexpr Pin QuadratureCountDownPin = EncoderMisoPin;
constexpr GpioPinFunction EncoderMisoPinPeriphMode = GpioPinFunction::C;

// Clock generator pin for external devices
constexpr uint8_t ClockGenGclkNumber = 6;
constexpr Pin ClockGenPin = PortAPin(22);
constexpr GpioPinFunction ClockGenPinPeriphMode = GpioPinFunction::H;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA00 attiny reset
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA01 driver CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 board type (analog)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA06 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"temp0"			},	// PA07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA08 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.buttons"	},	// PA09 2 buttons in (analog)
	{ TcOutput::none,	TccOutput::tcc1_0E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"			},	// PA10
	{ TcOutput::none,	TccOutput::tcc0_3F,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA12 driver MOSI (SERCOM4.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 driver SCK (SERCOM4.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 AS5047/attiny MOSI (SERCOM1.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	nullptr			},	// PA17 AS5047/attiny SCK (SERCOM1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA18 AS5047/attiny CS (SERCOM1.2)
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	nullptr			},	// PA19 AS5047/attiny MISO (SERCOM1.3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io1.out"		},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"io1.in"		},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 clock out to attiny (GCLK_IO6)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir"	},	// PA23 driver DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA27 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"ate.d0.diag"	},	// PA28 driver DIAG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk and LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio and LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB02 driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.step"	},	// PB03 driver STEP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB08 unused
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io0.out"		},	// PB09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB10 driver MISO (SERCOM4.2)
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB11 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB12 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"io0.in"		},	// PB22
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB23 unused
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static_assert(NumPins == 32 + 24);		// 32 pins on port A (some missing), 24 on port B

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER			TC2_Handler

// Timer/counter used to accumulate pulses from the quadrature decoder
Tcc * const QuadratureTcc = TCC2;
constexpr unsigned int QuadratureTccNumber = 2;
constexpr unsigned int QuadratureCountUpEventUser = 19;			// TCC2 EV0, see datasheet
constexpr unsigned int QuadratureCountDownEventUser = 20;		// TCC2 EV1, see datasheet

// Available UART ports
#define NUM_SERIAL_PORTS		0

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

// Event channel allocation (max 12 channels on SAMC21)
constexpr EventNumber CountUpEvent = 0;
constexpr EventNumber CountDownEvent = 1;

#endif /* SRC_CONFIG_EXP1CL_H_ */
