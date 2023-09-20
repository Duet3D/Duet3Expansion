/*
 * M23CL.h
 *
 *  Created on: 18 Oct 2022
 *      Author: David
 */

#ifndef SRC_CONFIG_M23CL_H_
#define SRC_CONFIG_M23CL_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"M23CL"
#define BOOTLOADER_NAME		"SAME5x"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			1
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0
#define DEDICATED_STEP_TIMER	1

// The SAMC21 can sink more current than it can source, therefore we use active low signals to drive external drivers
#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			1
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_CLOSED_LOOP		1
#define SUPPORT_BRAKE_PWM		1

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;
constexpr float MaxTmc5160Current = 3600.0;					// the maximum peak current we allow the TMC5160/5161 drivers to be set to in open loop mode
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.05;

constexpr Pin GlobalTmc51xxEnablePin = PortAPin(5);
constexpr Pin GlobalTmc51xxCSPin = PortAPin(10);
constexpr Pin DriverSdModePin = PortAPin(20);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM0;
constexpr uint8_t SERCOM_TMC51xx_NUMBER = 0;

constexpr Pin TMC51xxMosiPin = PortAPin(8);
constexpr GpioPinFunction TMC51xxMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC51xxSclkPin = PortAPin(9);
constexpr GpioPinFunction TMC51xxSclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC51xxMisoPin = PortAPin(11);
constexpr GpioPinFunction TMC51xxMisoPinPeriphMode = GpioPinFunction::C;

PortGroup * const StepPio = &(PORT->Group[1]);				// the PIO that all the step pins are on (port B)
constexpr Pin StepPins[NumDrivers] = { PortBPin(23) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(27) };
constexpr Pin DiagPins[NumDrivers] = { PortAPin(21) };

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0							// we have no SPI temperature sensors but we need the SharedSPI channel for the encoder
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			0
#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				1

constexpr bool UseAlternateCanPins = true;

constexpr size_t MaxPortsPerHeater = 1;

// TEMP0 uses a 3K9 series resistor and a 10K thermistor to ground. TEMP1 has pin PA07 assigned but it is not connected, so we ignore it.
constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 3900.0;
// Thermistor is a 10K Murata NCU15XH103J6SRC. B25/50 = 3380, B25/80 = 3428, B25/85 = 3434, B25/100 = 3455
// From this we deduce R25 = 10000, R50 = 4160.1, R80 = 1668.5, R85 = 1452.2, R100 = 973.8
// The following Beta and C values use the 25, 50 and 65C values
constexpr float DefaultThermistorR25_M23CL = 10000;
constexpr float DefaultThermistorBeta_M23CL = 3425.0;
constexpr float DefaultThermistorC_M23CL = 1.68e-7;

constexpr Pin BoardTypePin = PortAPin(3);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(12), PortAPin(13) };
constexpr bool LedActiveHigh = true;

constexpr Pin VinMonitorPin = PortAPin(2);
constexpr Pin V12MonitorPin = PortAPin(6);
constexpr float VinDividerRatio = (100.0 + 5.1)/5.1;
constexpr float V12DividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;
constexpr float V12MonitorVoltageRange = V12DividerRatio * 3.3;

constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(8), PortBPin(9) };
constexpr Pin ButtonPins[] = { PortAPin(0) };		// CAN reset jumper

// Brake
constexpr Pin BrakeOnPin = PortBPin(10);
constexpr Pin BrakePwmPin = PortAPin(1);

// Encoder and quadrature decoder interface
constexpr Pin EncoderCsPin = PortAPin(18);

// Shared SPI (used for interface to encoders, not for temperature sensors)
constexpr uint8_t SspiSercomNumber = 1;
constexpr uint32_t SspiDataInPad = 3;
constexpr Pin SSPIMosiPin = PortAPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;

constexpr Pin SSPISclkPin = PortAPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;

constexpr Pin SSPIMisoPin = PortAPin(19);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;

// Position decoder
constexpr Pin PositionDecoderPins[] = { PortAPin(24), PortAPin(25), PortBPin(22) };
constexpr GpioPinFunction PositionDecoderPinFunction = GpioPinFunction::G;

// Clock generator pin for TMC2160
constexpr uint8_t ClockGenGclkNumber = 5;
constexpr Pin ClockGenPin = PortBPin(11);
constexpr GpioPinFunction ClockGenPinPeriphMode = GpioPinFunction::M;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA00 CAN reset jumper
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.brakepwm"	},	// PA01 brake PWM
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 board type
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	SercomIo::none,		SercomIo::none,		4,	"io0.in"		},	// PA04 IO0_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		Nx,	"ate.v12"		},	// PA06 12v monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA07 labelled TEMP1 on schematic but not connected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA08 driver MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA09 driver SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA10 driver CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA11 driver MISO
	{ TcOutput::none,	TccOutput::tcc1_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA12 status LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 ACT LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 AS5047/SPI MOSI (SERCOM1.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 AS5047/SPI SCK (SERCOM1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA18 AS5047/SPI CS (SERCOM1.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA19 AS5047/SPI MISO (SERCOM1.3)
	{ TcOutput::tc7_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"fan"			},	// PA20 fan PWM
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"ate.d0.diag"	},	// PA21 driver DIAG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA23 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 PDEC0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 PDEC1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir"	},	// PA27 driver DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB02 not connected
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_14,	SercomIo::sercom5d,	SercomIo::none,		Nx,	nullptr			},	// PB03 not connected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"			},	// PB08 TEMP0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,	"hctemp"		},	// PB09 HC_TEMP
	{ TcOutput::none,	TccOutput::tcc0_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.brakeon"	},	// PB10 brake on
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB11 CLKOUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB12 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB22 not connected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.step"	},	// PB23 driver STEP
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 32 + 24;			// 32 pins on port A (some missing), 24 on port B
static_assert(NumPins == NumRealPins);					// no virtual pins in this table

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
constexpr unsigned int StepTcNumber = 0;
#define STEP_TC_HANDLER			TC0_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		0

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAME5x.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 3;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 3;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 4;
const NvicPriority NvicPriorityDmac = 5;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 5;

#endif /* SRC_CONFIG_M23CL_H_ */
