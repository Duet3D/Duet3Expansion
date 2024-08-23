/*
 * EXP1HCLv1_0.h
 *
 *  Created on: 3 Dec 2021
 *      Author: David
 */

#ifndef SRC_CONFIG_EXP1HCLV1_0_H_
#define SRC_CONFIG_EXP1HCLV1_0_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"EXP1HCL"
#define BOOTLOADER_NAME		"SAME5x"

// General features
#define HAS_VREF_MONITOR		1
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
constexpr float MaxTmc5160Current = 6300.0;					// the maximum current we allow the TMC5160/5161 drivers to be set to in open loop mode
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.050;

constexpr Pin GlobalTmc51xxEnablePin = PortAPin(5);
constexpr Pin GlobalTmc51xxCSPin = PortAPin(10);

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
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_DMA_NEOPIXEL	0

#ifdef DEBUG
# define SUPPORT_I2C_SENSORS	0							// in debug mode the SERCOM is used for debugging
# define SUPPORT_LIS3DH			0
#else
# define SUPPORT_I2C_SENSORS	1
# define SUPPORT_LIS3DH			1
#endif

#define SUPPORT_DHT_SENSOR		0
#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				1

constexpr bool UseAlternateCanPins = true;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float VrefTopResistor = 27.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);

constexpr Pin VrefPin = PortAPin(4);
constexpr Pin VssaPin = PortBPin(9);

constexpr Pin BoardTypePin = PortAPin(3);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

constexpr Pin VinMonitorPin = PortAPin(2);
constexpr Pin V12MonitorPin = PortAPin(6);
constexpr float VinDividerRatio = (100.0 + 5.1)/5.1;
constexpr float V12DividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;
constexpr float V12MonitorVoltageRange = V12DividerRatio * 3.3;

constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(8), PortAPin(7) };
constexpr Pin ButtonPins[] = { PortAPin(20) };

// Encoder and quadrature decoder interface
constexpr Pin EncoderCsPin = PortAPin(18);

#if SUPPORT_I2C_SENSORS

// I2C using pins PA12,13
constexpr uint8_t I2CSercomNumber = 2;
constexpr Pin I2CSDAPin = PortAPin(12);
constexpr GpioPinFunction I2CSDAPinPeriphMode = GpioPinFunction::C;
constexpr Pin I2CSCLPin = PortAPin(13);
constexpr GpioPinFunction I2CSCLPinPeriphMode = GpioPinFunction::C;
# define I2C_HANDLER0		SERCOM2_0_Handler
# define I2C_HANDLER1		SERCOM2_1_Handler
# define I2C_HANDLER2		SERCOM2_2_Handler
# define I2C_HANDLER3		SERCOM2_3_Handler

#endif

#if SUPPORT_LIS3DH

# if SUPPORT_I2C_SENSORS

#  define ACCELEROMETER_USES_SPI			(0)				// accelerometer is connected via I2C
constexpr Pin Lis3dhInt1Pin = PortAPin(20);					// same as io1.in

# else

#  define ACCELEROMETER_USES_SPI			(1)				// accelerometer is connected via SPI
constexpr Pin Lis3dhCsPin = PortAPin(18);					// same as encoder CS pin
constexpr Pin Lis3dhInt1Pin = PortAPin(13);					// same as io1.in

# endif

#endif

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

// Brake On pin for version 2.0 board. If the BrakwPwmPort is configured as the brake pin in M569.7 then the BrakeOnPin is used implicitly as well.
constexpr Pin BrakePwmPin = PortBPin(10);
constexpr Pin BrakeOnPin = PortAPin(20);

constexpr auto sercom2cPad0 = SercomIo::sercom2c + SercomIo::pad0;
constexpr auto sercom2cPad1 = SercomIo::sercom2c + SercomIo::pad1;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	"spi.cs1"		},	// PA00 CAN reset jumper on V0 boards, spi.cs1 on V1 boards
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA01 OUT0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 board type
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 VREF_MON
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		Nx,	"ate.v12"		},	// PA06 12v monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	SercomIo::none,		SercomIo::none,		7,	"temp1"			},	// PA07 TEMP1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA08 driver MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA09 driver SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA10 driver CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA11 driver MISO
	{ TcOutput::none,	TccOutput::tcc1_2F,	AdcInput::none,		SercomIo::none,		sercom2cPad0,		Nx,	"io1.out" 		},	// PA12 IO1 out, I2C capable
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		sercom2cPad1,		SercomIo::none,		13,	"io1.in"		},	// PA13 IO1 in, I2C capable
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 AS5047/SPI MOSI (SERCOM1.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 AS5047/SPI SCK (SERCOM1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"spi.cs0,ate.spi.cs" },	// PA18 AS5047/SPI CS (SERCOM1.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA19 AS5047/SPI MISO (SERCOM1.3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa20,brake.pos"	},	// PA20 test pad/spare on V1, Brake On on V2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"ate.d0.diag"	},	// PA21 driver DIAG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA23 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"pdec.a"		},	// PA24 PDEC0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"pdec.b"		},	// PA25 PDEC1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir"	},	// PA27 driver DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk and LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio and LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::sercom5d,	Nx,	"io0.out"		},	// PB02 IO0 out, UART available
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_15,	SercomIo::sercom5d,	SercomIo::none,		3,	"io0.in"		},	// PB03 IO0 in, UART available
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"			},	// PB08 TEMP0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB09 VSSA monitor
	{ TcOutput::none,	TccOutput::tcc0_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1,brake.neg"	},	// PB10 OUT1
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
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"pdec.n"		},	// PB22 PDEC2
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
constexpr DmaChannel DmacChanLedTx = 3;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAME5x.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 3;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityI2C = 3;
const NvicPriority NvicPriorityPins = 3;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 4;
const NvicPriority NvicPriorityDmac = 5;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 5;

#endif /* SRC_CONFIG_EXP1HCLV1_0_H_ */
