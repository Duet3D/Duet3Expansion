/*
 * Tool1_v01.h
 *
 *  Created on: 18 Aug 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_TOOL1_V01_H_
#define SRC_CONFIG_TOOL1_V01_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"TOOL1LC"
#define BOOTLOADER_NAME		"SAMC21"

// General features
#define HAS_VREF_MONITOR		1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1
#define USE_SERIAL_DEBUG		defined(DEBUG)

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0
#define DEDICATED_STEP_TIMER	1

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			0		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2240			0

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

#define TMC22xx_USES_SERCOM				1
#define TMC22xx_HAS_MUX					0
#define TMC22xx_SINGLE_DRIVER			1
#define TMC22xx_HAS_ENABLE_PINS			0
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_USE_SLAVEADDR			0

constexpr Pin GlobalTmc22xxEnablePin = PortBPin(2);

constexpr uint8_t TMC22xxSercomNumber = 3;
Sercom * const SERCOM_TMC22xx = SERCOM3;

constexpr Pin TMC22xxSercomTxPin = PortAPin(22);
constexpr GpioPinFunction TMC22xxSercomTxPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC22xxSercomRxPin = PortAPin(20);
constexpr GpioPinFunction TMC22xxSercomRxPinPeriphMode = GpioPinFunction::D;
constexpr uint8_t TMC22xxSercomRxPad = 2;

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable on the Duet Maestro, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 200000;
constexpr uint32_t TransferTimeout = 10;									// any transfer should complete within 10 ticks @ 1ms/tick

constexpr float DriverSenseResistor = 0.091 + 0.02 + 0.003;					// in ohms. Added the 0.003 to make the max current a round 1600mA.
constexpr float DriverVRef = 180.0;											// in mV
constexpr float DriverFullScaleCurrent = DriverVRef/DriverSenseResistor;	// in mA
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
constexpr float MaximumMotorCurrent = 1600.0;
constexpr float MaximumStandstillCurrent = 1200.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortAPin(27) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(28) };
constexpr Pin DriverDiagPins[NumDrivers] = { PortBPin(3) };

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		1
#define SUPPORT_LIS3DH			1
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			1
#define SUPPORT_LDC1612			0
#define SUPPORT_DMA_NEOPIXEL	0

#define USE_MPU					0
#define USE_CACHE				0

#define DIAG_SERCOM_NUMBER		4		// which SERCOM device we use for debugging output

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float VrefTopResistor = 10.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 1000.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 1000.0);
																			// there are 2 temperature sensing channels and a 1K load resistor (2K2 prior to version 1.3)
constexpr Pin BoardTypePin = PortAPin(5);

constexpr Pin VinMonitorPin = PortAPin(8);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 5.0;				// we use the 5V supply as the voltage reference

constexpr Pin VrefPin = PortAPin(7);
constexpr Pin VssaPin = PortAPin(6);

constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(9), PortAPin(2) };

constexpr Pin ButtonPins[] = { PortBPin(22), PortBPin(23) };

// Diagnostic LEDs
constexpr Pin LedPinsV10[] = { PortAPin(0), PortAPin(1) };
constexpr bool LedActiveHighV10 = true;
constexpr Pin LedPinsV11[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHighV11 = false;

#if SUPPORT_I2C_SENSORS

// I2C using pins PA16,17
constexpr uint8_t I2CSercomNumber = 1;
constexpr Pin I2CSDAPin = PortAPin(16);
constexpr GpioPinFunction I2CSDAPinPeriphMode = GpioPinFunction::C;
constexpr Pin I2CSCLPin = PortAPin(17);
constexpr GpioPinFunction I2CSCLPinPeriphMode = GpioPinFunction::C;
#define I2C_HANDLER		SERCOM1_Handler

#endif

#if SUPPORT_LIS3DH
# define ACCELEROMETER_USES_SPI			(0)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr bool Lis3dhAddressLsb = true;
constexpr Pin Lis3dhInt1Pin = PortAPin(0);
#endif

#if SUPPORT_LDC1612
constexpr Pin LDC1612ClockGenPin = PortAPin(23);
constexpr uint16_t LDC1612_I2CAddress = 0x2B;				// pin 4 is tied high on the Grove board
constexpr Pin LDC1612InterruptPin = PortAPin(23);			// this is brought out to a test pad
#endif

constexpr auto sercom2cPad0 = SercomIo::sercom2c + SercomIo::pad0;
constexpr auto sercom5dPad0 = SercomIo::sercom5d + SercomIo::pad0;
constexpr auto sercom2dPad0 = SercomIo::sercom2d + SercomIo::pad0;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SDADC				SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	nullptr			},	// PA00 LED0 (v1.0), accelerometer INT (v1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	"io3.in"		},	// PA01 LED1 (v1.0), IO3 IN (v1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "temp1"			},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 tied to +3.3V
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 VREFB for SDADC, tied to +3.3V
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 board type (analog)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA06 VssaMon (SDADC INN0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	AdcInput::sdadc_0,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA07 Vrefmon (SDADC INP0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA08 VinMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	AdcInput::none,		sercom2dPad0,		SercomIo::none,		9,	"io0.in"		},	// PA09 IO0.in
	{ TcOutput::none,	TccOutput::tcc0_2F,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"			},	// PA10
	{ TcOutput::none,	TccOutput::tcc1_1E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA11
	{ TcOutput::none,	TccOutput::tcc2_0E,	AdcInput::none,		AdcInput::none,		SercomIo::none,		sercom2cPad0,		12,	"io0.out" 		},	// PA12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	"out1.tach"		},	// PA13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 unused (v1.0), SDA (v1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 unused (v1.0), SCL (v1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	"io2.in" 		},	// PA18 IO2.in (v1.0 and later boards) and sercom1 pad 2
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa19"			},	// PA19 spare, potential PWM output
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA20 drivers USART Rx (sercom 3.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"io1.in"		},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 drivers USART Tx (sercom 3.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		sercom5dPad0,		Nx,	"pa23"			},	// PA23 spare, test pad on v1.1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 CAN0 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 CAN0 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.step"	},	// PA27 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir"	},	// PA28 driver0 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk, also LED0 on v1.1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio, also LED1 on v1.1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB02 driver enn
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	"ate.d0.diag"	},	// PB03 driver0 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB08 VssaMon (SDADC INN1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	AdcInput::sdadc_1,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"			},	// PB09 (SDADC INP1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"out2.tach"		},	// PB10
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out2"			},	// PB11
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
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"!^button0"		},	// PB22 button0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	"!^button1"		},	// PB23 button1

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.lis3dh,i2c.lis2dw"	},	// LIS3DH sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::ldc1612,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.ldc1612"	},	// LDC1612 sensor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 24;				// 32 pins on port A (some missing), 24 on port B
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612;

static_assert(NumPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcClockId = TC2_GCLK_ID;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		1
constexpr IRQn Serial0_IRQn = SERCOM4_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;
constexpr DmaChannel DmacChanSdadcRx = 3;
constexpr DmaChannel DmacChanLedTx = 4;

constexpr unsigned int NumDmaChannelsUsed = 5;			// must be at least the number of channels used, may be larger. Max 12 on the SAMC21.

// DMA priorities, higher is better. 0 to 3 are available.
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0 can't make RTOS calls. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C = 2;
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts

#endif /* SRC_CONFIG_TOOL1_V01_H_ */
