/*
 * TOOLINDX.h
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#ifndef SRC_CONFIG_TOOLINDX_H_
#define SRC_CONFIG_TOOLINDX_H_

#include <Hardware/PinDescription.h>
#include <SPI/SpiParameters.h>
#include <I2C/I2cParameters.h>

#define BOARD_TYPE_NAME		"NodeTrix"
#define BOOTLOADER_NAME		"STM32H5"

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
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_INPUT_SHAPING	1
#define SUPPORT_CLOSED_LOOP		1

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2208			0
#define SUPPORT_TMC2209			0
#define SUPPORT_TMC2240_SPI		1

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

constexpr Pin GlobalTmcEnablePin = PortCPin(14);
constexpr Pin GlobalTmcCSPin = PortAPin(4);

//#define TMC_USES_SERCOM			1
//constexpr uint8_t TmcSercomNumber = 0;
//Sercom * const SERCOM_TMC = SERCOM0;

constexpr Pin TMCMosiPin = PortAPin(7);
constexpr Pin TMCMisoPin = PortAPin(6);
constexpr Pin TMCSclkPin = PortAPin(5);
//constexpr GpioPinFunction TMCSpiPinsPeriphMode = GpioPinFunction::D;

constexpr uint32_t Tmc2240CurrentRange = 0x01;								// which current range we set the TMC2240 to (2A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 12.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 24000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x01
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;			// with RRef = 12K this works out as 2.0A so this is the maximum current we can ask for

constexpr float MaxMotorCurrent = DriverFullScaleCurrent;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

GPIO_TypeDef * const StepPort = GPIOD_NS;									// the port that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortDPin(2) };
constexpr Pin DirectionPins[NumDrivers] = { PortCPin(7) };
constexpr Pin DriverDiagPins[NumDrivers] = { PortCPin(13) };

#define SUPPORT_THERMISTORS			1
#define SUPPORT_SPI_SENSORS			0										// SPI temperature sensors not supported
#define SUPPORT_LDC1612				1
#define SUPPORT_TPiS_1T_1086_L5_5	0										// IR temperature sensor
#define SUPPORT_AS5601				1										// support direct-connected magnetic filament monitor encoder chip
#define SUPPORT_DMA_NEOPIXEL		1										// using QSPI for Neopixels
#define NEOPIXEL_USES_QSPI			0										// using QSPI for Neopixels
#define SUPPORT_INDUCTIVE_HEATER	0										// Inductive heater support
#define SUPPORT_LP5817				0										// LP5817 LED driver support
#define SUPPORT_ADS131M02			1										// ADS131M02 ADC support
#define SUPPORT_LOADCELL_DIAGNOSTICS	1									// load cell baseline drift reported by M122
#define SUPPORT_LOADCELL_FFT		1										// load cell spectra reported by M122, costs 16KiB of RAM
#define NUM_CURRENT_SENSORS			1										// board has dedicated heater output with current measurement

#define NUM_I2C_CHANNELS		1
#define SUPPORT_LIS3DH			1

#define NUM_SHARED_SPI			1											// we use a SharedSpi for the closed loop encoder

#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				1

constexpr int CANInstanceNumber = 0;										// FDCAN1 (not FDCAN2)
constexpr bool UseLaterCanPins = true;

constexpr size_t MaxPortsPerHeater = 1;										// we support a single heater

constexpr Pin BoardTypePin = PortAPin(3);

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanLedTx = 2;
constexpr DmaChannel DmacChanSspiTx = 3;
constexpr DmaChannel DmacChanSspiRx = 4;
constexpr DmaChannel DmacChanADS131M02Tx = 5;
constexpr DmaChannel DmacChanADS131M02Rx = 6;
//TODO add DMA channels for I2C as needed

constexpr unsigned int NumDmaChannelsUsed = 7;			// must be at least the number of channels used, may be larger. Max 12 on the SAME5x.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;
constexpr DmaPriority DmacPrioSspiTx = 0;
constexpr DmaPriority DmacPrioSspiRx = 3;
constexpr DmaPriority DmacPrioADS131M02Tx = 0;
constexpr DmaPriority DmacPrioADS131M02Rx = 3;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
constexpr NvicPriority NvicPriorityStep = 3;			// step interrupt is next highest, it can preempt most other interrupts
constexpr NvicPriority NvicPriorityDmac = 3;			// priority for DMA complete interrupts
constexpr NvicPriority NvicPriorityUart = 3;			// serial driver makes RTOS calls
constexpr NvicPriority NvicPriorityI2C = 3;
constexpr NvicPriority NvicPriorityPins = 3;			// priority for GPIO pin interrupts
constexpr NvicPriority NvicPriorityCan = 4;
constexpr NvicPriority NvicPriorityAdc = 5;

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(14), PortAPin(13) };					// the SWDEBUG pins
constexpr bool LedActiveHigh = false;

constexpr Pin VinMonitorPin = PortCPin(3);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;

// Thermistor inputs. 0 = nozzle environment, 1 = board temperature, 2 = LDC coil temperature.
// Thermistor 0 is Tewa TT7-10KX3-11 (10kOhm, B3977), https://www.tme.eu/Document/32a31570f1c819f9b3730213e5eca259/TT7-10KC3-11.pdf
// R25 = 10000, R75 = 1480, R125 = 338. From these and using the SRS calculator we deduce R25=10000, B=4333, C=1.03958e-7.
// Thermistors 1 and 2 are 10K Murata NCU15XH103J6SRC. B25/50 = 3380, B25/80 = 3428, B25/85 = 3434, B25/100 = 3455
// From this we deduce R25 = 10000, R50 = 4160.1, R80 = 1668.5, R85 = 1452.2, R100 = 973.8
// The following Beta and C values use the 25, 50 and 65C values
// We don't use a Vref/Vssa calibration chain on this board
#define CUSTOM_THERMISTORS			1										// we provide nonstandard R25, beta and series resistor values

constexpr size_t NumThermistorInputs = 4;
constexpr Pin TempSensePins[NumThermistorInputs] = { PortCPin(0), PortCPin(1), PortCPin(2), PortAPin(0) };
constexpr float ThermistorSeriesR[NumThermistorInputs] = { 2200, 2200, 2200, 3900 };
constexpr float ThermistorR25[NumThermistorInputs] = { 100000, 100000, 100000, 10000 };
constexpr float ThermistorBeta[NumThermistorInputs] = { 4725.0, 4725.0, 4725.0, 3425.0 };
constexpr float ThermistorShC[NumThermistorInputs] = { 7.060e-8, 7.060e-8, 7.060e-8, 1.68e-7 };
constexpr float DefaultThermistorSeriesR = 2200.0;							// needed by initialisation but not actually used

#if SUPPORT_CLOSED_LOOP

// Shared SPI definitions
constexpr SpiParameters SharedSpiParams =
{
	.instanceNumber = 2,
	.mosiPin = PortBPin(15),
	.misoPin = PortBPin(14),
	.sclkPin = PortBPin(13),
	.pinFunction = GpioPinFunction::AF5,
	.dmaChanTx = DmacChanSspiTx,
	.dmaChanRx = DmacChanSspiRx,
	.dmaPrioTx = DmacPrioSspiTx,
	.dmaPrioRx = DmacPrioSspiRx,
};

constexpr Pin EncoderCsPin = PortBPin(1);

// Clock generator pin for TMC2240
//constexpr uint8_t TmcClockGclkNumber = 7;
constexpr Pin TmcClockPin = PortAPin(8);
constexpr GpioPinFunction TmcClockPinPeriphMode = GpioPinFunction::AF0;

# define SUPPORT_MT6835					0
# define SUPPORT_QUADRATURE_ENCODER		0
# define SUPPORT_COMPOSITE_ENCODER		0

#endif

#if NUM_I2C_CHANNELS >= 1

// I2C0 using pins PA22,23 (SERCOM 3)
constexpr I2cParameters I2C0Params =
{
	.instanceNumber = 2,
	.sclPin = PortBPin(10),
	.sdaPin = PortBPin(12),
	.pinFunction = GpioPinFunction::AF5,
	.irqPriority = NvicPriorityI2C
};

#endif

#if SUPPORT_LIS3DH												// we actually use a LIS2DW
# define ACCELEROMETER_USES_SPI			(0)						// accelerometer is connected via I2C
constexpr unsigned int Lis_I2CChannel = 0;
constexpr Pin Lis3dhInt1Pin = PortBPin(5);
#endif

#if SUPPORT_LDC1612
constexpr unsigned int LDC1612_I2CChannel = 0;
constexpr uint16_t LDC1612_I2CAddress = 0x2A;					// pin 4 is tied low
//constexpr unsigned int LDC1612GClkNumber = 5;
constexpr Pin LDC1612ClockGenPin = PortCPin(9);
constexpr Pin LDC1612InterruptPin = PortBPin(0);
#endif

#if SUPPORT_AS5601
constexpr unsigned int AS5601_I2CChannel = 0;					//TODO which I2C?
constexpr uint16_t AS5601_I2CAddress = 0x36;					// I2C address of the AS5601
#endif

#if SUPPORT_ADS131M02

constexpr SpiParameters Ads131M02SpiParams =
{
	.instanceNumber = 3,
	.mosiPin = PortCPin(12),
	.misoPin = PortCPin(11),
	.sclkPin = PortCPin(10),
	.pinFunction = GpioPinFunction::AF6,
	.dmaChanTx = DmacChanADS131M02Tx,
	.dmaChanRx = DmacChanADS131M02Rx,
	.dmaPrioTx = DmacPrioADS131M02Tx,
	.dmaPrioRx = DmacPrioADS131M02Rx,
};

constexpr Pin ADS131M02_CsPin = PortAPin(15);
constexpr Pin ADS131M02_DRDYPin = PortAPin(1);
constexpr Pin ADS131M02_GclkPin = PortBPin(6);
constexpr GpioPinFunction ADS131M02_GclkPinFunction = GpioPinFunction::AF2;		// TIM4_CH1

#endif

#if NUM_CURRENT_SENSORS != 0

constexpr Pin CurrentSensorPins[] = { PortCPin(5) };						// ADC pin that reads the current
constexpr float CurrentSensorFullScaleCurrents[] = { 3300/(0.01 * 50) };	// ADC Vref divided by (sense resistor value times gain before the ADC), times 1000 if we want the result in mA
constexpr const char *CurrentSensorNames[] = { "heater" };					// name of the sensor
constexpr size_t HeaterCurrentSensorNumber = 0;

#endif

// Misc definitions
constexpr Pin NeopixelOutPin = PortAPin(2);
constexpr GpioPinFunction NeopixelOutPinFunction = GpioPinFunction::AF4;	// TIM15_CH1

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	Timer					ADC					Exint PinName
	// Port A
	{ TimerOutput::none,		AdcInput::adc12_0,	Nx,	"temp3"			},	// PA00 thermistor 3
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA01 ads132m02 drdy
	{ TimerOutput::tim15_ch1,	AdcInput::none,		Nx,	"led"			},	// PA02 NP out via timer 15
	{ TimerOutput::none,		AdcInput::adc12_15,	Nx, nullptr			},	// PA03 board type
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA04 driver CS
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA05 driver SCK
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA06 driver MISO
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA07 driver MOSI
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA08 driver TMC clock
	{ TimerOutput::tim1_ch2,	AdcInput::none,		Nx,	"out0"			},	// PA09 OUT0
	{ TimerOutput::none,		AdcInput::none,		Nx,	"out2.tach"		},	// PA10
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA11 USB D-
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr 		},	// PA12 USB D+
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA13 SWDIO, ACT LED
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA14 SWCLK, STATUS LED
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PA15 adc131m02 CS

	// Port B
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB00 LDC interrupt
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB01 AS5047D CS
	{ TimerOutput::lptim_ch1,	AdcInput::none,		Nx, "io0.out"		},	// PB02 IO0 out
	{ TimerOutput::none,		AdcInput::none,		Nx, "io2.in"		},	// PB03 IO2 in
	{ TimerOutput::none,		AdcInput::none,		Nx,	"out1.tach"		},	// PB04 OUT1 tacho input
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB05 accelerometer interrupt
	{ TimerOutput::tim4_ch1,	AdcInput::none,		Nx,	nullptr			},	// PB06 ADC clock via timer 4
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB07 CAN1 Tx
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB08 CAN1 Rx
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB09 not on chip
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB10 I2C SCL
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB11 not on chip
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB12 I2C SDA
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB13 shared SPI SCK
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB14 shared SPI MISO
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PB15 shared SPI SCK

	// Port C
	{ TimerOutput::none,		AdcInput::adc12_10,	Nx,	"temp0"			},	// PC00 thermistor 0
	{ TimerOutput::none,		AdcInput::adc12_11,	Nx,	"temp1"			},	// PC01 thermistor 1
	{ TimerOutput::none,		AdcInput::adc12_12,	Nx, "temp2"			},	// PC02 thermistor 2
	{ TimerOutput::none,		AdcInput::adc12_13,	Nx, nullptr			},	// PC03 VIN monitor
	{ TimerOutput::tim2_ch4,	AdcInput::none,		Nx,	"out2"			},	// PC04 OUT2
	{ TimerOutput::none,		AdcInput::adc12_8,	Nx,	nullptr			},	// PC05 heater current
	{ TimerOutput::tim8_ch1,	AdcInput::none,		Nx,	"out1"			},	// PC06 OUT1
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC07 driver dir
	{ TimerOutput::none,		AdcInput::none,		8,	"io0.in"		},	// PC08 IO0 in
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC09 LDC1612 clock
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC10 loadcell SPI SCK
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC11 loadcell SPI MISO
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC12 loadcell SPI MOSI
	{ TimerOutput::none,		AdcInput::none,		13,	nullptr			},	// PC13 driver diag
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PC14 driver enable
	{ TimerOutput::none,		AdcInput::none,		14,	"io1.in"		},	// PC15 IO1 in

	// Port D
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PD00 not on chip
	{ TimerOutput::none,		AdcInput::none,		Nx,	nullptr			},	// PD01 not on chip
	{ TimerOutput::none,		AdcInput::none,		Nx, nullptr			},	// PD02 driver step

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TimerOutput::none,		AdcInput::none,		Nx,	"i2c.lis3dh,i2c.lis2dw,i2c.accelerometer"	},	// LIS3DH or LIS2DW12 sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TimerOutput::none,		AdcInput::ldc1612,	Nx,	"i2c.ldc1612"	},	// LDC1612 sensor connected via I2C
#endif
#if SUPPORT_AS5601
	{ TimerOutput::none,		AdcInput::none,		Nx,	"i2c.mfm"		},	// AS5601+TCA6408A filament monitor connected via I2C
#endif
#if SUPPORT_ADS131M02
	{ TimerOutput::none,		AdcInput::ads131m02, Nx, "loadcell"		},	// load cell connected to ADA131M02
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = (3 * 16) + 3;							// 16 pins each on ports A thru C, 3 pins on port D
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601 + SUPPORT_ADS131M02;

static_assert(NumPins == NumRealPins + NumVirtualPins);

#if SUPPORT_AS5601
constexpr Pin MfmPin = NumRealPins + SUPPORT_LIS3DH + SUPPORT_LDC1612;																		// pin number when the user selects magnetic filament monitor on I2C bus
#endif
#if SUPPORT_INDUCTIVE_HEATER
constexpr Pin InductiveHeaterPin = NumRealPins + SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601;											// pin number when the user selects the inductive nozzle heater
#endif
#if SUPPORT_ADS131M02
constexpr Pin LoadCellPin = NumRealPins + SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601 + SUPPORT_INDUCTIVE_HEATER;						// pin number when the user selects the load cell
#endif
#if SUPPORT_LP5817
constexpr Pin LP5817Pin = NumRealPins + SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601 + SUPPORT_INDUCTIVE_HEATER + SUPPORT_ADS131M02;	// pin number when the user selects the LP5817
#endif

// Timer/counter used to generate step pulses and other sub-millisecond timings
//constexpr unsigned int StepTcNumber = 0;
//TcCount32 * const StepTc = &(TC0->COUNT32);
//constexpr IRQn StepTcIRQn = TC0_IRQn;
//#define STEP_TC_HANDLER			TC0_Handler

// Available UART ports
#define NUM_ASYNC_PORTS		0

#endif /* SRC_CONFIG_TOOLINDX_H_ */
