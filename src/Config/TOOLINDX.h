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

#define BOARD_TYPE_NAME		"TOOLINDX"
#define BOOTLOADER_NAME		"SAME5x_CAN_USB"

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

constexpr Pin GlobalTmcEnablePin = PortBPin(4);
constexpr Pin GlobalTmcCSPin = PortAPin(10);

#define TMC_USES_SERCOM			1
constexpr uint8_t TmcSercomNumber = 0;
Sercom * const SERCOM_TMC = SERCOM0;

constexpr Pin TMCMosiPin = PortAPin(4);
constexpr Pin TMCMisoPin = PortAPin(7);
constexpr Pin TMCSclkPin = PortAPin(5);
constexpr GpioPinFunction TMCSpiPinsPeriphMode = GpioPinFunction::D;

constexpr uint32_t Tmc2240CurrentRange = 0x01;								// which current range we set the TMC2240 to (2A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 24.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 24000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x01
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;			// with RRef = 15K this works out as 1.6A so this is the maximum current we can ask for

constexpr float MaxMotorCurrent = DriverFullScaleCurrent;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

PortGroup * const StepPio = &(PORT->Group[1]);								// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortBPin(12) };
constexpr Pin DirectionPins[NumDrivers] = { PortBPin(23) };
constexpr Pin DriverDiagPins[NumDrivers] = { PortBPin(07) };

#define SUPPORT_THERMISTORS			1
#define SUPPORT_SPI_SENSORS			0										// SPI temperature sensors not supported
#define SUPPORT_LDC1612				1
#define SUPPORT_TPiS_1T_1086_L5_5	1										// IR temperature sensor
#define SUPPORT_AS5601				0										// support direct-connected magnetic filament monitor encoder chip
#define SUPPORT_DMA_NEOPIXEL		1										// using QSPI for Neopixels
#define NEOPIXEL_USES_QSPI			1										// using QSPI for Neopixels
#define SUPPORT_INDUCTIVE_HEATER	1										// Inductive heater support
#define SUPPORT_LP5817				1										// LP5817 LED driver support
#define SUPPORT_ADS131M02			1										// ADS131M02 ADC support
#define NUM_CURRENT_SENSORS			1										// board has dedicated heater output with current measurement

#define NUM_I2C_CHANNELS		2
#define SUPPORT_LIS3DH			1

#define NUM_SHARED_SPI			1											// we use a SharedSpi for the closed loop encoder

#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				1

constexpr int CANInstanceNumber = 1;
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
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

constexpr Pin VinMonitorPin = PortAPin(2);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;							// to be confirmed
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;

// Thermistor inputs. 0 = nozzle environment, 1 = board temperature, 2 = LDC coil temperature.
// Thermistor 0 is Tewa TT7-10KX3-11 (10kOhm, B3977), https://www.tme.eu/Document/32a31570f1c819f9b3730213e5eca259/TT7-10KC3-11.pdf
// R25 = 10000, R75 = 1480, R125 = 338. From these and using the SRS calculator we deduce R25=10000, B=4333, C=1.03958e-7.
// Thermistors 1 and 2 are 10K Murata NCU15XH103J6SRC. B25/50 = 3380, B25/80 = 3428, B25/85 = 3434, B25/100 = 3455
// From this we deduce R25 = 10000, R50 = 4160.1, R80 = 1668.5, R85 = 1452.2, R100 = 973.8
// The following Beta and C values use the 25, 50 and 65C values
// We don't use a Vref/Vssa calibration chain on this board
#define CUSTOM_THERMISTORS			1										// we provide nonstandard R25, beta and series resistor values

constexpr size_t NumThermistorInputs = 3;
constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(11), PortBPin(8), PortBPin(9) };
constexpr float ThermistorSeriesR[NumThermistorInputs] = { 3900, 3900, 3900 };
constexpr float ThermistorR25[NumThermistorInputs] = { 10000, 10000, 10000 };
constexpr float ThermistorBeta[NumThermistorInputs] = { 4333, 3425.0, 3425.0 };
constexpr float ThermistorShC[NumThermistorInputs] = { 1.03958e-7, 1.68e-7, 1.68e-7 };
constexpr unsigned int envThermistorAdcFilterChannel = 0;

constexpr float DefaultThermistorSeriesR = 3900;							// needed for initialisation but not actually used

#if SUPPORT_CLOSED_LOOP

// Shared SPI definitions
constexpr SpiParameters SharedSpiParams =
{
	.sercomNumber = 5,
	.mosiPin = PortBPin(2),
	.misoPin = PortBPin(1),
	.sclkPin = PortBPin(3),
	.pinFunction = GpioPinFunction::D,
	.dataInPad = 3,
	.dataOutPad = 0,
	.dmaChanTx = DmacChanSspiTx,
	.dmaChanRx = DmacChanSspiRx,
	.dmaPrioTx = DmacPrioSspiTx,
	.dmaPrioRx = DmacPrioSspiRx,
};

constexpr Pin EncoderCsPin = PortBPin(0);

// Clock generator pin for TMC2240
constexpr uint8_t TmcClockGclkNumber = 7;
constexpr Pin TmcClockPin = PortBPin(13);
constexpr GpioPinFunction TmcClockPinPeriphMode = GpioPinFunction::M;

# define SUPPORT_MT6835					0
# define SUPPORT_QUADRATURE_ENCODER		0
# define SUPPORT_COMPOSITE_ENCODER		0

#endif

#if NUM_I2C_CHANNELS >= 1

// I2C0 using pins PA22,23 (SERCOM 3)
const I2cParameters I2C0Params =
{
	.sercomNumber = 3,
	.sclPin = PortAPin(22),
	.sdaPin = PortAPin(23),
	.pinFunction = GpioPinFunction::C,
	.irqPriority = NvicPriorityI2C
};

#endif

#if NUM_I2C_CHANNELS >= 2

// I2C1 using pins PA12,13 (SERCOM 4)
const I2cParameters I2C1Params =
{
	.sercomNumber = 4,
	.sclPin = PortAPin(12),
	.sdaPin = PortAPin(13),
	.pinFunction = GpioPinFunction::D,
	.irqPriority = NvicPriorityI2C
};

#endif

#if SUPPORT_LIS3DH												// we actually use a LIS2DW
# define ACCELEROMETER_USES_SPI			(0)						// accelerometer is connected via I2C
constexpr unsigned int Lis_I2CChannel = 0;
constexpr Pin Lis3dhInt1Pin = PortAPin(27);
#endif

#if SUPPORT_LDC1612
constexpr unsigned int LDC1612_I2CChannel = 0;
constexpr uint16_t LDC1612_I2CAddress = 0x2A;					// pin 4 is tied low
constexpr unsigned int LDC1612GClkNumber = 5;
constexpr Pin LDC1612ClockGenPin = PortBPin(11);
constexpr Pin LDC1612InterruptPin = PortBPin(10);
#endif

#if SUPPORT_TPiS_1T_1086_L5_5
constexpr unsigned int TPiS_I2CChannel = 1;
constexpr uint16_t TPiS_I2CAddress = 0x0C;
#endif

#if SUPPORT_LP5817
constexpr unsigned int LP5817_I2CChannel = 1;
constexpr uint16_t LP5817_I2CAddress = 0x2D;
#endif

#if SUPPORT_AS5601
constexpr unsigned int AS5601_I2CChannel = 0;					//TODO which I2C?
constexpr uint16_t AS5601_I2CAddress = 0x36;					// I2C address of the AS5601
#endif

#if SUPPORT_ADS131M02

constexpr SpiParameters Ads131M02SpiParams =
{
	.sercomNumber = 1,
	.mosiPin = PortAPin(16),
	.misoPin = PortAPin(19),
	.sclkPin = PortAPin(17),
	.pinFunction = GpioPinFunction::C,
	.dataInPad = 3,
	.dataOutPad = 0,
	.dmaChanTx = DmacChanADS131M02Tx,
	.dmaChanRx = DmacChanADS131M02Rx,
	.dmaPrioTx = DmacPrioADS131M02Tx,
	.dmaPrioRx = DmacPrioADS131M02Rx,
};

constexpr Pin ADS131M02_CsPin = PortAPin(18);
constexpr Pin ADS131M02_DRDYPin = PortBPin(22);
constexpr Pin ADS131M02_GclkPin = PortBPin(16);
constexpr GpioPinFunction ADS131M02_GclkPinFunction = GpioPinFunction::M;
constexpr unsigned int ADA131M02_GclkNumber = 2;
#endif

#if SUPPORT_INDUCTIVE_HEATER

// Definitions for inductive heater support

#define HEATER_POLL_RATE_MULTIPLIER		(10)					// how many times faster we run the temperature control loop than the standard 4Hz. Preferably a divisor of HeatSampleIntervalMillis.

constexpr unsigned int InductiveHeaterOscTccDeviceNumber = 3;	// number of the TC we use to generate the ~120kHz signal to excite the resonant circuit
constexpr unsigned int InductiveHeaterOscTccOutputNumber = 0;	// which output from the TCC we are using
constexpr unsigned int InductiveHeaterPwmTccDeviceNumber = 0;	// number of the TCC we use to generate the PWM signal that is gated with the osc signal
constexpr unsigned int InductiveHeaterPwmTccOutputNumber = 0;	// which output from the TCC we are using
constexpr unsigned int InductiveHeaterCCLNumber = 3;			// number of the CCL that we use to gate the TC and TCC output together
constexpr unsigned int InductiveHeaterCCLOutPin = PortBPin(17);	// the CCL output pin that drive the inductive heater mosfet
constexpr unsigned int InductiveHeaterAuxCCLNumber = 0;			// number of the second CCL that we need to use to gate two TCCs together

constexpr GpioPinFunction InductiveHeaterCCLOutPinPeriphMode = GpioPinFunction::N;
constexpr Pin InductiveHeaterVoltageFeedbackAdcPin = PortAPin(5);
constexpr Pin InductiveHeaterVoltageFeedbackAcPin = PortAPin(6);
constexpr float InductiveHeaterVoltageFeedbackRatio = (3 * 15 + 1.5)/1.5;							// 3 series resistors of 15K each, one 1.5K shunt resistor.
constexpr float InductiveHeaterVoltageFeedbackRange = 3.3 * InductiveHeaterVoltageFeedbackRatio;	// this comes to 102.3V

#include <HeaterModel.h>

constexpr HeaterModel InductiveHeaterDefaultModel =
{
	.heatingRate = 25.0,
	.basicCoolingRate = 1.9,
	.fanCoolingRate = 0.0,
	.coolingRateExponent = 1.35,
	.deadTime = 0.30,
	.temperatureCoefficient = 0.0,
	.typicalTemperature = 220.0,
	.standardVoltage = 24.0,
	.fzero = 0.0,
	.usePid = true,
	.zero = 0
};

constexpr float InductiveHeaterMaxFaultTime = 0.8;							// needs to be short enough to detect that there is no tool before damage is caused

#endif

#if NUM_CURRENT_SENSORS != 0

constexpr Pin CurrentSensorPins[] = { PortBPin(6) };						// ADC pin that reads the current
constexpr float CurrentSensorFullScaleCurrents[] = { 3300/(0.02 * 50) };	// ADC Vref divided by (sense resistor value times gain before the ADC), times 1000 if we want the result in mA
constexpr const char *CurrentSensorNames[] = { "heater" };					// name of the sensor
constexpr size_t InductiveHeaterCurrentSensorNumber = 0;

#endif

// Misc definitions
constexpr Pin UsbNotCanSelectPin = PortBPin(31);

constexpr Pin NeopixelOutPin = PortAPin(8);
constexpr GpioPinFunction NeopixelOutPinFunction = GpioPinFunction::H;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	"pcfan.tach"	},	// PA00 print cooling fan tacho
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pcfan"			},	// PA01 print cooling fan out
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 board type
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 SPI0 MOSI (Stepper, sercom0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 SPI0 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA06 heater voltage feedback (also on PB05). This pin also feeds AC2.
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA07 SPI0 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"led"			},	// PA08 NP out (QSPI D0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"io0.in"		},	// PA09 endstop
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA10 SPI0_CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		Nx,	"ate.envtemp"	},	// PA11 hot end surround thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA12 I2C1 SCL (sercom4)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 I2C1 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 SPI1 MOSI (ADC, sercom1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 SPI1 SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA18 SPI1 CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA19 SPI1 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"hsfan.tach"	},	// PA20 heatsink fan tacho
	{ TcOutput::none,	TccOutput::tcc1_5F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"hsfan"			},	// PA21 heatsink fan
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 I2C0 SCL (sercom3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA23 I2C0 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 USB DN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 USB DP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr			},	// PA27 accelerometer interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk and LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio and LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 SPI2_CS0 (AS5047D,sercom5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 SPI2 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PB02 SPI2 MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PB03 SPI2 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 Driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	"ate.heaterv"	},	// PB05 Heater voltage feedback (also on PA06)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 Heater current
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	nullptr			},	// PB07 Driver diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,	"boardtemp"		},	// PB08 board thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,	"coiltemp"		},	// PB09 LDC coil temperature
	{ TcOutput::none,	TccOutput::tcc0_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	nullptr			},	// PB10 LDC interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB11 LDC1612 clock (GCLK5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB12 Driver step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB13 Driver clock (GCLK7)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB14 CAN1 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB15 CAN1 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB16 ADC clock (GCLK2)
	{ TcOutput::none,	TccOutput::tcc3_1F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB17 CCL_OUT3 for heater FET drive
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	nullptr			},	// PB22 ADC !DRDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB23 driver DIR and BOOTLOADER_RESET jumper
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB24 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB25 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB27 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB29 not on chip
	{ TcOutput::none,	TccOutput::tcc4_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pb30"			},	// PB30 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB31 USB/!CAN select

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.lis3dh,i2c.lis2dw,i2c.accelerometer"	},	// LIS3DH or LIS2DW12 sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::ldc1612,	SercomIo::none,		SercomIo::none,		Nx,	"i2c.ldc1612"	},	// LDC1612 sensor connected via I2C
#endif
#if SUPPORT_AS5601
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.mfm"		},	// AS5601+TCA6408A filament monitor connected via I2C
#endif
#if SUPPORT_INDUCTIVE_HEATER
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "nozzleheat"	},	// inductive heater
#endif
#if SUPPORT_ADS131M02
	{ TcOutput::none,	TccOutput::none,	AdcInput::ads131m02, SercomIo::none,	SercomIo::none,		Nx, "loadcell"		},	// load cell connected to ADA131M02
#endif
#if SUPPORT_LP5817
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, "ate.lp5817"	},	// LP5817
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 32;			// 32 pins on port A (some missing), 32 on port B (some missing)
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601 + SUPPORT_INDUCTIVE_HEATER + SUPPORT_ADS131M02 + SUPPORT_LP5817;

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
constexpr unsigned int StepTcNumber = 0;
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
#define STEP_TC_HANDLER			TC0_Handler

// Available UART ports
#define NUM_ASYNC_PORTS		0

#endif /* SRC_CONFIG_TOOLINDX_H_ */
