/*
 * Fly36_RRF.h
 *
 *  Created on: 24 Dec 2022
 *      Author: David
 */

#ifndef SRC_CONFIG_FLY36_RRF_H_
#define SRC_CONFIG_FLY36_RRF_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"Fly36RRF"
#define BOOTLOADER_NAME		"Fly36RRF"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2240			0

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

#define TMC22xx_HAS_MUX					0
#define TMC22xx_SINGLE_DRIVER			1
#define TMC22xx_HAS_ENABLE_PINS			0
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_USE_SLAVEADDR			0

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable on the Duet Maestro, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 200000;
constexpr uint32_t TransferTimeout = 10;									// any transfer should complete within 10 ticks @ 1ms/tick

constexpr float DriverSenseResistor = 0.11 + 0.02;							// in ohms
constexpr float DriverVRef = 180.0;											// in mV
constexpr float DriverFullScaleCurrent = DriverVRef/DriverSenseResistor;	// in mA
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
constexpr float MaximumMotorCurrent = 1600.0;
constexpr float MaximumStandstillCurrent = 1200.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

constexpr Pin GlobalTmc22xxEnablePin = GpioPin(25);
constexpr Pin Tmc22xxUartPin = GpioPin(1);

constexpr Pin StepPins[NumDrivers] = { GpioPin(24) };
constexpr Pin DirectionPins[NumDrivers] = { GpioPin(23) };
constexpr Pin DriverDiagPins[NumDrivers] = { GpioPin(22) };

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			1
#define SUPPORT_DHT_SENSOR		0

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO	GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 4700.0;		// TEMP0 has 1K or 4K7 pullup, chamber thermistor has 4K7

constexpr Pin TempSensePins[NumThermistorInputs] = { GpioPin(26), GpioPin(27) };

constexpr Pin CanTxPin = GpioPin(5);
constexpr Pin CanRxPin = GpioPin(4);

constexpr Pin ButtonPins[] = { PIN_TODO };

// VIN voltage monitor
constexpr Pin VinMonitorPin = GpioPin(28);
constexpr float VinDividerRatio = (47.0 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;				// the Pico uses the 3.3V supply as the voltage reference

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(20) };
constexpr bool LedActiveHigh = false;

#if SUPPORT_SPI_SENSORS

// Shared SPI pin connections
constexpr uint8_t SspiSpiInstanceNumber = 1;
constexpr Pin SSPIMosiPin = GpioPin(11);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPISclkPin = GpioPin(10);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPIMisoPin = GpioPin(12);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::Spi;

#endif

#if SUPPORT_LIS3DH

#define ACCELEROMETER_USES_SPI			(1)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr Pin Lis3dhCsPin = GpioPin(9);
constexpr Pin Lis3dhInt1Pin = GpioPin(29);

#endif

// Table of pin functions that we are allowed to use
//TODO restrict each of pwm0 to pwm7 to just one output, to prevent users trying to use the same PWM unit for more than one pin
constexpr PinDescription PinTable[] =
{
	//	PWM					ADC				PinName
	// Port A
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO0 TMC UART through 1K resistor
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO1 TMC UART
	{ PwmOutput::pwm1a,	AdcInput::none,		"io2.in"	},	// GPIO2 ENDSTOP2
	{ PwmOutput::pwm1b,	AdcInput::none,		"io0.in"	},	// GPIO3 PROBE (BLTouch in)
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO4 CAN RX
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO5 CAN TX
	{ PwmOutput::pwm3a,	AdcInput::none,		"rgbled"	},	// GPIO6 RGB
	{ PwmOutput::pwm3b,	AdcInput::none,		"io0.out"	},	// GPIO7 SERVO (BLTouch out)
	{ PwmOutput::pwm4a,	AdcInput::none,		"out0"		},	// GPIO8 HEAT0
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO9 accelerometer CS
	{ PwmOutput::pwm5a,	AdcInput::none,		nullptr		},	// GPIO10 SPI SCLK
	{ PwmOutput::pwm5b,	AdcInput::none,		nullptr		},	// GPIO11 SPI MOSI
	{ PwmOutput::pwm6a,	AdcInput::none,		nullptr 	},	// GPIO12 SPI MISO
	{ PwmOutput::pwm6b,	AdcInput::none,		"max31865cs,rtdcs" },	// GPIO13 SPI MAX31856 CS
	{ PwmOutput::pwm7a,	AdcInput::none,		"out2"		},	// GPIO14 FAN1
	{ PwmOutput::pwm7b,	AdcInput::none,		"out1"		},	// GPIO15 FAN0
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO16 SPI0 MISO
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO17 SPI0 AS5047D CS
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr 	},	// GPIO18 SPI0 SCLK
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO19 SPI0 MOSI
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO20 status LED
	{ PwmOutput::pwm2b,	AdcInput::none,		"io1.in"	},	// GPIO21 ENDSTOP1
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO22 DIAG
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO23 DIR
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO24 STEP
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO25 EN
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"temp0"		},	// GPIO26 T0_TEMP
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	"temp1"		},	// GPIO27 CHAMBER_TEMP
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	nullptr		},	// GPIO28 VIN ADC
	{ PwmOutput::none,	AdcInput::adc0_3,	nullptr		},	// GPIO29 ACC_INT1
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static_assert(NumPins == 30);		// 30 GPIO pins on RP2040

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTimerAlarmNumber = 0;
constexpr unsigned int StepTcIRQn = TIMER_IRQ_0;

// Available UART ports
#define NUM_SERIAL_PORTS		1
//constexpr IRQn Serial0_IRQn = SERCOM5_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanCAN = 0;					// this must match the value used in the RP2040 CAN driver in CoreN2G!
constexpr DmaChannel DmacChanAdcRx = 1;
constexpr DmaChannel DmacChanTmcTx = 2;
constexpr DmaChannel DmacChanTmcRx = 3;					// this must be one higher than DmacChanTmcTx for RP2040 build configurations
constexpr DmaChannel DmacChanCRC = 4;

constexpr unsigned int NumDmaChannelsUsed = 5;			// must be at least the number of channels used, may be larger. Max 12 on the RP2040.

// DMA priorities, higher is better. RP2040 has only 0 and 1.
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 1;
constexpr DmaPriority DmacPrioAdcRx = 1;

// Interrupt priorities, lower means higher priority. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C = 2;
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 3;
const NvicPriority NvicPriorityUSB = 3;

#endif /* SRC_CONFIG_FLY36_RRF_H_ */
