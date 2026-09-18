/*
 * RPI_PICO.h
 *
 *  Created on: 24 Aug 2022
 *      Author: David
 */

#ifndef SRC_CONFIG_RPI_PICO_H_
#define SRC_CONFIG_RPI_PICO_H_

#define BOARD_TYPE_NAME		"RPi_Pico"
#define BOOTLOADER_NAME		"RPi_Pico"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0	//TEMP!
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				1
#define USE_SERIAL_DEBUG		1

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_INPUT_SHAPING	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2240			0
#define SUPPORT_TMC2240_SPI		0

// DMA channel assignments
constexpr DmaChannel DmacChanCAN = 0;					// this must match the value used in the RP2040 CAN driver in CoreN2G!
constexpr DmaChannel DmacChanAdcRx = 1;
constexpr DmaChannel DmacChanTmcTx = 2;
constexpr DmaChannel DmacChanTmcRx = 3;					// this must be one higher than DmacChanTmcTx for RP2040 build configurations
constexpr DmaChannel DmacChanCRC = 4;
constexpr DmaChannel DmaChanWS2812 = 5;

constexpr unsigned int NumDmaChannelsUsed = 6;			// must be at least the number of channels used, may be larger. Max 12 on the RP2040.

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

// Stepper driver configuration
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
constexpr float MaxMotorCurrent = 1600.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

constexpr Pin GlobalTmcEnablePin = GpioPin(0);							// this is different from the Fly board because GPIO25 on the Pico is the LED
constexpr Pin Tmc22xxUartPin = GpioPin(21);

constexpr Pin StepPins[NumDrivers] = { GpioPin(24) };
constexpr Pin DirectionPins[NumDrivers] = { GpioPin(23) };
constexpr Pin DriverDiagPins[NumDrivers] = { GpioPin(22) };

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_LIS3DH			0

#define NUM_I2C_CHANNELS		0
#define NUM_SHARED_SPI			(SUPPORT_SPI_SENSORS)

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO	GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr unsigned int CANInstanceNumber = 0;		// not used for this configuration
constexpr bool UseLaterCanPins = false;				// not used for this configuration

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 2200.0;

constexpr Pin TempSensePins[NumThermistorInputs] = { GpioPin(26), GpioPin(27) };

constexpr CanParameters CanParams =
{
	.instanceNumber = 0,						// not used on RP2040
	.txPin = GpioPin(5),
	.rxPin = GpioPin(4),
	.pinsFunction = GpioPinFunction::None		// not used on RP2040
};

constexpr Pin ButtonPins[] = { PIN_TODO };

// VSYS voltage monitor
constexpr Pin VinMonitorPin = GpioPin(29);
constexpr float VinDividerRatio = (200.0 + 100.0)/100.0;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;				// the Pico uses the 3.3V supply as the voltage reference

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(25) };
constexpr bool LedActiveHigh = true;

#if SUPPORT_SPI_SENSORS

// Shared SPI using pins 12,13,14,15. If changing this, also change the available pins in the pin table.
// Shared SPI definitions
constexpr SpiParameters SharedSpiParams =
{
	.instanceNumber = 1,
	.mosiPin = 15,
	.misoPin = 12,
	.sclkPin = 14,
};

#endif

#if NUM_I2C_CHANNELS != 0

//****** This code cannot be used because we haven't implemented I2C on the RP2040 yet ******
// I2C using pins GPIO8,9. If changing this, also change the available pins in the pin table.
constexpr I2cParameters I2C0Params =
{
	.instanceNumber = 0;
	.sclPin = 9,
	.sdaPin = 8,
	.irqPriority = NvicPriorityI2C
};

#endif

#if SUPPORT_LIS3DH
constexpr unsigned int Lis_I2CChannel = 0;
constexpr Pin Lis3dhInt1Pin = GpioPin(10);
#endif

// Table of pin functions that we are allowed to use
//TODO restrict each of pwm0 to pwm7 to just one output, to prevent users trying to use the same PWM unit for more than one pin
constexpr PinDescription PinTable[] =
{
	//	PWM					ADC				PinName
	// Port A
	{ PwmOutput::pwm0a,	AdcInput::none,		"gpio0"		},	// GPIO0
	{ PwmOutput::pwm0b,	AdcInput::none,		"gpio1"		},	// GPIO1
	{ PwmOutput::pwm1a,	AdcInput::none,		"gpio2"		},	// GPIO2
	{ PwmOutput::pwm1b,	AdcInput::none,		"gpio3"		},	// GPIO3
	{ PwmOutput::pwm2a,	AdcInput::none,		"gpio4"		},	// GPIO4
	{ PwmOutput::pwm2b,	AdcInput::none,		"gpio5"		},	// GPIO5
	{ PwmOutput::pwm3a,	AdcInput::none,		"gpio6"		},	// GPIO6
	{ PwmOutput::pwm3b,	AdcInput::none,		"gpio7"		},	// GPIO7
#if NUM_I2C_CHANNELS != 0
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO8 reserved for I2C SDA
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO9 reserved for I2C SCL
#else
	{ PwmOutput::pwm4a,	AdcInput::none,		"gpio8"		},	// GPIO8
	{ PwmOutput::pwm4b,	AdcInput::none,		"gpio9"		},	// GPIO9
#endif
#if SUPPORT_LIS3DH
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO10 reserved for LIS3DH INT1
#else
	{ PwmOutput::pwm5a,	AdcInput::none,		"gpio10"	},	// GPIO10
#endif
	{ PwmOutput::pwm5b,	AdcInput::none,		"gpio11"	},	// GPIO11
#if SUPPORT_SPI_SENSORS
	{ PwmOutput::none,	AdcInput::none,		nullptr 	},	// GPIO12 SPI1 Rx
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO13 SPI1 CS
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO14 SPI1 Sclk
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO15 SPI1 Tx
#else
	{ PwmOutput::pwm6a,	AdcInput::none,		"gpio12" 	},	// GPIO12
	{ PwmOutput::pwm6b,	AdcInput::none,		"gpio13"	},	// GPIO13
	{ PwmOutput::pwm7a,	AdcInput::none,		"gpio14"	},	// GPIO14
	{ PwmOutput::pwm7b,	AdcInput::none,		"gpio15"	},	// GPIO15
#endif
	{ PwmOutput::pwm0a,	AdcInput::none,		"gpio16"	},	// GPIO16
	{ PwmOutput::pwm0b,	AdcInput::none,		"gpio17"	},	// GPIO17
	{ PwmOutput::pwm1a,	AdcInput::none,		"gpio18" 	},	// GPIO18
	{ PwmOutput::pwm1b,	AdcInput::none,		"gpio19"	},	// GPIO19
	{ PwmOutput::pwm2a,	AdcInput::none,		"gpio20"	},	// GPIO20
	{ PwmOutput::pwm2b,	AdcInput::none,		"gpio21"	},	// GPIO21
	{ PwmOutput::pwm3a,	AdcInput::none,		"gpio22"	},	// GPIO22
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO23 used to control the voltage regulator on the Pico
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO24 used to detect VBUS on the Pico
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO25 on-board LED which we use as the STATUS LED
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"gpio26"	},	// GPIO26
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	"gpio27"	},	// GPIO27
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	"gpio28"	},	// GPIO28
	{ PwmOutput::none,	AdcInput::adc0_3,	nullptr		},	// GPIO29 used to measure VSYS on the Pico
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 30;						// 30 GPIO pins on RP2040
static_assert(NumPins == NumRealPins);					// no virtual pins

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTimerAlarmNumber = 0;
constexpr unsigned int StepTcIRQn = TIMER_IRQ_0;

// Available UART ports
#define NUM_ASYNC_PORTS		0

#endif /* SRC_CONFIG_RPI_PICO_H_ */
