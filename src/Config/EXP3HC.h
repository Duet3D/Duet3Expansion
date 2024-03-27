/*
 * Expansion1_v07.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_EXP3HC_H_
#define SRC_CONFIG_EXP3HC_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"EXP3HC"
#define BOOTLOADER_NAME		"SAME5x"

#define HAS_VREF_MONITOR		1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			1
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	1
#define HAS_BUTTONS				0
#define USE_SERIAL_DEBUG		defined(DEBUG)

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			0
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_BRAKE_PWM		1

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			1
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2660			0

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_DMA_NEOPIXEL	0

#define USE_MPU					0
#define USE_CACHE				1

constexpr bool UseAlternateCanPins = false;

constexpr size_t NumDrivers = 3;
constexpr size_t MaxSmartDrivers = 3;
constexpr float MaxTmc5160Current = 6300.0;									// the maximum current we allow the TMC5160/5161 drivers to be set to
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.050;

constexpr size_t NumAddressBits = 4;
constexpr size_t NumBoardTypeBits = 3;

constexpr size_t MaxPortsPerHeater = 3;

constexpr size_t NumThermistorInputs = 3;
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float VrefTopResistor = 15.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);
																			// there are 3 temperature sensing channels and a 4K7 load resistor
constexpr Pin GlobalTmc51xxEnablePin = PortBPin(23);
constexpr Pin GlobalTmc51xxCSPin = PortBPin(22);

#define TMC51xx_USES_SERCOM	1
Sercom * const SERCOM_TMC51xx = SERCOM0;
constexpr uint8_t SERCOM_TMC51xx_NUMBER = 0;

constexpr Pin TMC51xxMosiPin = PortBPin(24);
constexpr GpioPinFunction TMC51xxMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC51xxSclkPin = PortBPin(25);
constexpr GpioPinFunction TMC51xxSclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC51xxMisoPin = PortCPin(25);
constexpr GpioPinFunction TMC51xxMisoPinPeriphMode = GpioPinFunction::C;

PortGroup * const StepPio = &(PORT->Group[0]);		// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortAPin(25), PortAPin(27), PortAPin(1) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(23), PortCPin(28), PortAPin(0) };

constexpr Pin BoardTypePins[NumBoardTypeBits] = { PortBPin(18), PortCPin(18), PortCPin(13) };

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortCPin(10), PortCPin(7) };
constexpr bool LedActiveHigh = true;

constexpr Pin VinMonitorPin = PortAPin(10);

constexpr float VinDividerRatioPre102 = (60.4 + 4.7)/4.7;
constexpr float VinMonitorVoltageRangePre102 = VinDividerRatioPre102 * 3.3;				// We use the 3.3V supply as the voltage reference

constexpr float VinDividerRatio102AndLater = (100.0 + 5.1)/5.1;
constexpr float VinMonitorVoltageRange102AndLater = VinDividerRatio102AndLater * 3.3;	// We use the 3.3V supply as the voltage reference

constexpr Pin V12MonitorPin = PortBPin(5);
constexpr float V12DividerRatio = (60.4 + 4.7)/4.7;
constexpr float V12MonitorVoltageRange = V12DividerRatio * 3.3;							// We use the 3.3V supply as the voltage reference

constexpr Pin VrefPin = PortBPin(4);
constexpr Pin VssaPin = PortBPin(6);

constexpr Pin BoardAddressPins[4] = { PortCPin(11), PortCPin(12), PortCPin(14), PortCPin(15) };
constexpr Pin TempSensePins[NumThermistorInputs] = { PortCPin(3), PortBPin(8), PortBPin(7) };

// Shared SPI
constexpr uint8_t SspiSercomNumber = 6;
constexpr uint32_t SspiDataInPad = 3;
constexpr Pin SSPIMosiPin = PortCPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPISclkPin = PortCPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPIMisoPin = PortCPin(19);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;

constexpr auto sercom1cPad0 = SercomIo::sercom1c + SercomIo::pad0;
constexpr auto sercom1cPad1 = SercomIo::sercom1c + SercomIo::pad1;
constexpr auto sercom3cPad0 = SercomIo::sercom3c + SercomIo::pad0;
constexpr auto sercom3cPad1 = SercomIo::sercom3c + SercomIo::pad1;
constexpr auto sercom5cPad0 = SercomIo::sercom5c + SercomIo::pad0;
constexpr auto sercom5cPad1 = SercomIo::sercom5c + SercomIo::pad1;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d2.dir"	},	// PA00 driver2 DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d2.step"	},	// PA01 driver2 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA02 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io3.out"		},	// PA04
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out4"			},	// PA05
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		6,	"io0.in"		},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_7,	SercomIo::none,		SercomIo::none,		7,	"spi.cs2"		},	// PA07
	{ TcOutput::none,	TccOutput::tcc0_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out5"			},	// PA08
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	"ate.d0.diag0"	},	// PA09 driver0 diag0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA10 VINmon
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out6"			},	// PA11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	"out3.tach" 	},	// PA12
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out8"			},	// PA13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		sercom1cPad0,		Nx,	"io0.out,uart0.tx" },	// PA16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		sercom1cPad1,		SercomIo::none,		Nx,	"uart0.rx"		},	// PA17
	{ TcOutput::none,	TccOutput::tcc1_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0" 			},	// PA18
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out1"			},	// PA19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"io5.out"		},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA21 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.diag1"	},	// PA22 driver0 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.dir"	},	// PA23 driver0 dir
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out2"			},	// PA24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d0.step"	},	// PA25 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d1.step"	},	// PA27 driver1 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d2.diag1"	},	// PB00 driver2 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB02 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB03 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_6,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 VrefMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	"ate.v12"		},	// PB05 12VMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 VssaMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_9,	SercomIo::none,		SercomIo::none,		Nx,	"temp2"			},	// PB07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_0,	SercomIo::none,		SercomIo::none,		Nx,	"temp1"			},	// PB08
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out3"			},	// PB09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	"spi.cs0"		},	// PB10
	{ TcOutput::tc5_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out7"			},	// PB11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB12 CANtx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB13 CANrx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"spi.cs3"		},	// PB14 don't allow DHT11 on this pin, no EXINT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	"spi.cs1"		},	// PB15
	{ TcOutput::none,	TccOutput::tcc3_0F,	AdcInput::none,		SercomIo::none,		sercom5cPad0,		Nx,	"io1.out,uart1.tx"	},	// PB16
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		sercom5cPad1,		SercomIo::none,		Nx,	"uart1.rx"		},	// PB17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB18 board type 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,	"out4.tach"		},	// PB19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		sercom3cPad0,		Nx,	"io2.out,uart2.tx" },	// PB20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		sercom3cPad1,		SercomIo::none,		Nx,	"uart2.rx"		},	// PB21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB22 drivers CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB23 drivers ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB24 spi0 mosi
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB25 spi0 clock
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB26 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB27 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB28 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB29 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,	"ate.d2.diag0"	},	// PB30 driver2 diag0
	{ TcOutput::none,	TccOutput::tcc4_1F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	"io4.out"	},	// PB31

	// Port C
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_10,	SercomIo::none,		SercomIo::none,		0,	"io2.in"		},	// PC00 IO2 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_11,	SercomIo::none,		SercomIo::none,		1,	"io5.in"		},	// PC01 IO5 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_4,	SercomIo::none,		SercomIo::none,		2,	"io1.in"		},	// PC02 IO1 in
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_5,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"			},	// PC03 thermistor 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC04 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC05
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC06 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC07
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC08 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC09 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC10 diag LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC11 board ID 0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC12 board ID 1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC13
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC14 board ID 2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC15 board ID 3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC16 SPI1_MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC17 SPI1_CLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC18 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC19 SPI1_MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"io3.in"		},	// PC20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	"out5.tach"		},	// PC21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC22 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC23 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,	"io4.in"		},	// PC24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC25 spi0 mosi
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d1.diag1"	},	// PC26 driver1 diag1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	"ate.d1.diag0"	},	// PC27 driver1 diag0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"ate.d1.dir"	},	// PC28 driver1 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC30 not on 100-pin chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PC31 not on 100-pin chip
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 3 * 32;			// 32 pins on each of ports A, B, C
static_assert(NumPins == NumRealPins);					// no virtual pins in this table

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC6->COUNT32);
constexpr IRQn StepTcIRQn = TC6_IRQn;
constexpr unsigned int StepTcNumber = 6;
#define STEP_TC_HANDLER		TC6_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		2
constexpr IRQn Serial0_IRQn = SERCOM3_0_IRQn;
constexpr IRQn Serial1_IRQn = SERCOM5_0_IRQn;

// DMA channel assignments. Channels 0-3 have individual interrupt vectors, channels 4-31 share an interrupt vector.
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanLedTx = 2;

constexpr unsigned int NumDmaChannelsUsed = 3;			// must be at least the number of channels used, may be larger. Max 32 on the SAME51.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 3;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 3;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 4;
const NvicPriority NvicPriorityDmac = 5;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 5;

#endif /* SRC_CONFIG_EXP3HC_H_ */
