/*
 * TMC22xx.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#include "TMC22xx.h"

#if SUPPORT_TMC22xx

#ifndef TMC22xx_HAS_ENABLE_PINS
# error TMC22xx_HAS_ENABLE_PINS not defined
#endif

#ifndef TMC22xx_SINGLE_DRIVER
# error TMC22xx_SINGLE_DRIVER not defined
#endif

#ifndef TMC22xx_HAS_MUX
# error TMC22xx_HAS_MUX not defined
#endif

#ifndef TMC22xx_VARIABLE_NUM_DRIVERS
# error TMC22xx_VARIABLE_NUM_DRIVERS not defined
#endif

#if defined(SAME51) || defined(SAMC21)
# include "Movement/Move.h"
# include "Movement/StepTimer.h"
# include <Hardware/IoPorts.h>
# include <Hardware/DmacManager.h>
# include <Hardware/Serial.h>
# include <component/sercom.h>
#else
# include "RepRap.h"
# include "Movement/Move.h"
# include "Movement/StepTimer.h"
# include "sam/drivers/pdc/pdc.h"
# include "sam/drivers/uart/uart.h"
#endif

// Important note:
// The TMC2224 does handle a write request immediately followed by a read request.
// The TMC2224 does _not_ handle back-to-back read requests, it needs a short delay between them.

constexpr float MaximumMotorCurrent = 1600.0;				// we can't go any higher withour switching to the low sensitivity range
constexpr float MaximumStandstillCurrent = 1400.0;
constexpr float MinimumOpenLoadMotorCurrent = 300;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr size_t TmcTaskStackWords = 100;

#if TMC22xx_VARIABLE_NUM_DRIVERS

static size_t numTmc22xxDrivers;
static inline size_t GetNumTmcDrivers() { return numTmc22xxDrivers; }

#else

static inline constexpr size_t GetNumTmcDrivers() { return MaxSmartDrivers; }

#endif

enum class DriversState : uint8_t
{
	noPower = 0,
	notInitialised,
	initialising,
	ready
};

static DriversState driversState = DriversState::noPower;

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;
constexpr uint32_t GCONF_USE_VREF = 1 << 0;					// use external VRef
constexpr uint32_t GCONF_INT_RSENSE = 1 << 1;				// use internal sense resistors
constexpr uint32_t GCONF_SPREAD_CYCLE = 1 << 2;				// use spread cycle mode (else stealthchop mode)
constexpr uint32_t GCONF_REV_DIR = 1 << 3;					// reverse motor direction
constexpr uint32_t GCONF_INDEX_OTPW = 1 << 4;				// INDEX output shows over temperature warning (else it shows first microstep position)
constexpr uint32_t GCONF_INDEX_PULSE = 1 << 5;				// INDEX output shows pulses from internal pulse generator, else as set by GCONF_INDEX_OTPW
constexpr uint32_t GCONF_UART = 1 << 6;						// PDN_UART used for UART interface (else used for power down)
constexpr uint32_t GCONF_MSTEP_REG = 1 << 7;				// microstep resolution set by MSTEP register (else by MS1 and MS2 pins)
constexpr uint32_t GCONF_MULTISTEP_FILT = 1 << 8;			// pulse generation optimised for >750Hz full stepping frequency
constexpr uint32_t GCONF_TEST_MODE = 1 << 9;				// test mode, do not set this bit for normal operation

constexpr uint32_t DefaultGConfReg = GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT;

// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1 << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1 << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1 << 2;					// undervoltage on charge pump, driver disabled. Not latched so does not need to be cleared.

// IFCOUNT register (0x02, RO)
constexpr uint8_t REGNUM_IFCOUNT = 0x02;
constexpr uint32_t IFCOUNT_MASK = 0x000F;					// interface transmission counter

// SLAVECONF register (0x03, WO)
constexpr uint8_t REGNUM_SLAVECONF = 0x03;
constexpr uint32_t SLAVECONF_SENDDLY_8_BITS = 0 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_24_BITS = 2 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_40_BITS = 4 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_56_BITS = 6 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_72_BITS = 8 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_88_BITS = 10 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_104_BITS = 12 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_120_BITS = 14 << 8;

constexpr uint32_t DefaultSlaveConfReg = SLAVECONF_SENDDLY_8_BITS;	// we don't need any delay between transmission and reception

// OTP_PROG register (0x04, WO)
constexpr uint8_t REGNUM_OTP_PROG = 0x04;
constexpr uint32_t OTP_PROG_BIT_SHIFT = 0;
constexpr uint32_t OTP_PROG_BIT_MASK = 7 << OTP_PROG_BIT_SHIFT;
constexpr uint32_t OTP_PROG_BYTE_SHIFT = 4;
constexpr uint32_t OTP_PROG_BYTE_MASK = 3 << OTP_PROG_BYTE_SHIFT;
constexpr uint32_t OTP_PROG_MAGIC = 0xBD << 8;

// OTP_READ register (0x05, RO)
constexpr uint8_t REGNUM_OTP_READ = 0x05;
constexpr uint32_t OTP_READ_BYTE0_SHIFT = 0;
constexpr uint32_t OTP_READ_BYTE0_MASK = 0xFF << OTP_READ_BYTE0_SHIFT;
constexpr uint32_t OTP_READ_BYTE1_SHIFT = 8;
constexpr uint32_t OTP_READ_BYTE1_MASK = 0xFF << OTP_READ_BYTE1_SHIFT;
constexpr uint32_t OTP_READ_BYTE2_SHIFT = 16;
constexpr uint32_t OTP_READ_BYTE2_MASK = 0xFF << OTP_READ_BYTE2_SHIFT;

// IOIN register (0x06, RO)
constexpr uint8_t REGNUM_IOIN = 0x06;
constexpr uint32_t IOIN_220x_ENN = 1 << 0;
constexpr uint32_t IOIN_222x_PDN_UART = 1 << 1;
constexpr uint32_t IOIN_220x_MS1 = 1 << 2;
constexpr uint32_t IOIN_222x_SPREAD = 2 << 1;
constexpr uint32_t IOIN_220x_MS2 = 1 << 3;
constexpr uint32_t IOIN_222x_DIR = 1 << 3;
constexpr uint32_t IOIN_220x_DIAG = 1 << 4;
constexpr uint32_t IOIN_222x_ENN = 1 << 4;
constexpr uint32_t IOIN_222x_STEP = 1 << 5;
constexpr uint32_t IOIN_220x_PDN_UART = 1 << 6;
constexpr uint32_t IOIN_222x_MS1 = 1 << 6;
constexpr uint32_t IOIN_220x_STEP = 1 << 7;
constexpr uint32_t IOIN_222x_MS2 = 1 << 7;
constexpr uint32_t IOIN_IS_220x = 1 << 8;					// 1 if TMC220x, 0 if TMC222x
constexpr uint32_t IOIN_2209_SPREAD_EN = 1 << 8;
constexpr uint32_t IOIN_220x_DIR = 1 << 9;
constexpr uint32_t IOIN_VERSION_SHIFT = 24;
constexpr uint32_t IOIN_VERSION_MASK = 0xFF << IOIN_VERSION_SHIFT;

// FACTORY_CONF register (0x07, RW)
constexpr uint8_t REGNUM_FACTORY_CONF = 0x07;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_SHIFT = 0;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_MASK = 0x0F << FACTORY_CONF_FCLKTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_SHIFT = 8;
constexpr uint32_t FACTORY_CONF_OTTRIM_MASK = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_143_120 = 0x00 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_120 = 0x01 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_143 = 0x02 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_157_143 = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;

// Velocity dependent control registers

// IHOLD_IRUN register (WO)
constexpr uint8_t REGNUM_IHOLDIRUN = 0x10;
constexpr uint32_t IHOLDIRUN_IHOLD_SHIFT = 0;				// standstill current
constexpr uint32_t IHOLDIRUN_IHOLD_MASK = 0x1F << IHOLDIRUN_IHOLD_SHIFT;
constexpr uint32_t IHOLDIRUN_IRUN_SHIFT = 8;
constexpr uint32_t IHOLDIRUN_IRUN_MASK = 0x1F << IHOLDIRUN_IRUN_SHIFT;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_SHIFT = 16;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_MASK = 0x0F << IHOLDIRUN_IHOLDDELAY_SHIFT;

constexpr uint32_t DefaultIholdIrunReg = (0 << IHOLDIRUN_IHOLD_SHIFT) | (0 << IHOLDIRUN_IRUN_SHIFT) | (2 << IHOLDIRUN_IHOLDDELAY_SHIFT);
															// approx. 0.5 sec motor current reduction to low power

constexpr uint8_t REGNUM_TPOWER_DOWN = 0x11;
constexpr uint8_t REGNUM_TSTEP = 0x12;
constexpr uint8_t REGNUM_TPWMTHRS = 0x13;
constexpr uint8_t REGNUM_VACTUAL = 0x22;

// Stallguard registers (TMC2209 only)
constexpr uint8_t REGNUM_TCOOLTHRS = 0x14;
constexpr uint8_t REGNUM_SGTHRS = 0x40;
constexpr uint8_t REGNUM_SG_RESULT = 0x41;
constexpr uint8_t REGNUM_COOLCONF = 0x42;

// Sequencer registers (read only)
constexpr uint8_t REGNUM_MSCNT = 0x6A;
constexpr uint8_t REGNUM_MSCURACT = 0x6B;

// Chopper control registers

// CHOPCONF register
constexpr uint8_t REGNUM_CHOPCONF = 0x6C;
constexpr uint32_t CHOPCONF_TOFF_SHIFT = 0;					// off time setting, 0 = disable driver
constexpr uint32_t CHOPCONF_TOFF_MASK = 0x0F << CHOPCONF_TOFF_SHIFT;
constexpr uint32_t CHOPCONF_HSTRT_SHIFT = 4;				// hysteresis start
constexpr uint32_t CHOPCONF_HSTRT_MASK = 0x07 << CHOPCONF_HSTRT_SHIFT;
constexpr uint32_t CHOPCONF_HEND_SHIFT = 7;					// hysteresis end
constexpr uint32_t CHOPCONF_HEND_MASK = 0x0F << CHOPCONF_HEND_SHIFT;
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_VSENSE_HIGH = 1 << 17;			// use high sensitivity current scaling
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1 << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1 << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1 << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1 << 31;				// disable low side short protection

constexpr uint32_t DefaultChopConfReg = 0x10000053 | CHOPCONF_VSENSE_HIGH;	// this is the reset default + CHOPCONF_VSENSE_HIGH - try it until we find something better

// DRV_STATUS register. See the .h file for the bit definitions.
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC10D0024;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Send/receive data and CRC stuff

// Data format to write a driver register:
// Byte 0 sync byte, 0xA0 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register register address to write | 0x80
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC of bytes 0-6

// Data format to read a driver register:
// Byte 0 sync byte, 0xA0 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register address to read (top bit clear)
// Byte 3 8-bit CRC of bytes 0-2

// Reply to a read request:
// Byte 0 sync byte, 0xA0
// Byte 1 master address, 0xFF
// Byte 2 register address (top bit clear)
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC

// Add 1 bit to a CRC
static inline constexpr uint8_t CRCAddBit(uint8_t crc, uint8_t currentByte, uint8_t bit)
{
	return (((crc ^ (currentByte << (7 - bit))) & 0x80) != 0)
			? (crc << 1) ^ 0x07
				: (crc << 1);
}

// Add a byte to a CRC
static inline constexpr uint8_t CRCAddByte(uint8_t crc, uint8_t currentByte)
{
	crc = CRCAddBit(crc, currentByte, 0);
	crc = CRCAddBit(crc, currentByte, 1);
	crc = CRCAddBit(crc, currentByte, 2);
	crc = CRCAddBit(crc, currentByte, 3);
	crc = CRCAddBit(crc, currentByte, 4);
	crc = CRCAddBit(crc, currentByte, 5);
	crc = CRCAddBit(crc, currentByte, 6);
	crc = CRCAddBit(crc, currentByte, 7);
	return crc;
}

// CRC of the first 2 bytes we send in any request
static constexpr uint8_t InitialSendCRC = CRCAddByte(CRCAddByte(0, 0x05), 0x00);

// CRC of a request to read the IFCOUNT register
static constexpr uint8_t ReadIfcountCRC = CRCAddByte(InitialSendCRC, REGNUM_IFCOUNT);

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
#if TMC22xx_HAS_ENABLE_PINS
	void Init(uint32_t p_driverNumber, Pin p_pin);
#else
	void Init(uint32_t p_driverNumber);
#endif
	void SetAxisNumber(size_t p_axisNumber);
	uint32_t GetAxisNumber() const { return axisNumber; }
	void WriteAll();
	bool SetMicrostepping(uint32_t shift, bool interpolate);
	unsigned int GetMicrostepping(bool& interpolation) const;		// Get microstepping
	bool SetDriverMode(unsigned int mode);
	DriverMode GetDriverMode() const;
	void SetCurrent(float current);
	void Enable(bool en);
	void AppendDriverStatus(const StringRef& reply);
	uint8_t GetDriverNumber() const { return driverNumber; }
	bool UpdatePending() const { return registersToUpdate != 0; }

#if TMC22xx_HAS_ENABLE_PINS
	bool UsesGlobalEnable() const { return enablePin == NoPin; }
#endif

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal);
	uint32_t GetRegister(SmartDriverRegister reg) const;

	float GetStandstillCurrentPercent() const;
	void SetStandstillCurrentPercent(float percent);

	void TransferDone() __attribute__ ((hot));				// called by the ISR when the SPI transfer has completed
	void StartTransfer() __attribute__ ((hot));				// called to start a transfer
	void TransferTimedOut() { ++numTimeouts; }
	void AbortTransfer();

	uint32_t ReadLiveStatus() const;
	uint32_t ReadAccumulatedStatus(uint32_t bitsToKeep);

	// Variables used by the ISR
	static uint32_t transferStartedTime;

	void UartTmcHandler();									// core of the ISR for this driver

private:
	bool SetChopConf(uint32_t newVal);
	void UpdateRegister(size_t regIndex, uint32_t regVal);
	void UpdateChopConfRegister();							// calculate the chopper control register and flag it for sending
	void UpdateCurrent();
	void UpdateMaxOpenLoadStepInterval();

#if TMC22xx_HAS_MUX
	void SetUartMux();
#endif
#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
	static void SetupDMASend(uint8_t regnum, uint32_t outVal, uint8_t crc) __attribute__ ((hot));	// set up the PDC to send a register
	static void SetupDMAReceive(uint8_t regnum, uint8_t crc) __attribute__ ((hot));					// set up the PDC to receive a register
#else
	void SetupDMASend(uint8_t regnum, uint32_t outVal, uint8_t crc) __attribute__ ((hot));			// set up the PDC to send a register
	void SetupDMAReceive(uint8_t regnum, uint8_t crc) __attribute__ ((hot));						// set up the PDC to receive a register
#endif

	static constexpr unsigned int NumWriteRegisters = 6;	// the number of registers that we write to
	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;			// microstepping
	static constexpr unsigned int WriteSlaveConf = 1;		// read response timing
	static constexpr unsigned int WriteChopConf = 2;		// enable/disable and microstep setting
	static constexpr unsigned int WriteIholdIrun = 3;		// current setting
	static constexpr unsigned int WritePwmConf = 4;			// read register select, sense voltage high/low sensitivity
	static constexpr unsigned int WriteTpwmthrs = 5;		// upper step rate limit for stealthchop

	static constexpr unsigned int NumReadRegisters = 4;		// the number of registers that we read from
	static const uint8_t ReadRegNumbers[NumReadRegisters];	// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;
	static constexpr unsigned int ReadDrvStat = 1;
	static constexpr unsigned int ReadMsCnt = 2;
	static constexpr unsigned int ReadPwmScale = 3;

	volatile uint32_t writeRegisters[NumWriteRegisters];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedReadRegisters[NumReadRegisters];

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	volatile uint32_t registersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip
	volatile uint32_t registerBeingUpdated;					// which register we are sending

	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	uint32_t motorCurrent;									// the configured motor current
	uint32_t maxOpenLoadStepInterval;						// the maximum step pulse interval for which we consider open load detection to be reliable

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
# if TMC22xx_USES_SERCOM
	static Sercom * const sercom;
	static uint8_t const sercomNumber;
# else
	static Uart * const uart;								// the UART that controls all drivers
# endif
#else
# if TMC22xx_USES_SERCOM
	Sercom * sercom;										// the SERCOM that controls this driver
	uint8_t sercomNumber;
# else
	Uart *uart;												// the UART that controls this driver
# endif
#endif

	// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
	// This is the message we send - volatile because we care about when it is written
	static volatile uint8_t sendData[12];

	// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
	static volatile uint8_t receiveData[20];

	uint16_t readErrors;									// how many read errors we had
	uint16_t writeErrors;									// how many write errors we had
	uint16_t numReads;										// how many successful reads we had
	uint16_t numTimeouts;									// how many times a transfer timed out

#if TMC22xx_HAS_ENABLE_PINS
	Pin enablePin;											// the enable pin of this driver, if it has its own
#endif
	uint8_t driverNumber;									// the number of this driver as addressed by the UART multiplexer
	uint8_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t registerToRead;									// the next register we need to read
	uint8_t lastIfCount;									// the value of the IFCNT register last time we read it
	volatile uint8_t writeRegCRCs[NumWriteRegisters];		// CRCs of the messages needed to update the registers
	static const uint8_t ReadRegCRCs[NumReadRegisters];		// CRCs of the messages needed to read the registers
	bool enabled;											// true if driver is enabled
};

// Static data members of class TmcDriverState

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
# if TMC22xx_USES_SERCOM
Sercom * const TmcDriverState::sercom = SERCOM_TMC22xx;
uint8_t const TmcDriverState::sercomNumber = TMC22xxSercomNumber;

# else
Uart * const TmcDriverState::uart = UART_TMC22xx;
# endif
#endif

// TMC22xx management task
static Task<TmcTaskStackWords> tmcTask;

static DmaCallbackReason dmaFinishedReason;

// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
// This is the message we send - volatile because we care about when it is written
volatile uint8_t TmcDriverState::sendData[12] =
{
	0x05, 0x00,							// sync byte and slave address
	0x00,								// register address and write flag (filled in)
	0x00, 0x00, 0x00, 0x00,				// value to write (if writing), or 1 byte of CRC if read request (filled in)
	0x00,								// CRC of write request (filled in)
	0x05, 0x00,							// sync byte and slave address
	REGNUM_IFCOUNT,						// register we want to read
	ReadIfcountCRC						// CRC
};

// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
volatile uint8_t TmcDriverState::receiveData[20];

const uint8_t TmcDriverState::WriteRegNumbers[NumWriteRegisters] =
{
	REGNUM_GCONF,
	REGNUM_SLAVECONF,
	REGNUM_CHOPCONF,
	REGNUM_IHOLDIRUN,
	REGNUM_PWMCONF,
	REGNUM_TPWMTHRS
};

const uint8_t TmcDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS,
	REGNUM_MSCNT,
	REGNUM_PWM_SCALE
};

const uint8_t TmcDriverState::ReadRegCRCs[NumReadRegisters] =
{
	CRCAddByte(InitialSendCRC, ReadRegNumbers[0]),
	CRCAddByte(InitialSendCRC, ReadRegNumbers[1]),
	CRCAddByte(InitialSendCRC, ReadRegNumbers[2]),
	CRCAddByte(InitialSendCRC, ReadRegNumbers[3])
};

// State structures for all drivers
static TmcDriverState driverStates[MaxSmartDrivers];

// Set up the PDC or DMAC to send a register
inline void TmcDriverState::SetupDMASend(uint8_t regNum, uint32_t regVal, uint8_t crc)
{
#if defined(SAME51) || defined(SAMC21)
	DmacManager::DisableChannel(TmcTxDmaChannel);
	DmacManager::DisableChannel(TmcRxDmaChannel);
#elif defined(SAM4E) || defined(SAM4S)
	Pdc * const pdc = uart_get_pdc_base(uart);
	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
#else
# error Unsupported processor
#endif

	sendData[2] = regNum | 0x80;
	sendData[3] = (uint8_t)(regVal >> 24);
	sendData[4] = (uint8_t)(regVal >> 16);
	sendData[5] = (uint8_t)(regVal >> 8);
	sendData[6] = (uint8_t)regVal;
	sendData[7] = crc;

#if defined(SAME51) || defined(SAMC21)
	DmacManager::SetDestinationAddress(TmcTxDmaChannel, &(sercom->USART.DATA));
	DmacManager::SetSourceAddress(TmcTxDmaChannel, sendData);
	DmacManager::SetTriggerSourceSercomTx(TmcTxDmaChannel, sercomNumber);
	DmacManager::SetDataLength(TmcTxDmaChannel, 12);
	DmacManager::SetBtctrl(TmcTxDmaChannel, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BLOCKACT_NOACT);

	DmacManager::SetDestinationAddress(TmcRxDmaChannel, receiveData);
	DmacManager::SetSourceAddress(TmcRxDmaChannel, &(sercom->USART.DATA));
	DmacManager::SetTriggerSourceSercomRx(TmcRxDmaChannel, sercomNumber);
	DmacManager::SetDataLength(TmcRxDmaChannel, 20);
	DmacManager::SetBtctrl(TmcRxDmaChannel, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BLOCKACT_INT);

	DmacManager::EnableChannel(TmcTxDmaChannel, TmcTxDmaPriority);
	DmacManager::EnableChannel(TmcRxDmaChannel, TmcRxDmaPriority);
#elif defined(SAM4E) || defined(SAM4S)
	pdc->PERIPH_TPR = reinterpret_cast<uint32_t>(sendData);
	pdc->PERIPH_TCR = 12;											// number of bytes to send: 8 bytes send request + 4 bytes read IFCOUNT request

	pdc->PERIPH_RPR = reinterpret_cast<uint32_t>(receiveData);
	pdc->PERIPH_RCR = 20;											// number of bytes to receive: the sent data + 8 bytes of received data

	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);		// enable the PDC to transmit and receive
#else
# error Unsupported processor
#endif
}

// Set up the PDC or DMAC to send a register and receive the status
inline void TmcDriverState::SetupDMAReceive(uint8_t regNum, uint8_t crc)
{
#if defined(SAME51) || defined(SAMC21)
	DmacManager::DisableChannel(TmcTxDmaChannel);
	DmacManager::DisableChannel(TmcRxDmaChannel);
#elif defined(SAM4E) || defined(SAM4S)
	Pdc * const pdc = uart_get_pdc_base(uart);
	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
#else
# error Unsupported processor
#endif

	sendData[2] = regNum;
	sendData[3] = crc;

#if defined(SAME51) || defined(SAMC21)
	DmacManager::SetDestinationAddress(TmcTxDmaChannel, &(sercom->USART.DATA));
	DmacManager::SetSourceAddress(TmcTxDmaChannel, sendData);
	DmacManager::SetTriggerSourceSercomTx(TmcTxDmaChannel, sercomNumber);
	DmacManager::SetDataLength(TmcTxDmaChannel, 4);
	DmacManager::SetBtctrl(TmcTxDmaChannel, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BLOCKACT_NOACT);

	DmacManager::SetDestinationAddress(TmcRxDmaChannel, receiveData);
	DmacManager::SetSourceAddress(TmcRxDmaChannel, &(sercom->USART.DATA));
	DmacManager::SetTriggerSourceSercomRx(TmcRxDmaChannel, sercomNumber);
	DmacManager::SetDataLength(TmcRxDmaChannel, 12);
	DmacManager::SetBtctrl(TmcRxDmaChannel, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BLOCKACT_INT);

	DmacManager::EnableChannel(TmcTxDmaChannel, TmcTxDmaPriority);
	DmacManager::EnableChannel(TmcRxDmaChannel, TmcRxDmaPriority);
#elif defined(SAM4E) || defined(SAM4S)
	pdc->PERIPH_TPR = reinterpret_cast<uint32_t>(sendData);
	pdc->PERIPH_TCR = 4;											// send a 4 byte read data request

	pdc->PERIPH_RPR = reinterpret_cast<uint32_t>(receiveData);
	pdc->PERIPH_RCR = 12;											// receive the 4 bytes we sent + 8 bytes of received data

	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);		// enable the PDC to transmit and receive
#else
# error Unsupported processor
#endif
}

// Update the maximum step pulse interval at wich we consider open load detection to be reliable
void TmcDriverState::UpdateMaxOpenLoadStepInterval()
{
	const uint32_t defaultMaxInterval = StepTimer::StepClockRate/MinimumOpenLoadFullStepsPerSec;
	if ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0)
	{
		maxOpenLoadStepInterval = defaultMaxInterval;
	}
	else
	{
		// In stealthchop mode open load detection in unreliable, so disable it below the speed at which we switch to spreadCycle
		const uint32_t tpwmthrs = writeRegisters[WriteTpwmthrs] & 0x000FFFFF;
		// tpwmthrs is the 20-bit interval between 1/256 microsteps threshold, in clock cycles @ 12MHz.
		// We need to convert it to the interval between full steps, measured in our step clocks, less about 20% to allow some margin.
		// So multiply by the step clock rate divided by 12MHz, also multiply by 256 less 20%.
		constexpr uint32_t conversionFactor = ((256 - 51) * (StepTimer::StepClockRate/1000000))/12;
		const uint32_t fullStepClocks = tpwmthrs * conversionFactor;
		maxOpenLoadStepInterval = min<uint32_t>(fullStepClocks, defaultMaxInterval);
	}
}

// Set a register value and flag it for updating
void TmcDriverState::UpdateRegister(size_t regIndex, uint32_t regVal)
{
	registersToUpdate &= ~(1u << regIndex);								// make sure it is not updated while we are changing it
	uint8_t crc = InitialSendCRC;
	crc = CRCAddByte(crc, WriteRegNumbers[regIndex] | 0x80);
	crc = CRCAddByte(crc, (uint8_t)(regVal >> 24));
	crc = CRCAddByte(crc, (uint8_t)(regVal >> 16));
	crc = CRCAddByte(crc, (uint8_t)(regVal >> 8));
	crc = CRCAddByte(crc, (uint8_t)regVal);
	const irqflags_t flags = cpu_irq_save();
	writeRegisters[regIndex] = regVal;
	writeRegCRCs[regIndex] = crc;
	registersToUpdate |= (1u << regIndex);								// flag it for sending
	cpu_irq_restore(flags);
	if (regIndex == WriteGConf || regIndex == WriteTpwmthrs)
	{
		UpdateMaxOpenLoadStepInterval();
	}
}

// Calculate the chopper control register and flag it for sending
void TmcDriverState::UpdateChopConfRegister()
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

// Initialise the state of the driver and its CS pin
#if TMC22xx_HAS_ENABLE_PINS
void TmcDriverState::Init(uint32_t p_driverNumber, Pin p_pin)
#else
void TmcDriverState::Init(uint32_t p_driverNumber)
#endif
pre(!driversPowered)
{
	driverNumber = p_driverNumber;
	axisNumber = p_driverNumber;										// assume straight-through axis mapping initially
#if TMC22xx_HAS_ENABLE_PINS
	enablePin = p_pin;													// this is NoPin for the built-in drivers

	if (p_pin != NoPin)
	{
		IoPort::SetPinMode(p_pin, OUTPUT_HIGH);
	}
#endif

#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
	sercom = TMC22xxSercoms[p_driverNumber];
	sercomNumber = TMC22xxSercomNumbers[p_driverNumber];
# else
	uart = TMC22xxUarts[p_driverNumber];
# endif
#endif

	enabled = false;
	registersToUpdate = 0;
	motorCurrent = 0;
	standstillCurrentFraction = (256 * 3)/4;							// default to 75%
	UpdateRegister(WriteGConf, DefaultGConfReg);
	UpdateRegister(WriteSlaveConf, DefaultSlaveConfReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		accumulatedReadRegisters[i] = readRegisters[i] = 0;
	}
	registerBeingUpdated = 0;
	registerToRead = 0;
	lastIfCount = 0;
	readErrors = writeErrors = numReads = numTimeouts = 0;
}

inline void TmcDriverState::SetAxisNumber(size_t p_axisNumber)
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void TmcDriverState::WriteAll()
{
	registersToUpdate = (1u << NumWriteRegisters) - 1;
}

float TmcDriverState::GetStandstillCurrentPercent() const
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void TmcDriverState::SetStandstillCurrentPercent(float percent)
{
	standstillCurrentFraction = (uint8_t)constrain<long>(lrintf((percent * 256)/100), 0, 255);
	UpdateCurrent();
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift).
bool TmcDriverState::SetMicrostepping(uint32_t shift, bool interpolate)
{
	microstepShiftFactor = shift;
	configuredChopConfReg = (configuredChopConfReg & ~(CHOPCONF_MRES_MASK | CHOPCONF_INTPOL)) | ((8 - shift) << CHOPCONF_MRES_SHIFT);
	if (interpolate)
	{
		configuredChopConfReg |= CHOPCONF_INTPOL;
	}
	UpdateChopConfRegister();
	return true;
}

// Get microstepping or chopper control register
unsigned int TmcDriverState::GetMicrostepping(bool& interpolation) const
{
	interpolation = (writeRegisters[WriteChopConf] & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

bool TmcDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal)
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return SetChopConf(regVal);

	case SmartDriverRegister::toff:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TOFF_MASK) | ((regVal << CHOPCONF_TOFF_SHIFT) & CHOPCONF_TOFF_MASK));

	case SmartDriverRegister::tblank:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TBL_MASK) | ((regVal << CHOPCONF_TBL_SHIFT) & CHOPCONF_TBL_MASK));

	case SmartDriverRegister::hstart:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HSTRT_MASK) | ((regVal << CHOPCONF_HSTRT_SHIFT) & CHOPCONF_HSTRT_MASK));

	case SmartDriverRegister::hend:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HEND_MASK) | ((regVal << CHOPCONF_HEND_SHIFT) & CHOPCONF_HEND_MASK));

	case SmartDriverRegister::tpwmthrs:
		UpdateRegister(WriteTpwmthrs, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::hdec:
	case SmartDriverRegister::coolStep:
	default:
		return false;
	}
}

uint32_t TmcDriverState::GetRegister(SmartDriverRegister reg) const
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return configuredChopConfReg & 0x01FFFF;

	case SmartDriverRegister::toff:
		return (configuredChopConfReg & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;

	case SmartDriverRegister::tblank:
		return (configuredChopConfReg & CHOPCONF_TBL_MASK) >> CHOPCONF_TBL_SHIFT;

	case SmartDriverRegister::hstart:
		return (configuredChopConfReg & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;

	case SmartDriverRegister::hend:
		return (configuredChopConfReg & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;

	case SmartDriverRegister::tpwmthrs:
		return writeRegisters[WriteTpwmthrs] & 0x000FFFFF;

	case SmartDriverRegister::mstepPos:
		return readRegisters[ReadMsCnt];

	case SmartDriverRegister::pwmScale:
		return readRegisters[ReadPwmScale];

	case SmartDriverRegister::hdec:
	case SmartDriverRegister::coolStep:
	default:
		return 0;
	}
}

// Set the chopper control register to the settings provided by the user. We allow only the lowest 17 bits to be set.
bool TmcDriverState::SetChopConf(uint32_t newVal)
{
	const uint32_t offTime = (newVal & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;
	if (offTime == 0 || (offTime == 1 && (newVal & CHOPCONF_TBL_MASK) < (2 << CHOPCONF_TBL_SHIFT)))
	{
		return false;
	}
	const uint32_t hstrt = (newVal & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;
	const uint32_t hend = (newVal & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;
	if (hstrt + hend > 16)
	{
		return false;
	}
	const uint32_t userMask = CHOPCONF_TBL_MASK | CHOPCONF_HSTRT_MASK | CHOPCONF_HEND_MASK | CHOPCONF_TOFF_MASK;	// mask of bits the user is allowed to change
	configuredChopConfReg = (configuredChopConfReg & ~userMask) | (newVal & userMask);
	UpdateChopConfRegister();
	return true;
}

// Set the driver mode
bool TmcDriverState::SetDriverMode(unsigned int mode)
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] | GCONF_SPREAD_CYCLE);
		return true;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_SPREAD_CYCLE);
		return true;

	default:
		return false;
	}
}

// Get the driver mode
DriverMode TmcDriverState::GetDriverMode() const
{
	return ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0) ? DriverMode::spreadCycle : DriverMode::stealthChop;
}

// Set the motor current
void TmcDriverState::SetCurrent(float current)
{
	motorCurrent = static_cast<uint32_t>(constrain<float>(current, 50.0, MaximumMotorCurrent));
	UpdateCurrent();
}

void TmcDriverState::UpdateCurrent()
{
	// The current sense resistor on the Duet M is 0.082 ohms, to which we must add 0.03 ohms internal resistance.
	// Full scale peak motor current in the high sensitivity range is give by I = 0.18/(R+0.03) = 0.18/0.105 ~= 1.6A
	// This gives us a range of 50mA to 1.6A in 50mA steps in the high sensitivity range (VSENSE = 1)
	const uint32_t iRunCsBits = (32 * motorCurrent - 800)/1615;		// formula checked by simulation on a spreadsheet
	const uint32_t iHoldCurrent = min<uint32_t>((motorCurrent * standstillCurrentFraction)/256, (uint32_t)MaximumStandstillCurrent);	// calculate standstill current
	const uint32_t iHoldCsBits = (32 * iHoldCurrent - 800)/1615;	// formula checked by simulation on a spreadsheet
	UpdateRegister(WriteIholdIrun,
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRunCsBits << IHOLDIRUN_IRUN_SHIFT) | (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
}

// Enable or disable the driver
void TmcDriverState::Enable(bool en)
{
	if (enabled != en)
	{
		enabled = en;
#if TMC22xx_HAS_ENABLE_PINS
		if (enablePin != NoPin)
		{
			digitalWrite(enablePin, !en);			// we assume that smart drivers always have active low enables
		}
#endif
		UpdateChopConfRegister();
	}
}

// Read the status
uint32_t TmcDriverState::ReadLiveStatus() const
{
	const uint32_t ret = readRegisters[ReadDrvStat] & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST | TMC_RR_TEMPBITS);
	return (enabled) ? ret : ret & ~(TMC_RR_OLA | TMC_RR_OLB);
}

// Read the status
uint32_t TmcDriverState::ReadAccumulatedStatus(uint32_t bitsToKeep)
{
	const uint32_t mask = (enabled) ? 0xFFFFFFFF : ~(TMC_RR_OLA | TMC_RR_OLB);
	bitsToKeep &= mask;
	const irqflags_t flags = cpu_irq_save();
	const uint32_t status = accumulatedReadRegisters[ReadDrvStat];
	accumulatedReadRegisters[ReadDrvStat] = (status & bitsToKeep) | readRegisters[ReadDrvStat];		// so that the next call to ReadAccumulatedStatus isn't missing some bits
	cpu_irq_restore(flags);
	return status & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST | TMC_RR_TEMPBITS) & mask;
}

// Append the driver status to a string, and reset the min/max load values
void TmcDriverState::AppendDriverStatus(const StringRef& reply)
{
	const uint32_t lastReadStatus = readRegisters[ReadDrvStat];
	if (lastReadStatus & TMC_RR_OT)
	{
		reply.cat(" temperature-shutdown!");
	}
	else if (lastReadStatus & TMC_RR_OTPW)
	{
		reply.cat(" temperature-warning");
	}
	if (lastReadStatus & TMC_RR_S2G)
	{
		reply.cat(" short-to-ground");
	}
	if (lastReadStatus & TMC_RR_OLA)
	{
		reply.cat(" open-load-A");
	}
	if (lastReadStatus & TMC_RR_OLB)
	{
		reply.cat(" open-load-B");
	}
	if (lastReadStatus & TMC_RR_STST)
	{
		reply.cat(" standstill");
	}
	else if ((lastReadStatus & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB)) == 0)
	{
		reply.cat(" ok");
	}

	reply.catf(", read errors %u, write errors %u, ifcount %u, reads %u, timeouts %u", readErrors, writeErrors, lastIfCount, numReads, numTimeouts);
	readErrors = writeErrors = numReads = numTimeouts = 0;
}

// This is called by the ISR when the SPI transfer has completed
inline void TmcDriverState::TransferDone()
{
	if (sendData[2] & 0x80)								// if we were writing a register
	{
		const uint8_t currentIfCount = receiveData[18];
		if (currentIfCount == (uint8_t)(lastIfCount + 1))
		{
			registersToUpdate &= ~registerBeingUpdated;
		}
		else
		{
			++writeErrors;
		}
		lastIfCount = currentIfCount;
	}
	else if (driversState != DriversState::noPower)		// we don't check the CRC, so only accept the result if power is still good
	{
		if (sendData[2] == ReadRegNumbers[registerToRead] && ReadRegNumbers[registerToRead] == receiveData[6] && receiveData[4] == 0x05 && receiveData[5] == 0xFF)
		{
			// We asked to read the scheduled read register, and the sync byte, slave address and register number in the received message match
			//TODO here we could check the CRC of the received message, but for now we assume that we won't get any corruption in the 32-bit received data
			uint32_t regVal = ((uint32_t)receiveData[7] << 24) | ((uint32_t)receiveData[8] << 16) | ((uint32_t)receiveData[9] << 8) | receiveData[10];
			if (registerToRead == ReadDrvStat)
			{
				uint32_t interval;
				if (   (regVal & TMC_RR_STST) != 0
#if defined(SAME51) || defined(SAMC21)
					|| (interval = moveInstance->GetStepInterval(axisNumber, microstepShiftFactor)) == 0				// get the full step interval
#else
					|| (interval = reprap.GetMove().GetStepInterval(axisNumber, microstepShiftFactor)) == 0		// get the full step interval
#endif
					|| interval > maxOpenLoadStepInterval
					|| motorCurrent < MinimumOpenLoadMotorCurrent
				   )
				{
					regVal &= ~(TMC_RR_OLA | TMC_RR_OLB);				// open load bits are unreliable at standstill and low speeds
				}
			}
			readRegisters[registerToRead] = regVal;
			accumulatedReadRegisters[registerToRead] |= regVal;

			++registerToRead;
			if (registerToRead == NumReadRegisters)
			{
				registerToRead = 0;
			}
			++numReads;
		}
		else
		{
			++readErrors;
		}
	}
}

// This is called to abandon the current transfer, if any
void TmcDriverState::AbortTransfer()
{
#if TMC22xx_USES_SERCOM
	DmacManager::DisableChannel(TmcTxDmaChannel);
	DmacManager::DisableChannel(TmcRxDmaChannel);
	sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
	while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
	uart->UART_IDR = UART_IDR_ENDRX;				// disable end-of-receive interrupt
	uart_get_pdc_base(uart)->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
	uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;
#endif
}

#if TMC22xx_HAS_MUX

// Set up the UART multiplexer to address the selected driver
inline void TmcDriverState::SetUartMux()
{
	if ((driverNumber & 0x01) != 0)
	{
		fastDigitalWriteHigh(TMC22xxMuxPins[0]);
	}
	else
	{
		fastDigitalWriteLow(TMC22xxMuxPins[0]);
	}
	if ((driverNumber & 0x02) != 0)
	{
		fastDigitalWriteHigh(TMC22xxMuxPins[1]);
	}
	else
	{
		fastDigitalWriteLow(TMC22xxMuxPins[1]);
	}
	if ((driverNumber & 0x04) != 0)
	{
		fastDigitalWriteHigh(TMC22xxMuxPins[2]);
	}
	else
	{
		fastDigitalWriteLow(TMC22xxMuxPins[2]);
	}
}

#endif

// This is called from the ISR or elsewhere to start a new SPI transfer. Inlined for ISR speed.
inline void TmcDriverState::StartTransfer()
{
#if TMC22xx_HAS_MUX
	SetUartMux();
#endif

	// Find which register to send. The common case is when no registers need to be updated.
	if (registersToUpdate == 0)
	{
		registerBeingUpdated = 0;

		// Read a register
		const irqflags_t flags = cpu_irq_save();		// avoid race condition
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;	// reset transmitter and receiver
#endif
		SetupDMAReceive(ReadRegNumbers[registerToRead], ReadRegCRCs[registerToRead]);	// set up the PDC
#if TMC22xx_USES_SERCOM
		DmacManager::EnableCompletedInterrupt(TmcRxDmaChannel);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;				// enable end-of-receive interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;	// enable transmitter and receiver
#endif
		cpu_irq_restore(flags);
	}
	else
	{
		// Pick a register to write
		const size_t regNum = LowestSetBit(registersToUpdate);

		// Kick off a transfer for that register
		const irqflags_t flags = cpu_irq_save();		// avoid race condition
		registerBeingUpdated = 1u << regNum;
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;	// reset transmitter and receiver
#endif
		SetupDMASend(WriteRegNumbers[regNum], writeRegisters[regNum], writeRegCRCs[regNum]);	// set up the PDC
#if TMC22xx_USES_SERCOM
		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(TmcRxDmaChannel);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;				// enable end-of-transfer interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;	// enable transmitter and receiver
#endif
		cpu_irq_restore(flags);
	}
}

// ISR(s) for the UART(s)

inline void TmcDriverState::UartTmcHandler()
{
#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
	DmacManager::DisableCompletedInterrupt(TmcRxDmaChannel);
# else
	uart->UART_IDR = UART_IDR_ENDRX;					// disable the PDC interrupt
# endif
#endif
	TransferDone();										// tidy up after the transfer we just completed
}

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER

# if TMC22xx_USES_SERCOM

// DMA complete callback
void TransferCompleteCallback(CallbackParameter, DmaCallbackReason reason)
{
	dmaFinishedReason = reason;
	tmcTask.GiveFromISR();
}

# else

#  ifndef TMC22xx_UART_Handler
#  error TMC handler name not defined
#  endif

// ISR for the single UART
extern "C" void TMC22xx_UART_Handler() __attribute__ ((hot));

void TMC22xx_UART_Handler()
{
	UART_TMC22xx->UART_IDR = UART_IDR_ENDRX;				// disable the interrupt
	TmcDriverState * const driver = TmcDriverState::currentDriver;	// capture volatile variable
	if (driver != nullptr)
	{
		driver->UartTmcHandler();
	}
}

# endif

#else

// ISRs for the individual UARTs
extern "C" void UART_TMC_DRV0_Handler() __attribute__ ((hot));
void UART_TMC_DRV0_Handler()
{
	driverStates[0].UartTmcHandler();
}

extern "C" void UART_TMC_DRV1_Handler() __attribute__ ((hot));
void UART_TMC_DRV1_Handler()
{
	driverStates[1].UartTmcHandler();
}

#endif

extern "C" [[noreturn]] void TmcLoop(void *)
{
	TmcDriverState * currentDriver = nullptr;
	for (;;)
	{
		if (driversState == DriversState::noPower)
		{
			currentDriver = 0;
			TaskBase::Take();
		}
		else
		{
			if (driversState == DriversState::notInitialised)
			{
				for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
				{
					driverStates[drive].WriteAll();
				}
				driversState = DriversState::initialising;
			}

			// Do a transaction
#if TMC22xx_SINGLE_DRIVER
			currentDriver = driverStates;
#else
			currentDriver = (currentDriver == nullptr || currentDriver + 1 == driverStates + GetNumTmcDrivers())
								? driverStates
									: currentDriver + 1;
#endif
			currentDriver->StartTransfer();

			// Wait for the end-of-transfer interrupt
			const bool timedOut = TaskBase::Take(TransferTimeout);
			DmacManager::DisableCompletedInterrupt(TmcRxDmaChannel);

			if (!timedOut && dmaFinishedReason == DmaCallbackReason::complete)
			{
				currentDriver->UartTmcHandler();

				if (driversState == DriversState::initialising)
				{
					// If all drivers that share the global enable have been initialised, set the global enable
					bool allInitialised = true;
					for (size_t i = 0; i < GetNumTmcDrivers(); ++i)
					{
#if TMC22xx_HAS_ENABLE_PINS
						if (driverStates[i].UsesGlobalEnable() && driverStates[i].UpdatePending())
#else
						if (driverStates[i].UpdatePending())
#endif
						{
							allInitialised = false;
							break;
						}
					}

					if (allInitialised)
					{
						fastDigitalWriteLow(GlobalTmc22xxEnablePin);
						driversState = DriversState::ready;
					}
				}
			}
			else
			{
				currentDriver->TransferTimedOut();
				currentDriver->AbortTransfer();
				currentDriver = nullptr;
			}
		}
	}
}

//--------------------------- Public interface ---------------------------------

// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
#if TMC22xx_HAS_ENABLE_PINS
void SmartDrivers::Init(const Pin driverSelectPins[MaxSmartDrivers], size_t numTmcDrivers)
#elif TMC22xx_VARIABLE_NUM_DRIVERS
void SmartDrivers::Init(size_t numTmcDrivers)
#else
void SmartDrivers::Init()
#endif
{
#if TMC22xx_VARIABLE_NUM_DRIVERS
	numTmc22xxDrivers = min<size_t>(numTmcDrivers, MaxSmartDrivers);
#endif
	// Make sure the ENN pins are high
	IoPort::SetPinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
# if TMC22xx_USES_SERCOM
	// Set up the single UART that communicates with all TMC22xx drivers
	gpio_set_pin_function(TMC22xxSercomTxPin, TMC22xxSercomTxPinPeriphMode);
	gpio_set_pin_function(TMC22xxSercomRxPin, TMC22xxSercomRxPinPeriphMode);

	Serial::InitUart(TMC22xxSercomNumber, DriversBaudRate, TMC22xxSercomRxPad);
	DmacManager::SetInterruptCallback(TmcRxDmaChannel, TransferCompleteCallback, CallbackParameter(0));
# else
	// Set up the single UART that communicates with all TMC22xx drivers
	ConfigurePin(TMC22xx_UART_PINS);									// the pins are already set up for UART use in the pins table

	// Enable the clock to the UART
	pmc_enable_periph_clk(ID_TMC22xx_UART);

	// Set the UART baud rate, 8 bits, 2 stop bits, no parity
	UART_TMC22xx->UART_IDR = ~0u;
	UART_TMC22xx->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
	UART_TMC22xx->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;
	UART_TMC22xx->UART_BRGR = SystemPeripheralClock()/(16 * DriversBaudRate);		// set baud rate
	UART_TMC22xx->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	NVIC_EnableIRQ(TMC22xx_UART_IRQn);
# endif
#endif

#if TMC22xx_HAS_MUX
	// Set up the multiplexer control pins as outputs
	pinMode(TMC22xxMuxPins[0], OUTPUT_LOW);
	pinMode(TMC22xxMuxPins[1], OUTPUT_LOW);
	pinMode(TMC22xxMuxPins[2], OUTPUT_LOW);
#endif

	driversState = DriversState::noPower;
	for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
	{
#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
		// Initialise the SERCOM that controls this driver
		gpio_set_pin_function(TMC22xxSercomTxPins[drive], TMC22xxSercomTxPinPeriphModes[drive]);
		gpio_set_pin_function(TMC22xxSercomRxPins[drive], TMC22xxSercomRxPinPeriphModes[drive]);

		Serial::InitUart(TMC22xxUarts[drive], TMC22xxSercomNumbers[drive], DriversBaudRate);
		NVIC_EnableIRQ(TMC22xxSercomIRQns[drive]);
# else
		// Initialise the UART that controls this driver
		// The pins are already set up for UART use in the pins table
		ConfigurePin(TMC22xxUartPins[drive]);

		// Enable the clock to the UART
		pmc_enable_periph_clk(TMC22xxUartIds[drive]);

		// Set the UART baud rate, 8 bits, 2 stop bits, no parity
		Uart * const uart = TMC22xxUarts[drive];
		uart->UART_IDR = ~0u;
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
		uart->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;
		uart->UART_BRGR = VARIANT_MCK/(16 * DriversBaudRate);		// set baud rate
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

		NVIC_EnableIRQ(TMC22xxUartIRQns[drive]);
# endif
#endif

#if TMC22xx_HAS_ENABLE_PINS
		driverStates[drive].Init(drive, driverSelectPins[drive]);	// axes are mapped straight through to drivers initially
#else
		driverStates[drive].Init(drive);							// axes are mapped straight through to drivers initially
#endif
	}

	tmcTask.Create(TmcLoop, "TMC", nullptr, TaskPriority::TmcPriority);
}

void SmartDrivers::SetAxisNumber(size_t drive, uint32_t axisNumber)
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetAxisNumber(axisNumber);
	}
}

uint32_t SmartDrivers::GetAxisNumber(size_t drive)
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetAxisNumber() : 0;
}

void SmartDrivers::SetCurrent(size_t drive, float current)
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetCurrent(current);
	}
}

void SmartDrivers::EnableDrive(size_t drive, bool en)
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].Enable(en);
	}
}

uint32_t SmartDrivers::GetLiveStatus(size_t drive)
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].ReadLiveStatus() : 0;
}

uint32_t SmartDrivers::GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep)
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].ReadAccumulatedStatus(bitsToKeep) : 0;
}

// Set microstepping or chopper control register
bool SmartDrivers::SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolate)
{
	if (drive < GetNumTmcDrivers() && microsteps > 0)
	{
		// Set the microstepping. We need to determine how many bits right to shift the desired microstepping to reach 1.
		unsigned int shift = 0;
		unsigned int uSteps = (unsigned int)microsteps;
		while ((uSteps & 1) == 0)
		{
			uSteps >>= 1;
			++shift;
		}
		if (uSteps == 1 && shift <= 8)
		{
			driverStates[drive].SetMicrostepping(shift, interpolate);
			return true;
		}
	}
	return false;
}

// Get microstepping or chopper control register
unsigned int SmartDrivers::GetMicrostepping(size_t drive, bool& interpolation)
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetMicrostepping(interpolation) : 1;
}

bool SmartDrivers::SetDriverMode(size_t driver, unsigned int mode)
{
	return driver < GetNumTmcDrivers() && driverStates[driver].SetDriverMode(mode);
}

DriverMode SmartDrivers::GetDriverMode(size_t driver)
{
	return (driver < GetNumTmcDrivers()) ? driverStates[driver].GetDriverMode() : DriverMode::unknown;
}

// Flag that the the drivers have been powered up or down
// Before the first call to this function with 'powered' true, you must call Init()
void SmartDrivers::Spin(bool powered)
{
	TaskCriticalSectionLocker lock;

	if (powered)
	{
		if (driversState == DriversState::noPower)
		{
			driversState = DriversState::notInitialised;
			tmcTask.Give();									// wake up the TMC task because the drivers need to be initialised
		}
	}
	else
	{
		driversState = DriversState::noPower;				// flag that there is no power to the drivers
		fastDigitalWriteHigh(GlobalTmc22xxEnablePin);		// disable the drivers
	}
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void SmartDrivers::TurnDriversOff()
{
	// When using TMC2660 drivers, this is called when an over-voltage event occurs, so that we can try to protect the drivers by disabling them.
	// We don't use it with TMC22xx drivers.
}

void SmartDrivers::AppendDriverStatus(size_t drive, const StringRef& reply)
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].AppendDriverStatus(reply);
	}
}

float SmartDrivers::GetStandstillCurrentPercent(size_t drive)
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetStandstillCurrentPercent() : 0.0;
}

void SmartDrivers::SetStandstillCurrentPercent(size_t drive, float percent)
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetStandstillCurrentPercent(percent);
	}
}

bool SmartDrivers::SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal)
{
	return (driver < GetNumTmcDrivers()) && driverStates[driver].SetRegister(reg, regVal);
}

uint32_t SmartDrivers::GetRegister(size_t driver, SmartDriverRegister reg)
{
	return (driver < GetNumTmcDrivers()) ? driverStates[driver].GetRegister(reg) : 0;
}

#endif

// End
