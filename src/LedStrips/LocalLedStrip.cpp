/*
 * LocalLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/LocalLedStrip.h>

#if SUPPORT_LED_STRIPS

#include <Movement/StepTimer.h>
#include <CanMessageGenericParser.h>

#if SUPPORT_DMA_NEOPIXEL
# include <Serial.h>
# include <DmacManager.h>
# if SAME5x
#  include <hri_sercom_e54.h>
#  include <hri_mclk_e54.h>
#  define USE_16BIT_SPI	0		// set to use 16-bit SPI transfers instead of 8-bit
# elif SAMC21
#  include <hri_sercom_c21.h>
#  define USE_16BIT_SPI	0		// set to use 16-bit SPI transfers instead of 8-bit
# elif SAME70
#  include <xdmac/xdmac.h>
#  include <pmc/pmc.h>
#  define USE_16BIT_SPI	0		// set to use 16-bit SPI transfers instead of 8-bit
# endif
#endif

LocalLedStrip::LocalLedStrip(LedStripType p_type, uint32_t p_freq) noexcept
	: LedStripBase(p_type), frequency(p_freq)
{
}

LocalLedStrip::~LocalLedStrip()
{
	delete chunkBuffer;
}

// Configure or reconfigure the LED strip. Bit 0 of 'extra' must be set on return iff interrupts do not need to be disabled for a long time when setting the strip colours.
GCodeResult LocalLedStrip::CommonConfigure(CanMessageGenericParser& parser, const StringRef& reply, bool& seen, uint8_t& extra) noexcept
{
	// See if the frequency was provided
	if (parser.GetUintParam('Q', frequency))
	{
		seen = true;
	}

	// Deal with the pin name
	String<StringLength50> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(pinName.c_str(), reply, PinUsedBy::led, PinAccess::write0))
		{
			return GCodeResult::error;
		}

#if SUPPORT_DMA_NEOPIXEL
# if SAME5x || SAMC21
		sercom = PinTable[port.GetPin()].sercomOut;
		useDma = (sercom != SercomIo::none);
# else
#  error Code not written for this processor
		//useDma = (port.GetCapability() & PinCapability::npDma) != PinCapability::none;
# endif
#endif
	}

	// See if the maximum strip length was provided (the default value is set up by the constructor)
	if (parser.GetUintParam('U', maxLeds))
	{
		DeleteArray(chunkBuffer);
	}

	GCodeResult rslt = GCodeResult::ok;

#if !SAME70
	if (chunkBuffer == nullptr)
	{
		rslt = AllocateChunkBuffer(reply);
	}
#endif

#if SUPPORT_DMA_NEOPIXEL
	if (seen && useDma && rslt <= GCodeResult::warning)
	{
		SetupSpi();
	}
#endif

	extra = (useDma) ? 0x01 : 0;
	return rslt;
}

// Allocate the chunk buffer and set up the useDma flag
// Must set up the frequency and useDma flag first
GCodeResult LocalLedStrip::AllocateChunkBuffer(const StringRef& reply) noexcept
{
	const size_t bytesPerLed = GetBytesPerLed();
	chunkBufferSize = maxLeds * bytesPerLed;
	chunkBuffer = new uint8_t[chunkBufferSize];
	return GCodeResult::ok;
}

#if SUPPORT_DMA_NEOPIXEL

// This DOES NOT_WORK. The reason is that on (at least) Sercom1 on the SAME51G19A the data output pin goes high as soon as the SERCOM device
// has been enabled in SPI mode, and also goes high at the end of any data transmission regardless of the value of the last bit transmitted.
// Opened case 01429968 with Microchip to see if there is any way to control the idle state of the output pin.
//
// Set up the SPI port
void LocalLedStrip::SetupSpi() noexcept
{
# if SAME5x || SAMC21
	const uint8_t sercomNumber = GetDeviceNumber(sercom);
	Serial::EnableSercomClock(sercomNumber);

	Sercom *const hardware = Serial::GetSercom(sercomNumber);
	const uint32_t regCtrlAMode = SERCOM_SPI_CTRLA_MODE(3);
	const uint32_t regCtrlA = regCtrlAMode | SERCOM_SPI_CTRLA_DIPO(0) | SERCOM_SPI_CTRLA_DOPO(GetPadNumber(sercom)) | SERCOM_SPI_CTRLA_FORM(0);
	const uint32_t regCtrlB = 0;												// 8 bits, slave select disabled, receiver disabled for now
#  if SAME5x
	const uint32_t regCtrlC = 0;												// not 32-bit mode
#  endif

	if (!hri_sercomspi_is_syncing(hardware, SERCOM_SPI_SYNCBUSY_SWRST))
	{
		if (hri_sercomspi_get_CTRLA_reg(hardware, SERCOM_SPI_CTRLA_ENABLE))
		{
			hri_sercomspi_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_SYNCBUSY_ENABLE);
		}
		hri_sercomspi_write_CTRLA_reg(hardware, SERCOM_SPI_CTRLA_SWRST | regCtrlAMode);
	}
	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_SYNCBUSY_SWRST);

	hri_sercomspi_write_CTRLA_reg(hardware, regCtrlA);
	hri_sercomspi_write_CTRLB_reg(hardware, regCtrlB);
#  if SAME5x
	hri_sercomspi_write_CTRLC_reg(hardware, regCtrlC);
#  endif
	hri_sercomspi_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * frequency) - 1));
	hri_sercomspi_write_DBGCTRL_reg(hardware, SERCOM_SPI_DBGCTRL_DBGSTOP);		// baud rate generator is stopped when CPU halted by debugger

	// The problem we have is that when we enable the SERCOM the data out pin goes high until we send some data.
	// The following SetPinMode call was an unsuccessful attempt to change that.
	//IoPort::SetPinMode(port.GetPin(), PinMode::OUTPUT_LOW);
	SetPinFunction(port.GetPin(), GetPeriNumber(sercom));
	hri_sercomspi_write_CTRLA_reg(hardware, SERCOM_SPI_CTRLA_ENABLE | regCtrlA);
	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_SYNCBUSY_ENABLE);
# else
#  error Code not written for this processor
# endif
}

#endif

// Report details that are common to all local LED strips i.e. port name, frequency, and whether DMA is used
GCodeResult LocalLedStrip::CommonReportDetails(const StringRef &reply) noexcept
{
	reply.printf("%s strip on port \"", GetTypeText());
	port.AppendPinName(reply);
	reply.cat("\" uses ");
	if (useDma)
	{
		reply.catf("DMA, frequency %" PRIu32 "Hz", frequency);
	}
	else
	{
		reply.cat("bit-banging");
	}
	if (IsNeoPixel())
	{
		reply.catf(", max strip length %" PRIu32, maxLeds);
	}
	return GCodeResult::ok;
}

void LocalLedStrip::LedParams::GetM150Params(CanMessageGenericParser& parser) noexcept
{
	red = green = blue = white = 0;
	brightness = 128;
	numLeds = 1;
	following = false;

	(void)parser.GetUintParam('R', red);
	(void)parser.GetUintParam('U', green);
	(void)parser.GetUintParam('B', blue);
	(void)parser.GetUintParam('W', white);									// W value is used by RGBW NeoPixels only

	if (!parser.GetUintParam('P', brightness))								// P takes precedence over Y
	{
		if (parser.GetUintParam('Y',  brightness))
		{
			brightness = (brightness * 255)/31;								// valid Y values are 0-31
		}
	}

	(void)parser.GetUintParam('S', numLeds);
	(void)parser.GetBoolParam('F', following);
}

// Apply the brightness value to the red/green/blue/white values. This is needed for Neopixel strips, which don't have a 'brightness' value sent to them.
void LocalLedStrip::LedParams::ApplyBrightness() noexcept
{
	red = ((red * brightness) + 255) >> 8;
	green = ((green * brightness) + 255) >> 8;
	blue = ((blue * brightness) + 255) >> 8;
	white = ((white * brightness) + 255) >> 8;
	brightness = 255;														// in case we call this again
}

#if SUPPORT_DMA_NEOPIXEL

// DMA the data. Must be a multiple of 2 bytes if USE_16BIT_SPI is true.
void LocalLedStrip::DmaSendChunkBuffer(size_t numBytes) noexcept
{
# if SAME5x || SAMC21
//	Sercom *const hardware = Serial::GetSercom(GetDeviceNumber(sercom));
//	hardware->SPI.CTRLA.bit.ENABLE = 0;
//	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_CTRLA_ENABLE);

	DmacManager::DisableChannel(DmacChanLedTx);
	DmacManager::SetTriggerSourceSercomTx(DmacChanLedTx, GetDeviceNumber(sercom));
	DmacManager::SetSourceAddress(DmacChanLedTx, chunkBuffer);
	DmacManager::SetDestinationAddress(DmacChanLedTx, &Serial::GetSercom(GetDeviceNumber(sercom))->SPI.DATA.reg);
#  if SAME5x
	if (0)//((numBytes & 3) == 0)
	{
//		hardware->SPI.CTRLC.reg = SERCOM_SPI_CTRLC_DATA32B;	// 32-bit transfers
		DmacManager::SetBtctrl(DmacChanLedTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
		DmacManager::SetDataLength(DmacChanLedTx, numBytes >> 2);			// must do this last!
	}
	else
	{
//		hardware->SPI.CTRLC.reg = 0;							// 8-bit transfers
		DmacManager::SetBtctrl(DmacChanLedTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
		DmacManager::SetDataLength(DmacChanLedTx, numBytes);				// must do this last!
	}
#  else
	DmacManager::SetBtctrl(DmacChanLedTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanLedTx, numBytes);					// must do this last!
#  endif
	DmacManager::EnableChannel(DmacChanLedTx, DmacPrioLed);
//	hardware->SPI.CTRLA.bit.ENABLE = 1;
//	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_CTRLA_ENABLE);

# elif SAME70
	xdmac_channel_disable(XDMAC, DmacChanLedTx);
	xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
	p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
					| XDMAC_CC_MBSIZE_SINGLE
					| XDMAC_CC_DSYNC_MEM2PER
					| XDMAC_CC_CSIZE_CHK_1
#  if USE_16BIT_SPI
					| XDMAC_CC_DWIDTH_HALFWORD
#  else
					| XDMAC_CC_DWIDTH_BYTE
#  endif
					| XDMAC_CC_SIF_AHB_IF0
					| XDMAC_CC_DIF_AHB_IF1
					| XDMAC_CC_SAM_INCREMENTED_AM
					| XDMAC_CC_DAM_FIXED_AM
					| XDMAC_CC_PERID((uint32_t)DmaTrigSource::qspitx);
#  if USE_16BIT_SPI
	p_cfg.mbr_ubc = numBytes/2;
#  else
	p_cfg.mbr_ubc = numBytes;
#  endif
	p_cfg.mbr_sa = reinterpret_cast<uint32_t>(chunkBuffer);
	p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(QSPI->QSPI_TDR));
	xdmac_configure_transfer(XDMAC, DmacChanLedTx, &p_cfg);
	xdmac_channel_enable(XDMAC, DmacChanLedTx);
# else
#  error Unsupported processor
# endif
	dmaBusy = true;
}

// Return true if DMA to the LEDs is in progress
bool LocalLedStrip::DmaInProgress() noexcept
{
	if (dmaBusy)																// if we sent something
	{
# if SAME5x
		if ((DmacManager::GetAndClearChannelStatus(DmacChanLedTx) & DMAC_CHINTFLAG_TCMPL) != 0)
# elif SAME70
		if ((xdmac_channel_get_interrupt_status(XDMAC, DmacChanLedTx) & XDMAC_CIS_BIS) != 0)	// if the last transfer has finished
# endif
		{
			dmaBusy = false;													// we finished the last transfer
			whenTransferFinished = StepTimer::GetTimerTicks();
		}
	}
	return dmaBusy;
}

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

#endif	// SUPPORT_LED_STRIPS

// End
