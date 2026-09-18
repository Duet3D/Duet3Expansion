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

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_PIO_NEOPIXEL
bool LocalLedStrip::dmaBusy = false;
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
# if SAME5x && NEOPIXEL_USES_QSPI
		useDma = port.GetPin() == NeopixelOutPin;
# elif SAME5x || SAMC21
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

	uint32_t order;
	if (parser.GetUintParam('K', order))
	{
		if (order >= (uint32_t)ColorOrder::count)
		{
			reply.printf("Invalid color order K=%lu", order);
			return GCodeResult::warning;
		}
		colorOrder = (ColorOrder)order;
		seen = true;
	}

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

// On the SAME51G19A the SERCOM data output pin idles high, both while the SERCOM is enabled but not transmitting and again at the end of each
// transmission, regardless of the last bit sent. A NeoPixel strip that is out of reset counts any such pulse as a bit, which shifts the whole frame.
// Padding the data with zeros does not help because a few microseconds of low is far short of the reset time.
// So the pin is left under PORT control driving low, and is handed to the SERCOM only while the SERCOM is actively shifting out a zero byte,
// see DmaSendChunkBuffer and ReleaseDataPin.
//
// Set up the SPI port
void LocalLedStrip::SetupSpi() noexcept
{
# if SAME5x && NEOPIXEL_USES_QSPI
	hri_mclk_set_AHBMASK_QSPI_bit(MCLK);
	hri_mclk_clear_AHBMASK_QSPI_2X_bit(MCLK);			// we don't need the 2x clock
	hri_mclk_set_APBCMASK_QSPI_bit(MCLK);

	QSPI->CTRLA.reg = QSPI_CTRLA_SWRST;										// software reset
#  if USE_16BIT_SPI
	QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_16BITS;							// SPI mode, 16 bits per transfer
#  else
	QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_8BITS;								// SPI mode, 8 bits per transfer
#  endif
	QSPI->BAUD.reg = QSPI_BAUD_CPOL | QSPI_BAUD_CPHA | QSPI_BAUD_BAUD(SystemCoreClockFreq/frequency - 1);
	QSPI->CTRLA.reg = QSPI_CTRLA_ENABLE;
	SetPinFunction(NeopixelOutPin, NeopixelOutPinFunction);

# elif SAME5x || SAMC21
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
	firstColour = secondColour = thirdColour = white = 0;
	brightness = 128;
	numLeds = 1;
	following = false;

	(void)parser.GetUintParam('R', firstColour);
	(void)parser.GetUintParam('U', secondColour);
	(void)parser.GetUintParam('B', thirdColour);
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

// Put the colours in the right order. On entry, firstColour is the red amount, secondColour is the green amount, thirdColor is the blue amount.
void LocalLedStrip::LedParams::SwapColours(ColorOrder order) noexcept
{
	switch (order)
	{
	case ColorOrder::BGR:
		std::swap(firstColour, thirdColour);
		break;

	case ColorOrder::BRG:
		{
			const uint32_t red = firstColour;
			firstColour = thirdColour;
			thirdColour = secondColour;
			secondColour = red;
		}
		break;

	case ColorOrder::RGB:
	default:
		break;																// already in the right order

	case ColorOrder::RBG:
		std::swap(secondColour, thirdColour);
		break;

	case ColorOrder::GBR:
		{
			const uint32_t red = firstColour;
			firstColour = secondColour;
			secondColour = thirdColour;
			thirdColour = red;
		}
		break;

	case ColorOrder::GRB:
		std::swap(firstColour, secondColour);
		break;
	}
}

// Apply the brightness value to the red/green/blue/white values. This is needed for Neopixel strips, which don't have a 'brightness' value sent to them.
void LocalLedStrip::LedParams::ApplyBrightness() noexcept
{
	firstColour = ((firstColour * brightness) + 255) >> 8;
	secondColour = ((secondColour * brightness) + 255) >> 8;
	thirdColour = ((thirdColour * brightness) + 255) >> 8;
	white = ((white * brightness) + 255) >> 8;
	brightness = 255;														// in case we call this again
}

#if SUPPORT_DMA_NEOPIXEL

// DMA the data. Must be a multiple of 2 bytes if USE_16BIT_SPI is true.
void LocalLedStrip::DmaSendChunkBuffer(size_t numBytes) noexcept
{
# if SAME5x && NEOPIXEL_USES_QSPI
	DmacManager::DisableChannel(DmacChanLedTx);
	DmacManager::SetTriggerSource(DmacChanLedTx, DmaTrigSource::qspi_tx);
	DmacManager::SetBtctrl(DmacChanLedTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetSourceAddress(DmacChanLedTx, chunkBuffer);
	DmacManager::SetDestinationAddress(DmacChanLedTx, &QSPI->TXDATA.reg);
	DmacManager::SetDataLength(DmacChanLedTx, numBytes);					// must do this last!
	DmacManager::EnableChannel(DmacChanLedTx, DmacPrioLed);
# elif SAME5x || SAMC21
	Sercom *const hardware = Serial::GetSercom(GetDeviceNumber(sercom));
	DmacManager::DisableChannel(DmacChanLedTx);
	DmacManager::SetTriggerSourceSercomTx(DmacChanLedTx, GetDeviceNumber(sercom));
	DmacManager::SetSourceAddress(DmacChanLedTx, chunkBuffer);
	DmacManager::SetDestinationAddress(DmacChanLedTx, &hardware->SPI.DATA.reg);
	DmacManager::SetBtctrl(DmacChanLedTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanLedTx, numBytes);					// must do this last!
	DmacManager::SetInterruptCallback(DmacChanLedTx, DmaCompleteCallback, CallbackParameter(this));	// all strips share the channel, so claim it for this one
	DmacManager::EnableCompletedInterrupt(DmacChanLedTx);

	// Prime the SERCOM with two zero bytes, which gives us two byte times during which the data output is driven low.
	// The DMA is armed within that window and the pin is handed over at the end of it, so the strip never sees the idle-high level
	{
		IrqDisable();
		hri_sercomspi_clear_INTFLAG_TXC_bit(hardware);
		hardware->SPI.DATA.reg = 0;
		const uint32_t start = GetCurrentCycles();
		while (!hri_sercomspi_get_INTFLAG_DRE_bit(hardware) && GetElapsedCycles(start) < NanosecondsToCycles(4000)) { }
		hardware->SPI.DATA.reg = 0;
		DmacManager::EnableChannel(DmacChanLedTx, DmacPrioLed);
		delayNanoseconds(1000);												// let the first zero bit reach the pin before we hand it over
		SetPinFunction(port.GetPin(), GetPeriNumber(sercom));
		IrqEnable();
	}

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

#if (SAME5x || SAMC21) && !NEOPIXEL_USES_QSPI

// Wait for the final byte to be shifted out, then give the data line back to the PORT so that it returns to driving low.
// This must happen at transmit-complete, because the SERCOM raises the output as soon as it has nothing left to send
void LocalLedStrip::ReleaseDataPin() noexcept
{
	Sercom *const hardware = Serial::GetSercom(GetDeviceNumber(sercom));
	const uint32_t start = GetCurrentCycles();
	while (!hri_sercomspi_get_INTFLAG_TXC_bit(hardware) && GetElapsedCycles(start) < NanosecondsToCycles(50000)) { }
	ClearPinFunction(port.GetPin());
	whenTransferFinished = StepTimer::GetTimerTicks();
	dmaBusy = false;
}

/*static*/ void LocalLedStrip::DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept
{
	static_cast<LocalLedStrip*>(cp.vp)->ReleaseDataPin();
}

#endif

// Return true if DMA to the LEDs is in progress
bool LocalLedStrip::DmaInProgress() noexcept
{
# if (SAME5x || SAMC21) && !NEOPIXEL_USES_QSPI
	return dmaBusy;																// cleared by the DMA completion interrupt, which also releases the data pin
# else
	if (dmaBusy)																// if we sent something
	{
#  if SAME5x
		if ((DmacManager::GetAndClearChannelStatus(DmacChanLedTx) & DMAC_CHINTFLAG_TCMPL) != 0)
#  elif SAME70
		if ((xdmac_channel_get_interrupt_status(XDMAC, DmacChanLedTx) & XDMAC_CIS_BIS) != 0)	// if the last transfer has finished
#  endif
		{
			dmaBusy = false;													// we finished the last transfer
			whenTransferFinished = StepTimer::GetTimerTicks();
		}
	}
	return dmaBusy;
# endif
}

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

#endif	// SUPPORT_LED_STRIPS

// End
