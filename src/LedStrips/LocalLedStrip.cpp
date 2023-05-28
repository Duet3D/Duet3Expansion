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
	GCodeResult rslt;
	String<StringLength50> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(pinName.c_str(), reply, PinUsedBy::led, PinAccess::write0))
		{
			return GCodeResult::error;
		}

		// See if the maximum strip length was provided (the default value is set up by the constructor)
		(void)parser.GetUintParam('U', maxLeds);
		rslt = AllocateChunkBuffer(reply);
	}
	else
	{
		rslt = GCodeResult::ok;
	}

	extra = (useDma) ? 0x01 : 0;
	return rslt;
}

// Allocate the chunk buffer and set up the useDma flag
GCodeResult LocalLedStrip::AllocateChunkBuffer(const StringRef& reply) noexcept
{
	// Must set up useDma before calling GetBytesPerLed
#if SUPPORT_DMA_NEOPIXEL
	useDma = (port.GetCapability() & PinCapability::npDma) != PinCapability::none;
	if (useDma)
	{
		// Set up the SPI port. Currently at most one port per configuration is flagged as npDma capable.
# if defined(DUET3MINI)
		SetPinFunction(NeopixelOutPin, NeopixelOutPinFunction);
		hri_mclk_set_AHBMASK_QSPI_bit(MCLK);
		hri_mclk_clear_AHBMASK_QSPI_2X_bit(MCLK);			// we don't need the 2x clock
		hri_mclk_set_APBCMASK_QSPI_bit(MCLK);
# elif defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(PCCB_10)
		SetPinFunction(DotStarMosiPin, DotStarPinMode);
		SetPinFunction(DotStarSclkPin, DotStarPinMode);
		pmc_enable_periph_clk(DotStarClockId);				// enable the clock to the USART or SPI peripheral
# endif
		SetupSpi();
	}
#endif

	const size_t bytesPerLed = GetBytesPerLed();
	chunkBufferSize = maxLeds * bytesPerLed;
	chunkBuffer = new uint8_t[chunkBufferSize];
	return GCodeResult::ok;
}

// Report details that are common to all local LED strips i.e. port name, frequency, and whether DMA is used
GCodeResult LocalLedStrip::CommonReportDetails(const StringRef &reply) noexcept
{
	reply.printf("%s strip on port \"", GetTypeText());
	port.AppendPinName(reply);
	reply.catf("\" uses %s, frequency %" PRIu32 "Hz", (useDma) ? "DMA" : "bit-banging", frequency);
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
# if LEDSTRIP_USES_USART
	DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_TXDIS;			// reset transmitter and receiver, disable transmitter
	Pdc * const usartPdc = usart_get_pdc_base(DotStarUsart);
	usartPdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;		// disable the PDC
	usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(chunkBuffer);
	usartPdc->PERIPH_TCR = numBytes;										// number of bytes to transfer
	usartPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;								// enable the PDC to send data
	DotStarUsart->US_CR = US_CR_TXEN;										// enable transmitter
# elif SAME5x
	DmacManager::DisableChannel(DmacChanDotStarTx);
	DmacManager::SetTriggerSource(DmacChanDotStarTx, DmaTrigSource::qspi_tx);
#  if USE_16BIT_SPI
	DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X2 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
#  else
	DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
#  endif
	DmacManager::SetSourceAddress(DmacChanDotStarTx, chunkBuffer);
	DmacManager::SetDestinationAddress(DmacChanDotStarTx, &QSPI->TXDATA.reg);
	DmacManager::SetDataLength(DmacChanDotStarTx, numBytes);				// must do this last!
	DmacManager::EnableChannel(DmacChanDotStarTx, DmacPrioDotStar);
# elif SAME70
	xdmac_channel_disable(XDMAC, DmacChanDotStarTx);
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
	xdmac_configure_transfer(XDMAC, DmacChanDotStarTx, &p_cfg);
	xdmac_channel_enable(XDMAC, DmacChanDotStarTx);
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
# if LEDSTRIP_USES_USART
		if ((DotStarUsart->US_CSR & US_CSR_ENDTX) != 0)							// if we are no longer sending
# elif SAME5x
		if ((DmacManager::GetAndClearChannelStatus(DmacChanDotStarTx) & DMAC_CHINTFLAG_TCMPL) != 0)
# elif SAME70
		if ((xdmac_channel_get_interrupt_status(XDMAC, DmacChanDotStarTx) & XDMAC_CIS_BIS) != 0)	// if the last transfer has finished
# endif
		{
			dmaBusy = false;													// we finished the last transfer
			whenTransferFinished = StepTimer::GetTimerTicks();
		}
	}
	return dmaBusy;
}

// Setup the SPI peripheral. Only call this when the busy flag is not set.
void LocalLedStrip::SetupSpi() noexcept
{
# if LEDSTRIP_USES_USART
	// Set the USART in SPI mode, with the clock high when inactive, data changing on the falling edge of the clock
	DotStarUsart->US_IDR = ~0u;
	DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
	DotStarUsart->US_MR = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL
					| US_MR_CPOL
					| US_MR_CLKO;
	DotStarUsart->US_BRGR = SystemPeripheralClock()/frequency;		// set SPI clock frequency
	DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
# elif SAME5x
	// DotStar on Duet 3 Mini uses the QSPI peripheral
	QSPI->CTRLA.reg = QSPI_CTRLA_SWRST;										// software reset
#  if USE_16BIT_SPI
	QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_16BITS;							// SPI mode, 8 bits per transfer
#  else
	QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_8BITS;								// SPI mode, 8 bits per transfer
#  endif
	QSPI->BAUD.reg = QSPI_BAUD_CPOL | QSPI_BAUD_CPHA | QSPI_BAUD_BAUD(SystemCoreClockFreq/frequency - 1);
	QSPI->CTRLA.reg = QSPI_CTRLA_ENABLE;
# elif SAME70
	// DotStar on Duet 3 uses the QSPI peripheral
	QSPI->QSPI_CR = QSPI_CR_SWRST;
#  if USE_16BIT_SPI
	QSPI->QSPI_MR = QSPI_MR_NBBITS_16_BIT;									// SPI mode, 16 bits per transfer
#  else
	QSPI->QSPI_MR = QSPI_MR_NBBITS_8_BIT;									// SPI mode, 8 bits per transfer
#  endif
	QSPI->QSPI_SCR = QSPI_SCR_CPOL | QSPI_SCR_CPHA | QSPI_SCR_SCBR(SystemPeripheralClock()/frequency - 1);
	QSPI->QSPI_CR = QSPI_CR_QSPIEN;
	if (IsNeoPixel())														// if it's a Neopixel strip
	{
		QSPI->QSPI_TDR = 0;													// send a word of zeros to set the data line low
	}
# endif
}

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

#endif	// SUPPORT_LED_STRIPS

// End
