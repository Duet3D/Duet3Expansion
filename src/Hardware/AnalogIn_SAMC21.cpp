/*
 * AnalogIn_SAMC21.cpp
 *
 *  Created on: 20 Aug 2019
 *      Author: David
 */

#include "Peripherals.h"

#ifdef SAMC21

#include "AnalogIn.h"
#include "RTOSIface/RTOSIface.h"
#include "Hardware/DmacManager.h"
#include "Hardware/IoPorts.h"

#define ADC_INPUTCTRL_MUXNEG_GND   (0x18 << ADC_INPUTCTRL_MUXNEG_Pos)			// this definition is missing from file adc.h for the SAMC21

static uint32_t conversionsStarted = 0;
static uint32_t conversionsCompleted = 0;

static AnalogInCallbackFunction tempCallbackFn = nullptr;
static CallbackParameter tempCallbackParam = 0;
static uint32_t tempTicksPerCall = 1;
static uint32_t tempTicksAtLastCall = 0;

class AdcClass
{
public:
	enum class State : uint8_t
	{
		noChannels = 0,
		starting,
		idle,
		converting,
		ready
	};

	AdcClass(Adc * const p_device, IRQn p_irqn, DmaChannel p_dmaChan, DmaTrigSource p_trigSrc);

	State GetState() const { return state; }
	bool EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall);
	bool IsChannelEnabled(unsigned int chan) const;
	bool StartConversion(TaskBase *p_taskToWake);
	uint16_t ReadChannel(unsigned int chan) const { return resultsByChannel[chan]; }

	void EnableTemperatureSensor();

	void ResultReadyCallback();
	void ExecuteCallbacks();

private:
	bool InternalEnableChannel(unsigned int chan, uint8_t refCtrl, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall);

	static void DmaCompleteCallback(CallbackParameter cp);

	static constexpr size_t NumAdcChannels = 32;		// number of channels per ADC including temperature sensor inputs etc.
	static constexpr size_t MaxSequenceLength = 16;		// the maximum length of the read sequence

	Adc * const device;
	const IRQn irqn;
	const DmaChannel dmaChan;
	const DmaTrigSource trigSrc;

	size_t numChannelsEnabled;
	uint32_t channelsEnabled;
	TaskBase *taskToWake;
	State state;
	bool justStarted;
	AnalogInCallbackFunction callbackFunctions[NumAdcChannels];
	CallbackParameter callbackParams[NumAdcChannels];
	uint32_t ticksPerCall[NumAdcChannels];
	uint32_t ticksAtLastCall[NumAdcChannels];
	volatile uint16_t results[MaxSequenceLength];
	volatile uint16_t resultsByChannel[NumAdcChannels];
};

AdcClass::AdcClass(Adc * const p_device, IRQn p_irqn, DmaChannel p_dmaChan, DmaTrigSource p_trigSrc)
	: device(p_device), irqn(p_irqn), dmaChan(p_dmaChan), trigSrc(p_trigSrc),
	  numChannelsEnabled(0), channelsEnabled(0), taskToWake(nullptr), state(State::noChannels)
{
	for (size_t i = 0; i < NumAdcChannels; ++i)
	{
		callbackFunctions[i] = nullptr;
		callbackParams[i].u32 = 0;
		resultsByChannel[i] = 0;
	}
}

// Try to enable this ADC on the specified pin returning true if successful
// Only single ended mode with gain x1 is supported
// There is no check to avoid adding the same channel twice. If you do that it will be converted twice.
bool AdcClass::EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall)
{
	if (numChannelsEnabled == MaxSequenceLength || chan >= NumAdcChannels)
	{
		return false;
	}

	return InternalEnableChannel(chan, ADC_REFCTRL_REFSEL_INTVCC1, fn, param, p_ticksPerCall);
}

bool AdcClass::IsChannelEnabled(unsigned int chan) const
{
	return (channelsEnabled & (1ul << chan)) != 0;
}

bool AdcClass::InternalEnableChannel(unsigned int chan, uint8_t refCtrl, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall)
{
	if (chan < 32)
	{
		TaskCriticalSectionLocker lock;

		// Set up the ADC
		callbackFunctions[chan] = fn;
		callbackParams[chan] = param;
		ticksPerCall[chan] = p_ticksPerCall;
		ticksAtLastCall[chan] = millis();
		resultsByChannel[chan] = 0;
		++numChannelsEnabled;
		channelsEnabled |= 1ul << chan;

		if (numChannelsEnabled == 1)
		{
			// First channel is being enabled, so initialise the ADC
			if (!hri_adc_is_syncing(device, ADC_SYNCBUSY_SWRST))
			{
				if (hri_adc_get_CTRLA_reg(device, ADC_CTRLA_ENABLE))
				{
					hri_adc_clear_CTRLA_ENABLE_bit(device);
					hri_adc_wait_for_sync(device, ADC_SYNCBUSY_ENABLE);
				}
				hri_adc_write_CTRLA_reg(device, ADC_CTRLA_SWRST);
			}
			hri_adc_wait_for_sync(device, ADC_SYNCBUSY_SWRST);

			hri_adc_write_CTRLB_reg(device, ADC_CTRLB_PRESCALER_DIV16);			// GCLK0 is 48MHz, divided by 16 is 3MHz
			hri_adc_write_REFCTRL_reg(device,  ADC_REFCTRL_REFSEL_INTVCC1);
			hri_adc_write_EVCTRL_reg(device, ADC_EVCTRL_RESRDYEO);
			hri_adc_write_INPUTCTRL_reg(device, ADC_INPUTCTRL_MUXNEG_GND);
			hri_adc_write_AVGCTRL_reg(device, 0);
			hri_adc_write_SAMPCTRL_reg(device, ADC_SAMPCTRL_SAMPLEN(10));
			hri_adc_write_WINLT_reg(device, 0);
			hri_adc_write_WINUT_reg(device, 0xFFFF);
			hri_adc_write_GAINCORR_reg(device, 1u << 11);
			hri_adc_write_OFFSETCORR_reg(device, 0);
			hri_adc_write_DBGCTRL_reg(device, 0);

			hri_adc_set_CTRLA_ENABLE_bit(device);

			// Initialise the DMAC to read the result
			DmacManager::SetSourceAddress(dmaChan, const_cast<uint16_t *>(&device->RESULT.reg));
			DmacManager::SetInterruptCallbacks(dmaChan, DmaCompleteCallback, nullptr, this);
			DmacManager::SetBtctrl(dmaChan, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_HWORD
										| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
			DmacManager::SetTriggerSource(dmaChan, trigSrc);
			state = State::starting;
		}

		channelsEnabled |= 1ul << chan;
		return true;
	}

	return false;
}

bool AdcClass::StartConversion(TaskBase *p_taskToWake)
{
	if (numChannelsEnabled == 0 || state == State::converting)
	{
		return false;
	}

	taskToWake = p_taskToWake;
	(void)device->RESULT.reg;			// make sure no result pending (this is necessary to make it work!)
	device->SEQCTRL.reg = channelsEnabled;

	// Set up DMA to read the results our of the ADC into the results array
	DmacManager::SetDestinationAddress(dmaChan, results);
	DmacManager::SetDataLength(dmaChan, numChannelsEnabled);
	DmacManager::EnableCompletedInterrupt(dmaChan);
	DmacManager::EnableChannel(dmaChan);

	state = State::converting;
	++conversionsStarted;
	return true;
}

void AdcClass::ExecuteCallbacks()
{
	TaskCriticalSectionLocker lock;
	const uint32_t now = millis();
	const volatile uint16_t *p = results;
	const uint32_t channelsPreviouslyEnabled = device->SEQCTRL.reg;
	for (size_t i = 0; i < NumAdcChannels; ++i)
	{
		if ((channelsPreviouslyEnabled & (1u << i)) != 0)
		{
			const uint16_t currentResult = *p++;
			resultsByChannel[i] = currentResult;
			if (now - ticksAtLastCall[i] >= ticksPerCall[i])
			{
				ticksAtLastCall[i] = now;
				const AnalogInCallbackFunction fn = callbackFunctions[i];
				if (fn != nullptr)
				{
					fn(callbackParams[i], currentResult);
				}
			}
		}
	}
}

// Indirect callback from the DMA controller ISR
void AdcClass::ResultReadyCallback()
{
	state = State::ready;
	++conversionsCompleted;
	DmacManager::DisableChannel(dmaChan);			// disable the sequencer DMA, just in case it is out of sync
	if (taskToWake != nullptr)
	{
		taskToWake->GiveFromISR();
	}
}

// Callback from the DMA controller ISR
/*static*/ void AdcClass::DmaCompleteCallback(CallbackParameter cp)
{
	static_cast<AdcClass *>(cp.vp)->ResultReadyCallback();
}

// ADC instances
static AdcClass Adcs[] =
{
	// We use only the first ADC
	AdcClass(ADC0, ADC0_IRQn, Adc0TxDmaChannel, DmaTrigSource::adc0_resrdy),
};

namespace AnalogIn
{
	// Analog input management task
	constexpr size_t AnalogInTaskStackWords = 200;
	static Task<AnalogInTaskStackWords> analogInTask;

	// Main loop executed by the AIN task
	extern "C" void AinLoop(void *)
	{
		// Loop taking readings and processing them
		for (;;)
		{
			// Loop through ADCs
			bool conversionStarted = false;
			for (size_t i = 0; i < ARRAY_SIZE(Adcs); ++i)
			{
				AdcClass& adc = Adcs[i];
				switch (adc.GetState())
				{
				case AdcClass::State::ready:
					adc.ExecuteCallbacks();
					//no break
				case AdcClass::State::idle:
				case AdcClass::State::starting:
					adc.StartConversion(&analogInTask);
					conversionStarted = true;
					break;

				default:	// no channels enabled, or conversion in progress
					break;
				}
			}

			// Do the temperature sensor
			const AnalogInCallbackFunction fn = tempCallbackFn;
			if (tempCallbackFn != nullptr && TSENS->INTFLAG.bit.RESRDY)
			{
				const uint32_t now = millis();
				if (now - tempTicksAtLastCall >= tempTicksPerCall)
				{
					tempTicksAtLastCall = now;
					fn(tempCallbackParam, TSENS->VALUE.bit.VALUE ^ (1u << 23));		// VALUE is 2's complement, but the filter expects unsigned values
					TSENS->CTRLB.bit.START = 1;
				}
			}

			if (conversionStarted)
			{
				if (TaskBase::Take(500) == 0)
				{
					//TODO we had a timeout so record an error
				}
				delay(2);
			}
			else
			{
				// No ADCs enabled yet, or all converting
				delay(10);
			}
		}
	}
}

// Initialise the analog input subsystem. Call this just once.
void AnalogIn::Init()
{
	// Enable ADC clocks
	// SAMC21 has 2 ADCs but we use only the first one
	hri_mclk_set_APBCMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

	analogInTask.Create(AinLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

// Enable analog input on a pin.
// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
// Set ticksPerCall to 0 to get a callback on every reading.
bool AnalogIn::EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall)
{
	if (pin < ARRAY_SIZE(PinTable))
	{
		const AdcInput adcin = IoPort::PinToAdcInput(pin);
		if (adcin != AdcInput::none)
		{
			IoPort::SetPinMode(pin, AIN);
			return Adcs[GetDeviceNumber(adcin)].EnableChannel(GetInputNumber(adcin), fn, param, ticksPerCall);
		}
	}
	return false;
}

// Return whether or not the channel is enabled
bool AnalogIn::IsChannelEnabled(Pin pin)
{
	if (pin < ARRAY_SIZE(PinTable))
	{
		const AdcInput adcin = IoPort::PinToAdcInput(pin);
		if (adcin != AdcInput::none)
		{
			return Adcs[GetDeviceNumber(adcin)].IsChannelEnabled(GetInputNumber(adcin));
		}
	}
	return false;
}

#if 0
// Disable a previously-enabled channel
bool AnalogIn::DisableChannel(Pin pin)
{
	//TODO not implemented yet (do we need it?)
	return false;
}
#endif

uint16_t AnalogIn::ReadChannel(AdcInput adcin)
{
	return (adcin != AdcInput::none) ? Adcs[GetDeviceNumber(adcin)].ReadChannel(GetInputNumber(adcin)) : 0;
}

// Enable an on-chip MCU temperature sensor
void AnalogIn::EnableTemperatureSensor(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall)
{
	tempCallbackParam = param;
	tempCallbackFn = fn;
	tempTicksPerCall = ticksPerCall;

	hri_mclk_set_APBAMASK_TSENS_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TSENS_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));		// Use thr DFLL as the clock source

	TSENS->CTRLA.bit.SWRST = 1;
	while (TSENS->CTRLA.bit.SWRST) { }
	TSENS->CTRLC.reg = 0;

	const uint32_t calGain0 = (*reinterpret_cast<const uint32_t*>(TSENS_FUSES_GAIN_0_ADDR) & TSENS_FUSES_GAIN_0_Msk) >> TSENS_FUSES_GAIN_0_Pos;
	const uint32_t calGain1 = (*reinterpret_cast<const uint32_t*>(TSENS_FUSES_GAIN_1_ADDR) & TSENS_FUSES_GAIN_1_Msk) >> TSENS_FUSES_GAIN_1_Pos;
	const uint32_t calOffset = (*reinterpret_cast<const uint32_t*>(TSENS_FUSES_OFFSET_ADDR) & TSENS_FUSES_OFFSET_Msk) >> TSENS_FUSES_OFFSET_Pos;
	const uint32_t calFcal = (*reinterpret_cast<const uint32_t*>(TSENS_FUSES_FCAL_ADDR) & TSENS_FUSES_FCAL_Msk) >> TSENS_FUSES_FCAL_Pos;
	const uint32_t calTcal = (*reinterpret_cast<const uint32_t*>(TSENS_FUSES_TCAL_ADDR) & TSENS_FUSES_TCAL_Msk) >> TSENS_FUSES_TCAL_Pos;

	TSENS->GAIN.reg = calGain0 | (calGain1 << 20);
	TSENS->OFFSET.reg = calOffset;
	TSENS->CAL.reg = TSENS_CAL_TCAL(calTcal) | TSENS_CAL_FCAL(calFcal);

	TSENS->CTRLA.bit.ENABLE = 1;
	while (TSENS->SYNCBUSY.bit.ENABLE) { }
	TSENS->CTRLB.reg = TSENS_CTRLB_START;
}

void AnalogIn::GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted)
{
	convsStarted = conversionsStarted;
	convsCompleted = conversionsCompleted;
}

#endif

// End
