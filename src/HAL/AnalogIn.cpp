/*
 * AnalogIn.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include "AnalogIn.h"
#include "RTOSIface/RTOSIface.h"
#include "HAL/DmacManager.h"

class AdcClass
{
public:
	enum class State : uint8_t
	{
		noChannels = 0,
		idle,
		converting,
		ready
	};

	AdcClass(Adc * const p_device, IRQn p_irqn, DmaChannel p_dmaChan, DmaTrigSource p_trigSrc, const int8_t *p_pinTable, size_t p_size);

	State GetState() const { return state; }
	bool EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall);
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall);
	bool StartConversion(TaskBase *p_taskToWake);

	void ResultReadyCallback();
	void ExecuteCallbacks();

private:
	bool InternalEnableChannel(uint8_t chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall);
	static void DmaCompleteCallback(CallbackParameter cp);

	static constexpr size_t MaxChannels = 16;

	Adc * const device;
	const int8_t * const pinTable;
	const size_t pinTableSize;
	const IRQn irqn;
	const DmaChannel dmaChan;
	const DmaTrigSource trigSrc;

	size_t numChannelsEnabled;
	TaskBase *taskToWake;
	State state;
	AnalogInCallbackFunction callbackFunctions[MaxChannels];
	CallbackParameter callbackParams[MaxChannels];
	uint32_t ticksPerCall[MaxChannels];
	uint32_t ticksAtLastCall[MaxChannels];
	uint32_t inputRegisters[MaxChannels];
	uint16_t results[MaxChannels];
};

AdcClass::AdcClass(Adc * const p_device, IRQn p_irqn, DmaChannel p_dmaChan, DmaTrigSource p_trigSrc, const int8_t *p_pinTable, size_t p_size)
	: device(p_device), pinTable(p_pinTable), pinTableSize(p_size), irqn(p_irqn), dmaChan(p_dmaChan), trigSrc(p_trigSrc),
	  numChannelsEnabled(0), taskToWake(nullptr), state(State::noChannels)
{
	for (size_t i = 0; i < MaxChannels; ++i)
	{
		callbackFunctions[i] = nullptr;
		callbackParams[i].u32 = 0;
	}
}

// Try to enable this ADC on the specified pin returning true if successful
// Only single ended mode with gain x1 is supported
// There is no check to avoid adding the same channel twice. If you do that it will be converted twice.
bool AdcClass::EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall)
{
	if (numChannelsEnabled == MaxChannels || pin >= pinTableSize)
	{
		return false;
	}

	const int8_t chan = pinTable[pin];
	if (chan < 0)
	{
		return false;
	}

	// We have a valid ADC number and channel for this pin
	gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);							// disable the data input buffer
	gpio_set_pin_function(pin, ((pin & 31) << 16) | GPIO_PIN_FUNCTION_B);		// ADC is always on peripheral B

	return InternalEnableChannel(chan, fn, param, p_ticksPerCall);
}

bool AdcClass::EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall)
{
	if (numChannelsEnabled == MaxChannels || sensorNumber >= 2)
	{
		return false;
	}

	return InternalEnableChannel(sensorNumber + 0x1C, fn, param, p_ticksPerCall);
}

bool AdcClass::InternalEnableChannel(uint8_t chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall)
{
	TaskCriticalSectionLocker lock;

	// Set up the ADC
	callbackFunctions[numChannelsEnabled] = fn;
	callbackParams[numChannelsEnabled] = param;
	ticksPerCall[numChannelsEnabled] = p_ticksPerCall;
	ticksAtLastCall[numChannelsEnabled] = millis();
	inputRegisters[numChannelsEnabled] = 0x1800 | (uint16_t)chan;
	++numChannelsEnabled;

	if (numChannelsEnabled == 1)
	{
		// First channel is being enabled, so initialise the DMAC
		// First the sequencer
		DmacSetSourceAddress(dmaChan, inputRegisters);
		DmacSetDestinationAddress(dmaChan, const_cast<uint32_t *>(&device->DSEQDATA.reg));
		DmacSetBtctrl(dmaChan, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSIZE_X1);
		DMAC->Channel[dmaChan].CHCTRLA.reg = DMAC_CHCTRLA_SWRST | DMAC_CHCTRLA_TRIGSRC((uint8_t)trigSrc + 1) | DMAC_CHCTRLA_TRIGACT_BURST
														| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
		// Now the result reader
		DmacSetSourceAddress(dmaChan + 1, const_cast<uint16_t *>(&device->RESULT.reg));
		DmacSetDestinationAddress(dmaChan + 1, results);
		DmacSetInterruptCallbacks(dmaChan + 1, DmaCompleteCallback, nullptr, this);
		DmacSetBtctrl(dmaChan + 1, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSIZE_X1);
		DMAC->Channel[dmaChan + 1].CHCTRLA.reg = DMAC_CHCTRLA_SWRST | DMAC_CHCTRLA_TRIGSRC((uint8_t)trigSrc) | DMAC_CHCTRLA_TRIGACT_BURST
														| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;

		// Initialise the ADC
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

		hri_adc_write_CTRLB_reg(device, 0);
		hri_adc_write_REFCTRL_reg(device, ADC_REFCTRL_REFSEL_INTVCC1);
		hri_adc_write_EVCTRL_reg(device, 0);
		hri_adc_write_INPUTCTRL_reg(device, ADC_INPUTCTRL_MUXNEG_GND);
		hri_adc_write_AVGCTRL_reg(device, 0);
		hri_adc_write_SAMPCTRL_reg(device, ADC_SAMPCTRL_OFFCOMP);
		hri_adc_write_WINLT_reg(device, 0);
		hri_adc_write_WINUT_reg(device, 0xFFFF);
		hri_adc_write_GAINCORR_reg(device, 1u << 11);
		hri_adc_write_OFFSETCORR_reg(device, 0);
		hri_adc_write_DBGCTRL_reg(device, 0);
		hri_adc_write_DSEQCTRL_reg(device, ADC_DSEQCTRL_INPUTCTRL);						// enable DMA sequencing, update just the input register
		hri_adc_write_CTRLA_reg(device, ADC_CTRLA_PRESCALER_DIV16 | ADC_CTRLA_ENABLE);

		state = State::idle;
	}

	return true;
}

bool AdcClass::StartConversion(TaskBase *p_taskToWake)
{
	if (numChannelsEnabled == 0 || state == State::converting)
	{
		return false;
	}

	DmacSetSourceAddress(dmaChan, inputRegisters);
	DmacSetDataLength(dmaChan, numChannelsEnabled * sizeof(inputRegisters[0]));
	DmacSetDestinationAddress(dmaChan + 1, results);
	DmacSetDataLength(dmaChan + 1, numChannelsEnabled * sizeof(results[0]));

	state = State::converting;
	taskToWake = p_taskToWake;
	DmacEnableCompletedInterrupt(dmaChan + 1);
	hri_adc_set_SWTRIG_START_bit(device);
	return true;
}

void AdcClass::ExecuteCallbacks()
{
	TaskCriticalSectionLocker lock;
	const uint32_t now = millis();
	for (size_t i = 0; i < numChannelsEnabled; ++i)
	{
		if (now - ticksAtLastCall[i] >= ticksPerCall[i])
		{
			ticksAtLastCall[i] = now;
			if (callbackFunctions[i] != nullptr)
			{
				callbackFunctions[i](callbackParams[i], results[i]);
			}
		}
	}
}

void AdcClass::ResultReadyCallback()
{
	state = State::ready;
	if (taskToWake != nullptr)
	{
		taskToWake->GiveFromISR();
	}
}

/*static*/ void AdcClass::DmaCompleteCallback(CallbackParameter cp)
{
	static_cast<AdcClass *>(cp.vp)->ResultReadyCallback();
}

// Device-specific parts

#ifdef __SAME51N19A__

// Tables to map pins to ADC inputs
static const int8_t Adc0PinTable[] =
{
	-1, -1,  0,  1,  4,  5,  6,  7,	// PA0-PA7
	 8,  9, 10, 11, -1, -1, -1, -1,	// PA8-PA15
	-1, -1, -1, -1, -1, -1, -1, -1,	// PA16-PA23
	-1, -1, -1, -1, -1, -1, -1, -1,	// PA24-PA31
	12, 13, 14, 15, -1, -1,  8,  9,	// PB0-PB7
	 2,  3							// PB8-PB9
};

static const int8_t Adc1PinTable[] =
{
	-1, -1, -1,  1, -1, -1, -1, -1,	// PA0-PA7
	 2,  3, -1, -1, -1, -1, -1, -1,	// PA8-PA15
	-1, -1, -1, -1, -1, -1, -1, -1,	// PA16-PA23
	-1, -1, -1, -1, -1, -1, -1, -1,	// PA24-PA31
	-1, -1, -1, -1,  6,  7,  8,  9,	// PB0-PB7
	 0,  1, -1, -1, -1, -1, -1, -1,	// PB8-PB15
	-1, -1, -1, -1, -1, -1, -1, -1,	// PB16-PB23
	-1, -1, -1, -1, -1, -1, -1, -1,	// PB24-PB31
	10, 11,  4,  5					// PC0-PC7
};

// ADC instances
static AdcClass Adc0(ADC0, ADC0_0_IRQn, Adc0TxDmaChannel, DmaTrigSource::adc0_resrdy, Adc0PinTable, ARRAY_SIZE(Adc0PinTable));
static AdcClass Adc1(ADC1, ADC1_0_IRQn, Adc1TxDmaChannel, DmaTrigSource::adc1_resrdy, Adc1PinTable, ARRAY_SIZE(Adc1PinTable));

// Table of available ADCs
static AdcClass * const Adcs[] = { &Adc0, &Adc1 };

#endif

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
				AdcClass& adc = *Adcs[i];
				switch (adc.GetState())
				{
				case AdcClass::State::ready:
					adc.ExecuteCallbacks();
					//no break
				case AdcClass::State::idle:
					adc.StartConversion(&analogInTask);
					conversionStarted = true;
					break;

				default:	// no channels enabled, or conversion in progress
					break;
				}
			}

			if (conversionStarted)
			{
				if (TaskBase::Take(500) == 0)
				{
					//TODO we had a timeout so record an error
				}
			}
			else
			{
				// No ADCs enabled yet, or all converting
				delay(10);
			}
		}
	}

	// Initialise the analog input subsystem. Call this just once.
	void Init()
	{
		analogInTask.Create(AinLoop, "AIN", nullptr, TaskBase::AinPriority);
	}

	// Enable analog input on a pin.
	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	// 'adcnum' specifies which ADC to use, for those pins that can be connected to more than one ADC.
	bool EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, int adcnum)
	{
		if (adcnum == -1)
		{
			adcnum = 0;
			while (adcnum < (int)ARRAY_SIZE(Adcs))
			{
				if (Adcs[adcnum]->EnableChannel(pin, fn, param, ticksPerCall))
				{
					return true;
				}
				++adcnum;
			}
			return false;
		}
		else if (adcnum >= 0 && adcnum < (int)ARRAY_SIZE(Adcs))
		{
			return Adcs[adcnum]->EnableChannel(pin, fn, param, ticksPerCall);
		}
		return false;
	}

	// Disable a previously-enabled channel
	bool DisableChannel(Pin pin)
	{
		//TODO not implemented yet (do we need it?)
		return false;
	}

	// Enable an on-chip MCU temperature sensor
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, int adcnum)
	{
		if (adcnum == -1)
		{
			adcnum = 0;
			while (adcnum < (int)ARRAY_SIZE(Adcs))
			{
				if (Adcs[adcnum]->EnableTemperatureSensor(sensorNumber, fn, param, ticksPerCall))
				{
					return true;
				}
				++adcnum;
			}
			return false;
		}
		else if (adcnum >= 0 && adcnum < (int)ARRAY_SIZE(Adcs))
		{
			return Adcs[adcnum]->EnableTemperatureSensor(sensorNumber, fn, param, ticksPerCall);
		}
		return false;
	}
}

// End
