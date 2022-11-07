/*
 * LinearEncoder.cpp
 *
 *  Created on: 6 Nov 2022
 *      Author: David
 */

#include <ClosedLoop/Encoders/LinearCompositeEncoder.h>

#if SUPPORT_CLOSED_LOOP

LinearCompositeEncoder::LinearCompositeEncoder(float p_countsPerRev, uint32_t p_stepsPerRev, SharedSpiDevice &spiDev, Pin p_csPin) noexcept
	: Encoder((4 * p_countsPerRev)/(float)p_stepsPerRev, p_stepsPerRev)
{
	shaftEncoder = new AS5047D(p_stepsPerRev, spiDev, p_csPin);
	linEncoder = new QuadratureEncoderPdec(p_countsPerRev, p_stepsPerRev);
}

LinearCompositeEncoder::~LinearCompositeEncoder()
{
	delete linEncoder;
	delete shaftEncoder;
}

// Take a reading and store at least currentCount and currentPhasePosition. Return true if error, false if success.
bool LinearCompositeEncoder::TakeReading() noexcept
{
	const bool shaftErr = shaftEncoder->TakeReading();
	const bool linErr = linEncoder->TakeReading();
	if (!shaftErr && !linErr)
	{
		currentCount = linEncoder->GetCurrentCount();
		currentPhasePosition = shaftEncoder->GetCurrentPhasePosition();
		return false;
	}
	return true;
}

GCodeResult LinearCompositeEncoder::Init(const StringRef &reply) noexcept
{
	GCodeResult ret = shaftEncoder->Init(reply);
	if (ret == GCodeResult::ok)
	{
		ret = linEncoder->Init(reply);
	}
	return ret;
}

void LinearCompositeEncoder::Enable() noexcept
{
	shaftEncoder->Enable();
	linEncoder->Enable();
}

void LinearCompositeEncoder::Disable() noexcept
{
	linEncoder->Disable();
	shaftEncoder->Disable();
}

void LinearCompositeEncoder::ClearFullRevs() noexcept
{
	shaftEncoder->ClearFullRevs();
}

// Encoder polarity for basic tuning purposes. Changing this will change the encoder reading.
void LinearCompositeEncoder::SetTuningBackwards(bool backwards) noexcept
{
	linEncoder->SetTuningBackwards(backwards);
	currentCount = linEncoder->GetCurrentCount();
}

// Process the tuning data. Only applicable if the encoder supports basic tuning.
TuningErrors LinearCompositeEncoder::ProcessTuningData() noexcept
{
	const TuningErrors rslt = linEncoder->ProcessTuningData();
	measuredCountsPerStep = linEncoder->GetMeasuredCountsPerStep();
	measuredHysteresis = linEncoder->GetMeasuredHysteresis();
	return rslt;
}

// Encoder polarity for calibration purposes. Changing this will change the encoder reading.
void LinearCompositeEncoder::SetCalibrationBackwards(bool backwards) noexcept
{
	shaftEncoder->SetCalibrationBackwards(backwards);
	currentPhasePosition = shaftEncoder->GetCurrentPhasePosition();
}

// Calibrate the encoder using the recorded data points. Only applicable if the encoder supports calibration.
TuningErrors LinearCompositeEncoder::Calibrate(bool store) noexcept
{
	const TuningErrors rslt = shaftEncoder->Calibrate(store);
	measuredCountsPerStep = shaftEncoder->GetMeasuredCountsPerStep();
	measuredHysteresis = shaftEncoder->GetMeasuredHysteresis();
	return rslt;
}

void LinearCompositeEncoder::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.cat("Shaft: ");
	shaftEncoder->AppendDiagnostics(reply);
	reply.lcat("Lin: ");
	linEncoder->AppendDiagnostics(reply);
}

void LinearCompositeEncoder::AppendStatus(const StringRef &reply) noexcept
{
	shaftEncoder->AppendStatus(reply);
	linEncoder->AppendStatus(reply);
}

#endif

// End
