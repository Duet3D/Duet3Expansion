/*
 * LinearEncoder.cpp
 *
 *  Created on: 6 Nov 2022
 *      Author: David
 */

#include <ClosedLoop/Encoders/LinearCompositeEncoder.h>

#if SUPPORT_CLOSED_LOOP

LinearCompositeEncoder::LinearCompositeEncoder(float p_countsPerRev, uint32_t p_stepsPerRev, SharedSpiDevice &spiDev, Pin p_csPin) noexcept
	: Encoder(p_stepsPerRev, p_countsPerRev)
{
	shaftEncoder = new AS5047D(p_stepsPerRev, spiDev, p_csPin);
	linEncoder = new QuadratureEncoderPdec(p_stepsPerRev, p_countsPerRev);
}

LinearCompositeEncoder::~LinearCompositeEncoder()
{
	delete linEncoder;
	delete shaftEncoder;
}

bool LinearCompositeEncoder::TakeReading() noexcept
{
	const bool shaftOk = shaftEncoder->TakeReading();
	const bool linOk = linEncoder->TakeReading();
	if (shaftOk && linOk)
	{
		//TODO copy counts etc. across
		return true;
	}
	return false;
}

GCodeResult LinearCompositeEncoder::Init(const StringRef &reply) noexcept
{
	GCodeResult ret = shaftEncoder->Init(reply);
	if (ret == GCodeResult::ok)
	{
		linEncoder->Init(reply);
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
	shaftEncoder->Disable();
	linEncoder->Disable();
}

void LinearCompositeEncoder::ClearFullRevs() noexcept
{
	shaftEncoder->ClearFullRevs();
	//TODO copy modified counts down
}

// Encoder polarity for basic tuning purposes. Changing this will change the encoder reading.
void LinearCompositeEncoder::SetTuningBackwards(bool backwards) noexcept
{
	linEncoder->SetTuningBackwards(backwards);
	//TODO copy data down if needed
}

// Encoder polarity for calibration purposes. Changing this will change the encoder reading.
void LinearCompositeEncoder::SetCalibrationBackwards(bool backwards) noexcept
{
	shaftEncoder->SetCalibrationBackwards(backwards);
	//TODO copy data down if needed
}

void LinearCompositeEncoder::AppendDiagnostics(const StringRef &reply) noexcept
{
	//TODO sort out the formatting
	reply.cat("Shaft: ");
	shaftEncoder->AppendDiagnostics(reply);
	reply.lcat("Lin: ");
	linEncoder->AppendDiagnostics(reply);
}

void LinearCompositeEncoder::AppendStatus(const StringRef &reply) noexcept
{
	//TODO sort out the formatting
	reply.lcat("Shaft");
	shaftEncoder->AppendStatus(reply);
	reply.lcat("Lin");
	linEncoder->AppendStatus(reply);
}

#endif

// End
