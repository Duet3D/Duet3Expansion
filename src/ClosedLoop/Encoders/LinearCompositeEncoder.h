/*
 * LinearEncoder.h
 *
 *  Created on: 6 Nov 2022
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_ENCODERS_LINEARCOMPOSITEENCODER_H_
#define SRC_CLOSEDLOOP_ENCODERS_LINEARCOMPOSITEENCODER_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP && SAME5x

#include "QuadratureEncoderPdec.h"
#include "AS5047D.h"

class LinearCompositeEncoder : public Encoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<LinearCompositeEncoder>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<LinearCompositeEncoder>(p); }

	LinearCompositeEncoder(float p_countsPerRev, uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept;
	~LinearCompositeEncoder();

	// Overridden virtual functions

	// Take a reading and store at least currentCount and currentPhasePosition. Return true if error, false if success.
	bool TakeReading() noexcept override;

	// Tell the encoder what the step phase is at the current count. Only applicable to relative encoders.
	void SetKnownPhaseAtCurrentCount(uint32_t phase) noexcept override { }

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	void ClearFullRevs() noexcept override;

	// Encoder polarity for basic tuning purposes. Changing this will change the encoder reading.
	void SetTuningBackwards(bool backwards) noexcept override;

	// Encoder polarity for calibration purposes. Changing this will change the encoder reading.
	void SetCalibrationBackwards(bool backwards) noexcept override;

	// Return true if rotary absolute encoder calibration is applicable to this encoder
	bool UsesCalibration() const noexcept override { return true; }

	// Return true if basic tuning is applicable to this encoder
	bool UsesBasicTuning() const noexcept override { return true; }

	EncoderType GetType() const noexcept override { return EncoderType::linearComposite; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

	// Set the forward tuning results. Only applicable if the encoder supports basic tuning.
	void SetForwardTuningResults(float slope, float xMean, float yMean) noexcept override { linEncoder->SetForwardTuningResults(slope, xMean, yMean); }

	// Set the reverse tuning results. Only applicable if the encoder supports basic tuning.
	void SetReverseTuningResults(float slope, float xMean, float yMean) noexcept override { linEncoder->SetReverseTuningResults(slope, xMean, yMean); }

	// Process the tuning data. Only applicable if the encoder supports basic tuning.
	TuningErrors ProcessTuningData() noexcept override;

	// Clear the encoder data collection. Only applicable if the encoder supports calibration.
	void ClearDataCollection(size_t p_numDataPoints) noexcept override { return shaftEncoder->ClearDataCollection(p_numDataPoints); }

	// Record a calibration data point. Only applicable if the encoder supports calibration.
	void RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept override { return shaftEncoder->RecordDataPoint(index, data, backwards); }

	// Calibrate the encoder using the recorded data points. Only applicable if the encoder supports calibration.
	TuningErrors Calibrate(bool store) noexcept override;

	// Load the calibration lookup table and clear bits TuningError:NeedsBasicTuning and/or TuningError::NotCalibrated in tuningNeeded as appropriate.
	void LoadLUT(TuningErrors& tuningNeeded) noexcept override;

	// Clear the calibration lookup table. Only applicable if the encoder supports calibration.
	void ClearLUT() noexcept override { return shaftEncoder->ClearLUT(); }

	// Clear the calibration lookup table and delete it from NVRAM. Only applicable if the encoder supports calibration.
	void ScrubLUT() noexcept override { return shaftEncoder->ScrubLUT(); }

	// Append a summary of calibration lookup table corrections to a string. Only applicable if the encoder supports calibration.
	void AppendLUTCorrections(const StringRef& reply) const noexcept override { return shaftEncoder->AppendLUTCorrections(reply); }

	// Append a summary of measured calibration errors to a string. Only applicable if the encoder supports calibration.
	void AppendCalibrationErrors(const StringRef& reply) const noexcept override{ return shaftEncoder->AppendCalibrationErrors(reply); }

	// Get the count if the shaft encoder. Only used by calibration. Normally the same as GetCurrentCount except for combination encoders.
	int32_t GetCurrentShaftCount() const noexcept override { return shaftEncoder->GetCurrentCount(); }

private:
	// Load the quadrature encoder direction from NVM
	__attribute__((noinline)) void LoadQuadratureDirectionFromNVM(TuningErrors& tuningNeeded) noexcept;

	QuadratureEncoderPdec *linEncoder;
	AS5047D *shaftEncoder;
};

#endif	//SUPPORT_CLOSED_LOOP

#endif /* SRC_CLOSEDLOOP_ENCODERS_LINEARCOMPOSITEENCODER_H_ */
