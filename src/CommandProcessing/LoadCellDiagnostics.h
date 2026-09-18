/*
 * LoadCellDiagnostics.h
 *
 *  Created on: 11 Aug 2026
 *      Author: Christian
 */

#ifndef SRC_COMMANDPROCESSING_LOADCELLDIAGNOSTICS_H_
#define SRC_COMMANDPROCESSING_LOADCELLDIAGNOSTICS_H_

#include <RepRapFirmware.h>

#if SUPPORT_LOADCELL_DIAGNOSTICS

// Load cell capture and analysis, reported by M122. Everything is in raw ADC counts, because the scale in grams per count
// is held by the main board and never sent here
namespace LoadCellDiagnostics
{
	void RecordSample(int32_t reading) noexcept;				// called from the ADC task for every sample
	void AppendDiagnostics(const StringRef& reply) noexcept;	// append the drift figures and, if configured, the fast spectrum
#if SUPPORT_LOADCELL_FFT
	void AppendSlowSpectrum(const StringRef& reply) noexcept;	// append the slow spectrum, which is a separate diagnostics part
#endif
}

#endif

#endif /* SRC_COMMANDPROCESSING_LOADCELLDIAGNOSTICS_H_ */
