/*
 * LoadCellDiagnostics.cpp
 *
 *  Created on: 11 Aug 2026
 *      Author: Christian
 */

#include "LoadCellDiagnostics.h"

#if SUPPORT_LOADCELL_DIAGNOSTICS

#include <cmath>

namespace LoadCellDiagnostics
{
	// Drift tracking, cheap enough to leave on whenever diagnostics are configured. Each tracker averages readings into blocks
	// of its own interval and reports the spread between the smallest and largest block mean since the previous report. The
	// averaging is what makes the figure meaningful: a one second block averages about 977 samples, so the noise floor shrinks
	// by a factor of about 31 and what remains is genuine baseline movement rather than sensor noise.
	//
	//   1s   roughly the length of a probing move, so this is the movement that a tare taken at the start of the probe cannot
	//        remove. It is the floor on trigger accuracy, and it says whether taring once per probe is sufficient on its own
	//   10s  the gap between consecutive taps when multi-tapping or running a mesh, so this says whether re-taring between taps
	//        buys anything at all
	//   60s  thermal, over the course of a mesh run or a print segment
	//
	// Together they dimension the drift filter: a small 1s figure against a large 60s one means a low corner frequency will
	// track the drift out without touching the tap, whereas a large 1s figure means no corner works, because a filter fast
	// enough to follow that drift is also fast enough to eat the leading edge of a tap. They cannot distinguish a monotonic
	// ramp from an oscillation, which is what the spectra are for, and a steady drift produces a figure that grows with the
	// time since the last report while a bounded wobble produces one that does not
	constexpr uint32_t DriftIntervals[] = { 1000, 10000, 60000 };
	constexpr size_t NumDriftIntervals = ARRAY_SIZE(DriftIntervals);

	struct DriftTracker
	{
		int64_t sum;
		uint32_t count;
		uint32_t whenBlockStarted;
		float minMean, maxMean;
		uint32_t blocks;

		void Reset() noexcept { minMean = maxMean = 0.0; blocks = 0; }
		void Add(int32_t reading, uint32_t now, uint32_t interval) noexcept;
	};

	static DriftTracker driftTrackers[NumDriftIntervals];
	static bool initialised = false;

	static volatile uint32_t samplesTaken = 0;
	static uint32_t whenCaptureStarted = 0;

#if SUPPORT_LOADCELL_FFT
	constexpr size_t NumSamples = 1024;						// 1.05s at 977 samples/sec, giving about 1Hz resolution up to 488Hz
	static_assert((NumSamples & (NumSamples - 1)) == 0);	// the transform below is radix-2
	constexpr size_t SlowDecimation = 64;					// the slow ring averages this many samples per entry, giving 15Hz and a 67s window

	// Bands reported for the fast spectrum. The last two cover the toolhead resonances that Kalico notches out
	constexpr float FastBandLimits[] = { 1.0, 10.0, 40.0, 100.0, 250.0 };
	// Bands reported for the slow spectrum, covering thermal drift, bowden forces and anything cycling slowly such as a heater or fan
	constexpr float SlowBandLimits[] = { 0.02, 0.1, 0.5, 1.0 };
	constexpr size_t MaxBands = 5;
	static_assert(ARRAY_SIZE(FastBandLimits) <= MaxBands && ARRAY_SIZE(SlowBandLimits) <= MaxBands);

	constexpr unsigned int NumPeaksToReport = 3;
	constexpr size_t PeakExclusionBins = 4;					// a Hann-windowed tone spans about 4 bins

	static int32_t fastSamples[NumSamples];					// int32 so that the ADC task never touches the FPU
	static int32_t slowSamples[NumSamples];
	static float re[NumSamples], im[NumSamples];			// analysis workspace, shared between the two spectra
	static volatile size_t fastWriteIndex = 0, slowWriteIndex = 0;
	static volatile uint32_t slowSamplesTaken = 0;
	static int64_t decimationSum = 0;
	static uint32_t decimationCount = 0;

	static void Fft() noexcept;
	static void AppendSpectrum(const int32_t (&buffer)[NumSamples], size_t startIndex, float rate,
								const float *_ecv_array bandLimits, size_t numBands, const char *_ecv_array label, const StringRef& reply) noexcept;
#endif
}

// Accumulate one block of this tracker's interval, and record how far the block means wander
void LoadCellDiagnostics::DriftTracker::Add(int32_t reading, uint32_t now, uint32_t interval) noexcept
{
	sum += reading;
	count++;
	if (now - whenBlockStarted >= interval)
	{
		const float blockMean = (float)sum / (float)count;
		if (blocks == 0)
		{
			minMean = maxMean = blockMean;
		}
		else if (blockMean < minMean)
		{
			minMean = blockMean;
		}
		else if (blockMean > maxMean)
		{
			maxMean = blockMean;
		}
		blocks++;
		sum = 0;
		count = 0;
		whenBlockStarted = now;
	}
}

// Record one sample. Called from the ADC task on every conversion, so it must stay trivial
void LoadCellDiagnostics::RecordSample(int32_t reading) noexcept
{
	const uint32_t now = millis();
	if (!initialised)
	{
		whenCaptureStarted = now;
		for (DriftTracker& t : driftTrackers)
		{
			t.sum = 0;
			t.count = 0;
			t.whenBlockStarted = now;
			t.Reset();
		}
		initialised = true;
	}

	if (samplesTaken < 0xFFFFFFFF)
	{
		samplesTaken = samplesTaken + 1;
	}

	for (size_t i = 0; i < NumDriftIntervals; i++)
	{
		driftTrackers[i].Add(reading, now, DriftIntervals[i]);
	}

#if SUPPORT_LOADCELL_FFT
	const size_t index = fastWriteIndex;
	fastSamples[index] = reading;
	fastWriteIndex = (index + 1) % NumSamples;

	decimationSum += reading;
	if (++decimationCount == SlowDecimation)
	{
		const size_t slowIndex = slowWriteIndex;
		slowSamples[slowIndex] = (int32_t)(decimationSum / (int64_t)SlowDecimation);	// averaging is also the anti-alias filter for the decimation
		slowWriteIndex = (slowIndex + 1) % NumSamples;
		if (slowSamplesTaken < 0xFFFFFFFF)
		{
			slowSamplesTaken = slowSamplesTaken + 1;
		}
		decimationSum = 0;
		decimationCount = 0;
	}
#endif
}

#if SUPPORT_LOADCELL_FFT

// In-place radix-2 decimation-in-time transform of re[]/im[]
void LoadCellDiagnostics::Fft() noexcept
{
	// Bit-reversal permutation
	for (size_t i = 1, j = 0; i < NumSamples; i++)
	{
		size_t bit = NumSamples >> 1;
		while ((j & bit) != 0)
		{
			j ^= bit;
			bit >>= 1;
		}
		j ^= bit;
		if (i < j)
		{
			const float tr = re[i];
			re[i] = re[j];
			re[j] = tr;
			const float ti = im[i];
			im[i] = im[j];
			im[j] = ti;
		}
	}

	for (size_t len = 2; len <= NumSamples; len <<= 1)
	{
		const size_t half = len / 2;
		const float ang = -Pi / (float)half;
		// The twiddle factor is hoisted out of the inner loop, which cuts the sinf/cosf calls from N/2*log2(N) to N-1
		for (size_t k = 0; k < half; k++)
		{
			const float wr = cosf(ang * (float)k);
			const float wi = sinf(ang * (float)k);
			for (size_t i = k; i < NumSamples; i += len)
			{
				const size_t j = i + half;
				const float vr = re[j] * wr - im[j] * wi;
				const float vi = re[j] * wi + im[j] * wr;
				re[j] = re[i] - vr;
				im[j] = im[i] - vi;
				re[i] += vr;
				im[i] += vi;
			}
		}
	}
}

// Transform one ring buffer and append the result. Deliberately reports evidence rather than recommended filter corners
void LoadCellDiagnostics::AppendSpectrum(const int32_t (&buffer)[NumSamples], size_t startIndex, float rate,
											const float *_ecv_array bandLimits, size_t numBands, const char *_ecv_array label, const StringRef& reply) noexcept
{
	const float binWidth = rate / (float)NumSamples;

	// Unwrap the ring into the workspace, oldest sample first. No lock is needed: the writer overwrites the oldest entries
	// next, which is where this copy starts, but it advances one entry per sample period while the whole copy takes microseconds
	for (size_t i = 0; i < NumSamples; i++)
	{
		re[i] = (float)buffer[(startIndex + i) % NumSamples];
	}

	float sum = 0.0;
	for (size_t i = 0; i < NumSamples; i++)
	{
		sum += re[i];
	}
	const float mean = sum / (float)NumSamples;

	// Remove the DC term, which would otherwise dominate the low bins, then apply the Hann window. Without the window a tone
	// lying between bins smears across the whole spectrum and reads as a wrong notch frequency
	for (size_t i = 0; i < NumSamples; i++)
	{
		re[i] = (re[i] - mean) * 0.5f * (1.0f - cosf((2.0f * Pi * (float)i) / (float)(NumSamples - 1)));
		im[i] = 0.0;
	}

	Fft();

	// Convert to single-sided amplitudes in place, so re[] then holds the amplitude of each bin
	// The 0.5 coherent gain of the Hann window is folded into the scale factor
	constexpr float AmplitudeScale = 4.0 / (float)NumSamples;
	constexpr size_t LastBin = NumSamples / 2 - 1;
	for (size_t k = 1; k <= LastBin; k++)
	{
		re[k] = sqrtf(re[k] * re[k] + im[k] * im[k]) * AmplitudeScale;
	}

	float bandSumSquares[MaxBands + 1];
	for (float& b : bandSumSquares)
	{
		b = 0.0;
	}
	for (size_t k = 1; k <= LastBin; k++)
	{
		size_t band = 0;
		while (band < numBands && (float)k * binWidth >= bandLimits[band])
		{
			band++;
		}
		bandSumSquares[band] += re[k] * re[k];
	}

	// Pick the strongest peaks, excluding the main lobe of one already picked - otherwise a single resonance fills every slot.
	// A peak must be a local maximum, otherwise the monotone skirt of the DC drift leakage fills the slots and the interpolation diverges
	float peakAmplitude[NumPeaksToReport] = { };
	float peakFrequency[NumPeaksToReport] = { };
	size_t peakBin[NumPeaksToReport] = { };
	for (unsigned int p = 0; p < NumPeaksToReport; p++)
	{
		size_t bestBin = 0;
		float bestAmplitude = 0.0;
		for (size_t k = 2; k < LastBin; k++)
		{
			bool excluded = false;
			for (unsigned int q = 0; q < p; q++)
			{
				if (k + PeakExclusionBins >= peakBin[q] && k <= peakBin[q] + PeakExclusionBins)
				{
					excluded = true;
				}
			}
			if (!excluded && re[k] > bestAmplitude && re[k] >= re[k - 1] && re[k] >= re[k + 1])
			{
				bestAmplitude = re[k];
				bestBin = k;
			}
		}
		if (bestBin == 0)
		{
			break;
		}

		// A Hann-windowed tone puts its energy in the peak bin and its two neighbours, so combining the three recovers the
		// amplitude even when the tone falls between bins; parabolic interpolation on the same three recovers the frequency
		peakBin[p] = bestBin;
		peakAmplitude[p] = sqrtf(re[bestBin - 1] * re[bestBin - 1] + re[bestBin] * re[bestBin] + re[bestBin + 1] * re[bestBin + 1]) / sqrtf(1.5);
		const float denominator = re[bestBin - 1] - 2 * re[bestBin] + re[bestBin + 1];
		const float offset = (denominator == 0.0f) ? 0.0f : constrain<float>(0.5f * (re[bestBin - 1] - re[bestBin + 1]) / denominator, -0.5f, 0.5f);
		peakFrequency[p] = ((float)bestBin + offset) * binWidth;
	}

	reply.lcatf("  %s spectrum, %.3fHz bins, band rms (counts):", label, (double)binWidth);
	for (size_t band = 1; band <= numBands; band++)			// band 0 is below the first limit, i.e. drift this window is too short to resolve
	{
		reply.catf(" %.3g-%.3gHz %.2f",
					(double)bandLimits[band - 1],
					(double)((band == numBands) ? rate / 2 : bandLimits[band]),
					(double)sqrtf(bandSumSquares[band] / 2));
	}
	reply.lcat("   peaks:");
	for (unsigned int p = 0; p < NumPeaksToReport; p++)
	{
		reply.catf(" %.3gHz %.2f", (double)peakFrequency[p], (double)peakAmplitude[p]);
	}
}

#endif

// Append the drift figures and, if configured, the spectra
void LoadCellDiagnostics::AppendDiagnostics(const StringRef& reply) noexcept
{
	if (!initialised || samplesTaken == 0)
	{
		reply.lcat("Load cell: no samples");
		return;
	}

	const uint32_t elapsed = millis() - whenCaptureStarted;
	const float measuredRate = (elapsed == 0) ? 0.0f : (float)samplesTaken * 1000.0f / (float)elapsed;

	reply.lcatf("Load cell: %" PRIu32 " samples at %.1fHz, baseline movement (counts):", samplesTaken, (double)measuredRate);
	for (size_t i = 0; i < NumDriftIntervals; i++)
	{
		DriftTracker& t = driftTrackers[i];
		if (t.blocks < 2)
		{
			reply.catf(" over %" PRIu32 "ms n/a", DriftIntervals[i]);
		}
		else
		{
			reply.catf(" over %" PRIu32 "ms %.1f (%" PRIu32 " blocks)", DriftIntervals[i], (double)(t.maxMean - t.minMean), t.blocks);
		}
		t.Reset();											// each report covers the period since the previous one
	}

#if SUPPORT_LOADCELL_FFT
	if (samplesTaken < NumSamples)
	{
		reply.lcatf("  fast spectrum: only %" PRIu32 " of %u samples", samplesTaken, (unsigned int)NumSamples);
	}
	else
	{
		AppendSpectrum(fastSamples, fastWriteIndex, measuredRate, FastBandLimits, ARRAY_SIZE(FastBandLimits), "fast", reply);
	}
#endif
}

#if SUPPORT_LOADCELL_FFT

// Append the slow spectrum. It gets its own diagnostics part because both spectra together overflow one reply string
void LoadCellDiagnostics::AppendSlowSpectrum(const StringRef& reply) noexcept
{
	if (!initialised || samplesTaken == 0)
	{
		return;
	}

	const uint32_t elapsed = millis() - whenCaptureStarted;
	const float measuredRate = (elapsed == 0) ? 0.0f : (float)samplesTaken * 1000.0f / (float)elapsed;

	if (slowSamplesTaken < NumSamples)
	{
		reply.lcatf("  slow spectrum: only %" PRIu32 " of %u samples, needs %" PRIu32 "s",
						slowSamplesTaken, (unsigned int)NumSamples,
						(uint32_t)((float)((NumSamples - slowSamplesTaken) * SlowDecimation) / measuredRate));
	}
	else
	{
		AppendSpectrum(slowSamples, slowWriteIndex, measuredRate / (float)SlowDecimation, SlowBandLimits, ARRAY_SIZE(SlowBandLimits), "slow", reply);
	}
}

#endif

#endif

// End
