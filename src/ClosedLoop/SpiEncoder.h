/*
 * SpiEncoder.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_SPIENCODER_H_
#define SRC_CLOSEDLOOP_SPIENCODER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include <Hardware/SharedSpiClient.h>

class SpiEncoder
{
public:
	SpiEncoder(uint32_t clockFreq, SpiMode m, bool polarity, Pin p_csPin) noexcept;
	virtual ~SpiEncoder() { }

	virtual EncoderType GetType() const noexcept = 0;
	virtual void Enable() noexcept = 0;
	virtual void Disable() noexcept = 0;
	virtual int32_t GetReading() noexcept = 0;
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	static void Init() noexcept;

protected:
	static void EnableSpi() noexcept;
	static void DisableSpi() noexcept;

	static SharedSpiDevice *encoderSpi;

	SharedSpiClient spi;
	Pin csPin;
};

#endif

#endif /* SRC_CLOSEDLOOP_SPIENCODER_H_ */
