/*
 * ZProbe.h
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#ifndef SRC_ZPROBE_H_
#define SRC_ZPROBE_H_

#include "Endstop.h"
#include "GCodes/GCodeResult.h"

struct CanMessageConfigureZProbe;
struct CanMessageCreateZProbe;
struct CanMessageGetZProbePinNames;
struct CanMessageSetProbing;

class ZProbe : public EndstopOrZProbe
{
public:
	ZProbe(unsigned int num, ZProbeType p_type);

	virtual void SetIREmitter(bool on) const = 0;
	virtual uint16_t GetRawReading() const = 0;
	virtual void SetProbing(bool isProbing) const = 0;
	virtual GCodeResult AppendPinNames(const StringRef& str) const = 0;
	virtual GCodeResult Configure(const CanMessageConfigureZProbe& msg, const StringRef& reply, uint8_t& extra);
	virtual GCodeResult ConfigurePorts(const CanMessageCreateZProbe& msg, size_t dataLength, const StringRef& reply) { return GCodeResult::ok; }
	virtual GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply);

	EndStopHit Stopped() const override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;

	ZProbeType GetProbeType() const { return type; }
	int GetAdcValue() const { return adcValue; }

	int GetReading() const;

protected:
	uint8_t number;
	ZProbeType type;
	int8_t sensor;					// the sensor number used for temperature calibration
	int16_t adcValue;				// the target ADC value, after inversion if enabled
};

// MotorStall Z probes have no port, also in a CAN environment the local and remote proxy versions are the same
class MotorStallZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) { return FreelistManager::Allocate<MotorStallZProbe>(); }
	void operator delete(void* p) { FreelistManager::Release<MotorStallZProbe>(p); }

	MotorStallZProbe(unsigned int num) : ZProbe(num, ZProbeType::zMotorStall) { }
	~MotorStallZProbe() override { }
	void SetIREmitter(bool on) const override { }
	uint16_t GetRawReading() const override { return 4000; }
	void SetProbing(bool isProbing) const override { }
	GCodeResult AppendPinNames(const StringRef& str) const override { return GCodeResult::ok; }

private:
};

#endif /* SRC_ZPROBE_H_ */
