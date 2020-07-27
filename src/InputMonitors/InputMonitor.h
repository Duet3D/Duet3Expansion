/*
 * InputMonitor.h
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_INPUTMONITOR_H_
#define SRC_ENDSTOPS_INPUTMONITOR_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <GCodes/GCodeResult.h>
#include <RTOSIface/RTOSIface.h>

struct CanMessageCreateInputMonitor;
struct CanMessageChangeInputMonitor;
struct CanMessageInputChanged;

class InputMonitor
{
public:
	InputMonitor() { }

	static void Init();

	static GCodeResult Create(const CanMessageCreateInputMonitor& msg, size_t dataLength, const StringRef& reply, uint8_t& extra);
	static GCodeResult Change(const CanMessageChangeInputMonitor& msg, const StringRef& reply, uint8_t& extra);

	static uint32_t AddStateChanges(CanMessageInputChanged *msg);

	static void CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept;
	static void CommonAnalogPortInterrupt(CallbackParameter cbp, uint16_t reading) noexcept;

private:
	bool Activate();
	void Deactivate();
	void DigitalInterrupt();
	void AnalogInterrupt(uint16_t reading);

	static bool Delete(uint16_t handle);
	static ReadLockedPointer<InputMonitor> Find(uint16_t handle);

	InputMonitor *next;
	IoPort port;
	uint32_t whenLastSent;
	uint16_t handle;
	uint16_t minInterval;
	uint16_t threshold;
	bool active;
	volatile bool state;
	volatile bool sendDue;

	static InputMonitor *monitorsList;
	static InputMonitor *freeList;

	static ReadWriteLock listLock;
};

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
