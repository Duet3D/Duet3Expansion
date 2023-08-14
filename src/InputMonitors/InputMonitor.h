/*
 * InputMonitor.h
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_INPUTMONITOR_H_
#define SRC_ENDSTOPS_INPUTMONITOR_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <Hardware/IoPorts.h>
#include <RTOSIface/RTOSIface.h>

struct CanMessageCreateInputMonitorNew;
struct CanMessageChangeInputMonitorNew;
struct CanMessageInputChanged;
class CanMessageBuffer;

class InputMonitor
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	InputMonitor() noexcept { }

	static void Init() noexcept;
	static void Spin() noexcept;

	static GCodeResult Create(const CanMessageCreateInputMonitorNew& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept;
	static GCodeResult Change(const CanMessageChangeInputMonitorNew& msg, const StringRef& reply, uint8_t& extra) noexcept;

	static uint32_t AddStateChanges(CanMessageInputChanged *msg) noexcept;
	static void ReadInputs(CanMessageBuffer *buf) noexcept;

	static void CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept;
	static void CommonAnalogPortInterrupt(CallbackParameter cbp, uint16_t reading) noexcept;

private:
	bool Activate() noexcept;
	void Deactivate() noexcept;
	void DigitalInterrupt() noexcept;
	void AnalogInterrupt(uint16_t reading) noexcept;
	uint32_t GetAnalogValue() const noexcept;
	GCodeResult SetDriveLevel(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept;

	static bool Delete(uint16_t hndl) noexcept;
	static ReadLockedPointer<InputMonitor> Find(uint16_t hndl) noexcept;

	InputMonitor *next;
	IoPort port;
	uint32_t whenLastSent;
	uint32_t threshold;
	uint16_t handle;
	uint16_t minInterval;
	bool active;
	volatile bool state;
	volatile bool sendDue;

	static InputMonitor * volatile monitorsList;
	static InputMonitor * volatile freeList;

	static ReadWriteLock listLock;
};

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
