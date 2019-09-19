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

class InputMonitor
{
public:
	InputMonitor() { }

	static GCodeResult Create(const CanMessageCreateInputMonitor& msg, size_t dataLength, const StringRef& reply, uint8_t& extra);
	static GCodeResult Change(const CanMessageChangeInputMonitor& msg, const StringRef& reply, uint8_t& extra);

private:
	void Activate();
	void Deactivate();

	static bool Delete(uint16_t handle);

	InputMonitor *next;
	IoPort port;
	uint16_t handle;
	uint16_t minInterval;
	uint16_t threshold;
	bool state;

	static InputMonitor *monitorsList;
	static InputMonitor *freeList;

	static ReadWriteLock listLock;
};

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
