/*
 * Can.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANSLAVEINTERFACE_H_
#define SRC_CAN_CANSLAVEINTERFACE_H_

#include <RepRapFirmware.h>

struct CanMovementMessage;

namespace CanSlaveInterface
{
	void Init();
	bool GetCanMove(CanMovementMessage& move);
	void MoveStoppedByZProbe();
}

#endif /* SRC_CAN_CANSLAVEINTERFACE_H_ */
