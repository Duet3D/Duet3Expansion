/*
 * Can.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANSLAVEINTERFACE_H_
#define SRC_CAN_CANSLAVEINTERFACE_H_

#include "RepRapFirmware.h"

struct CanMessageMovement;

namespace CanSlaveInterface
{
	void Init();
	bool GetCanMove(CanMessageMovement& move);
	void MoveStoppedByZProbe();
}

#endif /* SRC_CAN_CANSLAVEINTERFACE_H_ */
