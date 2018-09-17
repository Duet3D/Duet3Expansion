/*
 * Can.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CAN_H_
#define SRC_CAN_CAN_H_

#include <RepRapFirmware.h>

struct CanMovementMessage;

namespace CanManager
{
	const CanMovementMessage& GetCanMove();
	void MoveStoppedByZProbe();
}

#endif /* SRC_CAN_CAN_H_ */
