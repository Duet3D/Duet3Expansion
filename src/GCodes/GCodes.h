/*
 * GCodes.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_GCODES_H_
#define SRC_GCODES_H_

#include "RepRapFirmware.h"

namespace GCodes
{
	void Init();
	void Spin();

	inline bool IsPaused() { return false; }
	inline size_t GetTotalAxes() { return 3; }
	inline size_t GetVisibleAxes() { return 3; }
	inline const char *GetAxisLetters() { return "XYZ"; }
	inline void MoveStoppedByZProbe() {}
}

#endif /* SRC_GCODES_H_ */
