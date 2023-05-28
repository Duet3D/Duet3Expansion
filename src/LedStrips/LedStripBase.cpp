/*
 * LedStripBase.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include "LedStripBase.h"

#if SUPPORT_LED_STRIPS

const char *_ecv_array LedStripBase::GetTypeText() const noexcept
{
	return type.ToString();
}

#endif

// End
