/*
 * Trigonometry.h
 *
 *  Created on: 12 Jul 2021
 *      Author: Louis
 */

#ifndef SRC_CLOSEDLOOP_TRIGONOMETRY_H_
# define SRC_CLOSEDLOOP_TRIGONOMETRY_H_

#include <array>
#include <math.h>
#include <RepRapFirmware.h>

namespace Trigonometry
{
	float FastSin(uint16_t) noexcept;
	float FastCos(uint16_t) noexcept;
}

#endif /* SRC_CLOSEDLOOP_TRIGONOMETRY_H_ */
