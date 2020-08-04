/*
 * ClockGen.cpp
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#include "ClockGen.h"

#if SUPPORT_CLOSED_LOOP

#ifdef SAMC21
# include <hri_gclk_c21.h>
#endif

// Initialise the external 12MHz or 16MHz clock that feeds the TMC2160 and the Attiny
void ClockGen::Init()
{
	// Currently we program the DPLL to generate 48MHz output, so to get 16MHz we divide by 3 and set the Improve Duty Cycle bit
	// We could instead program the DPLL to generate 96MHz and divide it by an extra factor of 2 in the other GCLKs
	// Or we could divide by 4 and be content with 12MHz.
	hri_gclk_write_GENCTRL_reg(
	    GCLK,
		ClockGenGclkNumber,
	    	  GCLK_GENCTRL_DIV(3)
			| (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
	        | (0 << GCLK_GENCTRL_DIVSEL_Pos)
			| (1 << GCLK_GENCTRL_OE_Pos)
	        | (0 << GCLK_GENCTRL_OOV_Pos)
			| (1 << GCLK_GENCTRL_IDC_Pos)
	        | GCLK_GENCTRL_GENEN
			| GCLK_GENCTRL_SRC_DPLL96M
		);
	gpio_set_pin_function(ClockGenPin, ClockGenPinPeriphMode);
}

#endif

// End
