/****************************************************************************************************

RepRapFirmware - Configuration

This is where all machine-independent configuration and other definitions are set up. Nothing that
depends on any particular RepRap, RepRap component, or RepRap controller should go in here. Define
machine-dependent things in Platform.h

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <cstddef>			// for size_t

// String lengths. Keeping the number of distinct lengths small will reduce flash memory usage.
constexpr size_t StringLength20 = 20;
constexpr size_t StringLength50 = 50;
constexpr size_t StringLength100 = 100;
constexpr size_t StringLength256 = 256;
constexpr size_t StringLength500 = 500;
constexpr size_t FormatStringLength = StringLength256;
constexpr size_t MaxMessageLength = StringLength256;

#endif
