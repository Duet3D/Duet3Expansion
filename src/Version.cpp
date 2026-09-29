/*
 * Version.cpp
 *
 *  Created on: 12 Aug 2022
 *      Author: David
 */

#include "Version.h"
#include <General/IsoDate.h>

const char *const DateText = IsoDate;
// Keep firmware date stable across separately-built main/tool binaries to avoid
// false "incompatible software versions" warnings when only build time differs.
const char *const TimeSuffix = "";

// End
