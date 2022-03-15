/*
 * Version.h
 *
 *  Created on: 1 Sep 2019
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#define VERSION		"3.4.0"

#include <General/IsoDate.h>

#if 0
// Use this for official releases
# define TIME_SUFFIX
#else
// Use this for internal builds
# define TIME_SUFFIX		" " __TIME__
#endif

#endif /* SRC_VERSION_H_ */
