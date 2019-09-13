/*
 * Version.h
 *
 *  Created on: 1 Sep 2019
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

constexpr const char* FirmwareVersion = "3.0alpha 2019-09-13b2";

#if defined(SAME51)
constexpr const char* BoardTypeName = "EXP3HC";
#elif defined(SAMC21)
constexpr const char* BoardTypeName = "TOOL1LC";
#else
# error Unsupported processor
#endif

#endif /* SRC_VERSION_H_ */
