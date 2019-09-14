/*
 * Version.h
 *
 *  Created on: 1 Sep 2019
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

constexpr const char* FirmwareVersion = "3.0alpha 2019-09-14b1";

#if defined(EXPANSION_1_V09)
constexpr const char* BoardTypeName = "EXP3HC";
#elif defined(TOOL_1_V01)
constexpr const char* BoardTypeName = "TOOL1LC";
#else
# error Unsupported processor
#endif

#endif /* SRC_VERSION_H_ */
