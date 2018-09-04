/**
 * \file
 *
 * \brief Top header file for SAMC20
 *
 * Copyright (c) 2017 Atmel Corporation,
 *                    a wholly owned subsidiary of Microchip Technology Inc.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the Licence at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \asf_license_stop
 *
 */

#ifndef _SAMC20_
#define _SAMC20_

/**
 * \defgroup SAMC20_definitions SAMC20 Device Definitions
 * \brief SAMC20 CMSIS Definitions.
 */

#if   defined(__SAMC20N17A__) || defined(__ATSAMC20N17A__)
  #include "samc20n17a.h"
#elif defined(__SAMC20N18A__) || defined(__ATSAMC20N18A__)
  #include "samc20n18a.h"
#else
  #error Library does not support the specified device.
#endif

#endif /* _SAMC20_ */
