/**
 * \file
 *
 * \brief Top header file for SAMDA1
 *
 * Copyright (c) 2016 Atmel Corporation,
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

#ifndef _SAMDA1_
#define _SAMDA1_

/**
 * \defgroup SAMDA1_definitions SAMDA1 Device Definitions
 * \brief SAMDA1 CMSIS Definitions.
 */

#if   defined(__SAMDA1E14A__) || defined(__ATSAMDA1E14A__)
  #include "samda1e14a.h"
#elif defined(__SAMDA1E15A__) || defined(__ATSAMDA1E15A__)
  #include "samda1e15a.h"
#elif defined(__SAMDA1E16A__) || defined(__ATSAMDA1E16A__)
  #include "samda1e16a.h"
#elif defined(__SAMDA1G14A__) || defined(__ATSAMDA1G14A__)
  #include "samda1g14a.h"
#elif defined(__SAMDA1G15A__) || defined(__ATSAMDA1G15A__)
  #include "samda1g15a.h"
#elif defined(__SAMDA1G16A__) || defined(__ATSAMDA1G16A__)
  #include "samda1g16a.h"
#elif defined(__SAMDA1J14A__) || defined(__ATSAMDA1J14A__)
  #include "samda1j14a.h"
#elif defined(__SAMDA1J15A__) || defined(__ATSAMDA1J15A__)
  #include "samda1j15a.h"
#elif defined(__SAMDA1J16A__) || defined(__ATSAMDA1J16A__)
  #include "samda1j16a.h"
#else
  #error Library does not support the specified device.
#endif

#endif /* _SAMDA1_ */
