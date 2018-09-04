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

#if   defined(__SAMC20E15A__) || defined(__ATSAMC20E15A__)
  #include "samc20e15a.h"
#elif defined(__SAMC20E16A__) || defined(__ATSAMC20E16A__)
  #include "samc20e16a.h"
#elif defined(__SAMC20E17A__) || defined(__ATSAMC20E17A__)
  #include "samc20e17a.h"
#elif defined(__SAMC20E18A__) || defined(__ATSAMC20E18A__)
  #include "samc20e18a.h"
#elif defined(__SAMC20G15A__) || defined(__ATSAMC20G15A__)
  #include "samc20g15a.h"
#elif defined(__SAMC20G16A__) || defined(__ATSAMC20G16A__)
  #include "samc20g16a.h"
#elif defined(__SAMC20G17A__) || defined(__ATSAMC20G17A__)
  #include "samc20g17a.h"
#elif defined(__SAMC20G18A__) || defined(__ATSAMC20G18A__)
  #include "samc20g18a.h"
#elif defined(__SAMC20J15A__) || defined(__ATSAMC20J15A__)
  #include "samc20j15a.h"
#elif defined(__SAMC20J16A__) || defined(__ATSAMC20J16A__)
  #include "samc20j16a.h"
#elif defined(__SAMC20J17A__) || defined(__ATSAMC20J17A__)
  #include "samc20j17a.h"
#elif defined(__SAMC20J17AU__) || defined(__ATSAMC20J17AU__)
  #include "samc20j17au.h"
#elif defined(__SAMC20J18A__) || defined(__ATSAMC20J18A__)
  #include "samc20j18a.h"
#elif defined(__SAMC20J18AU__) || defined(__ATSAMC20J18AU__)
  #include "samc20j18au.h"
#else
  #error Library does not support the specified device.
#endif

#endif /* _SAMC20_ */
