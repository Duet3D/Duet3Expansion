/**
 * \file
 *
 * \brief Top level header file
 *
 * Copyright (c) 2017 Atmel Corporation, a wholly owned subsidiary of Microchip Technology Inc.
 *
 * \license_start
 *
 * \page License
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \license_stop
 *
 */

#ifndef _SAM_
#define _SAM_

#if   defined(__SAMD11C14A__) || defined(__ATSAMD11C14A__)
  #include "samd11c14a.h"
#elif defined(__SAMD11D14AM__) || defined(__ATSAMD11D14AM__)
  #include "samd11d14am.h"
#elif defined(__SAMD11D14AS__) || defined(__ATSAMD11D14AS__)
  #include "samd11d14as.h"
#elif defined(__SAMD11D14AU__) || defined(__ATSAMD11D14AU__)
  #include "samd11d14au.h"
#else
  #error Library does not support the specified device
#endif

#endif /* _SAM_ */

