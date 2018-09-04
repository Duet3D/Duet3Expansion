/**
 * \file
 *
 * \brief Top level header file
 *
 * Copyright (c) 2018 Atmel Corporation, a wholly owned subsidiary of Microchip Technology Inc.
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

#if   defined(__SAME70J19__) || defined(__ATSAME70J19__)
  #include "same70j19.h"
#elif defined(__SAME70J20__) || defined(__ATSAME70J20__)
  #include "same70j20.h"
#elif defined(__SAME70J21__) || defined(__ATSAME70J21__)
  #include "same70j21.h"
#elif defined(__SAME70N19__) || defined(__ATSAME70N19__)
  #include "same70n19.h"
#elif defined(__SAME70N20__) || defined(__ATSAME70N20__)
  #include "same70n20.h"
#elif defined(__SAME70N21__) || defined(__ATSAME70N21__)
  #include "same70n21.h"
#elif defined(__SAME70Q19__) || defined(__ATSAME70Q19__)
  #include "same70q19.h"
#elif defined(__SAME70Q20__) || defined(__ATSAME70Q20__)
  #include "same70q20.h"
#elif defined(__SAME70Q21__) || defined(__ATSAME70Q21__)
  #include "same70q21.h"
#else
  #error Library does not support the specified device
#endif

#endif /* _SAM_ */

