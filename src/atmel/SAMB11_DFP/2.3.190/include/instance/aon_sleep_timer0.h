/**
 * \file
 *
 * \brief Instance description for AON_SLEEP_TIMER0
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

/* file generated from device description version  */
#ifndef _SAMB11_AON_SLEEP_TIMER0_INSTANCE_H_
#define _SAMB11_AON_SLEEP_TIMER0_INSTANCE_H_

/* ========== Register definition for AON_SLEEP_TIMER0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_AON_SLEEP_TIMER0_CONTROL (0x4000D000) /**< (AON_SLEEP_TIMER0) Control for the Always On Sleep Timer */
#define REG_AON_SLEEP_TIMER0_SINGLE_COUNT_DURATION (0x4000D004) /**< (AON_SLEEP_TIMER0) Count for the single count AND reload */
#define REG_AON_SLEEP_TIMER0_CURRENT_COUNT_VALUE (0x4000D00C) /**< (AON_SLEEP_TIMER0) Current count of the sleep timer */

#else

#define REG_AON_SLEEP_TIMER0_CONTROL (*(__IO uint32_t*)0x4000D000U) /**< (AON_SLEEP_TIMER0) Control for the Always On Sleep Timer */
#define REG_AON_SLEEP_TIMER0_SINGLE_COUNT_DURATION (*(__IO uint32_t*)0x4000D004U) /**< (AON_SLEEP_TIMER0) Count for the single count AND reload */
#define REG_AON_SLEEP_TIMER0_CURRENT_COUNT_VALUE (*(__I  uint32_t*)0x4000D00CU) /**< (AON_SLEEP_TIMER0) Current count of the sleep timer */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_AON_SLEEP_TIMER0_INSTANCE_ */
