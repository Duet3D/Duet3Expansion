/**
 * \file
 *
 * \brief Instance description for LP_CLK_CAL_REGS0
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
#ifndef _SAMB11_LP_CLK_CAL_REGS0_INSTANCE_H_
#define _SAMB11_LP_CLK_CAL_REGS0_INSTANCE_H_

/* ========== Register definition for LP_CLK_CAL_REGS0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_LP_CLK_CAL_REGS0_CONFIG_REG (0x4000C000) /**< (LP_CLK_CAL_REGS0) Configuration of the calibration clocks and the enable of calibration */
#define REG_LP_CLK_CAL_REGS0_CALIB_OSC_COUNT_REG (0x4000C004) /**< (LP_CLK_CAL_REGS0) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31) */
#define REG_LP_CLK_CAL_REGS0_CALIB_RTC_COUNT_REG (0x4000C008) /**< (LP_CLK_CAL_REGS0) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31) */
#define REG_LP_CLK_CAL_REGS0_CALIB_STATUS_REG (0x4000C00C) /**< (LP_CLK_CAL_REGS0) Calibration Status Register */

#else

#define REG_LP_CLK_CAL_REGS0_CONFIG_REG (*(__IO uint16_t*)0x4000C000U) /**< (LP_CLK_CAL_REGS0) Configuration of the calibration clocks and the enable of calibration */
#define REG_LP_CLK_CAL_REGS0_CALIB_OSC_COUNT_REG (*(__I  uint32_t*)0x4000C004U) /**< (LP_CLK_CAL_REGS0) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31) */
#define REG_LP_CLK_CAL_REGS0_CALIB_RTC_COUNT_REG (*(__I  uint32_t*)0x4000C008U) /**< (LP_CLK_CAL_REGS0) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31) */
#define REG_LP_CLK_CAL_REGS0_CALIB_STATUS_REG (*(__I  uint8_t*)0x4000C00CU) /**< (LP_CLK_CAL_REGS0) Calibration Status Register */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_LP_CLK_CAL_REGS0_INSTANCE_ */
