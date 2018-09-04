/**
 * \file
 *
 * \brief Component description for LP_CLK_CAL_REGS
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
#ifndef _SAMB11_LP_CLK_CAL_REGS_COMPONENT_H_
#define _SAMB11_LP_CLK_CAL_REGS_COMPONENT_H_
#define _SAMB11_LP_CLK_CAL_REGS_COMPONENT_         /**< \deprecated  Backward compatibility for ASF */

/** \addtogroup SAMB_SAMB11 Low Power Clock Calibration Registers
 *  @{
 */
/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR LP_CLK_CAL_REGS */
/* ========================================================================== */
#ifndef COMPONENT_TYPEDEF_STYLE
  #define COMPONENT_TYPEDEF_STYLE 'N'  /**< Defines default style of typedefs for the component header files ('R' = RFO, 'N' = NTO)*/
#endif

#define LP_CLK_CAL_REGS_LPCCR1234                  /**< (LP_CLK_CAL_REGS) Module ID */
#define REV_LP_CLK_CAL_REGS 0x100                  /**< (LP_CLK_CAL_REGS) Module revision */

/* -------- LP_CLK_CAL_REGS_CONFIG_REG : (LP_CLK_CAL_REGS Offset: 0x00) (R/W 16) Configuration of the calibration clocks and the enable of calibration -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint16_t NUMBER_CALIB_CLKS:4;       /**< bit:   0..3  Multiplier value for the number of calibration clocks (1 shifted left by _value_) */
    uint16_t :2;                        /**< bit:   4..5  Reserved */
    uint16_t START_RTC_CALIB:1;         /**< bit:      6  If set, this will start the calibration of the RTC (32.768 kHz) clock */
    uint16_t START_OSC_CALIB:1;         /**< bit:      7  If set, this will start the calibration of the OSC (32 kHz) clock */
    uint16_t :6;                        /**< bit:  8..13  Reserved */
    uint16_t IRQ_CONTROL:2;             /**< bit: 14..15  Interrupt Control                        */
  } bit;                                /**< Structure used for bit  access */
  uint16_t reg;                         /**< Type used for register access */
} LP_CLK_CAL_REGS_CONFIG_REG_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define LP_CLK_CAL_REGS_CONFIG_REG_OFFSET   (0x00)                                        /**<  (LP_CLK_CAL_REGS_CONFIG_REG) Configuration of the calibration clocks and the enable of calibration  Offset */
#define LP_CLK_CAL_REGS_CONFIG_REG_RESETVALUE _U_(0x00)                                     /**<  (LP_CLK_CAL_REGS_CONFIG_REG) Configuration of the calibration clocks and the enable of calibration  Reset Value */

#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos 0                                              /**< (LP_CLK_CAL_REGS_CONFIG_REG) Multiplier value for the number of calibration clocks (1 shifted left by _value_) Position */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Msk (_U_(0xF) << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) Multiplier value for the number of calibration clocks (1 shifted left by _value_) Mask */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS(value) (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Msk & ((value) << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos))
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_0_Val _U_(0x0)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 1 clock  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_1_Val _U_(0x1)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 2 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_2_Val _U_(0x2)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 4 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_3_Val _U_(0x3)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 8 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_4_Val _U_(0x4)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 16 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_5_Val _U_(0x5)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 32 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_6_Val _U_(0x6)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 64 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_7_Val _U_(0x7)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 128 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_8_Val _U_(0x8)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 256 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_9_Val _U_(0x9)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 512 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_10_Val _U_(0xA)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 1024 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_11_Val _U_(0xB)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 2048 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_12_Val _U_(0xC)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 4096 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_13_Val _U_(0xD)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 8192 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_14_Val _U_(0xE)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 16384 clocks  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_15_Val _U_(0xF)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) 32768 clocks  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_0 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_0_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 1 clock Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_1 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_1_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 2 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_2 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_2_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 4 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_3 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_3_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 8 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_4 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_4_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 16 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_5 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_5_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 32 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_6 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_6_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 64 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_7 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_7_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 128 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_8 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_8_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 256 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_9 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_9_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 512 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_10 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_10_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 1024 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_11 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_11_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 2048 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_12 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_12_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 4096 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_13 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_13_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 8192 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_14 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_14_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 16384 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_15 (LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_15_Val << LP_CLK_CAL_REGS_CONFIG_REG_NUMBER_CALIB_CLKS_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) 32768 clocks Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB_Pos 6                                              /**< (LP_CLK_CAL_REGS_CONFIG_REG) If set, this will start the calibration of the RTC (32.768 kHz) clock Position */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) If set, this will start the calibration of the RTC (32.768 kHz) clock Mask */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CONFIG_REG_START_RTC_CALIB_Msk instead */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB_Pos 7                                              /**< (LP_CLK_CAL_REGS_CONFIG_REG) If set, this will start the calibration of the OSC (32 kHz) clock Position */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) If set, this will start the calibration of the OSC (32 kHz) clock Mask */
#define LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CONFIG_REG_START_OSC_CALIB_Msk instead */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos 14                                             /**< (LP_CLK_CAL_REGS_CONFIG_REG) Interrupt Control Position */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Msk (_U_(0x3) << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) Interrupt Control Mask */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL(value) (LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Msk & ((value) << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos))
#define   LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_0_Val _U_(0x0)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC calibration done  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_1_Val _U_(0x1)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when RTC calibration done  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_2_Val _U_(0x2)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC or RTC calibration done  */
#define   LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_3_Val _U_(0x3)                                       /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC and RTC calibration done  */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_0 (LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_0_Val << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC calibration done Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_1 (LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_1_Val << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when RTC calibration done Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_2 (LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_2_Val << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC or RTC calibration done Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_3 (LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_3_Val << LP_CLK_CAL_REGS_CONFIG_REG_IRQ_CONTROL_Pos)  /**< (LP_CLK_CAL_REGS_CONFIG_REG) IRQ when OSC and RTC calibration done Position  */
#define LP_CLK_CAL_REGS_CONFIG_REG_MASK     _U_(0xC0CF)                                    /**< \deprecated (LP_CLK_CAL_REGS_CONFIG_REG) Register MASK  (Use LP_CLK_CAL_REGS_CONFIG_REG_Msk instead)  */
#define LP_CLK_CAL_REGS_CONFIG_REG_Msk      _U_(0xC0CF)                                    /**< (LP_CLK_CAL_REGS_CONFIG_REG) Register Mask  */


/* -------- LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG : (LP_CLK_CAL_REGS Offset: 0x04) (R/ 32) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31) -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t CAL_FRAC_COUNT:15;         /**< bit:  0..14  This is the fractional part of the calibration counter */
    uint32_t NO_CAL_IN_PROGRESS:1;      /**< bit:     15  Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) */
    uint32_t CAL_INT_COUNT:15;          /**< bit: 16..30  This is the integer part of the calibration counter */
    uint32_t CAL_DONE:1;                /**< bit:     31  Indicates calibration is done (any write clears this to a 0) */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_OFFSET (0x04)                                        /**<  (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31)  Offset */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_RESETVALUE _U_(0x8000)                                   /**<  (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31)  Reset Value */

#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT_Pos 0                                              /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) This is the fractional part of the calibration counter Position */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT_Msk (_U_(0x7FFF) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) This is the fractional part of the calibration counter Mask */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT(value) (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT_Msk & ((value) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_FRAC_COUNT_Pos))
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS_Pos 15                                             /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) Position */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) Mask */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT_Pos 16                                             /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) This is the integer part of the calibration counter Position */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT_Msk (_U_(0x7FFF) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) This is the integer part of the calibration counter Mask */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT(value) (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT_Msk & ((value) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_INT_COUNT_Pos))
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE_Pos 31                                             /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Indicates calibration is done (any write clears this to a 0) Position */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Indicates calibration is done (any write clears this to a 0) Mask */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_CAL_DONE_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_MASK _U_(0xFFFFFFFF)                                /**< \deprecated (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Register MASK  (Use LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_Msk instead)  */
#define LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_Msk _U_(0xFFFFFFFF)                                /**< (LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG) Register Mask  */


/* -------- LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG : (LP_CLK_CAL_REGS Offset: 0x08) (R/ 32) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31) -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t CAL_FRAC_COUNT:15;         /**< bit:  0..14  This is the fractional part of the calibration counter */
    uint32_t NO_CAL_IN_PROGRESS:1;      /**< bit:     15  Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) */
    uint32_t CAL_INT_COUNT:15;          /**< bit: 16..30  This is the integer part of the calibration counter */
    uint32_t CAL_DONE:1;                /**< bit:     31  Indicates calibration is done (any write clears this to a 0) */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_OFFSET (0x08)                                        /**<  (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31)  Offset */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_RESETVALUE _U_(0x8000)                                   /**<  (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31)  Reset Value */

#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT_Pos 0                                              /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) This is the fractional part of the calibration counter Position */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT_Msk (_U_(0x7FFF) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) This is the fractional part of the calibration counter Mask */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT(value) (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT_Msk & ((value) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_FRAC_COUNT_Pos))
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS_Pos 15                                             /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) Position */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Indicates no calibration is running (any write sets this to a 1, 0 when calibration done) Mask */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_NO_CAL_IN_PROGRESS_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT_Pos 16                                             /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) This is the integer part of the calibration counter Position */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT_Msk (_U_(0x7FFF) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) This is the integer part of the calibration counter Mask */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT(value) (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT_Msk & ((value) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_INT_COUNT_Pos))
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE_Pos 31                                             /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Indicates calibration is done (any write clears this to a 0) Position */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Indicates calibration is done (any write clears this to a 0) Mask */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_CAL_DONE_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_MASK _U_(0xFFFFFFFF)                                /**< \deprecated (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Register MASK  (Use LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_Msk instead)  */
#define LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_Msk _U_(0xFFFFFFFF)                                /**< (LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG) Register Mask  */


/* -------- LP_CLK_CAL_REGS_CALIB_STATUS_REG : (LP_CLK_CAL_REGS Offset: 0x0c) (R/ 8) Calibration Status Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CALIBRATION_OSC_DONE:1;    /**< bit:      0  This is the done flag for calibrating the osc clock */
    uint8_t  CALIBRATION_RTC_DONE:1;    /**< bit:      1  This is the done flag for calibrating the rtc clock */
    uint8_t  :2;                        /**< bit:   2..3  Reserved */
    uint8_t  OSC_CAL_RUNNING:1;         /**< bit:      4  OSC clock calibration in progress        */
    uint8_t  RTC_CAL_RUNNING:1;         /**< bit:      5  RTC clock calibration in progress        */
    uint8_t  :2;                        /**< bit:   6..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} LP_CLK_CAL_REGS_CALIB_STATUS_REG_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_OFFSET (0x0C)                                        /**<  (LP_CLK_CAL_REGS_CALIB_STATUS_REG) Calibration Status Register  Offset */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_RESETVALUE _U_(0x00)                                     /**<  (LP_CLK_CAL_REGS_CALIB_STATUS_REG) Calibration Status Register  Reset Value */

#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE_Pos 0                                              /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) This is the done flag for calibrating the osc clock Position */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) This is the done flag for calibrating the osc clock Mask */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_OSC_DONE_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE_Pos 1                                              /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) This is the done flag for calibrating the rtc clock Position */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) This is the done flag for calibrating the rtc clock Mask */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_STATUS_REG_CALIBRATION_RTC_DONE_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING_Pos 4                                              /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) OSC clock calibration in progress Position */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) OSC clock calibration in progress Mask */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_STATUS_REG_OSC_CAL_RUNNING_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING_Pos 5                                              /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) RTC clock calibration in progress Position */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING_Msk (_U_(0x1) << LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING_Pos)  /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) RTC clock calibration in progress Mask */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING_Msk  /**< \deprecated Old style mask definition for 1 bit bitfield. Use LP_CLK_CAL_REGS_CALIB_STATUS_REG_RTC_CAL_RUNNING_Msk instead */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_MASK _U_(0x33)                                      /**< \deprecated (LP_CLK_CAL_REGS_CALIB_STATUS_REG) Register MASK  (Use LP_CLK_CAL_REGS_CALIB_STATUS_REG_Msk instead)  */
#define LP_CLK_CAL_REGS_CALIB_STATUS_REG_Msk _U_(0x33)                                      /**< (LP_CLK_CAL_REGS_CALIB_STATUS_REG) Register Mask  */


#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if COMPONENT_TYPEDEF_STYLE == 'R'
/** \brief LP_CLK_CAL_REGS hardware registers */
typedef struct {  /* Low Power Clock Calibration Registers */
  RwReg16 CONFIG_REG;     /**< (LP_CLK_CAL_REGS Offset: 0x00) Configuration of the calibration clocks and the enable of calibration */
  RoReg8  Reserved1[0x2];
  RoReg   CALIB_OSC_COUNT_REG; /**< (LP_CLK_CAL_REGS Offset: 0x04) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31) */
  RoReg   CALIB_RTC_COUNT_REG; /**< (LP_CLK_CAL_REGS Offset: 0x08) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31) */
  RoReg8  CALIB_STATUS_REG; /**< (LP_CLK_CAL_REGS Offset: 0x0C) Calibration Status Register */
} LpClkCalRegs;

#elif COMPONENT_TYPEDEF_STYLE == 'N'
/** \brief LP_CLK_CAL_REGS hardware registers */
typedef struct {  /* Low Power Clock Calibration Registers */
  __IO LP_CLK_CAL_REGS_CONFIG_REG_Type CONFIG_REG;     /**< Offset: 0x00 (R/W  16) Configuration of the calibration clocks and the enable of calibration */
  __I  uint8_t                        Reserved1[2];
  __I  LP_CLK_CAL_REGS_CALIB_OSC_COUNT_REG_Type CALIB_OSC_COUNT_REG; /**< Offset: 0x04 (R/   32) Calibration OSC Count Register (Any write sets bit 15 and clears bit 31) */
  __I  LP_CLK_CAL_REGS_CALIB_RTC_COUNT_REG_Type CALIB_RTC_COUNT_REG; /**< Offset: 0x08 (R/   32) Calibration RTC Count Register (Any write sets bit 15 and clears bit 31) */
  __I  LP_CLK_CAL_REGS_CALIB_STATUS_REG_Type CALIB_STATUS_REG; /**< Offset: 0x0C (R/    8) Calibration Status Register */
} LpClkCalRegs;

#else /* COMPONENT_TYPEDEF_STYLE */
#error Unknown component typedef style
#endif /* COMPONENT_TYPEDEF_STYLE */

#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of Low Power Clock Calibration Registers */

#endif /* _SAMB11_LP_CLK_CAL_REGS_COMPONENT_H_ */
