/**
 * \file
 *
 * \brief Component description for TIMER
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
#ifndef _SAMB11_TIMER_COMPONENT_H_
#define _SAMB11_TIMER_COMPONENT_H_
#define _SAMB11_TIMER_COMPONENT_         /**< \deprecated  Backward compatibility for ASF */

/** \addtogroup SAMB_SAMB11 ARM General Purpose Timer
 *  @{
 */
/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR TIMER */
/* ========================================================================== */
#ifndef COMPONENT_TYPEDEF_STYLE
  #define COMPONENT_TYPEDEF_STYLE 'N'  /**< Defines default style of typedefs for the component header files ('R' = RFO, 'N' = NTO)*/
#endif

#define TIMER_T11234                     /**< (TIMER) Module ID */
#define REV_TIMER 0x100                  /**< (TIMER) Module revision */

/* -------- TIMER_CTRL : (TIMER Offset: 0x00) (R/W 8) Timer Control -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ENABLE:1;                  /**< bit:      0  Enable Timer Operation                   */
    uint8_t  EXTERNAL_INPUT_ENABLE:1;   /**< bit:      1  Select external input as enable          */
    uint8_t  EXTERNAL_INPUT_CLOCK:1;    /**< bit:      2  Select external input as clock           */
    uint8_t  INTERRUPT_ENABLE:1;        /**< bit:      3  Enable Timer Interrupt                   */
    uint8_t  :4;                        /**< bit:   4..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_CTRL_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_CTRL_OFFSET                   (0x00)                                        /**<  (TIMER_CTRL) Timer Control  Offset */
#define TIMER_CTRL_RESETVALUE               _U_(0x00)                                     /**<  (TIMER_CTRL) Timer Control  Reset Value */

#define TIMER_CTRL_ENABLE_Pos               0                                              /**< (TIMER_CTRL) Enable Timer Operation Position */
#define TIMER_CTRL_ENABLE_Msk               (_U_(0x1) << TIMER_CTRL_ENABLE_Pos)            /**< (TIMER_CTRL) Enable Timer Operation Mask */
#define TIMER_CTRL_ENABLE                   TIMER_CTRL_ENABLE_Msk                          /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_CTRL_ENABLE_Msk instead */
#define TIMER_CTRL_EXTERNAL_INPUT_ENABLE_Pos 1                                              /**< (TIMER_CTRL) Select external input as enable Position */
#define TIMER_CTRL_EXTERNAL_INPUT_ENABLE_Msk (_U_(0x1) << TIMER_CTRL_EXTERNAL_INPUT_ENABLE_Pos)  /**< (TIMER_CTRL) Select external input as enable Mask */
#define TIMER_CTRL_EXTERNAL_INPUT_ENABLE    TIMER_CTRL_EXTERNAL_INPUT_ENABLE_Msk           /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_CTRL_EXTERNAL_INPUT_ENABLE_Msk instead */
#define TIMER_CTRL_EXTERNAL_INPUT_CLOCK_Pos 2                                              /**< (TIMER_CTRL) Select external input as clock Position */
#define TIMER_CTRL_EXTERNAL_INPUT_CLOCK_Msk (_U_(0x1) << TIMER_CTRL_EXTERNAL_INPUT_CLOCK_Pos)  /**< (TIMER_CTRL) Select external input as clock Mask */
#define TIMER_CTRL_EXTERNAL_INPUT_CLOCK     TIMER_CTRL_EXTERNAL_INPUT_CLOCK_Msk            /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_CTRL_EXTERNAL_INPUT_CLOCK_Msk instead */
#define TIMER_CTRL_INTERRUPT_ENABLE_Pos     3                                              /**< (TIMER_CTRL) Enable Timer Interrupt Position */
#define TIMER_CTRL_INTERRUPT_ENABLE_Msk     (_U_(0x1) << TIMER_CTRL_INTERRUPT_ENABLE_Pos)  /**< (TIMER_CTRL) Enable Timer Interrupt Mask */
#define TIMER_CTRL_INTERRUPT_ENABLE         TIMER_CTRL_INTERRUPT_ENABLE_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_CTRL_INTERRUPT_ENABLE_Msk instead */
#define TIMER_CTRL_MASK                     _U_(0x0F)                                      /**< \deprecated (TIMER_CTRL) Register MASK  (Use TIMER_CTRL_Msk instead)  */
#define TIMER_CTRL_Msk                      _U_(0x0F)                                      /**< (TIMER_CTRL) Register Mask  */


/* -------- TIMER_VALUE : (TIMER Offset: 0x04) (R/W 32) Current Value -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t VALUE:32;                  /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} TIMER_VALUE_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_VALUE_OFFSET                  (0x04)                                        /**<  (TIMER_VALUE) Current Value  Offset */
#define TIMER_VALUE_RESETVALUE              _U_(0x00)                                     /**<  (TIMER_VALUE) Current Value  Reset Value */

#define TIMER_VALUE_VALUE_Pos               0                                              /**< (TIMER_VALUE)  Position */
#define TIMER_VALUE_VALUE_Msk               (_U_(0xFFFFFFFF) << TIMER_VALUE_VALUE_Pos)     /**< (TIMER_VALUE)  Mask */
#define TIMER_VALUE_VALUE(value)            (TIMER_VALUE_VALUE_Msk & ((value) << TIMER_VALUE_VALUE_Pos))
#define TIMER_VALUE_MASK                    _U_(0xFFFFFFFF)                                /**< \deprecated (TIMER_VALUE) Register MASK  (Use TIMER_VALUE_Msk instead)  */
#define TIMER_VALUE_Msk                     _U_(0xFFFFFFFF)                                /**< (TIMER_VALUE) Register Mask  */


/* -------- TIMER_RELOAD : (TIMER Offset: 0x08) (R/W 32) Reload Value -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t RELOAD:32;                 /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} TIMER_RELOAD_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_RELOAD_OFFSET                 (0x08)                                        /**<  (TIMER_RELOAD) Reload Value  Offset */
#define TIMER_RELOAD_RESETVALUE             _U_(0x00)                                     /**<  (TIMER_RELOAD) Reload Value  Reset Value */

#define TIMER_RELOAD_RELOAD_Pos             0                                              /**< (TIMER_RELOAD)  Position */
#define TIMER_RELOAD_RELOAD_Msk             (_U_(0xFFFFFFFF) << TIMER_RELOAD_RELOAD_Pos)   /**< (TIMER_RELOAD)  Mask */
#define TIMER_RELOAD_RELOAD(value)          (TIMER_RELOAD_RELOAD_Msk & ((value) << TIMER_RELOAD_RELOAD_Pos))
#define TIMER_RELOAD_MASK                   _U_(0xFFFFFFFF)                                /**< \deprecated (TIMER_RELOAD) Register MASK  (Use TIMER_RELOAD_Msk instead)  */
#define TIMER_RELOAD_Msk                    _U_(0xFFFFFFFF)                                /**< (TIMER_RELOAD) Register Mask  */


/* -------- TIMER_INTSTATUSCLEAR : (TIMER Offset: 0x0c) (R/W 8) Timer Interrupt, write 1 to clear -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  INTSTATUSCLEAR:1;          /**< bit:      0                                           */
    uint8_t  :7;                        /**< bit:   1..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_INTSTATUSCLEAR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_INTSTATUSCLEAR_OFFSET         (0x0C)                                        /**<  (TIMER_INTSTATUSCLEAR) Timer Interrupt, write 1 to clear  Offset */
#define TIMER_INTSTATUSCLEAR_RESETVALUE     _U_(0x00)                                     /**<  (TIMER_INTSTATUSCLEAR) Timer Interrupt, write 1 to clear  Reset Value */

#define TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR_Pos 0                                              /**< (TIMER_INTSTATUSCLEAR)  Position */
#define TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR_Msk (_U_(0x1) << TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR_Pos)  /**< (TIMER_INTSTATUSCLEAR)  Mask */
#define TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR_Msk        /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_INTSTATUSCLEAR_INTSTATUSCLEAR_Msk instead */
#define TIMER_INTSTATUSCLEAR_MASK           _U_(0x01)                                      /**< \deprecated (TIMER_INTSTATUSCLEAR) Register MASK  (Use TIMER_INTSTATUSCLEAR_Msk instead)  */
#define TIMER_INTSTATUSCLEAR_Msk            _U_(0x01)                                      /**< (TIMER_INTSTATUSCLEAR) Register Mask  */


/* -------- TIMER_PID4 : (TIMER Offset: 0xfd0) (R/ 8) Peripheral ID Register 4 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_C_CODE:4;           /**< bit:   0..3  JEP 106 C Code                           */
    uint8_t  BLOCK_COUNT:4;             /**< bit:   4..7  Block Count                              */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID4_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID4_OFFSET                   (0xFD0)                                       /**<  (TIMER_PID4) Peripheral ID Register 4  Offset */
#define TIMER_PID4_RESETVALUE               _U_(0x04)                                     /**<  (TIMER_PID4) Peripheral ID Register 4  Reset Value */

#define TIMER_PID4_JEP106_C_CODE_Pos        0                                              /**< (TIMER_PID4) JEP 106 C Code Position */
#define TIMER_PID4_JEP106_C_CODE_Msk        (_U_(0xF) << TIMER_PID4_JEP106_C_CODE_Pos)     /**< (TIMER_PID4) JEP 106 C Code Mask */
#define TIMER_PID4_JEP106_C_CODE(value)     (TIMER_PID4_JEP106_C_CODE_Msk & ((value) << TIMER_PID4_JEP106_C_CODE_Pos))
#define TIMER_PID4_BLOCK_COUNT_Pos          4                                              /**< (TIMER_PID4) Block Count Position */
#define TIMER_PID4_BLOCK_COUNT_Msk          (_U_(0xF) << TIMER_PID4_BLOCK_COUNT_Pos)       /**< (TIMER_PID4) Block Count Mask */
#define TIMER_PID4_BLOCK_COUNT(value)       (TIMER_PID4_BLOCK_COUNT_Msk & ((value) << TIMER_PID4_BLOCK_COUNT_Pos))
#define TIMER_PID4_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID4) Register MASK  (Use TIMER_PID4_Msk instead)  */
#define TIMER_PID4_Msk                      _U_(0xFF)                                      /**< (TIMER_PID4) Register Mask  */


/* -------- TIMER_PID5 : (TIMER Offset: 0xfd4) (R/ 8) Peripheral ID Register 5 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PID5:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID5_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID5_OFFSET                   (0xFD4)                                       /**<  (TIMER_PID5) Peripheral ID Register 5  Offset */
#define TIMER_PID5_RESETVALUE               _U_(0x00)                                     /**<  (TIMER_PID5) Peripheral ID Register 5  Reset Value */

#define TIMER_PID5_PID5_Pos                 0                                              /**< (TIMER_PID5)  Position */
#define TIMER_PID5_PID5_Msk                 (_U_(0xFF) << TIMER_PID5_PID5_Pos)             /**< (TIMER_PID5)  Mask */
#define TIMER_PID5_PID5(value)              (TIMER_PID5_PID5_Msk & ((value) << TIMER_PID5_PID5_Pos))
#define TIMER_PID5_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID5) Register MASK  (Use TIMER_PID5_Msk instead)  */
#define TIMER_PID5_Msk                      _U_(0xFF)                                      /**< (TIMER_PID5) Register Mask  */


/* -------- TIMER_PID6 : (TIMER Offset: 0xfd8) (R/ 8) Peripheral ID Register 6 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PID6:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID6_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID6_OFFSET                   (0xFD8)                                       /**<  (TIMER_PID6) Peripheral ID Register 6  Offset */
#define TIMER_PID6_RESETVALUE               _U_(0x00)                                     /**<  (TIMER_PID6) Peripheral ID Register 6  Reset Value */

#define TIMER_PID6_PID6_Pos                 0                                              /**< (TIMER_PID6)  Position */
#define TIMER_PID6_PID6_Msk                 (_U_(0xFF) << TIMER_PID6_PID6_Pos)             /**< (TIMER_PID6)  Mask */
#define TIMER_PID6_PID6(value)              (TIMER_PID6_PID6_Msk & ((value) << TIMER_PID6_PID6_Pos))
#define TIMER_PID6_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID6) Register MASK  (Use TIMER_PID6_Msk instead)  */
#define TIMER_PID6_Msk                      _U_(0xFF)                                      /**< (TIMER_PID6) Register Mask  */


/* -------- TIMER_PID7 : (TIMER Offset: 0xfdc) (R/ 8) Peripheral ID Register 7 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PID7:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID7_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID7_OFFSET                   (0xFDC)                                       /**<  (TIMER_PID7) Peripheral ID Register 7  Offset */
#define TIMER_PID7_RESETVALUE               _U_(0x00)                                     /**<  (TIMER_PID7) Peripheral ID Register 7  Reset Value */

#define TIMER_PID7_PID7_Pos                 0                                              /**< (TIMER_PID7)  Position */
#define TIMER_PID7_PID7_Msk                 (_U_(0xFF) << TIMER_PID7_PID7_Pos)             /**< (TIMER_PID7)  Mask */
#define TIMER_PID7_PID7(value)              (TIMER_PID7_PID7_Msk & ((value) << TIMER_PID7_PID7_Pos))
#define TIMER_PID7_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID7) Register MASK  (Use TIMER_PID7_Msk instead)  */
#define TIMER_PID7_Msk                      _U_(0xFF)                                      /**< (TIMER_PID7) Register Mask  */


/* -------- TIMER_PID0 : (TIMER Offset: 0xfe0) (R/ 8) Peripheral ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:8;             /**< bit:   0..7  Part Number Bits 7:0                     */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID0_OFFSET                   (0xFE0)                                       /**<  (TIMER_PID0) Peripheral ID Register 0  Offset */
#define TIMER_PID0_RESETVALUE               _U_(0x22)                                     /**<  (TIMER_PID0) Peripheral ID Register 0  Reset Value */

#define TIMER_PID0_PART_NUMBER_Pos          0                                              /**< (TIMER_PID0) Part Number Bits 7:0 Position */
#define TIMER_PID0_PART_NUMBER_Msk          (_U_(0xFF) << TIMER_PID0_PART_NUMBER_Pos)      /**< (TIMER_PID0) Part Number Bits 7:0 Mask */
#define TIMER_PID0_PART_NUMBER(value)       (TIMER_PID0_PART_NUMBER_Msk & ((value) << TIMER_PID0_PART_NUMBER_Pos))
#define TIMER_PID0_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID0) Register MASK  (Use TIMER_PID0_Msk instead)  */
#define TIMER_PID0_Msk                      _U_(0xFF)                                      /**< (TIMER_PID0) Register Mask  */


/* -------- TIMER_PID1 : (TIMER Offset: 0xfe4) (R/ 8) Peripheral ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:4;             /**< bit:   0..3  Part Number Bits 11:8                    */
    uint8_t  JEP106_ID_3_0:4;           /**< bit:   4..7  JEP106 ID Bits 3:0                       */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID1_OFFSET                   (0xFE4)                                       /**<  (TIMER_PID1) Peripheral ID Register 1  Offset */
#define TIMER_PID1_RESETVALUE               _U_(0xB8)                                     /**<  (TIMER_PID1) Peripheral ID Register 1  Reset Value */

#define TIMER_PID1_PART_NUMBER_Pos          0                                              /**< (TIMER_PID1) Part Number Bits 11:8 Position */
#define TIMER_PID1_PART_NUMBER_Msk          (_U_(0xF) << TIMER_PID1_PART_NUMBER_Pos)       /**< (TIMER_PID1) Part Number Bits 11:8 Mask */
#define TIMER_PID1_PART_NUMBER(value)       (TIMER_PID1_PART_NUMBER_Msk & ((value) << TIMER_PID1_PART_NUMBER_Pos))
#define TIMER_PID1_JEP106_ID_3_0_Pos        4                                              /**< (TIMER_PID1) JEP106 ID Bits 3:0 Position */
#define TIMER_PID1_JEP106_ID_3_0_Msk        (_U_(0xF) << TIMER_PID1_JEP106_ID_3_0_Pos)     /**< (TIMER_PID1) JEP106 ID Bits 3:0 Mask */
#define TIMER_PID1_JEP106_ID_3_0(value)     (TIMER_PID1_JEP106_ID_3_0_Msk & ((value) << TIMER_PID1_JEP106_ID_3_0_Pos))
#define TIMER_PID1_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID1) Register MASK  (Use TIMER_PID1_Msk instead)  */
#define TIMER_PID1_Msk                      _U_(0xFF)                                      /**< (TIMER_PID1) Register Mask  */


/* -------- TIMER_PID2 : (TIMER Offset: 0xfe8) (R/ 8) Peripheral ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_ID_6_4:3;           /**< bit:   0..2  JEP106 ID Bits 6:4                       */
    uint8_t  JEDEC_USED:1;              /**< bit:      3  JEDEC Used                               */
    uint8_t  REVISION:4;                /**< bit:   4..7  Revision                                 */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID2_OFFSET                   (0xFE8)                                       /**<  (TIMER_PID2) Peripheral ID Register 2  Offset */
#define TIMER_PID2_RESETVALUE               _U_(0x1B)                                     /**<  (TIMER_PID2) Peripheral ID Register 2  Reset Value */

#define TIMER_PID2_JEP106_ID_6_4_Pos        0                                              /**< (TIMER_PID2) JEP106 ID Bits 6:4 Position */
#define TIMER_PID2_JEP106_ID_6_4_Msk        (_U_(0x7) << TIMER_PID2_JEP106_ID_6_4_Pos)     /**< (TIMER_PID2) JEP106 ID Bits 6:4 Mask */
#define TIMER_PID2_JEP106_ID_6_4(value)     (TIMER_PID2_JEP106_ID_6_4_Msk & ((value) << TIMER_PID2_JEP106_ID_6_4_Pos))
#define TIMER_PID2_JEDEC_USED_Pos           3                                              /**< (TIMER_PID2) JEDEC Used Position */
#define TIMER_PID2_JEDEC_USED_Msk           (_U_(0x1) << TIMER_PID2_JEDEC_USED_Pos)        /**< (TIMER_PID2) JEDEC Used Mask */
#define TIMER_PID2_JEDEC_USED               TIMER_PID2_JEDEC_USED_Msk                      /**< \deprecated Old style mask definition for 1 bit bitfield. Use TIMER_PID2_JEDEC_USED_Msk instead */
#define TIMER_PID2_REVISION_Pos             4                                              /**< (TIMER_PID2) Revision Position */
#define TIMER_PID2_REVISION_Msk             (_U_(0xF) << TIMER_PID2_REVISION_Pos)          /**< (TIMER_PID2) Revision Mask */
#define TIMER_PID2_REVISION(value)          (TIMER_PID2_REVISION_Msk & ((value) << TIMER_PID2_REVISION_Pos))
#define TIMER_PID2_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID2) Register MASK  (Use TIMER_PID2_Msk instead)  */
#define TIMER_PID2_Msk                      _U_(0xFF)                                      /**< (TIMER_PID2) Register Mask  */


/* -------- TIMER_PID3 : (TIMER Offset: 0xfec) (R/ 8) Peripheral ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CUSTOMER_MOD_NUMBER:4;     /**< bit:   0..3  Customer Modification Number             */
    uint8_t  ECO_REV_NUMBER:4;          /**< bit:   4..7  ECO Revision Number                      */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_PID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_PID3_OFFSET                   (0xFEC)                                       /**<  (TIMER_PID3) Peripheral ID Register 3  Offset */
#define TIMER_PID3_RESETVALUE               _U_(0x00)                                     /**<  (TIMER_PID3) Peripheral ID Register 3  Reset Value */

#define TIMER_PID3_CUSTOMER_MOD_NUMBER_Pos  0                                              /**< (TIMER_PID3) Customer Modification Number Position */
#define TIMER_PID3_CUSTOMER_MOD_NUMBER_Msk  (_U_(0xF) << TIMER_PID3_CUSTOMER_MOD_NUMBER_Pos)  /**< (TIMER_PID3) Customer Modification Number Mask */
#define TIMER_PID3_CUSTOMER_MOD_NUMBER(value) (TIMER_PID3_CUSTOMER_MOD_NUMBER_Msk & ((value) << TIMER_PID3_CUSTOMER_MOD_NUMBER_Pos))
#define TIMER_PID3_ECO_REV_NUMBER_Pos       4                                              /**< (TIMER_PID3) ECO Revision Number Position */
#define TIMER_PID3_ECO_REV_NUMBER_Msk       (_U_(0xF) << TIMER_PID3_ECO_REV_NUMBER_Pos)    /**< (TIMER_PID3) ECO Revision Number Mask */
#define TIMER_PID3_ECO_REV_NUMBER(value)    (TIMER_PID3_ECO_REV_NUMBER_Msk & ((value) << TIMER_PID3_ECO_REV_NUMBER_Pos))
#define TIMER_PID3_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_PID3) Register MASK  (Use TIMER_PID3_Msk instead)  */
#define TIMER_PID3_Msk                      _U_(0xFF)                                      /**< (TIMER_PID3) Register Mask  */


/* -------- TIMER_CID0 : (TIMER Offset: 0xff0) (R/ 8) Component ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CID0:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_CID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_CID0_OFFSET                   (0xFF0)                                       /**<  (TIMER_CID0) Component ID Register 0  Offset */
#define TIMER_CID0_RESETVALUE               _U_(0x0D)                                     /**<  (TIMER_CID0) Component ID Register 0  Reset Value */

#define TIMER_CID0_CID0_Pos                 0                                              /**< (TIMER_CID0)  Position */
#define TIMER_CID0_CID0_Msk                 (_U_(0xFF) << TIMER_CID0_CID0_Pos)             /**< (TIMER_CID0)  Mask */
#define TIMER_CID0_CID0(value)              (TIMER_CID0_CID0_Msk & ((value) << TIMER_CID0_CID0_Pos))
#define TIMER_CID0_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_CID0) Register MASK  (Use TIMER_CID0_Msk instead)  */
#define TIMER_CID0_Msk                      _U_(0xFF)                                      /**< (TIMER_CID0) Register Mask  */


/* -------- TIMER_CID1 : (TIMER Offset: 0xff4) (R/ 8) Component ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CID1:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_CID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_CID1_OFFSET                   (0xFF4)                                       /**<  (TIMER_CID1) Component ID Register 1  Offset */
#define TIMER_CID1_RESETVALUE               _U_(0xF0)                                     /**<  (TIMER_CID1) Component ID Register 1  Reset Value */

#define TIMER_CID1_CID1_Pos                 0                                              /**< (TIMER_CID1)  Position */
#define TIMER_CID1_CID1_Msk                 (_U_(0xFF) << TIMER_CID1_CID1_Pos)             /**< (TIMER_CID1)  Mask */
#define TIMER_CID1_CID1(value)              (TIMER_CID1_CID1_Msk & ((value) << TIMER_CID1_CID1_Pos))
#define TIMER_CID1_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_CID1) Register MASK  (Use TIMER_CID1_Msk instead)  */
#define TIMER_CID1_Msk                      _U_(0xFF)                                      /**< (TIMER_CID1) Register Mask  */


/* -------- TIMER_CID2 : (TIMER Offset: 0xff8) (R/ 8) Component ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CID2:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_CID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_CID2_OFFSET                   (0xFF8)                                       /**<  (TIMER_CID2) Component ID Register 2  Offset */
#define TIMER_CID2_RESETVALUE               _U_(0x05)                                     /**<  (TIMER_CID2) Component ID Register 2  Reset Value */

#define TIMER_CID2_CID2_Pos                 0                                              /**< (TIMER_CID2)  Position */
#define TIMER_CID2_CID2_Msk                 (_U_(0xFF) << TIMER_CID2_CID2_Pos)             /**< (TIMER_CID2)  Mask */
#define TIMER_CID2_CID2(value)              (TIMER_CID2_CID2_Msk & ((value) << TIMER_CID2_CID2_Pos))
#define TIMER_CID2_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_CID2) Register MASK  (Use TIMER_CID2_Msk instead)  */
#define TIMER_CID2_Msk                      _U_(0xFF)                                      /**< (TIMER_CID2) Register Mask  */


/* -------- TIMER_CID3 : (TIMER Offset: 0xffc) (R/ 8) Component ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CID3:8;                    /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} TIMER_CID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define TIMER_CID3_OFFSET                   (0xFFC)                                       /**<  (TIMER_CID3) Component ID Register 3  Offset */
#define TIMER_CID3_RESETVALUE               _U_(0xB1)                                     /**<  (TIMER_CID3) Component ID Register 3  Reset Value */

#define TIMER_CID3_CID3_Pos                 0                                              /**< (TIMER_CID3)  Position */
#define TIMER_CID3_CID3_Msk                 (_U_(0xFF) << TIMER_CID3_CID3_Pos)             /**< (TIMER_CID3)  Mask */
#define TIMER_CID3_CID3(value)              (TIMER_CID3_CID3_Msk & ((value) << TIMER_CID3_CID3_Pos))
#define TIMER_CID3_MASK                     _U_(0xFF)                                      /**< \deprecated (TIMER_CID3) Register MASK  (Use TIMER_CID3_Msk instead)  */
#define TIMER_CID3_Msk                      _U_(0xFF)                                      /**< (TIMER_CID3) Register Mask  */


#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if COMPONENT_TYPEDEF_STYLE == 'R'
/** \brief TIMER hardware registers */
typedef struct {  /* ARM General Purpose Timer */
  RwReg8  CTRL;           /**< (TIMER Offset: 0x00) Timer Control */
  RoReg8  Reserved1[0x3];
  RwReg   VALUE;          /**< (TIMER Offset: 0x04) Current Value */
  RwReg   RELOAD;         /**< (TIMER Offset: 0x08) Reload Value */
  RwReg8  INTSTATUSCLEAR; /**< (TIMER Offset: 0x0C) Timer Interrupt, write 1 to clear */
  RoReg8  Reserved2[0xFC3];
  RoReg8  PID4;           /**< (TIMER Offset: 0xFD0) Peripheral ID Register 4 */
  RoReg8  Reserved3[0x3];
  RoReg8  PID5;           /**< (TIMER Offset: 0xFD4) Peripheral ID Register 5 */
  RoReg8  Reserved4[0x3];
  RoReg8  PID6;           /**< (TIMER Offset: 0xFD8) Peripheral ID Register 6 */
  RoReg8  Reserved5[0x3];
  RoReg8  PID7;           /**< (TIMER Offset: 0xFDC) Peripheral ID Register 7 */
  RoReg8  Reserved6[0x3];
  RoReg8  PID0;           /**< (TIMER Offset: 0xFE0) Peripheral ID Register 0 */
  RoReg8  Reserved7[0x3];
  RoReg8  PID1;           /**< (TIMER Offset: 0xFE4) Peripheral ID Register 1 */
  RoReg8  Reserved8[0x3];
  RoReg8  PID2;           /**< (TIMER Offset: 0xFE8) Peripheral ID Register 2 */
  RoReg8  Reserved9[0x3];
  RoReg8  PID3;           /**< (TIMER Offset: 0xFEC) Peripheral ID Register 3 */
  RoReg8  Reserved10[0x3];
  RoReg8  CID0;           /**< (TIMER Offset: 0xFF0) Component ID Register 0 */
  RoReg8  Reserved11[0x3];
  RoReg8  CID1;           /**< (TIMER Offset: 0xFF4) Component ID Register 1 */
  RoReg8  Reserved12[0x3];
  RoReg8  CID2;           /**< (TIMER Offset: 0xFF8) Component ID Register 2 */
  RoReg8  Reserved13[0x3];
  RoReg8  CID3;           /**< (TIMER Offset: 0xFFC) Component ID Register 3 */
} Timer;

#elif COMPONENT_TYPEDEF_STYLE == 'N'
/** \brief TIMER hardware registers */
typedef struct {  /* ARM General Purpose Timer */
  __IO TIMER_CTRL_Type                CTRL;           /**< Offset: 0x00 (R/W   8) Timer Control */
  __I  uint8_t                        Reserved1[3];
  __IO TIMER_VALUE_Type               VALUE;          /**< Offset: 0x04 (R/W  32) Current Value */
  __IO TIMER_RELOAD_Type              RELOAD;         /**< Offset: 0x08 (R/W  32) Reload Value */
  __IO TIMER_INTSTATUSCLEAR_Type      INTSTATUSCLEAR; /**< Offset: 0x0C (R/W   8) Timer Interrupt, write 1 to clear */
  __I  uint8_t                        Reserved2[4035];
  __I  TIMER_PID4_Type                PID4;           /**< Offset: 0xFD0 (R/    8) Peripheral ID Register 4 */
  __I  uint8_t                        Reserved3[3];
  __I  TIMER_PID5_Type                PID5;           /**< Offset: 0xFD4 (R/    8) Peripheral ID Register 5 */
  __I  uint8_t                        Reserved4[3];
  __I  TIMER_PID6_Type                PID6;           /**< Offset: 0xFD8 (R/    8) Peripheral ID Register 6 */
  __I  uint8_t                        Reserved5[3];
  __I  TIMER_PID7_Type                PID7;           /**< Offset: 0xFDC (R/    8) Peripheral ID Register 7 */
  __I  uint8_t                        Reserved6[3];
  __I  TIMER_PID0_Type                PID0;           /**< Offset: 0xFE0 (R/    8) Peripheral ID Register 0 */
  __I  uint8_t                        Reserved7[3];
  __I  TIMER_PID1_Type                PID1;           /**< Offset: 0xFE4 (R/    8) Peripheral ID Register 1 */
  __I  uint8_t                        Reserved8[3];
  __I  TIMER_PID2_Type                PID2;           /**< Offset: 0xFE8 (R/    8) Peripheral ID Register 2 */
  __I  uint8_t                        Reserved9[3];
  __I  TIMER_PID3_Type                PID3;           /**< Offset: 0xFEC (R/    8) Peripheral ID Register 3 */
  __I  uint8_t                        Reserved10[3];
  __I  TIMER_CID0_Type                CID0;           /**< Offset: 0xFF0 (R/    8) Component ID Register 0 */
  __I  uint8_t                        Reserved11[3];
  __I  TIMER_CID1_Type                CID1;           /**< Offset: 0xFF4 (R/    8) Component ID Register 1 */
  __I  uint8_t                        Reserved12[3];
  __I  TIMER_CID2_Type                CID2;           /**< Offset: 0xFF8 (R/    8) Component ID Register 2 */
  __I  uint8_t                        Reserved13[3];
  __I  TIMER_CID3_Type                CID3;           /**< Offset: 0xFFC (R/    8) Component ID Register 3 */
} Timer;

#else /* COMPONENT_TYPEDEF_STYLE */
#error Unknown component typedef style
#endif /* COMPONENT_TYPEDEF_STYLE */

#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of ARM General Purpose Timer */

#endif /* _SAMB11_TIMER_COMPONENT_H_ */
