/**
 * \file
 *
 * \brief Component description for ARM_ROM
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
#ifndef _SAMB11_ARM_ROM_COMPONENT_H_
#define _SAMB11_ARM_ROM_COMPONENT_H_
#define _SAMB11_ARM_ROM_COMPONENT_         /**< \deprecated  Backward compatibility for ASF */

/** \addtogroup SAMB_SAMB11 ARM ROM
 *  @{
 */
/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR ARM_ROM */
/* ========================================================================== */
#ifndef COMPONENT_TYPEDEF_STYLE
  #define COMPONENT_TYPEDEF_STYLE 'N'  /**< Defines default style of typedefs for the component header files ('R' = RFO, 'N' = NTO)*/
#endif

#define ARM_ROM_AR1234                     /**< (ARM_ROM) Module ID */
#define REV_ARM_ROM 0x100                  /**< (ARM_ROM) Module revision */

/* -------- ARM_ROM_ROM_SCS : (ARM_ROM Offset: 0x00) (R/ 32) Points to the SCS at 0xE000E000 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ROM_SCS:32;                /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_ROM_ROM_SCS_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_SCS_OFFSET              (0x00)                                        /**<  (ARM_ROM_ROM_SCS) Points to the SCS at 0xE000E000  Offset */
#define ARM_ROM_ROM_SCS_RESETVALUE          _U_(0xFFF0F003)                               /**<  (ARM_ROM_ROM_SCS) Points to the SCS at 0xE000E000  Reset Value */

#define ARM_ROM_ROM_SCS_ROM_SCS_Pos         0                                              /**< (ARM_ROM_ROM_SCS)  Position */
#define ARM_ROM_ROM_SCS_ROM_SCS_Msk         (_U_(0xFFFFFFFF) << ARM_ROM_ROM_SCS_ROM_SCS_Pos)  /**< (ARM_ROM_ROM_SCS)  Mask */
#define ARM_ROM_ROM_SCS_ROM_SCS(value)      (ARM_ROM_ROM_SCS_ROM_SCS_Msk & ((value) << ARM_ROM_ROM_SCS_ROM_SCS_Pos))
#define ARM_ROM_ROM_SCS_MASK                _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_ROM_ROM_SCS) Register MASK  (Use ARM_ROM_ROM_SCS_Msk instead)  */
#define ARM_ROM_ROM_SCS_Msk                 _U_(0xFFFFFFFF)                                /**< (ARM_ROM_ROM_SCS) Register Mask  */


/* -------- ARM_ROM_ROM_DWT : (ARM_ROM Offset: 0x10) (R/ 32) Points to the DWT at 0xE0001000 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ROM_DWT:32;                /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_ROM_ROM_DWT_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_DWT_OFFSET              (0x10)                                        /**<  (ARM_ROM_ROM_DWT) Points to the DWT at 0xE0001000  Offset */
#define ARM_ROM_ROM_DWT_RESETVALUE          _U_(0xFFF02003)                               /**<  (ARM_ROM_ROM_DWT) Points to the DWT at 0xE0001000  Reset Value */

#define ARM_ROM_ROM_DWT_ROM_DWT_Pos         0                                              /**< (ARM_ROM_ROM_DWT)  Position */
#define ARM_ROM_ROM_DWT_ROM_DWT_Msk         (_U_(0xFFFFFFFF) << ARM_ROM_ROM_DWT_ROM_DWT_Pos)  /**< (ARM_ROM_ROM_DWT)  Mask */
#define ARM_ROM_ROM_DWT_ROM_DWT(value)      (ARM_ROM_ROM_DWT_ROM_DWT_Msk & ((value) << ARM_ROM_ROM_DWT_ROM_DWT_Pos))
#define ARM_ROM_ROM_DWT_MASK                _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_ROM_ROM_DWT) Register MASK  (Use ARM_ROM_ROM_DWT_Msk instead)  */
#define ARM_ROM_ROM_DWT_Msk                 _U_(0xFFFFFFFF)                                /**< (ARM_ROM_ROM_DWT) Register Mask  */


/* -------- ARM_ROM_ROM_BPU : (ARM_ROM Offset: 0x20) (R/ 32) Points to the BPU at 0xE0002000 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ROM_BPU:32;                /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_ROM_ROM_BPU_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_BPU_OFFSET              (0x20)                                        /**<  (ARM_ROM_ROM_BPU) Points to the BPU at 0xE0002000  Offset */
#define ARM_ROM_ROM_BPU_RESETVALUE          _U_(0xFFF03003)                               /**<  (ARM_ROM_ROM_BPU) Points to the BPU at 0xE0002000  Reset Value */

#define ARM_ROM_ROM_BPU_ROM_BPU_Pos         0                                              /**< (ARM_ROM_ROM_BPU)  Position */
#define ARM_ROM_ROM_BPU_ROM_BPU_Msk         (_U_(0xFFFFFFFF) << ARM_ROM_ROM_BPU_ROM_BPU_Pos)  /**< (ARM_ROM_ROM_BPU)  Mask */
#define ARM_ROM_ROM_BPU_ROM_BPU(value)      (ARM_ROM_ROM_BPU_ROM_BPU_Msk & ((value) << ARM_ROM_ROM_BPU_ROM_BPU_Pos))
#define ARM_ROM_ROM_BPU_MASK                _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_ROM_ROM_BPU) Register MASK  (Use ARM_ROM_ROM_BPU_Msk instead)  */
#define ARM_ROM_ROM_BPU_Msk                 _U_(0xFFFFFFFF)                                /**< (ARM_ROM_ROM_BPU) Register Mask  */


/* -------- ARM_ROM_ROM_EOT : (ARM_ROM Offset: 0x30) (R/ 32) End of Table Marker -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ROM_EOT:32;                /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_ROM_ROM_EOT_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_EOT_OFFSET              (0x30)                                        /**<  (ARM_ROM_ROM_EOT) End of Table Marker  Offset */
#define ARM_ROM_ROM_EOT_RESETVALUE          _U_(0x00)                                     /**<  (ARM_ROM_ROM_EOT) End of Table Marker  Reset Value */

#define ARM_ROM_ROM_EOT_ROM_EOT_Pos         0                                              /**< (ARM_ROM_ROM_EOT)  Position */
#define ARM_ROM_ROM_EOT_ROM_EOT_Msk         (_U_(0xFFFFFFFF) << ARM_ROM_ROM_EOT_ROM_EOT_Pos)  /**< (ARM_ROM_ROM_EOT)  Mask */
#define ARM_ROM_ROM_EOT_ROM_EOT(value)      (ARM_ROM_ROM_EOT_ROM_EOT_Msk & ((value) << ARM_ROM_ROM_EOT_ROM_EOT_Pos))
#define ARM_ROM_ROM_EOT_MASK                _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_ROM_ROM_EOT) Register MASK  (Use ARM_ROM_ROM_EOT_Msk instead)  */
#define ARM_ROM_ROM_EOT_Msk                 _U_(0xFFFFFFFF)                                /**< (ARM_ROM_ROM_EOT) Register Mask  */


/* -------- ARM_ROM_ROM_CSMT : (ARM_ROM Offset: 0xfcc) (R/ 8) System Memory accessible through DAP -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ROM_CSMT:1;                /**< bit:      0                                           */
    uint8_t  :7;                        /**< bit:   1..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_CSMT_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_CSMT_OFFSET             (0xFCC)                                       /**<  (ARM_ROM_ROM_CSMT) System Memory accessible through DAP  Offset */
#define ARM_ROM_ROM_CSMT_RESETVALUE         _U_(0x01)                                     /**<  (ARM_ROM_ROM_CSMT) System Memory accessible through DAP  Reset Value */

#define ARM_ROM_ROM_CSMT_ROM_CSMT_Pos       0                                              /**< (ARM_ROM_ROM_CSMT)  Position */
#define ARM_ROM_ROM_CSMT_ROM_CSMT_Msk       (_U_(0x1) << ARM_ROM_ROM_CSMT_ROM_CSMT_Pos)    /**< (ARM_ROM_ROM_CSMT)  Mask */
#define ARM_ROM_ROM_CSMT_ROM_CSMT           ARM_ROM_ROM_CSMT_ROM_CSMT_Msk                  /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_ROM_ROM_CSMT_ROM_CSMT_Msk instead */
#define ARM_ROM_ROM_CSMT_MASK               _U_(0x01)                                      /**< \deprecated (ARM_ROM_ROM_CSMT) Register MASK  (Use ARM_ROM_ROM_CSMT_Msk instead)  */
#define ARM_ROM_ROM_CSMT_Msk                _U_(0x01)                                      /**< (ARM_ROM_ROM_CSMT) Register Mask  */


/* -------- ARM_ROM_ROM_PID4 : (ARM_ROM Offset: 0xfd0) (R/ 8) Peripheral ID Register 4 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_C_CODE:4;           /**< bit:   0..3  JEP106 C Code                            */
    uint8_t  BLOCK_COUNT:4;             /**< bit:   4..7  Block Count                              */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_PID4_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_PID4_OFFSET             (0xFD0)                                       /**<  (ARM_ROM_ROM_PID4) Peripheral ID Register 4  Offset */
#define ARM_ROM_ROM_PID4_RESETVALUE         _U_(0x04)                                     /**<  (ARM_ROM_ROM_PID4) Peripheral ID Register 4  Reset Value */

#define ARM_ROM_ROM_PID4_JEP106_C_CODE_Pos  0                                              /**< (ARM_ROM_ROM_PID4) JEP106 C Code Position */
#define ARM_ROM_ROM_PID4_JEP106_C_CODE_Msk  (_U_(0xF) << ARM_ROM_ROM_PID4_JEP106_C_CODE_Pos)  /**< (ARM_ROM_ROM_PID4) JEP106 C Code Mask */
#define ARM_ROM_ROM_PID4_JEP106_C_CODE(value) (ARM_ROM_ROM_PID4_JEP106_C_CODE_Msk & ((value) << ARM_ROM_ROM_PID4_JEP106_C_CODE_Pos))
#define ARM_ROM_ROM_PID4_BLOCK_COUNT_Pos    4                                              /**< (ARM_ROM_ROM_PID4) Block Count Position */
#define ARM_ROM_ROM_PID4_BLOCK_COUNT_Msk    (_U_(0xF) << ARM_ROM_ROM_PID4_BLOCK_COUNT_Pos)  /**< (ARM_ROM_ROM_PID4) Block Count Mask */
#define ARM_ROM_ROM_PID4_BLOCK_COUNT(value) (ARM_ROM_ROM_PID4_BLOCK_COUNT_Msk & ((value) << ARM_ROM_ROM_PID4_BLOCK_COUNT_Pos))
#define ARM_ROM_ROM_PID4_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_PID4) Register MASK  (Use ARM_ROM_ROM_PID4_Msk instead)  */
#define ARM_ROM_ROM_PID4_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_PID4) Register Mask  */


/* -------- ARM_ROM_ROM_PID0 : (ARM_ROM Offset: 0xfe0) (R/ 8) Peripheral ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:8;             /**< bit:   0..7  Part Number Bits 7:0                     */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_PID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_PID0_OFFSET             (0xFE0)                                       /**<  (ARM_ROM_ROM_PID0) Peripheral ID Register 0  Offset */
#define ARM_ROM_ROM_PID0_RESETVALUE         _U_(0x71)                                     /**<  (ARM_ROM_ROM_PID0) Peripheral ID Register 0  Reset Value */

#define ARM_ROM_ROM_PID0_PART_NUMBER_Pos    0                                              /**< (ARM_ROM_ROM_PID0) Part Number Bits 7:0 Position */
#define ARM_ROM_ROM_PID0_PART_NUMBER_Msk    (_U_(0xFF) << ARM_ROM_ROM_PID0_PART_NUMBER_Pos)  /**< (ARM_ROM_ROM_PID0) Part Number Bits 7:0 Mask */
#define ARM_ROM_ROM_PID0_PART_NUMBER(value) (ARM_ROM_ROM_PID0_PART_NUMBER_Msk & ((value) << ARM_ROM_ROM_PID0_PART_NUMBER_Pos))
#define ARM_ROM_ROM_PID0_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_PID0) Register MASK  (Use ARM_ROM_ROM_PID0_Msk instead)  */
#define ARM_ROM_ROM_PID0_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_PID0) Register Mask  */


/* -------- ARM_ROM_ROM_PID1 : (ARM_ROM Offset: 0xfe4) (R/ 8) Peripheral ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:4;             /**< bit:   0..3  Part Number Bits 11:8                    */
    uint8_t  JEP106_ID_3_0:4;           /**< bit:   4..7  JEP106 ID Code Bits 6:4                  */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_PID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_PID1_OFFSET             (0xFE4)                                       /**<  (ARM_ROM_ROM_PID1) Peripheral ID Register 1  Offset */
#define ARM_ROM_ROM_PID1_RESETVALUE         _U_(0xB4)                                     /**<  (ARM_ROM_ROM_PID1) Peripheral ID Register 1  Reset Value */

#define ARM_ROM_ROM_PID1_PART_NUMBER_Pos    0                                              /**< (ARM_ROM_ROM_PID1) Part Number Bits 11:8 Position */
#define ARM_ROM_ROM_PID1_PART_NUMBER_Msk    (_U_(0xF) << ARM_ROM_ROM_PID1_PART_NUMBER_Pos)  /**< (ARM_ROM_ROM_PID1) Part Number Bits 11:8 Mask */
#define ARM_ROM_ROM_PID1_PART_NUMBER(value) (ARM_ROM_ROM_PID1_PART_NUMBER_Msk & ((value) << ARM_ROM_ROM_PID1_PART_NUMBER_Pos))
#define ARM_ROM_ROM_PID1_JEP106_ID_3_0_Pos  4                                              /**< (ARM_ROM_ROM_PID1) JEP106 ID Code Bits 6:4 Position */
#define ARM_ROM_ROM_PID1_JEP106_ID_3_0_Msk  (_U_(0xF) << ARM_ROM_ROM_PID1_JEP106_ID_3_0_Pos)  /**< (ARM_ROM_ROM_PID1) JEP106 ID Code Bits 6:4 Mask */
#define ARM_ROM_ROM_PID1_JEP106_ID_3_0(value) (ARM_ROM_ROM_PID1_JEP106_ID_3_0_Msk & ((value) << ARM_ROM_ROM_PID1_JEP106_ID_3_0_Pos))
#define ARM_ROM_ROM_PID1_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_PID1) Register MASK  (Use ARM_ROM_ROM_PID1_Msk instead)  */
#define ARM_ROM_ROM_PID1_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_PID1) Register Mask  */


/* -------- ARM_ROM_ROM_PID2 : (ARM_ROM Offset: 0xfe8) (R/ 8) Peripheral ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_ID_6_4:3;           /**< bit:   0..2  JEP106 ID Code Bits 6:4                  */
    uint8_t  JEDEC_USED:1;              /**< bit:      3  JEDEC Used                               */
    uint8_t  REVISION:4;                /**< bit:   4..7  Revision                                 */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_PID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_PID2_OFFSET             (0xFE8)                                       /**<  (ARM_ROM_ROM_PID2) Peripheral ID Register 2  Offset */
#define ARM_ROM_ROM_PID2_RESETVALUE         _U_(0x0B)                                     /**<  (ARM_ROM_ROM_PID2) Peripheral ID Register 2  Reset Value */

#define ARM_ROM_ROM_PID2_JEP106_ID_6_4_Pos  0                                              /**< (ARM_ROM_ROM_PID2) JEP106 ID Code Bits 6:4 Position */
#define ARM_ROM_ROM_PID2_JEP106_ID_6_4_Msk  (_U_(0x7) << ARM_ROM_ROM_PID2_JEP106_ID_6_4_Pos)  /**< (ARM_ROM_ROM_PID2) JEP106 ID Code Bits 6:4 Mask */
#define ARM_ROM_ROM_PID2_JEP106_ID_6_4(value) (ARM_ROM_ROM_PID2_JEP106_ID_6_4_Msk & ((value) << ARM_ROM_ROM_PID2_JEP106_ID_6_4_Pos))
#define ARM_ROM_ROM_PID2_JEDEC_USED_Pos     3                                              /**< (ARM_ROM_ROM_PID2) JEDEC Used Position */
#define ARM_ROM_ROM_PID2_JEDEC_USED_Msk     (_U_(0x1) << ARM_ROM_ROM_PID2_JEDEC_USED_Pos)  /**< (ARM_ROM_ROM_PID2) JEDEC Used Mask */
#define ARM_ROM_ROM_PID2_JEDEC_USED         ARM_ROM_ROM_PID2_JEDEC_USED_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_ROM_ROM_PID2_JEDEC_USED_Msk instead */
#define ARM_ROM_ROM_PID2_REVISION_Pos       4                                              /**< (ARM_ROM_ROM_PID2) Revision Position */
#define ARM_ROM_ROM_PID2_REVISION_Msk       (_U_(0xF) << ARM_ROM_ROM_PID2_REVISION_Pos)    /**< (ARM_ROM_ROM_PID2) Revision Mask */
#define ARM_ROM_ROM_PID2_REVISION(value)    (ARM_ROM_ROM_PID2_REVISION_Msk & ((value) << ARM_ROM_ROM_PID2_REVISION_Pos))
#define ARM_ROM_ROM_PID2_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_PID2) Register MASK  (Use ARM_ROM_ROM_PID2_Msk instead)  */
#define ARM_ROM_ROM_PID2_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_PID2) Register Mask  */


/* -------- ARM_ROM_ROM_PID3 : (ARM_ROM Offset: 0xfec) (R/ 8) Peripheral ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CUSTOMER_MOD_NUMBER:4;     /**< bit:   0..3  Customer Modification Number             */
    uint8_t  ECO_REV_NUMBER:4;          /**< bit:   4..7  ECO Revision Number                      */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_PID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_PID3_OFFSET             (0xFEC)                                       /**<  (ARM_ROM_ROM_PID3) Peripheral ID Register 3  Offset */
#define ARM_ROM_ROM_PID3_RESETVALUE         _U_(0x00)                                     /**<  (ARM_ROM_ROM_PID3) Peripheral ID Register 3  Reset Value */

#define ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER_Pos 0                                              /**< (ARM_ROM_ROM_PID3) Customer Modification Number Position */
#define ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER_Msk (_U_(0xF) << ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER_Pos)  /**< (ARM_ROM_ROM_PID3) Customer Modification Number Mask */
#define ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER(value) (ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER_Msk & ((value) << ARM_ROM_ROM_PID3_CUSTOMER_MOD_NUMBER_Pos))
#define ARM_ROM_ROM_PID3_ECO_REV_NUMBER_Pos 4                                              /**< (ARM_ROM_ROM_PID3) ECO Revision Number Position */
#define ARM_ROM_ROM_PID3_ECO_REV_NUMBER_Msk (_U_(0xF) << ARM_ROM_ROM_PID3_ECO_REV_NUMBER_Pos)  /**< (ARM_ROM_ROM_PID3) ECO Revision Number Mask */
#define ARM_ROM_ROM_PID3_ECO_REV_NUMBER(value) (ARM_ROM_ROM_PID3_ECO_REV_NUMBER_Msk & ((value) << ARM_ROM_ROM_PID3_ECO_REV_NUMBER_Pos))
#define ARM_ROM_ROM_PID3_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_PID3) Register MASK  (Use ARM_ROM_ROM_PID3_Msk instead)  */
#define ARM_ROM_ROM_PID3_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_PID3) Register Mask  */


/* -------- ARM_ROM_ROM_CID0 : (ARM_ROM Offset: 0xff0) (R/ 8) Component ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ROM_CID0:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_CID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_CID0_OFFSET             (0xFF0)                                       /**<  (ARM_ROM_ROM_CID0) Component ID Register 0  Offset */
#define ARM_ROM_ROM_CID0_RESETVALUE         _U_(0x0D)                                     /**<  (ARM_ROM_ROM_CID0) Component ID Register 0  Reset Value */

#define ARM_ROM_ROM_CID0_ROM_CID0_Pos       0                                              /**< (ARM_ROM_ROM_CID0)  Position */
#define ARM_ROM_ROM_CID0_ROM_CID0_Msk       (_U_(0xFF) << ARM_ROM_ROM_CID0_ROM_CID0_Pos)   /**< (ARM_ROM_ROM_CID0)  Mask */
#define ARM_ROM_ROM_CID0_ROM_CID0(value)    (ARM_ROM_ROM_CID0_ROM_CID0_Msk & ((value) << ARM_ROM_ROM_CID0_ROM_CID0_Pos))
#define ARM_ROM_ROM_CID0_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_CID0) Register MASK  (Use ARM_ROM_ROM_CID0_Msk instead)  */
#define ARM_ROM_ROM_CID0_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_CID0) Register Mask  */


/* -------- ARM_ROM_ROM_CID1 : (ARM_ROM Offset: 0xff4) (R/ 8) Component ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ROM_CID1:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_CID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_CID1_OFFSET             (0xFF4)                                       /**<  (ARM_ROM_ROM_CID1) Component ID Register 1  Offset */
#define ARM_ROM_ROM_CID1_RESETVALUE         _U_(0x10)                                     /**<  (ARM_ROM_ROM_CID1) Component ID Register 1  Reset Value */

#define ARM_ROM_ROM_CID1_ROM_CID1_Pos       0                                              /**< (ARM_ROM_ROM_CID1)  Position */
#define ARM_ROM_ROM_CID1_ROM_CID1_Msk       (_U_(0xFF) << ARM_ROM_ROM_CID1_ROM_CID1_Pos)   /**< (ARM_ROM_ROM_CID1)  Mask */
#define ARM_ROM_ROM_CID1_ROM_CID1(value)    (ARM_ROM_ROM_CID1_ROM_CID1_Msk & ((value) << ARM_ROM_ROM_CID1_ROM_CID1_Pos))
#define ARM_ROM_ROM_CID1_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_CID1) Register MASK  (Use ARM_ROM_ROM_CID1_Msk instead)  */
#define ARM_ROM_ROM_CID1_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_CID1) Register Mask  */


/* -------- ARM_ROM_ROM_CID2 : (ARM_ROM Offset: 0xff8) (R/ 8) Component ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ROM_CID2:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_CID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_CID2_OFFSET             (0xFF8)                                       /**<  (ARM_ROM_ROM_CID2) Component ID Register 2  Offset */
#define ARM_ROM_ROM_CID2_RESETVALUE         _U_(0x05)                                     /**<  (ARM_ROM_ROM_CID2) Component ID Register 2  Reset Value */

#define ARM_ROM_ROM_CID2_ROM_CID2_Pos       0                                              /**< (ARM_ROM_ROM_CID2)  Position */
#define ARM_ROM_ROM_CID2_ROM_CID2_Msk       (_U_(0xFF) << ARM_ROM_ROM_CID2_ROM_CID2_Pos)   /**< (ARM_ROM_ROM_CID2)  Mask */
#define ARM_ROM_ROM_CID2_ROM_CID2(value)    (ARM_ROM_ROM_CID2_ROM_CID2_Msk & ((value) << ARM_ROM_ROM_CID2_ROM_CID2_Pos))
#define ARM_ROM_ROM_CID2_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_CID2) Register MASK  (Use ARM_ROM_ROM_CID2_Msk instead)  */
#define ARM_ROM_ROM_CID2_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_CID2) Register Mask  */


/* -------- ARM_ROM_ROM_CID3 : (ARM_ROM Offset: 0xffc) (R/ 8) Component ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  ROM_CID3:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_ROM_ROM_CID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_ROM_ROM_CID3_OFFSET             (0xFFC)                                       /**<  (ARM_ROM_ROM_CID3) Component ID Register 3  Offset */
#define ARM_ROM_ROM_CID3_RESETVALUE         _U_(0xB1)                                     /**<  (ARM_ROM_ROM_CID3) Component ID Register 3  Reset Value */

#define ARM_ROM_ROM_CID3_ROM_CID3_Pos       0                                              /**< (ARM_ROM_ROM_CID3)  Position */
#define ARM_ROM_ROM_CID3_ROM_CID3_Msk       (_U_(0xFF) << ARM_ROM_ROM_CID3_ROM_CID3_Pos)   /**< (ARM_ROM_ROM_CID3)  Mask */
#define ARM_ROM_ROM_CID3_ROM_CID3(value)    (ARM_ROM_ROM_CID3_ROM_CID3_Msk & ((value) << ARM_ROM_ROM_CID3_ROM_CID3_Pos))
#define ARM_ROM_ROM_CID3_MASK               _U_(0xFF)                                      /**< \deprecated (ARM_ROM_ROM_CID3) Register MASK  (Use ARM_ROM_ROM_CID3_Msk instead)  */
#define ARM_ROM_ROM_CID3_Msk                _U_(0xFF)                                      /**< (ARM_ROM_ROM_CID3) Register Mask  */


#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if COMPONENT_TYPEDEF_STYLE == 'R'
/** \brief ARM_ROM hardware registers */
typedef struct {  /* ARM ROM */
  RoReg   ROM_SCS;        /**< (ARM_ROM Offset: 0x00) Points to the SCS at 0xE000E000 */
  RoReg8  Reserved1[0xC];
  RoReg   ROM_DWT;        /**< (ARM_ROM Offset: 0x10) Points to the DWT at 0xE0001000 */
  RoReg8  Reserved2[0xC];
  RoReg   ROM_BPU;        /**< (ARM_ROM Offset: 0x20) Points to the BPU at 0xE0002000 */
  RoReg8  Reserved3[0xC];
  RoReg   ROM_EOT;        /**< (ARM_ROM Offset: 0x30) End of Table Marker */
  RoReg8  Reserved4[0xF98];
  RoReg8  ROM_CSMT;       /**< (ARM_ROM Offset: 0xFCC) System Memory accessible through DAP */
  RoReg8  Reserved5[0x3];
  RoReg8  ROM_PID4;       /**< (ARM_ROM Offset: 0xFD0) Peripheral ID Register 4 */
  RoReg8  Reserved6[0xF];
  RoReg8  ROM_PID0;       /**< (ARM_ROM Offset: 0xFE0) Peripheral ID Register 0 */
  RoReg8  Reserved7[0x3];
  RoReg8  ROM_PID1;       /**< (ARM_ROM Offset: 0xFE4) Peripheral ID Register 1 */
  RoReg8  Reserved8[0x3];
  RoReg8  ROM_PID2;       /**< (ARM_ROM Offset: 0xFE8) Peripheral ID Register 2 */
  RoReg8  Reserved9[0x3];
  RoReg8  ROM_PID3;       /**< (ARM_ROM Offset: 0xFEC) Peripheral ID Register 3 */
  RoReg8  Reserved10[0x3];
  RoReg8  ROM_CID0;       /**< (ARM_ROM Offset: 0xFF0) Component ID Register 0 */
  RoReg8  Reserved11[0x3];
  RoReg8  ROM_CID1;       /**< (ARM_ROM Offset: 0xFF4) Component ID Register 1 */
  RoReg8  Reserved12[0x3];
  RoReg8  ROM_CID2;       /**< (ARM_ROM Offset: 0xFF8) Component ID Register 2 */
  RoReg8  Reserved13[0x3];
  RoReg8  ROM_CID3;       /**< (ARM_ROM Offset: 0xFFC) Component ID Register 3 */
} ArmRom;

#elif COMPONENT_TYPEDEF_STYLE == 'N'
/** \brief ARM_ROM hardware registers */
typedef struct {  /* ARM ROM */
  __I  ARM_ROM_ROM_SCS_Type           ROM_SCS;        /**< Offset: 0x00 (R/   32) Points to the SCS at 0xE000E000 */
  __I  uint32_t                       Reserved1[3];
  __I  ARM_ROM_ROM_DWT_Type           ROM_DWT;        /**< Offset: 0x10 (R/   32) Points to the DWT at 0xE0001000 */
  __I  uint32_t                       Reserved2[3];
  __I  ARM_ROM_ROM_BPU_Type           ROM_BPU;        /**< Offset: 0x20 (R/   32) Points to the BPU at 0xE0002000 */
  __I  uint32_t                       Reserved3[3];
  __I  ARM_ROM_ROM_EOT_Type           ROM_EOT;        /**< Offset: 0x30 (R/   32) End of Table Marker */
  __I  uint32_t                       Reserved4[998];
  __I  ARM_ROM_ROM_CSMT_Type          ROM_CSMT;       /**< Offset: 0xFCC (R/    8) System Memory accessible through DAP */
  __I  uint8_t                        Reserved5[3];
  __I  ARM_ROM_ROM_PID4_Type          ROM_PID4;       /**< Offset: 0xFD0 (R/    8) Peripheral ID Register 4 */
  __I  uint8_t                        Reserved6[15];
  __I  ARM_ROM_ROM_PID0_Type          ROM_PID0;       /**< Offset: 0xFE0 (R/    8) Peripheral ID Register 0 */
  __I  uint8_t                        Reserved7[3];
  __I  ARM_ROM_ROM_PID1_Type          ROM_PID1;       /**< Offset: 0xFE4 (R/    8) Peripheral ID Register 1 */
  __I  uint8_t                        Reserved8[3];
  __I  ARM_ROM_ROM_PID2_Type          ROM_PID2;       /**< Offset: 0xFE8 (R/    8) Peripheral ID Register 2 */
  __I  uint8_t                        Reserved9[3];
  __I  ARM_ROM_ROM_PID3_Type          ROM_PID3;       /**< Offset: 0xFEC (R/    8) Peripheral ID Register 3 */
  __I  uint8_t                        Reserved10[3];
  __I  ARM_ROM_ROM_CID0_Type          ROM_CID0;       /**< Offset: 0xFF0 (R/    8) Component ID Register 0 */
  __I  uint8_t                        Reserved11[3];
  __I  ARM_ROM_ROM_CID1_Type          ROM_CID1;       /**< Offset: 0xFF4 (R/    8) Component ID Register 1 */
  __I  uint8_t                        Reserved12[3];
  __I  ARM_ROM_ROM_CID2_Type          ROM_CID2;       /**< Offset: 0xFF8 (R/    8) Component ID Register 2 */
  __I  uint8_t                        Reserved13[3];
  __I  ARM_ROM_ROM_CID3_Type          ROM_CID3;       /**< Offset: 0xFFC (R/    8) Component ID Register 3 */
} ArmRom;

#else /* COMPONENT_TYPEDEF_STYLE */
#error Unknown component typedef style
#endif /* COMPONENT_TYPEDEF_STYLE */

#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of ARM ROM */

#endif /* _SAMB11_ARM_ROM_COMPONENT_H_ */
