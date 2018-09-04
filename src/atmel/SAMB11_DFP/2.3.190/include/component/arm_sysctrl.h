/**
 * \file
 *
 * \brief Component description for ARM_SYSCTRL
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
#ifndef _SAMB11_ARM_SYSCTRL_COMPONENT_H_
#define _SAMB11_ARM_SYSCTRL_COMPONENT_H_
#define _SAMB11_ARM_SYSCTRL_COMPONENT_         /**< \deprecated  Backward compatibility for ASF */

/** \addtogroup SAMB_SAMB11 ARM System Control
 *  @{
 */
/* ========================================================================== */
/**  SOFTWARE API DEFINITION FOR ARM_SYSCTRL */
/* ========================================================================== */
#ifndef COMPONENT_TYPEDEF_STYLE
  #define COMPONENT_TYPEDEF_STYLE 'N'  /**< Defines default style of typedefs for the component header files ('R' = RFO, 'N' = NTO)*/
#endif

#define ARM_SYSCTRL_ASC1234                    /**< (ARM_SYSCTRL) Module ID */
#define REV_ARM_SYSCTRL 0x100                  /**< (ARM_SYSCTRL) Module revision */

/* -------- ARM_SYSCTRL_ACTLR : (ARM_SYSCTRL Offset: 0x08) (R/ 32) Auxiliary Control Register (not implemented) -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ACTLR:32;                  /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_ACTLR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_ACTLR_OFFSET            (0x08)                                        /**<  (ARM_SYSCTRL_ACTLR) Auxiliary Control Register (not implemented)  Offset */
#define ARM_SYSCTRL_ACTLR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_ACTLR) Auxiliary Control Register (not implemented)  Reset Value */

#define ARM_SYSCTRL_ACTLR_ACTLR_Pos         0                                              /**< (ARM_SYSCTRL_ACTLR)  Position */
#define ARM_SYSCTRL_ACTLR_ACTLR_Msk         (_U_(0xFFFFFFFF) << ARM_SYSCTRL_ACTLR_ACTLR_Pos)  /**< (ARM_SYSCTRL_ACTLR)  Mask */
#define ARM_SYSCTRL_ACTLR_ACTLR(value)      (ARM_SYSCTRL_ACTLR_ACTLR_Msk & ((value) << ARM_SYSCTRL_ACTLR_ACTLR_Pos))
#define ARM_SYSCTRL_ACTLR_MASK              _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_ACTLR) Register MASK  (Use ARM_SYSCTRL_ACTLR_Msk instead)  */
#define ARM_SYSCTRL_ACTLR_Msk               _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_ACTLR) Register Mask  */


/* -------- ARM_SYSCTRL_SYST_CSR : (ARM_SYSCTRL Offset: 0x10) (R/W 32) SysTick Control and Status Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t ENABLE:1;                  /**< bit:      0  Indicates the enabled status of the SysTick counter */
    uint32_t TICKINT:1;                 /**< bit:      1  Indicates whether counting to 0 causes the status of the SysTick exception to change to pending */
    uint32_t CLKSOURCE:1;               /**< bit:      2  SysTick uses the processor clock (writes are ignored) */
    uint32_t :13;                       /**< bit:  3..15  Reserved */
    uint32_t COUNTFLAG:1;               /**< bit:     16  If 1 then the timer has counted to 0     */
    uint32_t :15;                       /**< bit: 17..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SYST_CSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SYST_CSR_OFFSET         (0x10)                                        /**<  (ARM_SYSCTRL_SYST_CSR) SysTick Control and Status Register  Offset */
#define ARM_SYSCTRL_SYST_CSR_RESETVALUE     _U_(0x04)                                     /**<  (ARM_SYSCTRL_SYST_CSR) SysTick Control and Status Register  Reset Value */

#define ARM_SYSCTRL_SYST_CSR_ENABLE_Pos     0                                              /**< (ARM_SYSCTRL_SYST_CSR) Indicates the enabled status of the SysTick counter Position */
#define ARM_SYSCTRL_SYST_CSR_ENABLE_Msk     (_U_(0x1) << ARM_SYSCTRL_SYST_CSR_ENABLE_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Indicates the enabled status of the SysTick counter Mask */
#define ARM_SYSCTRL_SYST_CSR_ENABLE         ARM_SYSCTRL_SYST_CSR_ENABLE_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CSR_ENABLE_Msk instead */
#define   ARM_SYSCTRL_SYST_CSR_ENABLE_0_Val _U_(0x0)                                       /**< (ARM_SYSCTRL_SYST_CSR) Counter Disabled  */
#define   ARM_SYSCTRL_SYST_CSR_ENABLE_1_Val _U_(0x1)                                       /**< (ARM_SYSCTRL_SYST_CSR) Counter Operating  */
#define ARM_SYSCTRL_SYST_CSR_ENABLE_0       (ARM_SYSCTRL_SYST_CSR_ENABLE_0_Val << ARM_SYSCTRL_SYST_CSR_ENABLE_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Counter Disabled Position  */
#define ARM_SYSCTRL_SYST_CSR_ENABLE_1       (ARM_SYSCTRL_SYST_CSR_ENABLE_1_Val << ARM_SYSCTRL_SYST_CSR_ENABLE_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Counter Operating Position  */
#define ARM_SYSCTRL_SYST_CSR_TICKINT_Pos    1                                              /**< (ARM_SYSCTRL_SYST_CSR) Indicates whether counting to 0 causes the status of the SysTick exception to change to pending Position */
#define ARM_SYSCTRL_SYST_CSR_TICKINT_Msk    (_U_(0x1) << ARM_SYSCTRL_SYST_CSR_TICKINT_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Indicates whether counting to 0 causes the status of the SysTick exception to change to pending Mask */
#define ARM_SYSCTRL_SYST_CSR_TICKINT        ARM_SYSCTRL_SYST_CSR_TICKINT_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CSR_TICKINT_Msk instead */
#define   ARM_SYSCTRL_SYST_CSR_TICKINT_0_Val _U_(0x0)                                       /**< (ARM_SYSCTRL_SYST_CSR) Count to 0 does not affect the SysTick exception status  */
#define   ARM_SYSCTRL_SYST_CSR_TICKINT_1_Val _U_(0x1)                                       /**< (ARM_SYSCTRL_SYST_CSR) Count to 0 changes the SysTick excpetion status to pending  */
#define ARM_SYSCTRL_SYST_CSR_TICKINT_0      (ARM_SYSCTRL_SYST_CSR_TICKINT_0_Val << ARM_SYSCTRL_SYST_CSR_TICKINT_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Count to 0 does not affect the SysTick exception status Position  */
#define ARM_SYSCTRL_SYST_CSR_TICKINT_1      (ARM_SYSCTRL_SYST_CSR_TICKINT_1_Val << ARM_SYSCTRL_SYST_CSR_TICKINT_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) Count to 0 changes the SysTick excpetion status to pending Position  */
#define ARM_SYSCTRL_SYST_CSR_CLKSOURCE_Pos  2                                              /**< (ARM_SYSCTRL_SYST_CSR) SysTick uses the processor clock (writes are ignored) Position */
#define ARM_SYSCTRL_SYST_CSR_CLKSOURCE_Msk  (_U_(0x1) << ARM_SYSCTRL_SYST_CSR_CLKSOURCE_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) SysTick uses the processor clock (writes are ignored) Mask */
#define ARM_SYSCTRL_SYST_CSR_CLKSOURCE      ARM_SYSCTRL_SYST_CSR_CLKSOURCE_Msk             /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CSR_CLKSOURCE_Msk instead */
#define ARM_SYSCTRL_SYST_CSR_COUNTFLAG_Pos  16                                             /**< (ARM_SYSCTRL_SYST_CSR) If 1 then the timer has counted to 0 Position */
#define ARM_SYSCTRL_SYST_CSR_COUNTFLAG_Msk  (_U_(0x1) << ARM_SYSCTRL_SYST_CSR_COUNTFLAG_Pos)  /**< (ARM_SYSCTRL_SYST_CSR) If 1 then the timer has counted to 0 Mask */
#define ARM_SYSCTRL_SYST_CSR_COUNTFLAG      ARM_SYSCTRL_SYST_CSR_COUNTFLAG_Msk             /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CSR_COUNTFLAG_Msk instead */
#define ARM_SYSCTRL_SYST_CSR_MASK           _U_(0x10007)                                   /**< \deprecated (ARM_SYSCTRL_SYST_CSR) Register MASK  (Use ARM_SYSCTRL_SYST_CSR_Msk instead)  */
#define ARM_SYSCTRL_SYST_CSR_Msk            _U_(0x10007)                                   /**< (ARM_SYSCTRL_SYST_CSR) Register Mask  */


/* -------- ARM_SYSCTRL_SYST_RVR : (ARM_SYSCTRL Offset: 0x14) (R/W 32) SysTick Reload Value Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t SYST_RVR:24;               /**< bit:  0..23                                           */
    uint32_t :8;                        /**< bit: 24..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SYST_RVR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SYST_RVR_OFFSET         (0x14)                                        /**<  (ARM_SYSCTRL_SYST_RVR) SysTick Reload Value Register  Offset */
#define ARM_SYSCTRL_SYST_RVR_RESETVALUE     _U_(0x00)                                     /**<  (ARM_SYSCTRL_SYST_RVR) SysTick Reload Value Register  Reset Value */

#define ARM_SYSCTRL_SYST_RVR_SYST_RVR_Pos   0                                              /**< (ARM_SYSCTRL_SYST_RVR)  Position */
#define ARM_SYSCTRL_SYST_RVR_SYST_RVR_Msk   (_U_(0xFFFFFF) << ARM_SYSCTRL_SYST_RVR_SYST_RVR_Pos)  /**< (ARM_SYSCTRL_SYST_RVR)  Mask */
#define ARM_SYSCTRL_SYST_RVR_SYST_RVR(value) (ARM_SYSCTRL_SYST_RVR_SYST_RVR_Msk & ((value) << ARM_SYSCTRL_SYST_RVR_SYST_RVR_Pos))
#define ARM_SYSCTRL_SYST_RVR_MASK           _U_(0xFFFFFF)                                  /**< \deprecated (ARM_SYSCTRL_SYST_RVR) Register MASK  (Use ARM_SYSCTRL_SYST_RVR_Msk instead)  */
#define ARM_SYSCTRL_SYST_RVR_Msk            _U_(0xFFFFFF)                                  /**< (ARM_SYSCTRL_SYST_RVR) Register Mask  */


/* -------- ARM_SYSCTRL_SYST_CVR : (ARM_SYSCTRL Offset: 0x18) (R/W 32) SysTick Current Value Register (Any Write Clears to 0) -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t SYST_CVR:24;               /**< bit:  0..23                                           */
    uint32_t :8;                        /**< bit: 24..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SYST_CVR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SYST_CVR_OFFSET         (0x18)                                        /**<  (ARM_SYSCTRL_SYST_CVR) SysTick Current Value Register (Any Write Clears to 0)  Offset */
#define ARM_SYSCTRL_SYST_CVR_RESETVALUE     _U_(0x00)                                     /**<  (ARM_SYSCTRL_SYST_CVR) SysTick Current Value Register (Any Write Clears to 0)  Reset Value */

#define ARM_SYSCTRL_SYST_CVR_SYST_CVR_Pos   0                                              /**< (ARM_SYSCTRL_SYST_CVR)  Position */
#define ARM_SYSCTRL_SYST_CVR_SYST_CVR_Msk   (_U_(0xFFFFFF) << ARM_SYSCTRL_SYST_CVR_SYST_CVR_Pos)  /**< (ARM_SYSCTRL_SYST_CVR)  Mask */
#define ARM_SYSCTRL_SYST_CVR_SYST_CVR(value) (ARM_SYSCTRL_SYST_CVR_SYST_CVR_Msk & ((value) << ARM_SYSCTRL_SYST_CVR_SYST_CVR_Pos))
#define ARM_SYSCTRL_SYST_CVR_MASK           _U_(0xFFFFFF)                                  /**< \deprecated (ARM_SYSCTRL_SYST_CVR) Register MASK  (Use ARM_SYSCTRL_SYST_CVR_Msk instead)  */
#define ARM_SYSCTRL_SYST_CVR_Msk            _U_(0xFFFFFF)                                  /**< (ARM_SYSCTRL_SYST_CVR) Register Mask  */


/* -------- ARM_SYSCTRL_SYST_CALIB : (ARM_SYSCTRL Offset: 0x1c) (R/ 32) SysTick Calibration Value Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t TENMS:24;                  /**< bit:  0..23  Holds a reload value to be used for 10ms timing. If 0 then calibration value is not known. */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t SKEW:1;                    /**< bit:     30  If 1 then 10ms calibration value is inexact */
    uint32_t NOREF:1;                   /**< bit:     31  If 1 then reference clock is not implemented */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SYST_CALIB_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SYST_CALIB_OFFSET       (0x1C)                                        /**<  (ARM_SYSCTRL_SYST_CALIB) SysTick Calibration Value Register  Offset */
#define ARM_SYSCTRL_SYST_CALIB_RESETVALUE   _U_(0x00)                                     /**<  (ARM_SYSCTRL_SYST_CALIB) SysTick Calibration Value Register  Reset Value */

#define ARM_SYSCTRL_SYST_CALIB_TENMS_Pos    0                                              /**< (ARM_SYSCTRL_SYST_CALIB) Holds a reload value to be used for 10ms timing. If 0 then calibration value is not known. Position */
#define ARM_SYSCTRL_SYST_CALIB_TENMS_Msk    (_U_(0xFFFFFF) << ARM_SYSCTRL_SYST_CALIB_TENMS_Pos)  /**< (ARM_SYSCTRL_SYST_CALIB) Holds a reload value to be used for 10ms timing. If 0 then calibration value is not known. Mask */
#define ARM_SYSCTRL_SYST_CALIB_TENMS(value) (ARM_SYSCTRL_SYST_CALIB_TENMS_Msk & ((value) << ARM_SYSCTRL_SYST_CALIB_TENMS_Pos))
#define ARM_SYSCTRL_SYST_CALIB_SKEW_Pos     30                                             /**< (ARM_SYSCTRL_SYST_CALIB) If 1 then 10ms calibration value is inexact Position */
#define ARM_SYSCTRL_SYST_CALIB_SKEW_Msk     (_U_(0x1) << ARM_SYSCTRL_SYST_CALIB_SKEW_Pos)  /**< (ARM_SYSCTRL_SYST_CALIB) If 1 then 10ms calibration value is inexact Mask */
#define ARM_SYSCTRL_SYST_CALIB_SKEW         ARM_SYSCTRL_SYST_CALIB_SKEW_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CALIB_SKEW_Msk instead */
#define ARM_SYSCTRL_SYST_CALIB_NOREF_Pos    31                                             /**< (ARM_SYSCTRL_SYST_CALIB) If 1 then reference clock is not implemented Position */
#define ARM_SYSCTRL_SYST_CALIB_NOREF_Msk    (_U_(0x1) << ARM_SYSCTRL_SYST_CALIB_NOREF_Pos)  /**< (ARM_SYSCTRL_SYST_CALIB) If 1 then reference clock is not implemented Mask */
#define ARM_SYSCTRL_SYST_CALIB_NOREF        ARM_SYSCTRL_SYST_CALIB_NOREF_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SYST_CALIB_NOREF_Msk instead */
#define ARM_SYSCTRL_SYST_CALIB_MASK         _U_(0xC0FFFFFF)                                /**< \deprecated (ARM_SYSCTRL_SYST_CALIB) Register MASK  (Use ARM_SYSCTRL_SYST_CALIB_Msk instead)  */
#define ARM_SYSCTRL_SYST_CALIB_Msk          _U_(0xC0FFFFFF)                                /**< (ARM_SYSCTRL_SYST_CALIB) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_ISER : (ARM_SYSCTRL Offset: 0x100) (R/W 32) Interrupt Set-Enable Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t SETENA:32;                 /**< bit:  0..31  Write 1 to enable the associated interrupt */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_ISER_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_ISER_OFFSET        (0x100)                                       /**<  (ARM_SYSCTRL_NVIC_ISER) Interrupt Set-Enable Register  Offset */
#define ARM_SYSCTRL_NVIC_ISER_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_ISER) Interrupt Set-Enable Register  Reset Value */

#define ARM_SYSCTRL_NVIC_ISER_SETENA_Pos    0                                              /**< (ARM_SYSCTRL_NVIC_ISER) Write 1 to enable the associated interrupt Position */
#define ARM_SYSCTRL_NVIC_ISER_SETENA_Msk    (_U_(0xFFFFFFFF) << ARM_SYSCTRL_NVIC_ISER_SETENA_Pos)  /**< (ARM_SYSCTRL_NVIC_ISER) Write 1 to enable the associated interrupt Mask */
#define ARM_SYSCTRL_NVIC_ISER_SETENA(value) (ARM_SYSCTRL_NVIC_ISER_SETENA_Msk & ((value) << ARM_SYSCTRL_NVIC_ISER_SETENA_Pos))
#define ARM_SYSCTRL_NVIC_ISER_MASK          _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_NVIC_ISER) Register MASK  (Use ARM_SYSCTRL_NVIC_ISER_Msk instead)  */
#define ARM_SYSCTRL_NVIC_ISER_Msk           _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_NVIC_ISER) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_ICER : (ARM_SYSCTRL Offset: 0x180) (R/W 32) Interrupt Clear Enable Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t CLRENA:32;                 /**< bit:  0..31  Write 1 to disable the associated interrupt */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_ICER_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_ICER_OFFSET        (0x180)                                       /**<  (ARM_SYSCTRL_NVIC_ICER) Interrupt Clear Enable Register  Offset */
#define ARM_SYSCTRL_NVIC_ICER_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_ICER) Interrupt Clear Enable Register  Reset Value */

#define ARM_SYSCTRL_NVIC_ICER_CLRENA_Pos    0                                              /**< (ARM_SYSCTRL_NVIC_ICER) Write 1 to disable the associated interrupt Position */
#define ARM_SYSCTRL_NVIC_ICER_CLRENA_Msk    (_U_(0xFFFFFFFF) << ARM_SYSCTRL_NVIC_ICER_CLRENA_Pos)  /**< (ARM_SYSCTRL_NVIC_ICER) Write 1 to disable the associated interrupt Mask */
#define ARM_SYSCTRL_NVIC_ICER_CLRENA(value) (ARM_SYSCTRL_NVIC_ICER_CLRENA_Msk & ((value) << ARM_SYSCTRL_NVIC_ICER_CLRENA_Pos))
#define ARM_SYSCTRL_NVIC_ICER_MASK          _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_NVIC_ICER) Register MASK  (Use ARM_SYSCTRL_NVIC_ICER_Msk instead)  */
#define ARM_SYSCTRL_NVIC_ICER_Msk           _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_NVIC_ICER) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_ISPR : (ARM_SYSCTRL Offset: 0x200) (R/W 32) Interrupt Set-Pending Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t SETPEND:32;                /**< bit:  0..31  Change the state of the associated interrupt to pending */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_ISPR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_ISPR_OFFSET        (0x200)                                       /**<  (ARM_SYSCTRL_NVIC_ISPR) Interrupt Set-Pending Register  Offset */
#define ARM_SYSCTRL_NVIC_ISPR_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_ISPR) Interrupt Set-Pending Register  Reset Value */

#define ARM_SYSCTRL_NVIC_ISPR_SETPEND_Pos   0                                              /**< (ARM_SYSCTRL_NVIC_ISPR) Change the state of the associated interrupt to pending Position */
#define ARM_SYSCTRL_NVIC_ISPR_SETPEND_Msk   (_U_(0xFFFFFFFF) << ARM_SYSCTRL_NVIC_ISPR_SETPEND_Pos)  /**< (ARM_SYSCTRL_NVIC_ISPR) Change the state of the associated interrupt to pending Mask */
#define ARM_SYSCTRL_NVIC_ISPR_SETPEND(value) (ARM_SYSCTRL_NVIC_ISPR_SETPEND_Msk & ((value) << ARM_SYSCTRL_NVIC_ISPR_SETPEND_Pos))
#define ARM_SYSCTRL_NVIC_ISPR_MASK          _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_NVIC_ISPR) Register MASK  (Use ARM_SYSCTRL_NVIC_ISPR_Msk instead)  */
#define ARM_SYSCTRL_NVIC_ISPR_Msk           _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_NVIC_ISPR) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_ICPR : (ARM_SYSCTRL Offset: 0x280) (R/W 32) Interrupt Clear-Pending Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t CLRPEND:32;                /**< bit:  0..31  Change the state of the associated interrupt to not pending */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_ICPR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_ICPR_OFFSET        (0x280)                                       /**<  (ARM_SYSCTRL_NVIC_ICPR) Interrupt Clear-Pending Register  Offset */
#define ARM_SYSCTRL_NVIC_ICPR_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_ICPR) Interrupt Clear-Pending Register  Reset Value */

#define ARM_SYSCTRL_NVIC_ICPR_CLRPEND_Pos   0                                              /**< (ARM_SYSCTRL_NVIC_ICPR) Change the state of the associated interrupt to not pending Position */
#define ARM_SYSCTRL_NVIC_ICPR_CLRPEND_Msk   (_U_(0xFFFFFFFF) << ARM_SYSCTRL_NVIC_ICPR_CLRPEND_Pos)  /**< (ARM_SYSCTRL_NVIC_ICPR) Change the state of the associated interrupt to not pending Mask */
#define ARM_SYSCTRL_NVIC_ICPR_CLRPEND(value) (ARM_SYSCTRL_NVIC_ICPR_CLRPEND_Msk & ((value) << ARM_SYSCTRL_NVIC_ICPR_CLRPEND_Pos))
#define ARM_SYSCTRL_NVIC_ICPR_MASK          _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_NVIC_ICPR) Register MASK  (Use ARM_SYSCTRL_NVIC_ICPR_Msk instead)  */
#define ARM_SYSCTRL_NVIC_ICPR_Msk           _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_NVIC_ICPR) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR0 : (ARM_SYSCTRL Offset: 0x400) (R/W 32) Interrupt Priority Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :6;                        /**< bit:   0..5  Reserved */
    uint32_t PRI_N0:2;                  /**< bit:   6..7  Priority of Interrupt 0                  */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N1:2;                  /**< bit: 14..15  Priority of Interrupt 1                  */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N2:2;                  /**< bit: 22..23  Priority of Interrupt 2                  */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N3:2;                  /**< bit: 30..31  Priority of Interrupt 3                  */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR0_OFFSET        (0x400)                                       /**<  (ARM_SYSCTRL_NVIC_IPR0) Interrupt Priority Register 0  Offset */
#define ARM_SYSCTRL_NVIC_IPR0_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR0) Interrupt Priority Register 0  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR0_PRI_N0_Pos    6                                              /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 0 Position */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N0_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR0_PRI_N0_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 0 Mask */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N0(value) (ARM_SYSCTRL_NVIC_IPR0_PRI_N0_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR0_PRI_N0_Pos))
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N1_Pos    14                                             /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 1 Position */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N1_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR0_PRI_N1_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 1 Mask */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N1(value) (ARM_SYSCTRL_NVIC_IPR0_PRI_N1_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR0_PRI_N1_Pos))
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N2_Pos    22                                             /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 2 Position */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N2_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR0_PRI_N2_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 2 Mask */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N2(value) (ARM_SYSCTRL_NVIC_IPR0_PRI_N2_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR0_PRI_N2_Pos))
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N3_Pos    30                                             /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 3 Position */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N3_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR0_PRI_N3_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR0) Priority of Interrupt 3 Mask */
#define ARM_SYSCTRL_NVIC_IPR0_PRI_N3(value) (ARM_SYSCTRL_NVIC_IPR0_PRI_N3_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR0_PRI_N3_Pos))
#define ARM_SYSCTRL_NVIC_IPR0_MASK          _U_(0xC0C0C0C0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR0) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR0_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR0_Msk           _U_(0xC0C0C0C0)                                /**< (ARM_SYSCTRL_NVIC_IPR0) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR1 : (ARM_SYSCTRL Offset: 0x404) (R/W 32) Interrupt Priority Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :6;                        /**< bit:   0..5  Reserved */
    uint32_t PRI_N4:2;                  /**< bit:   6..7  Priority of Interrupt 4                  */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N5:2;                  /**< bit: 14..15  Priority of Interrupt 5                  */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N6:2;                  /**< bit: 22..23  Priority of Interrupt 6                  */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N7:2;                  /**< bit: 30..31  Priority of Interrupt 7                  */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR1_OFFSET        (0x404)                                       /**<  (ARM_SYSCTRL_NVIC_IPR1) Interrupt Priority Register 1  Offset */
#define ARM_SYSCTRL_NVIC_IPR1_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR1) Interrupt Priority Register 1  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR1_PRI_N4_Pos    6                                              /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 4 Position */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N4_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR1_PRI_N4_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 4 Mask */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N4(value) (ARM_SYSCTRL_NVIC_IPR1_PRI_N4_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR1_PRI_N4_Pos))
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N5_Pos    14                                             /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 5 Position */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N5_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR1_PRI_N5_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 5 Mask */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N5(value) (ARM_SYSCTRL_NVIC_IPR1_PRI_N5_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR1_PRI_N5_Pos))
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N6_Pos    22                                             /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 6 Position */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N6_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR1_PRI_N6_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 6 Mask */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N6(value) (ARM_SYSCTRL_NVIC_IPR1_PRI_N6_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR1_PRI_N6_Pos))
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N7_Pos    30                                             /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 7 Position */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N7_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR1_PRI_N7_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR1) Priority of Interrupt 7 Mask */
#define ARM_SYSCTRL_NVIC_IPR1_PRI_N7(value) (ARM_SYSCTRL_NVIC_IPR1_PRI_N7_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR1_PRI_N7_Pos))
#define ARM_SYSCTRL_NVIC_IPR1_MASK          _U_(0xC0C0C0C0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR1) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR1_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR1_Msk           _U_(0xC0C0C0C0)                                /**< (ARM_SYSCTRL_NVIC_IPR1) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR2 : (ARM_SYSCTRL Offset: 0x408) (R/W 32) Interrupt Priority Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :6;                        /**< bit:   0..5  Reserved */
    uint32_t PRI_N8:2;                  /**< bit:   6..7  Priority of Interrupt 8                  */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N9:2;                  /**< bit: 14..15  Priority of Interrupt 9                  */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N10:2;                 /**< bit: 22..23  Priority of Interrupt 10                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N11:2;                 /**< bit: 30..31  Priority of Interrupt 11                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR2_OFFSET        (0x408)                                       /**<  (ARM_SYSCTRL_NVIC_IPR2) Interrupt Priority Register 2  Offset */
#define ARM_SYSCTRL_NVIC_IPR2_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR2) Interrupt Priority Register 2  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR2_PRI_N8_Pos    6                                              /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 8 Position */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N8_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR2_PRI_N8_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 8 Mask */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N8(value) (ARM_SYSCTRL_NVIC_IPR2_PRI_N8_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR2_PRI_N8_Pos))
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N9_Pos    14                                             /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 9 Position */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N9_Msk    (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR2_PRI_N9_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 9 Mask */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N9(value) (ARM_SYSCTRL_NVIC_IPR2_PRI_N9_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR2_PRI_N9_Pos))
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N10_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 10 Position */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N10_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR2_PRI_N10_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 10 Mask */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N10(value) (ARM_SYSCTRL_NVIC_IPR2_PRI_N10_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR2_PRI_N10_Pos))
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N11_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 11 Position */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N11_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR2_PRI_N11_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR2) Priority of Interrupt 11 Mask */
#define ARM_SYSCTRL_NVIC_IPR2_PRI_N11(value) (ARM_SYSCTRL_NVIC_IPR2_PRI_N11_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR2_PRI_N11_Pos))
#define ARM_SYSCTRL_NVIC_IPR2_MASK          _U_(0xC0C0C0C0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR2) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR2_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR2_Msk           _U_(0xC0C0C0C0)                                /**< (ARM_SYSCTRL_NVIC_IPR2) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR3 : (ARM_SYSCTRL Offset: 0x40c) (R/W 32) Interrupt Priority Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :6;                        /**< bit:   0..5  Reserved */
    uint32_t PRI_N12:2;                 /**< bit:   6..7  Priority of Interrupt 12                 */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N13:2;                 /**< bit: 14..15  Priority of Interrupt 13                 */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N14:2;                 /**< bit: 22..23  Priority of Interrupt 14                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N15:2;                 /**< bit: 30..31  Priority of Interrupt 15                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR3_OFFSET        (0x40C)                                       /**<  (ARM_SYSCTRL_NVIC_IPR3) Interrupt Priority Register 3  Offset */
#define ARM_SYSCTRL_NVIC_IPR3_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR3) Interrupt Priority Register 3  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR3_PRI_N12_Pos   6                                              /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 12 Position */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N12_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR3_PRI_N12_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 12 Mask */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N12(value) (ARM_SYSCTRL_NVIC_IPR3_PRI_N12_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR3_PRI_N12_Pos))
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N13_Pos   14                                             /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 13 Position */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N13_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR3_PRI_N13_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 13 Mask */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N13(value) (ARM_SYSCTRL_NVIC_IPR3_PRI_N13_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR3_PRI_N13_Pos))
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N14_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 14 Position */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N14_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR3_PRI_N14_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 14 Mask */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N14(value) (ARM_SYSCTRL_NVIC_IPR3_PRI_N14_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR3_PRI_N14_Pos))
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N15_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 15 Position */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N15_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR3_PRI_N15_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR3) Priority of Interrupt 15 Mask */
#define ARM_SYSCTRL_NVIC_IPR3_PRI_N15(value) (ARM_SYSCTRL_NVIC_IPR3_PRI_N15_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR3_PRI_N15_Pos))
#define ARM_SYSCTRL_NVIC_IPR3_MASK          _U_(0xC0C0C0C0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR3) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR3_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR3_Msk           _U_(0xC0C0C0C0)                                /**< (ARM_SYSCTRL_NVIC_IPR3) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR4 : (ARM_SYSCTRL Offset: 0x410) (R/W 32) Interrupt Priority Register 4 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :5;                        /**< bit:   0..4  Reserved */
    uint32_t PRI_N16:3;                 /**< bit:   5..7  Priority of Interrupt 16                 */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N17:2;                 /**< bit: 14..15  Priority of Interrupt 17                 */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N18:2;                 /**< bit: 22..23  Priority of Interrupt 18                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N19:2;                 /**< bit: 30..31  Priority of Interrupt 19                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR4_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR4_OFFSET        (0x410)                                       /**<  (ARM_SYSCTRL_NVIC_IPR4) Interrupt Priority Register 4  Offset */
#define ARM_SYSCTRL_NVIC_IPR4_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR4) Interrupt Priority Register 4  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR4_PRI_N16_Pos   5                                              /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 16 Position */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N16_Msk   (_U_(0x7) << ARM_SYSCTRL_NVIC_IPR4_PRI_N16_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 16 Mask */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N16(value) (ARM_SYSCTRL_NVIC_IPR4_PRI_N16_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR4_PRI_N16_Pos))
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N17_Pos   14                                             /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 17 Position */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N17_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR4_PRI_N17_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 17 Mask */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N17(value) (ARM_SYSCTRL_NVIC_IPR4_PRI_N17_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR4_PRI_N17_Pos))
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N18_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 18 Position */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N18_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR4_PRI_N18_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 18 Mask */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N18(value) (ARM_SYSCTRL_NVIC_IPR4_PRI_N18_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR4_PRI_N18_Pos))
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N19_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 19 Position */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N19_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR4_PRI_N19_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR4) Priority of Interrupt 19 Mask */
#define ARM_SYSCTRL_NVIC_IPR4_PRI_N19(value) (ARM_SYSCTRL_NVIC_IPR4_PRI_N19_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR4_PRI_N19_Pos))
#define ARM_SYSCTRL_NVIC_IPR4_MASK          _U_(0xC0C0C0E0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR4) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR4_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR4_Msk           _U_(0xC0C0C0E0)                                /**< (ARM_SYSCTRL_NVIC_IPR4) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR5 : (ARM_SYSCTRL Offset: 0x414) (R/W 32) Interrupt Priority Register 5 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :5;                        /**< bit:   0..4  Reserved */
    uint32_t PRI_N20:3;                 /**< bit:   5..7  Priority of Interrupt 20                 */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N21:2;                 /**< bit: 14..15  Priority of Interrupt 21                 */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N22:2;                 /**< bit: 22..23  Priority of Interrupt 22                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N23:2;                 /**< bit: 30..31  Priority of Interrupt 23                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR5_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR5_OFFSET        (0x414)                                       /**<  (ARM_SYSCTRL_NVIC_IPR5) Interrupt Priority Register 5  Offset */
#define ARM_SYSCTRL_NVIC_IPR5_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR5) Interrupt Priority Register 5  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR5_PRI_N20_Pos   5                                              /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 20 Position */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N20_Msk   (_U_(0x7) << ARM_SYSCTRL_NVIC_IPR5_PRI_N20_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 20 Mask */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N20(value) (ARM_SYSCTRL_NVIC_IPR5_PRI_N20_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR5_PRI_N20_Pos))
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N21_Pos   14                                             /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 21 Position */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N21_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR5_PRI_N21_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 21 Mask */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N21(value) (ARM_SYSCTRL_NVIC_IPR5_PRI_N21_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR5_PRI_N21_Pos))
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N22_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 22 Position */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N22_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR5_PRI_N22_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 22 Mask */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N22(value) (ARM_SYSCTRL_NVIC_IPR5_PRI_N22_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR5_PRI_N22_Pos))
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N23_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 23 Position */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N23_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR5_PRI_N23_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR5) Priority of Interrupt 23 Mask */
#define ARM_SYSCTRL_NVIC_IPR5_PRI_N23(value) (ARM_SYSCTRL_NVIC_IPR5_PRI_N23_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR5_PRI_N23_Pos))
#define ARM_SYSCTRL_NVIC_IPR5_MASK          _U_(0xC0C0C0E0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR5) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR5_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR5_Msk           _U_(0xC0C0C0E0)                                /**< (ARM_SYSCTRL_NVIC_IPR5) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR6 : (ARM_SYSCTRL Offset: 0x418) (R/W 32) Interrupt Priority Register 6 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :5;                        /**< bit:   0..4  Reserved */
    uint32_t PRI_N24:3;                 /**< bit:   5..7  Priority of Interrupt 24                 */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N25:2;                 /**< bit: 14..15  Priority of Interrupt 25                 */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N26:2;                 /**< bit: 22..23  Priority of Interrupt 26                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N27:2;                 /**< bit: 30..31  Priority of Interrupt 27                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR6_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR6_OFFSET        (0x418)                                       /**<  (ARM_SYSCTRL_NVIC_IPR6) Interrupt Priority Register 6  Offset */
#define ARM_SYSCTRL_NVIC_IPR6_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR6) Interrupt Priority Register 6  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR6_PRI_N24_Pos   5                                              /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 24 Position */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N24_Msk   (_U_(0x7) << ARM_SYSCTRL_NVIC_IPR6_PRI_N24_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 24 Mask */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N24(value) (ARM_SYSCTRL_NVIC_IPR6_PRI_N24_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR6_PRI_N24_Pos))
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N25_Pos   14                                             /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 25 Position */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N25_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR6_PRI_N25_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 25 Mask */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N25(value) (ARM_SYSCTRL_NVIC_IPR6_PRI_N25_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR6_PRI_N25_Pos))
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N26_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 26 Position */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N26_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR6_PRI_N26_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 26 Mask */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N26(value) (ARM_SYSCTRL_NVIC_IPR6_PRI_N26_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR6_PRI_N26_Pos))
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N27_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 27 Position */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N27_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR6_PRI_N27_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR6) Priority of Interrupt 27 Mask */
#define ARM_SYSCTRL_NVIC_IPR6_PRI_N27(value) (ARM_SYSCTRL_NVIC_IPR6_PRI_N27_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR6_PRI_N27_Pos))
#define ARM_SYSCTRL_NVIC_IPR6_MASK          _U_(0xC0C0C0E0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR6) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR6_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR6_Msk           _U_(0xC0C0C0E0)                                /**< (ARM_SYSCTRL_NVIC_IPR6) Register Mask  */


/* -------- ARM_SYSCTRL_NVIC_IPR7 : (ARM_SYSCTRL Offset: 0x41c) (R/W 32) Interrupt Priority Register 7 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :5;                        /**< bit:   0..4  Reserved */
    uint32_t PRI_N28:3;                 /**< bit:   5..7  Priority of Interrupt 28                 */
    uint32_t :6;                        /**< bit:  8..13  Reserved */
    uint32_t PRI_N29:2;                 /**< bit: 14..15  Priority of Interrupt 29                 */
    uint32_t :6;                        /**< bit: 16..21  Reserved */
    uint32_t PRI_N30:2;                 /**< bit: 22..23  Priority of Interrupt 30                 */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_N31:2;                 /**< bit: 30..31  Priority of Interrupt 31                 */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_NVIC_IPR7_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_NVIC_IPR7_OFFSET        (0x41C)                                       /**<  (ARM_SYSCTRL_NVIC_IPR7) Interrupt Priority Register 7  Offset */
#define ARM_SYSCTRL_NVIC_IPR7_RESETVALUE    _U_(0x00)                                     /**<  (ARM_SYSCTRL_NVIC_IPR7) Interrupt Priority Register 7  Reset Value */

#define ARM_SYSCTRL_NVIC_IPR7_PRI_N28_Pos   5                                              /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 28 Position */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N28_Msk   (_U_(0x7) << ARM_SYSCTRL_NVIC_IPR7_PRI_N28_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 28 Mask */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N28(value) (ARM_SYSCTRL_NVIC_IPR7_PRI_N28_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR7_PRI_N28_Pos))
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N29_Pos   14                                             /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 29 Position */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N29_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR7_PRI_N29_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 29 Mask */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N29(value) (ARM_SYSCTRL_NVIC_IPR7_PRI_N29_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR7_PRI_N29_Pos))
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N30_Pos   22                                             /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 30 Position */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N30_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR7_PRI_N30_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 30 Mask */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N30(value) (ARM_SYSCTRL_NVIC_IPR7_PRI_N30_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR7_PRI_N30_Pos))
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N31_Pos   30                                             /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 31 Position */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N31_Msk   (_U_(0x3) << ARM_SYSCTRL_NVIC_IPR7_PRI_N31_Pos)  /**< (ARM_SYSCTRL_NVIC_IPR7) Priority of Interrupt 31 Mask */
#define ARM_SYSCTRL_NVIC_IPR7_PRI_N31(value) (ARM_SYSCTRL_NVIC_IPR7_PRI_N31_Msk & ((value) << ARM_SYSCTRL_NVIC_IPR7_PRI_N31_Pos))
#define ARM_SYSCTRL_NVIC_IPR7_MASK          _U_(0xC0C0C0E0)                                /**< \deprecated (ARM_SYSCTRL_NVIC_IPR7) Register MASK  (Use ARM_SYSCTRL_NVIC_IPR7_Msk instead)  */
#define ARM_SYSCTRL_NVIC_IPR7_Msk           _U_(0xC0C0C0E0)                                /**< (ARM_SYSCTRL_NVIC_IPR7) Register Mask  */


/* -------- ARM_SYSCTRL_CPUID : (ARM_SYSCTRL Offset: 0xd00) (R/ 32) CPU Identification Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t CPUID:32;                  /**< bit:  0..31                                           */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_CPUID_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_CPUID_OFFSET            (0xD00)                                       /**<  (ARM_SYSCTRL_CPUID) CPU Identification Register  Offset */
#define ARM_SYSCTRL_CPUID_RESETVALUE        _U_(0x410CC200)                               /**<  (ARM_SYSCTRL_CPUID) CPU Identification Register  Reset Value */

#define ARM_SYSCTRL_CPUID_CPUID_Pos         0                                              /**< (ARM_SYSCTRL_CPUID)  Position */
#define ARM_SYSCTRL_CPUID_CPUID_Msk         (_U_(0xFFFFFFFF) << ARM_SYSCTRL_CPUID_CPUID_Pos)  /**< (ARM_SYSCTRL_CPUID)  Mask */
#define ARM_SYSCTRL_CPUID_CPUID(value)      (ARM_SYSCTRL_CPUID_CPUID_Msk & ((value) << ARM_SYSCTRL_CPUID_CPUID_Pos))
#define ARM_SYSCTRL_CPUID_MASK              _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_CPUID) Register MASK  (Use ARM_SYSCTRL_CPUID_Msk instead)  */
#define ARM_SYSCTRL_CPUID_Msk               _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_CPUID) Register Mask  */


/* -------- ARM_SYSCTRL_ICSR : (ARM_SYSCTRL Offset: 0xd04) (R/ 32) Interrupt Control State Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t VECTACTIVE:9;              /**< bit:   0..8  Exception number for the current executing exception */
    uint32_t :3;                        /**< bit:  9..11  Reserved */
    uint32_t VECTPENDING:9;             /**< bit: 12..20  Exception number for highest priority pending exception */
    uint32_t :1;                        /**< bit:     21  Reserved */
    uint32_t ISRPENDING:1;              /**< bit:     22  If 1, Interrupt is pending               */
    uint32_t ISRPREEMPT:1;              /**< bit:     23  If 1, will service a pending exception   */
    uint32_t :1;                        /**< bit:     24  Reserved */
    uint32_t PENDSTCLR:1;               /**< bit:     25  Clear Pending SysTick                    */
    uint32_t PENDSTSET:1;               /**< bit:     26  Set pending SysTick                      */
    uint32_t PENDSVCLR:1;               /**< bit:     27  Clearing pending PendSV                  */
    uint32_t PENDSVSET:1;               /**< bit:     28  Set pending PendSV interrupt             */
    uint32_t :2;                        /**< bit: 29..30  Reserved */
    uint32_t NMIPENDSET:1;              /**< bit:     31  Activate NMI Exception                   */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_ICSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_ICSR_OFFSET             (0xD04)                                       /**<  (ARM_SYSCTRL_ICSR) Interrupt Control State Register  Offset */
#define ARM_SYSCTRL_ICSR_RESETVALUE         _U_(0x00)                                     /**<  (ARM_SYSCTRL_ICSR) Interrupt Control State Register  Reset Value */

#define ARM_SYSCTRL_ICSR_VECTACTIVE_Pos     0                                              /**< (ARM_SYSCTRL_ICSR) Exception number for the current executing exception Position */
#define ARM_SYSCTRL_ICSR_VECTACTIVE_Msk     (_U_(0x1FF) << ARM_SYSCTRL_ICSR_VECTACTIVE_Pos)  /**< (ARM_SYSCTRL_ICSR) Exception number for the current executing exception Mask */
#define ARM_SYSCTRL_ICSR_VECTACTIVE(value)  (ARM_SYSCTRL_ICSR_VECTACTIVE_Msk & ((value) << ARM_SYSCTRL_ICSR_VECTACTIVE_Pos))
#define ARM_SYSCTRL_ICSR_VECTPENDING_Pos    12                                             /**< (ARM_SYSCTRL_ICSR) Exception number for highest priority pending exception Position */
#define ARM_SYSCTRL_ICSR_VECTPENDING_Msk    (_U_(0x1FF) << ARM_SYSCTRL_ICSR_VECTPENDING_Pos)  /**< (ARM_SYSCTRL_ICSR) Exception number for highest priority pending exception Mask */
#define ARM_SYSCTRL_ICSR_VECTPENDING(value) (ARM_SYSCTRL_ICSR_VECTPENDING_Msk & ((value) << ARM_SYSCTRL_ICSR_VECTPENDING_Pos))
#define ARM_SYSCTRL_ICSR_ISRPENDING_Pos     22                                             /**< (ARM_SYSCTRL_ICSR) If 1, Interrupt is pending Position */
#define ARM_SYSCTRL_ICSR_ISRPENDING_Msk     (_U_(0x1) << ARM_SYSCTRL_ICSR_ISRPENDING_Pos)  /**< (ARM_SYSCTRL_ICSR) If 1, Interrupt is pending Mask */
#define ARM_SYSCTRL_ICSR_ISRPENDING         ARM_SYSCTRL_ICSR_ISRPENDING_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_ISRPENDING_Msk instead */
#define ARM_SYSCTRL_ICSR_ISRPREEMPT_Pos     23                                             /**< (ARM_SYSCTRL_ICSR) If 1, will service a pending exception Position */
#define ARM_SYSCTRL_ICSR_ISRPREEMPT_Msk     (_U_(0x1) << ARM_SYSCTRL_ICSR_ISRPREEMPT_Pos)  /**< (ARM_SYSCTRL_ICSR) If 1, will service a pending exception Mask */
#define ARM_SYSCTRL_ICSR_ISRPREEMPT         ARM_SYSCTRL_ICSR_ISRPREEMPT_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_ISRPREEMPT_Msk instead */
#define ARM_SYSCTRL_ICSR_PENDSTCLR_Pos      25                                             /**< (ARM_SYSCTRL_ICSR) Clear Pending SysTick Position */
#define ARM_SYSCTRL_ICSR_PENDSTCLR_Msk      (_U_(0x1) << ARM_SYSCTRL_ICSR_PENDSTCLR_Pos)   /**< (ARM_SYSCTRL_ICSR) Clear Pending SysTick Mask */
#define ARM_SYSCTRL_ICSR_PENDSTCLR          ARM_SYSCTRL_ICSR_PENDSTCLR_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_PENDSTCLR_Msk instead */
#define ARM_SYSCTRL_ICSR_PENDSTSET_Pos      26                                             /**< (ARM_SYSCTRL_ICSR) Set pending SysTick Position */
#define ARM_SYSCTRL_ICSR_PENDSTSET_Msk      (_U_(0x1) << ARM_SYSCTRL_ICSR_PENDSTSET_Pos)   /**< (ARM_SYSCTRL_ICSR) Set pending SysTick Mask */
#define ARM_SYSCTRL_ICSR_PENDSTSET          ARM_SYSCTRL_ICSR_PENDSTSET_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_PENDSTSET_Msk instead */
#define ARM_SYSCTRL_ICSR_PENDSVCLR_Pos      27                                             /**< (ARM_SYSCTRL_ICSR) Clearing pending PendSV Position */
#define ARM_SYSCTRL_ICSR_PENDSVCLR_Msk      (_U_(0x1) << ARM_SYSCTRL_ICSR_PENDSVCLR_Pos)   /**< (ARM_SYSCTRL_ICSR) Clearing pending PendSV Mask */
#define ARM_SYSCTRL_ICSR_PENDSVCLR          ARM_SYSCTRL_ICSR_PENDSVCLR_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_PENDSVCLR_Msk instead */
#define ARM_SYSCTRL_ICSR_PENDSVSET_Pos      28                                             /**< (ARM_SYSCTRL_ICSR) Set pending PendSV interrupt Position */
#define ARM_SYSCTRL_ICSR_PENDSVSET_Msk      (_U_(0x1) << ARM_SYSCTRL_ICSR_PENDSVSET_Pos)   /**< (ARM_SYSCTRL_ICSR) Set pending PendSV interrupt Mask */
#define ARM_SYSCTRL_ICSR_PENDSVSET          ARM_SYSCTRL_ICSR_PENDSVSET_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_PENDSVSET_Msk instead */
#define ARM_SYSCTRL_ICSR_NMIPENDSET_Pos     31                                             /**< (ARM_SYSCTRL_ICSR) Activate NMI Exception Position */
#define ARM_SYSCTRL_ICSR_NMIPENDSET_Msk     (_U_(0x1) << ARM_SYSCTRL_ICSR_NMIPENDSET_Pos)  /**< (ARM_SYSCTRL_ICSR) Activate NMI Exception Mask */
#define ARM_SYSCTRL_ICSR_NMIPENDSET         ARM_SYSCTRL_ICSR_NMIPENDSET_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_ICSR_NMIPENDSET_Msk instead */
#define ARM_SYSCTRL_ICSR_MASK               _U_(0x9EDFF1FF)                                /**< \deprecated (ARM_SYSCTRL_ICSR) Register MASK  (Use ARM_SYSCTRL_ICSR_Msk instead)  */
#define ARM_SYSCTRL_ICSR_Msk                _U_(0x9EDFF1FF)                                /**< (ARM_SYSCTRL_ICSR) Register Mask  */


/* -------- ARM_SYSCTRL_AIRCR : (ARM_SYSCTRL Offset: 0xd0c) (R/ 32) Application Interrupt and Reset Control Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :1;                        /**< bit:      0  Reserved */
    uint32_t VECTCLRACTIVE:1;           /**< bit:      1  Clears all active state information for fixed and configurable exceptions */
    uint32_t SYSRESETREQ:1;             /**< bit:      2  Active High System Reset Request         */
    uint32_t :12;                       /**< bit:  3..14  Reserved */
    uint32_t ENDIANNESS:1;              /**< bit:     15  If 1 then big endian                     */
    uint32_t VECTKEY:16;                /**< bit: 16..31  Vector Key, must write 0x05FA to this otherwise the write is UNPREDICTABLE */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_AIRCR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_AIRCR_OFFSET            (0xD0C)                                       /**<  (ARM_SYSCTRL_AIRCR) Application Interrupt and Reset Control Register  Offset */
#define ARM_SYSCTRL_AIRCR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_AIRCR) Application Interrupt and Reset Control Register  Reset Value */

#define ARM_SYSCTRL_AIRCR_VECTCLRACTIVE_Pos 1                                              /**< (ARM_SYSCTRL_AIRCR) Clears all active state information for fixed and configurable exceptions Position */
#define ARM_SYSCTRL_AIRCR_VECTCLRACTIVE_Msk (_U_(0x1) << ARM_SYSCTRL_AIRCR_VECTCLRACTIVE_Pos)  /**< (ARM_SYSCTRL_AIRCR) Clears all active state information for fixed and configurable exceptions Mask */
#define ARM_SYSCTRL_AIRCR_VECTCLRACTIVE     ARM_SYSCTRL_AIRCR_VECTCLRACTIVE_Msk            /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_AIRCR_VECTCLRACTIVE_Msk instead */
#define ARM_SYSCTRL_AIRCR_SYSRESETREQ_Pos   2                                              /**< (ARM_SYSCTRL_AIRCR) Active High System Reset Request Position */
#define ARM_SYSCTRL_AIRCR_SYSRESETREQ_Msk   (_U_(0x1) << ARM_SYSCTRL_AIRCR_SYSRESETREQ_Pos)  /**< (ARM_SYSCTRL_AIRCR) Active High System Reset Request Mask */
#define ARM_SYSCTRL_AIRCR_SYSRESETREQ       ARM_SYSCTRL_AIRCR_SYSRESETREQ_Msk              /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_AIRCR_SYSRESETREQ_Msk instead */
#define ARM_SYSCTRL_AIRCR_ENDIANNESS_Pos    15                                             /**< (ARM_SYSCTRL_AIRCR) If 1 then big endian Position */
#define ARM_SYSCTRL_AIRCR_ENDIANNESS_Msk    (_U_(0x1) << ARM_SYSCTRL_AIRCR_ENDIANNESS_Pos)  /**< (ARM_SYSCTRL_AIRCR) If 1 then big endian Mask */
#define ARM_SYSCTRL_AIRCR_ENDIANNESS        ARM_SYSCTRL_AIRCR_ENDIANNESS_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_AIRCR_ENDIANNESS_Msk instead */
#define ARM_SYSCTRL_AIRCR_VECTKEY_Pos       16                                             /**< (ARM_SYSCTRL_AIRCR) Vector Key, must write 0x05FA to this otherwise the write is UNPREDICTABLE Position */
#define ARM_SYSCTRL_AIRCR_VECTKEY_Msk       (_U_(0xFFFF) << ARM_SYSCTRL_AIRCR_VECTKEY_Pos)  /**< (ARM_SYSCTRL_AIRCR) Vector Key, must write 0x05FA to this otherwise the write is UNPREDICTABLE Mask */
#define ARM_SYSCTRL_AIRCR_VECTKEY(value)    (ARM_SYSCTRL_AIRCR_VECTKEY_Msk & ((value) << ARM_SYSCTRL_AIRCR_VECTKEY_Pos))
#define ARM_SYSCTRL_AIRCR_MASK              _U_(0xFFFF8006)                                /**< \deprecated (ARM_SYSCTRL_AIRCR) Register MASK  (Use ARM_SYSCTRL_AIRCR_Msk instead)  */
#define ARM_SYSCTRL_AIRCR_Msk               _U_(0xFFFF8006)                                /**< (ARM_SYSCTRL_AIRCR) Register Mask  */


/* -------- ARM_SYSCTRL_SCR : (ARM_SYSCTRL Offset: 0xd10) (R/W 8) System Control Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  :1;                        /**< bit:      0  Reserved */
    uint8_t  SLEEPONEXIT:1;             /**< bit:      1  Enter sleep state on exit from an ISR    */
    uint8_t  SLEEPDEEP:1;               /**< bit:      2  Selected sleep state is deep sleep       */
    uint8_t  :1;                        /**< bit:      3  Reserved */
    uint8_t  SEVONPEND:1;               /**< bit:      4  Transitions from inactive to pending are wakeup events */
    uint8_t  :3;                        /**< bit:   5..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCR_OFFSET              (0xD10)                                       /**<  (ARM_SYSCTRL_SCR) System Control Register  Offset */
#define ARM_SYSCTRL_SCR_RESETVALUE          _U_(0x00)                                     /**<  (ARM_SYSCTRL_SCR) System Control Register  Reset Value */

#define ARM_SYSCTRL_SCR_SLEEPONEXIT_Pos     1                                              /**< (ARM_SYSCTRL_SCR) Enter sleep state on exit from an ISR Position */
#define ARM_SYSCTRL_SCR_SLEEPONEXIT_Msk     (_U_(0x1) << ARM_SYSCTRL_SCR_SLEEPONEXIT_Pos)  /**< (ARM_SYSCTRL_SCR) Enter sleep state on exit from an ISR Mask */
#define ARM_SYSCTRL_SCR_SLEEPONEXIT         ARM_SYSCTRL_SCR_SLEEPONEXIT_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SCR_SLEEPONEXIT_Msk instead */
#define ARM_SYSCTRL_SCR_SLEEPDEEP_Pos       2                                              /**< (ARM_SYSCTRL_SCR) Selected sleep state is deep sleep Position */
#define ARM_SYSCTRL_SCR_SLEEPDEEP_Msk       (_U_(0x1) << ARM_SYSCTRL_SCR_SLEEPDEEP_Pos)    /**< (ARM_SYSCTRL_SCR) Selected sleep state is deep sleep Mask */
#define ARM_SYSCTRL_SCR_SLEEPDEEP           ARM_SYSCTRL_SCR_SLEEPDEEP_Msk                  /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SCR_SLEEPDEEP_Msk instead */
#define ARM_SYSCTRL_SCR_SEVONPEND_Pos       4                                              /**< (ARM_SYSCTRL_SCR) Transitions from inactive to pending are wakeup events Position */
#define ARM_SYSCTRL_SCR_SEVONPEND_Msk       (_U_(0x1) << ARM_SYSCTRL_SCR_SEVONPEND_Pos)    /**< (ARM_SYSCTRL_SCR) Transitions from inactive to pending are wakeup events Mask */
#define ARM_SYSCTRL_SCR_SEVONPEND           ARM_SYSCTRL_SCR_SEVONPEND_Msk                  /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SCR_SEVONPEND_Msk instead */
#define ARM_SYSCTRL_SCR_MASK                _U_(0x16)                                      /**< \deprecated (ARM_SYSCTRL_SCR) Register MASK  (Use ARM_SYSCTRL_SCR_Msk instead)  */
#define ARM_SYSCTRL_SCR_Msk                 _U_(0x16)                                      /**< (ARM_SYSCTRL_SCR) Register Mask  */


/* -------- ARM_SYSCTRL_CCR : (ARM_SYSCTRL Offset: 0xd14) (R/ 16) Configuration and Control Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint16_t :3;                        /**< bit:   0..2  Reserved */
    uint16_t UNALIGN_TRP:1;             /**< bit:      3  Unaligned work and halfword accesses generate a HardFault exception */
    uint16_t :5;                        /**< bit:   4..8  Reserved */
    uint16_t STKALIGN:1;                /**< bit:      9  On exception entry, SP used prior to exception is adjusted to be 8-byte aligned and context to restore it is saved */
    uint16_t :6;                        /**< bit: 10..15  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint16_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_CCR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_CCR_OFFSET              (0xD14)                                       /**<  (ARM_SYSCTRL_CCR) Configuration and Control Register  Offset */
#define ARM_SYSCTRL_CCR_RESETVALUE          _U_(0x00)                                     /**<  (ARM_SYSCTRL_CCR) Configuration and Control Register  Reset Value */

#define ARM_SYSCTRL_CCR_UNALIGN_TRP_Pos     3                                              /**< (ARM_SYSCTRL_CCR) Unaligned work and halfword accesses generate a HardFault exception Position */
#define ARM_SYSCTRL_CCR_UNALIGN_TRP_Msk     (_U_(0x1) << ARM_SYSCTRL_CCR_UNALIGN_TRP_Pos)  /**< (ARM_SYSCTRL_CCR) Unaligned work and halfword accesses generate a HardFault exception Mask */
#define ARM_SYSCTRL_CCR_UNALIGN_TRP         ARM_SYSCTRL_CCR_UNALIGN_TRP_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_CCR_UNALIGN_TRP_Msk instead */
#define ARM_SYSCTRL_CCR_STKALIGN_Pos        9                                              /**< (ARM_SYSCTRL_CCR) On exception entry, SP used prior to exception is adjusted to be 8-byte aligned and context to restore it is saved Position */
#define ARM_SYSCTRL_CCR_STKALIGN_Msk        (_U_(0x1) << ARM_SYSCTRL_CCR_STKALIGN_Pos)     /**< (ARM_SYSCTRL_CCR) On exception entry, SP used prior to exception is adjusted to be 8-byte aligned and context to restore it is saved Mask */
#define ARM_SYSCTRL_CCR_STKALIGN            ARM_SYSCTRL_CCR_STKALIGN_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_CCR_STKALIGN_Msk instead */
#define ARM_SYSCTRL_CCR_MASK                _U_(0x208)                                     /**< \deprecated (ARM_SYSCTRL_CCR) Register MASK  (Use ARM_SYSCTRL_CCR_Msk instead)  */
#define ARM_SYSCTRL_CCR_Msk                 _U_(0x208)                                     /**< (ARM_SYSCTRL_CCR) Register Mask  */


/* -------- ARM_SYSCTRL_SHPR2 : (ARM_SYSCTRL Offset: 0xd1c) (R/W 32) System Handler Priority Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :30;                       /**< bit:  0..29  Reserved */
    uint32_t PRI_11:2;                  /**< bit: 30..31  Priority of system handler 11, SVCall    */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SHPR2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SHPR2_OFFSET            (0xD1C)                                       /**<  (ARM_SYSCTRL_SHPR2) System Handler Priority Register 2  Offset */
#define ARM_SYSCTRL_SHPR2_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_SHPR2) System Handler Priority Register 2  Reset Value */

#define ARM_SYSCTRL_SHPR2_PRI_11_Pos        30                                             /**< (ARM_SYSCTRL_SHPR2) Priority of system handler 11, SVCall Position */
#define ARM_SYSCTRL_SHPR2_PRI_11_Msk        (_U_(0x3) << ARM_SYSCTRL_SHPR2_PRI_11_Pos)     /**< (ARM_SYSCTRL_SHPR2) Priority of system handler 11, SVCall Mask */
#define ARM_SYSCTRL_SHPR2_PRI_11(value)     (ARM_SYSCTRL_SHPR2_PRI_11_Msk & ((value) << ARM_SYSCTRL_SHPR2_PRI_11_Pos))
#define ARM_SYSCTRL_SHPR2_MASK              _U_(0xC0000000)                                /**< \deprecated (ARM_SYSCTRL_SHPR2) Register MASK  (Use ARM_SYSCTRL_SHPR2_Msk instead)  */
#define ARM_SYSCTRL_SHPR2_Msk               _U_(0xC0000000)                                /**< (ARM_SYSCTRL_SHPR2) Register Mask  */


/* -------- ARM_SYSCTRL_SHPR3 : (ARM_SYSCTRL Offset: 0xd20) (R/W 32) System Handler Priority Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t :22;                       /**< bit:  0..21  Reserved */
    uint32_t PRI_14:2;                  /**< bit: 22..23  Priority of system handler 14, PendSV    */
    uint32_t :6;                        /**< bit: 24..29  Reserved */
    uint32_t PRI_15:2;                  /**< bit: 30..31  Priority of system handler 15, SysTick   */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SHPR3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SHPR3_OFFSET            (0xD20)                                       /**<  (ARM_SYSCTRL_SHPR3) System Handler Priority Register 3  Offset */
#define ARM_SYSCTRL_SHPR3_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_SHPR3) System Handler Priority Register 3  Reset Value */

#define ARM_SYSCTRL_SHPR3_PRI_14_Pos        22                                             /**< (ARM_SYSCTRL_SHPR3) Priority of system handler 14, PendSV Position */
#define ARM_SYSCTRL_SHPR3_PRI_14_Msk        (_U_(0x3) << ARM_SYSCTRL_SHPR3_PRI_14_Pos)     /**< (ARM_SYSCTRL_SHPR3) Priority of system handler 14, PendSV Mask */
#define ARM_SYSCTRL_SHPR3_PRI_14(value)     (ARM_SYSCTRL_SHPR3_PRI_14_Msk & ((value) << ARM_SYSCTRL_SHPR3_PRI_14_Pos))
#define ARM_SYSCTRL_SHPR3_PRI_15_Pos        30                                             /**< (ARM_SYSCTRL_SHPR3) Priority of system handler 15, SysTick Position */
#define ARM_SYSCTRL_SHPR3_PRI_15_Msk        (_U_(0x3) << ARM_SYSCTRL_SHPR3_PRI_15_Pos)     /**< (ARM_SYSCTRL_SHPR3) Priority of system handler 15, SysTick Mask */
#define ARM_SYSCTRL_SHPR3_PRI_15(value)     (ARM_SYSCTRL_SHPR3_PRI_15_Msk & ((value) << ARM_SYSCTRL_SHPR3_PRI_15_Pos))
#define ARM_SYSCTRL_SHPR3_MASK              _U_(0xC0C00000)                                /**< \deprecated (ARM_SYSCTRL_SHPR3) Register MASK  (Use ARM_SYSCTRL_SHPR3_Msk instead)  */
#define ARM_SYSCTRL_SHPR3_Msk               _U_(0xC0C00000)                                /**< (ARM_SYSCTRL_SHPR3) Register Mask  */


/* -------- ARM_SYSCTRL_SHCSR : (ARM_SYSCTRL Offset: 0xd24) (R/W 16) System Handler Control and State Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint16_t :15;                       /**< bit:  0..14  Reserved */
    uint16_t SVCALLPENDED:1;            /**< bit:     15  SVCall is pending                        */
  } bit;                                /**< Structure used for bit  access */
  uint16_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SHCSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SHCSR_OFFSET            (0xD24)                                       /**<  (ARM_SYSCTRL_SHCSR) System Handler Control and State Register  Offset */
#define ARM_SYSCTRL_SHCSR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_SHCSR) System Handler Control and State Register  Reset Value */

#define ARM_SYSCTRL_SHCSR_SVCALLPENDED_Pos  15                                             /**< (ARM_SYSCTRL_SHCSR) SVCall is pending Position */
#define ARM_SYSCTRL_SHCSR_SVCALLPENDED_Msk  (_U_(0x1) << ARM_SYSCTRL_SHCSR_SVCALLPENDED_Pos)  /**< (ARM_SYSCTRL_SHCSR) SVCall is pending Mask */
#define ARM_SYSCTRL_SHCSR_SVCALLPENDED      ARM_SYSCTRL_SHCSR_SVCALLPENDED_Msk             /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SHCSR_SVCALLPENDED_Msk instead */
#define ARM_SYSCTRL_SHCSR_MASK              _U_(0x8000)                                    /**< \deprecated (ARM_SYSCTRL_SHCSR) Register MASK  (Use ARM_SYSCTRL_SHCSR_Msk instead)  */
#define ARM_SYSCTRL_SHCSR_Msk               _U_(0x8000)                                    /**< (ARM_SYSCTRL_SHCSR) Register Mask  */


/* -------- ARM_SYSCTRL_DFSR : (ARM_SYSCTRL Offset: 0xd30) (R/W 8) Debug Fault Status Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  HALTED:1;                  /**< bit:      0  Halt Request Debug Event Active          */
    uint8_t  BKPT:1;                    /**< bit:      1  At least one breakpoint debug event      */
    uint8_t  DWTTRAP:1;                 /**< bit:      2  At least one debug event generated by DWT */
    uint8_t  VCATCH:1;                  /**< bit:      3  Vector catch debug event generated       */
    uint8_t  EXTERNAL:1;                /**< bit:      4  EDBGRQ debug event                       */
    uint8_t  :3;                        /**< bit:   5..7  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_DFSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_DFSR_OFFSET             (0xD30)                                       /**<  (ARM_SYSCTRL_DFSR) Debug Fault Status Register  Offset */
#define ARM_SYSCTRL_DFSR_RESETVALUE         _U_(0x00)                                     /**<  (ARM_SYSCTRL_DFSR) Debug Fault Status Register  Reset Value */

#define ARM_SYSCTRL_DFSR_HALTED_Pos         0                                              /**< (ARM_SYSCTRL_DFSR) Halt Request Debug Event Active Position */
#define ARM_SYSCTRL_DFSR_HALTED_Msk         (_U_(0x1) << ARM_SYSCTRL_DFSR_HALTED_Pos)      /**< (ARM_SYSCTRL_DFSR) Halt Request Debug Event Active Mask */
#define ARM_SYSCTRL_DFSR_HALTED             ARM_SYSCTRL_DFSR_HALTED_Msk                    /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DFSR_HALTED_Msk instead */
#define ARM_SYSCTRL_DFSR_BKPT_Pos           1                                              /**< (ARM_SYSCTRL_DFSR) At least one breakpoint debug event Position */
#define ARM_SYSCTRL_DFSR_BKPT_Msk           (_U_(0x1) << ARM_SYSCTRL_DFSR_BKPT_Pos)        /**< (ARM_SYSCTRL_DFSR) At least one breakpoint debug event Mask */
#define ARM_SYSCTRL_DFSR_BKPT               ARM_SYSCTRL_DFSR_BKPT_Msk                      /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DFSR_BKPT_Msk instead */
#define ARM_SYSCTRL_DFSR_DWTTRAP_Pos        2                                              /**< (ARM_SYSCTRL_DFSR) At least one debug event generated by DWT Position */
#define ARM_SYSCTRL_DFSR_DWTTRAP_Msk        (_U_(0x1) << ARM_SYSCTRL_DFSR_DWTTRAP_Pos)     /**< (ARM_SYSCTRL_DFSR) At least one debug event generated by DWT Mask */
#define ARM_SYSCTRL_DFSR_DWTTRAP            ARM_SYSCTRL_DFSR_DWTTRAP_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DFSR_DWTTRAP_Msk instead */
#define ARM_SYSCTRL_DFSR_VCATCH_Pos         3                                              /**< (ARM_SYSCTRL_DFSR) Vector catch debug event generated Position */
#define ARM_SYSCTRL_DFSR_VCATCH_Msk         (_U_(0x1) << ARM_SYSCTRL_DFSR_VCATCH_Pos)      /**< (ARM_SYSCTRL_DFSR) Vector catch debug event generated Mask */
#define ARM_SYSCTRL_DFSR_VCATCH             ARM_SYSCTRL_DFSR_VCATCH_Msk                    /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DFSR_VCATCH_Msk instead */
#define ARM_SYSCTRL_DFSR_EXTERNAL_Pos       4                                              /**< (ARM_SYSCTRL_DFSR) EDBGRQ debug event Position */
#define ARM_SYSCTRL_DFSR_EXTERNAL_Msk       (_U_(0x1) << ARM_SYSCTRL_DFSR_EXTERNAL_Pos)    /**< (ARM_SYSCTRL_DFSR) EDBGRQ debug event Mask */
#define ARM_SYSCTRL_DFSR_EXTERNAL           ARM_SYSCTRL_DFSR_EXTERNAL_Msk                  /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DFSR_EXTERNAL_Msk instead */
#define ARM_SYSCTRL_DFSR_MASK               _U_(0x1F)                                      /**< \deprecated (ARM_SYSCTRL_DFSR) Register MASK  (Use ARM_SYSCTRL_DFSR_Msk instead)  */
#define ARM_SYSCTRL_DFSR_Msk                _U_(0x1F)                                      /**< (ARM_SYSCTRL_DFSR) Register Mask  */


/* -------- ARM_SYSCTRL_DHCSR : (ARM_SYSCTRL Offset: 0xdf0) (R/W 32) Debug Halting Control and Status Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t C_DEBUGEN:1;               /**< bit:      0  Halting Debug Enabled (Must write 0xA05F to bits 31:16 to access) */
    uint32_t C_HALT:1;                  /**< bit:      1  Request a running processor to halt (Must write 0xA05F to bits 31:16 to access) */
    uint32_t C_STEP:1;                  /**< bit:      2  Single-Stepping Enabled (Must write 0xA05F to bits 31:16 to access) */
    uint32_t C_MASKINTS:1;              /**< bit:      3  Mask PendSV, SysTick and external configurable interrupts (Must write 0xA05F to bits 31:16 to access) */
    uint32_t :12;                       /**< bit:  4..15  Reserved */
    uint32_t S_REGRDY:1;                /**< bit:     16  Transfer to or from the DCRDR is complete */
    uint32_t S_HALT:1;                  /**< bit:     17  In Debug State                           */
    uint32_t S_SLEEP:1;                 /**< bit:     18  Sleeping                                 */
    uint32_t S_LOCKUP:1;                /**< bit:     19  Locked Up                                */
    uint32_t :4;                        /**< bit: 20..23  Reserved */
    uint32_t S_RETIRE_ST:1;             /**< bit:     24  At least one instruction has completed since last DHCSR read */
    uint32_t S_RESET_ST:1;              /**< bit:     25  At least one reset since last DHCSR read */
    uint32_t :6;                        /**< bit: 26..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_DHCSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_DHCSR_OFFSET            (0xDF0)                                       /**<  (ARM_SYSCTRL_DHCSR) Debug Halting Control and Status Register  Offset */
#define ARM_SYSCTRL_DHCSR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_DHCSR) Debug Halting Control and Status Register  Reset Value */

#define ARM_SYSCTRL_DHCSR_C_DEBUGEN_Pos     0                                              /**< (ARM_SYSCTRL_DHCSR) Halting Debug Enabled (Must write 0xA05F to bits 31:16 to access) Position */
#define ARM_SYSCTRL_DHCSR_C_DEBUGEN_Msk     (_U_(0x1) << ARM_SYSCTRL_DHCSR_C_DEBUGEN_Pos)  /**< (ARM_SYSCTRL_DHCSR) Halting Debug Enabled (Must write 0xA05F to bits 31:16 to access) Mask */
#define ARM_SYSCTRL_DHCSR_C_DEBUGEN         ARM_SYSCTRL_DHCSR_C_DEBUGEN_Msk                /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_C_DEBUGEN_Msk instead */
#define ARM_SYSCTRL_DHCSR_C_HALT_Pos        1                                              /**< (ARM_SYSCTRL_DHCSR) Request a running processor to halt (Must write 0xA05F to bits 31:16 to access) Position */
#define ARM_SYSCTRL_DHCSR_C_HALT_Msk        (_U_(0x1) << ARM_SYSCTRL_DHCSR_C_HALT_Pos)     /**< (ARM_SYSCTRL_DHCSR) Request a running processor to halt (Must write 0xA05F to bits 31:16 to access) Mask */
#define ARM_SYSCTRL_DHCSR_C_HALT            ARM_SYSCTRL_DHCSR_C_HALT_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_C_HALT_Msk instead */
#define ARM_SYSCTRL_DHCSR_C_STEP_Pos        2                                              /**< (ARM_SYSCTRL_DHCSR) Single-Stepping Enabled (Must write 0xA05F to bits 31:16 to access) Position */
#define ARM_SYSCTRL_DHCSR_C_STEP_Msk        (_U_(0x1) << ARM_SYSCTRL_DHCSR_C_STEP_Pos)     /**< (ARM_SYSCTRL_DHCSR) Single-Stepping Enabled (Must write 0xA05F to bits 31:16 to access) Mask */
#define ARM_SYSCTRL_DHCSR_C_STEP            ARM_SYSCTRL_DHCSR_C_STEP_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_C_STEP_Msk instead */
#define ARM_SYSCTRL_DHCSR_C_MASKINTS_Pos    3                                              /**< (ARM_SYSCTRL_DHCSR) Mask PendSV, SysTick and external configurable interrupts (Must write 0xA05F to bits 31:16 to access) Position */
#define ARM_SYSCTRL_DHCSR_C_MASKINTS_Msk    (_U_(0x1) << ARM_SYSCTRL_DHCSR_C_MASKINTS_Pos)  /**< (ARM_SYSCTRL_DHCSR) Mask PendSV, SysTick and external configurable interrupts (Must write 0xA05F to bits 31:16 to access) Mask */
#define ARM_SYSCTRL_DHCSR_C_MASKINTS        ARM_SYSCTRL_DHCSR_C_MASKINTS_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_C_MASKINTS_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_REGRDY_Pos      16                                             /**< (ARM_SYSCTRL_DHCSR) Transfer to or from the DCRDR is complete Position */
#define ARM_SYSCTRL_DHCSR_S_REGRDY_Msk      (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_REGRDY_Pos)   /**< (ARM_SYSCTRL_DHCSR) Transfer to or from the DCRDR is complete Mask */
#define ARM_SYSCTRL_DHCSR_S_REGRDY          ARM_SYSCTRL_DHCSR_S_REGRDY_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_REGRDY_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_HALT_Pos        17                                             /**< (ARM_SYSCTRL_DHCSR) In Debug State Position */
#define ARM_SYSCTRL_DHCSR_S_HALT_Msk        (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_HALT_Pos)     /**< (ARM_SYSCTRL_DHCSR) In Debug State Mask */
#define ARM_SYSCTRL_DHCSR_S_HALT            ARM_SYSCTRL_DHCSR_S_HALT_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_HALT_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_SLEEP_Pos       18                                             /**< (ARM_SYSCTRL_DHCSR) Sleeping Position */
#define ARM_SYSCTRL_DHCSR_S_SLEEP_Msk       (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_SLEEP_Pos)    /**< (ARM_SYSCTRL_DHCSR) Sleeping Mask */
#define ARM_SYSCTRL_DHCSR_S_SLEEP           ARM_SYSCTRL_DHCSR_S_SLEEP_Msk                  /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_SLEEP_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_LOCKUP_Pos      19                                             /**< (ARM_SYSCTRL_DHCSR) Locked Up Position */
#define ARM_SYSCTRL_DHCSR_S_LOCKUP_Msk      (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_LOCKUP_Pos)   /**< (ARM_SYSCTRL_DHCSR) Locked Up Mask */
#define ARM_SYSCTRL_DHCSR_S_LOCKUP          ARM_SYSCTRL_DHCSR_S_LOCKUP_Msk                 /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_LOCKUP_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_RETIRE_ST_Pos   24                                             /**< (ARM_SYSCTRL_DHCSR) At least one instruction has completed since last DHCSR read Position */
#define ARM_SYSCTRL_DHCSR_S_RETIRE_ST_Msk   (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_RETIRE_ST_Pos)  /**< (ARM_SYSCTRL_DHCSR) At least one instruction has completed since last DHCSR read Mask */
#define ARM_SYSCTRL_DHCSR_S_RETIRE_ST       ARM_SYSCTRL_DHCSR_S_RETIRE_ST_Msk              /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_RETIRE_ST_Msk instead */
#define ARM_SYSCTRL_DHCSR_S_RESET_ST_Pos    25                                             /**< (ARM_SYSCTRL_DHCSR) At least one reset since last DHCSR read Position */
#define ARM_SYSCTRL_DHCSR_S_RESET_ST_Msk    (_U_(0x1) << ARM_SYSCTRL_DHCSR_S_RESET_ST_Pos)  /**< (ARM_SYSCTRL_DHCSR) At least one reset since last DHCSR read Mask */
#define ARM_SYSCTRL_DHCSR_S_RESET_ST        ARM_SYSCTRL_DHCSR_S_RESET_ST_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DHCSR_S_RESET_ST_Msk instead */
#define ARM_SYSCTRL_DHCSR_Msk               _U_(0x30F000F)                                 /**< (ARM_SYSCTRL_DHCSR) Register Mask  */


/* -------- ARM_SYSCTRL_DCRSR : (ARM_SYSCTRL Offset: 0xdf4) (R/W 32) Debug Core Register Selector Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t REGSEL:5;                  /**< bit:   0..4  Specifies the ARM core register or special purpose register to transfer */
    uint32_t :11;                       /**< bit:  5..15  Reserved */
    uint32_t REGWNR:1;                  /**< bit:     16  If 1 then transfer is a write            */
    uint32_t :15;                       /**< bit: 17..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_DCRSR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_DCRSR_OFFSET            (0xDF4)                                       /**<  (ARM_SYSCTRL_DCRSR) Debug Core Register Selector Register  Offset */
#define ARM_SYSCTRL_DCRSR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_DCRSR) Debug Core Register Selector Register  Reset Value */

#define ARM_SYSCTRL_DCRSR_REGSEL_Pos        0                                              /**< (ARM_SYSCTRL_DCRSR) Specifies the ARM core register or special purpose register to transfer Position */
#define ARM_SYSCTRL_DCRSR_REGSEL_Msk        (_U_(0x1F) << ARM_SYSCTRL_DCRSR_REGSEL_Pos)    /**< (ARM_SYSCTRL_DCRSR) Specifies the ARM core register or special purpose register to transfer Mask */
#define ARM_SYSCTRL_DCRSR_REGSEL(value)     (ARM_SYSCTRL_DCRSR_REGSEL_Msk & ((value) << ARM_SYSCTRL_DCRSR_REGSEL_Pos))
#define   ARM_SYSCTRL_DCRSR_REGSEL_0_Val    _U_(0x0)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R0  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_1_Val    _U_(0x1)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R1  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_2_Val    _U_(0x2)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R2  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_3_Val    _U_(0x3)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R3  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_4_Val    _U_(0x4)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R4  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_5_Val    _U_(0x5)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R5  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_6_Val    _U_(0x6)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R6  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_7_Val    _U_(0x7)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R7  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_8_Val    _U_(0x8)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R8  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_9_Val    _U_(0x9)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R9  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_10_Val   _U_(0xA)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R10  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_11_Val   _U_(0xB)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R11  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_12_Val   _U_(0xC)                                       /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R12  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_13_Val   _U_(0xD)                                       /**< (ARM_SYSCTRL_DCRSR) Current SP  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_14_Val   _U_(0xE)                                       /**< (ARM_SYSCTRL_DCRSR) LR  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_15_Val   _U_(0xF)                                       /**< (ARM_SYSCTRL_DCRSR) DebugReturnAddress  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_16_Val   _U_(0x10)                                      /**< (ARM_SYSCTRL_DCRSR) xPSR  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_17_Val   _U_(0x11)                                      /**< (ARM_SYSCTRL_DCRSR) MSP  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_18_Val   _U_(0x12)                                      /**< (ARM_SYSCTRL_DCRSR) PSP  */
#define   ARM_SYSCTRL_DCRSR_REGSEL_20_Val   _U_(0x14)                                      /**< (ARM_SYSCTRL_DCRSR) CONTROL and PRIMASK  */
#define ARM_SYSCTRL_DCRSR_REGSEL_0          (ARM_SYSCTRL_DCRSR_REGSEL_0_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R0 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_1          (ARM_SYSCTRL_DCRSR_REGSEL_1_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R1 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_2          (ARM_SYSCTRL_DCRSR_REGSEL_2_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R2 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_3          (ARM_SYSCTRL_DCRSR_REGSEL_3_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R3 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_4          (ARM_SYSCTRL_DCRSR_REGSEL_4_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R4 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_5          (ARM_SYSCTRL_DCRSR_REGSEL_5_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R5 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_6          (ARM_SYSCTRL_DCRSR_REGSEL_6_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R6 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_7          (ARM_SYSCTRL_DCRSR_REGSEL_7_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R7 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_8          (ARM_SYSCTRL_DCRSR_REGSEL_8_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R8 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_9          (ARM_SYSCTRL_DCRSR_REGSEL_9_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R9 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_10         (ARM_SYSCTRL_DCRSR_REGSEL_10_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R10 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_11         (ARM_SYSCTRL_DCRSR_REGSEL_11_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R11 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_12         (ARM_SYSCTRL_DCRSR_REGSEL_12_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) ARM Core Register R12 Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_13         (ARM_SYSCTRL_DCRSR_REGSEL_13_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) Current SP Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_14         (ARM_SYSCTRL_DCRSR_REGSEL_14_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) LR Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_15         (ARM_SYSCTRL_DCRSR_REGSEL_15_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) DebugReturnAddress Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_16         (ARM_SYSCTRL_DCRSR_REGSEL_16_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) xPSR Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_17         (ARM_SYSCTRL_DCRSR_REGSEL_17_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) MSP Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_18         (ARM_SYSCTRL_DCRSR_REGSEL_18_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) PSP Position  */
#define ARM_SYSCTRL_DCRSR_REGSEL_20         (ARM_SYSCTRL_DCRSR_REGSEL_20_Val << ARM_SYSCTRL_DCRSR_REGSEL_Pos)  /**< (ARM_SYSCTRL_DCRSR) CONTROL and PRIMASK Position  */
#define ARM_SYSCTRL_DCRSR_REGWNR_Pos        16                                             /**< (ARM_SYSCTRL_DCRSR) If 1 then transfer is a write Position */
#define ARM_SYSCTRL_DCRSR_REGWNR_Msk        (_U_(0x1) << ARM_SYSCTRL_DCRSR_REGWNR_Pos)     /**< (ARM_SYSCTRL_DCRSR) If 1 then transfer is a write Mask */
#define ARM_SYSCTRL_DCRSR_REGWNR            ARM_SYSCTRL_DCRSR_REGWNR_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DCRSR_REGWNR_Msk instead */
#define ARM_SYSCTRL_DCRSR_MASK              _U_(0x1001F)                                   /**< \deprecated (ARM_SYSCTRL_DCRSR) Register MASK  (Use ARM_SYSCTRL_DCRSR_Msk instead)  */
#define ARM_SYSCTRL_DCRSR_Msk               _U_(0x1001F)                                   /**< (ARM_SYSCTRL_DCRSR) Register Mask  */


/* -------- ARM_SYSCTRL_DCRDR : (ARM_SYSCTRL Offset: 0xdf8) (R/W 32) Debug Core Register Data Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t DBGTMP:32;                 /**< bit:  0..31  Data Temporary Cache                     */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_DCRDR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_DCRDR_OFFSET            (0xDF8)                                       /**<  (ARM_SYSCTRL_DCRDR) Debug Core Register Data Register  Offset */
#define ARM_SYSCTRL_DCRDR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_DCRDR) Debug Core Register Data Register  Reset Value */

#define ARM_SYSCTRL_DCRDR_DBGTMP_Pos        0                                              /**< (ARM_SYSCTRL_DCRDR) Data Temporary Cache Position */
#define ARM_SYSCTRL_DCRDR_DBGTMP_Msk        (_U_(0xFFFFFFFF) << ARM_SYSCTRL_DCRDR_DBGTMP_Pos)  /**< (ARM_SYSCTRL_DCRDR) Data Temporary Cache Mask */
#define ARM_SYSCTRL_DCRDR_DBGTMP(value)     (ARM_SYSCTRL_DCRDR_DBGTMP_Msk & ((value) << ARM_SYSCTRL_DCRDR_DBGTMP_Pos))
#define ARM_SYSCTRL_DCRDR_MASK              _U_(0xFFFFFFFF)                                /**< \deprecated (ARM_SYSCTRL_DCRDR) Register MASK  (Use ARM_SYSCTRL_DCRDR_Msk instead)  */
#define ARM_SYSCTRL_DCRDR_Msk               _U_(0xFFFFFFFF)                                /**< (ARM_SYSCTRL_DCRDR) Register Mask  */


/* -------- ARM_SYSCTRL_DEMCR : (ARM_SYSCTRL Offset: 0xdfc) (R/W 32) Debug Exception and Monitor Control Register -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint32_t VC_CORERESET:1;            /**< bit:      0  Reset Vector Catch Enabled               */
    uint32_t :9;                        /**< bit:   1..9  Reserved */
    uint32_t VC_HARDERR:1;              /**< bit:     10  Halting Debug Trap Enabled               */
    uint32_t :13;                       /**< bit: 11..23  Reserved */
    uint32_t DWTENA:1;                  /**< bit:     24  DWT Enabled                              */
    uint32_t :7;                        /**< bit: 25..31  Reserved */
  } bit;                                /**< Structure used for bit  access */
  uint32_t reg;                         /**< Type used for register access */
} ARM_SYSCTRL_DEMCR_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_DEMCR_OFFSET            (0xDFC)                                       /**<  (ARM_SYSCTRL_DEMCR) Debug Exception and Monitor Control Register  Offset */
#define ARM_SYSCTRL_DEMCR_RESETVALUE        _U_(0x00)                                     /**<  (ARM_SYSCTRL_DEMCR) Debug Exception and Monitor Control Register  Reset Value */

#define ARM_SYSCTRL_DEMCR_VC_CORERESET_Pos  0                                              /**< (ARM_SYSCTRL_DEMCR) Reset Vector Catch Enabled Position */
#define ARM_SYSCTRL_DEMCR_VC_CORERESET_Msk  (_U_(0x1) << ARM_SYSCTRL_DEMCR_VC_CORERESET_Pos)  /**< (ARM_SYSCTRL_DEMCR) Reset Vector Catch Enabled Mask */
#define ARM_SYSCTRL_DEMCR_VC_CORERESET      ARM_SYSCTRL_DEMCR_VC_CORERESET_Msk             /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DEMCR_VC_CORERESET_Msk instead */
#define ARM_SYSCTRL_DEMCR_VC_HARDERR_Pos    10                                             /**< (ARM_SYSCTRL_DEMCR) Halting Debug Trap Enabled Position */
#define ARM_SYSCTRL_DEMCR_VC_HARDERR_Msk    (_U_(0x1) << ARM_SYSCTRL_DEMCR_VC_HARDERR_Pos)  /**< (ARM_SYSCTRL_DEMCR) Halting Debug Trap Enabled Mask */
#define ARM_SYSCTRL_DEMCR_VC_HARDERR        ARM_SYSCTRL_DEMCR_VC_HARDERR_Msk               /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DEMCR_VC_HARDERR_Msk instead */
#define ARM_SYSCTRL_DEMCR_DWTENA_Pos        24                                             /**< (ARM_SYSCTRL_DEMCR) DWT Enabled Position */
#define ARM_SYSCTRL_DEMCR_DWTENA_Msk        (_U_(0x1) << ARM_SYSCTRL_DEMCR_DWTENA_Pos)     /**< (ARM_SYSCTRL_DEMCR) DWT Enabled Mask */
#define ARM_SYSCTRL_DEMCR_DWTENA            ARM_SYSCTRL_DEMCR_DWTENA_Msk                   /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_DEMCR_DWTENA_Msk instead */
#define ARM_SYSCTRL_DEMCR_MASK              _U_(0x1000401)                                 /**< \deprecated (ARM_SYSCTRL_DEMCR) Register MASK  (Use ARM_SYSCTRL_DEMCR_Msk instead)  */
#define ARM_SYSCTRL_DEMCR_Msk               _U_(0x1000401)                                 /**< (ARM_SYSCTRL_DEMCR) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_PID4 : (ARM_SYSCTRL Offset: 0xfd0) (R/ 8) Peripheral ID Register 4 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_C_CODE:4;           /**< bit:   0..3  JEP106 C Code                            */
    uint8_t  BLOCK_COUNT:4;             /**< bit:   4..7  Block Count                              */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_PID4_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_PID4_OFFSET         (0xFD0)                                       /**<  (ARM_SYSCTRL_SCS_PID4) Peripheral ID Register 4  Offset */
#define ARM_SYSCTRL_SCS_PID4_RESETVALUE     _U_(0x04)                                     /**<  (ARM_SYSCTRL_SCS_PID4) Peripheral ID Register 4  Reset Value */

#define ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE_Pos 0                                              /**< (ARM_SYSCTRL_SCS_PID4) JEP106 C Code Position */
#define ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE_Pos)  /**< (ARM_SYSCTRL_SCS_PID4) JEP106 C Code Mask */
#define ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE(value) (ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE_Msk & ((value) << ARM_SYSCTRL_SCS_PID4_JEP106_C_CODE_Pos))
#define ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT_Pos 4                                              /**< (ARM_SYSCTRL_SCS_PID4) Block Count Position */
#define ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT_Pos)  /**< (ARM_SYSCTRL_SCS_PID4) Block Count Mask */
#define ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT(value) (ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT_Msk & ((value) << ARM_SYSCTRL_SCS_PID4_BLOCK_COUNT_Pos))
#define ARM_SYSCTRL_SCS_PID4_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_PID4) Register MASK  (Use ARM_SYSCTRL_SCS_PID4_Msk instead)  */
#define ARM_SYSCTRL_SCS_PID4_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_PID4) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_PID0 : (ARM_SYSCTRL Offset: 0xfe0) (R/ 8) Peripheral ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:8;             /**< bit:   0..7  Part Number Bits 7:0                     */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_PID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_PID0_OFFSET         (0xFE0)                                       /**<  (ARM_SYSCTRL_SCS_PID0) Peripheral ID Register 0  Offset */
#define ARM_SYSCTRL_SCS_PID0_RESETVALUE     _U_(0x08)                                     /**<  (ARM_SYSCTRL_SCS_PID0) Peripheral ID Register 0  Reset Value */

#define ARM_SYSCTRL_SCS_PID0_PART_NUMBER_Pos 0                                              /**< (ARM_SYSCTRL_SCS_PID0) Part Number Bits 7:0 Position */
#define ARM_SYSCTRL_SCS_PID0_PART_NUMBER_Msk (_U_(0xFF) << ARM_SYSCTRL_SCS_PID0_PART_NUMBER_Pos)  /**< (ARM_SYSCTRL_SCS_PID0) Part Number Bits 7:0 Mask */
#define ARM_SYSCTRL_SCS_PID0_PART_NUMBER(value) (ARM_SYSCTRL_SCS_PID0_PART_NUMBER_Msk & ((value) << ARM_SYSCTRL_SCS_PID0_PART_NUMBER_Pos))
#define ARM_SYSCTRL_SCS_PID0_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_PID0) Register MASK  (Use ARM_SYSCTRL_SCS_PID0_Msk instead)  */
#define ARM_SYSCTRL_SCS_PID0_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_PID0) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_PID1 : (ARM_SYSCTRL Offset: 0xfe4) (R/ 8) Peripheral ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  PART_NUMBER:4;             /**< bit:   0..3  Part Number Bits 11:8                    */
    uint8_t  JEP106_ID_3_0:4;           /**< bit:   4..7  JEP106 ID Code Bits 3:0                  */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_PID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_PID1_OFFSET         (0xFE4)                                       /**<  (ARM_SYSCTRL_SCS_PID1) Peripheral ID Register 1  Offset */
#define ARM_SYSCTRL_SCS_PID1_RESETVALUE     _U_(0xB0)                                     /**<  (ARM_SYSCTRL_SCS_PID1) Peripheral ID Register 1  Reset Value */

#define ARM_SYSCTRL_SCS_PID1_PART_NUMBER_Pos 0                                              /**< (ARM_SYSCTRL_SCS_PID1) Part Number Bits 11:8 Position */
#define ARM_SYSCTRL_SCS_PID1_PART_NUMBER_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID1_PART_NUMBER_Pos)  /**< (ARM_SYSCTRL_SCS_PID1) Part Number Bits 11:8 Mask */
#define ARM_SYSCTRL_SCS_PID1_PART_NUMBER(value) (ARM_SYSCTRL_SCS_PID1_PART_NUMBER_Msk & ((value) << ARM_SYSCTRL_SCS_PID1_PART_NUMBER_Pos))
#define ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0_Pos 4                                              /**< (ARM_SYSCTRL_SCS_PID1) JEP106 ID Code Bits 3:0 Position */
#define ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0_Pos)  /**< (ARM_SYSCTRL_SCS_PID1) JEP106 ID Code Bits 3:0 Mask */
#define ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0(value) (ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0_Msk & ((value) << ARM_SYSCTRL_SCS_PID1_JEP106_ID_3_0_Pos))
#define ARM_SYSCTRL_SCS_PID1_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_PID1) Register MASK  (Use ARM_SYSCTRL_SCS_PID1_Msk instead)  */
#define ARM_SYSCTRL_SCS_PID1_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_PID1) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_PID2 : (ARM_SYSCTRL Offset: 0xfe8) (R/ 8) Peripheral ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  JEP106_ID_6_4:3;           /**< bit:   0..2  JEP106 ID Code Bits 6:4                  */
    uint8_t  JEDEC_USED:1;              /**< bit:      3  JEDEC Used                               */
    uint8_t  REVISION:4;                /**< bit:   4..7  Revision                                 */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_PID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_PID2_OFFSET         (0xFE8)                                       /**<  (ARM_SYSCTRL_SCS_PID2) Peripheral ID Register 2  Offset */
#define ARM_SYSCTRL_SCS_PID2_RESETVALUE     _U_(0x0B)                                     /**<  (ARM_SYSCTRL_SCS_PID2) Peripheral ID Register 2  Reset Value */

#define ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4_Pos 0                                              /**< (ARM_SYSCTRL_SCS_PID2) JEP106 ID Code Bits 6:4 Position */
#define ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4_Msk (_U_(0x7) << ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4_Pos)  /**< (ARM_SYSCTRL_SCS_PID2) JEP106 ID Code Bits 6:4 Mask */
#define ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4(value) (ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4_Msk & ((value) << ARM_SYSCTRL_SCS_PID2_JEP106_ID_6_4_Pos))
#define ARM_SYSCTRL_SCS_PID2_JEDEC_USED_Pos 3                                              /**< (ARM_SYSCTRL_SCS_PID2) JEDEC Used Position */
#define ARM_SYSCTRL_SCS_PID2_JEDEC_USED_Msk (_U_(0x1) << ARM_SYSCTRL_SCS_PID2_JEDEC_USED_Pos)  /**< (ARM_SYSCTRL_SCS_PID2) JEDEC Used Mask */
#define ARM_SYSCTRL_SCS_PID2_JEDEC_USED     ARM_SYSCTRL_SCS_PID2_JEDEC_USED_Msk            /**< \deprecated Old style mask definition for 1 bit bitfield. Use ARM_SYSCTRL_SCS_PID2_JEDEC_USED_Msk instead */
#define ARM_SYSCTRL_SCS_PID2_REVISION_Pos   4                                              /**< (ARM_SYSCTRL_SCS_PID2) Revision Position */
#define ARM_SYSCTRL_SCS_PID2_REVISION_Msk   (_U_(0xF) << ARM_SYSCTRL_SCS_PID2_REVISION_Pos)  /**< (ARM_SYSCTRL_SCS_PID2) Revision Mask */
#define ARM_SYSCTRL_SCS_PID2_REVISION(value) (ARM_SYSCTRL_SCS_PID2_REVISION_Msk & ((value) << ARM_SYSCTRL_SCS_PID2_REVISION_Pos))
#define ARM_SYSCTRL_SCS_PID2_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_PID2) Register MASK  (Use ARM_SYSCTRL_SCS_PID2_Msk instead)  */
#define ARM_SYSCTRL_SCS_PID2_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_PID2) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_PID3 : (ARM_SYSCTRL Offset: 0xfec) (R/ 8) Peripheral ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  CUSTOMER_MOD_NUMBER:4;     /**< bit:   0..3  Customer Modification Number             */
    uint8_t  ECO_REV_NUMBER:4;          /**< bit:   4..7  ECO Revision Number                      */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_PID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_PID3_OFFSET         (0xFEC)                                       /**<  (ARM_SYSCTRL_SCS_PID3) Peripheral ID Register 3  Offset */
#define ARM_SYSCTRL_SCS_PID3_RESETVALUE     _U_(0x00)                                     /**<  (ARM_SYSCTRL_SCS_PID3) Peripheral ID Register 3  Reset Value */

#define ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER_Pos 0                                              /**< (ARM_SYSCTRL_SCS_PID3) Customer Modification Number Position */
#define ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER_Pos)  /**< (ARM_SYSCTRL_SCS_PID3) Customer Modification Number Mask */
#define ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER(value) (ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER_Msk & ((value) << ARM_SYSCTRL_SCS_PID3_CUSTOMER_MOD_NUMBER_Pos))
#define ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER_Pos 4                                              /**< (ARM_SYSCTRL_SCS_PID3) ECO Revision Number Position */
#define ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER_Msk (_U_(0xF) << ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER_Pos)  /**< (ARM_SYSCTRL_SCS_PID3) ECO Revision Number Mask */
#define ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER(value) (ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER_Msk & ((value) << ARM_SYSCTRL_SCS_PID3_ECO_REV_NUMBER_Pos))
#define ARM_SYSCTRL_SCS_PID3_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_PID3) Register MASK  (Use ARM_SYSCTRL_SCS_PID3_Msk instead)  */
#define ARM_SYSCTRL_SCS_PID3_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_PID3) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_CID0 : (ARM_SYSCTRL Offset: 0xff0) (R/ 8) Component ID Register 0 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  SCS_CID0:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_CID0_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_CID0_OFFSET         (0xFF0)                                       /**<  (ARM_SYSCTRL_SCS_CID0) Component ID Register 0  Offset */
#define ARM_SYSCTRL_SCS_CID0_RESETVALUE     _U_(0x0D)                                     /**<  (ARM_SYSCTRL_SCS_CID0) Component ID Register 0  Reset Value */

#define ARM_SYSCTRL_SCS_CID0_SCS_CID0_Pos   0                                              /**< (ARM_SYSCTRL_SCS_CID0)  Position */
#define ARM_SYSCTRL_SCS_CID0_SCS_CID0_Msk   (_U_(0xFF) << ARM_SYSCTRL_SCS_CID0_SCS_CID0_Pos)  /**< (ARM_SYSCTRL_SCS_CID0)  Mask */
#define ARM_SYSCTRL_SCS_CID0_SCS_CID0(value) (ARM_SYSCTRL_SCS_CID0_SCS_CID0_Msk & ((value) << ARM_SYSCTRL_SCS_CID0_SCS_CID0_Pos))
#define ARM_SYSCTRL_SCS_CID0_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_CID0) Register MASK  (Use ARM_SYSCTRL_SCS_CID0_Msk instead)  */
#define ARM_SYSCTRL_SCS_CID0_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_CID0) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_CID1 : (ARM_SYSCTRL Offset: 0xff4) (R/ 8) Component ID Register 1 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  SCS_CID1:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_CID1_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_CID1_OFFSET         (0xFF4)                                       /**<  (ARM_SYSCTRL_SCS_CID1) Component ID Register 1  Offset */
#define ARM_SYSCTRL_SCS_CID1_RESETVALUE     _U_(0xE0)                                     /**<  (ARM_SYSCTRL_SCS_CID1) Component ID Register 1  Reset Value */

#define ARM_SYSCTRL_SCS_CID1_SCS_CID1_Pos   0                                              /**< (ARM_SYSCTRL_SCS_CID1)  Position */
#define ARM_SYSCTRL_SCS_CID1_SCS_CID1_Msk   (_U_(0xFF) << ARM_SYSCTRL_SCS_CID1_SCS_CID1_Pos)  /**< (ARM_SYSCTRL_SCS_CID1)  Mask */
#define ARM_SYSCTRL_SCS_CID1_SCS_CID1(value) (ARM_SYSCTRL_SCS_CID1_SCS_CID1_Msk & ((value) << ARM_SYSCTRL_SCS_CID1_SCS_CID1_Pos))
#define ARM_SYSCTRL_SCS_CID1_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_CID1) Register MASK  (Use ARM_SYSCTRL_SCS_CID1_Msk instead)  */
#define ARM_SYSCTRL_SCS_CID1_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_CID1) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_CID2 : (ARM_SYSCTRL Offset: 0xff8) (R/ 8) Component ID Register 2 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  SCS_CID2:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_CID2_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_CID2_OFFSET         (0xFF8)                                       /**<  (ARM_SYSCTRL_SCS_CID2) Component ID Register 2  Offset */
#define ARM_SYSCTRL_SCS_CID2_RESETVALUE     _U_(0x05)                                     /**<  (ARM_SYSCTRL_SCS_CID2) Component ID Register 2  Reset Value */

#define ARM_SYSCTRL_SCS_CID2_SCS_CID2_Pos   0                                              /**< (ARM_SYSCTRL_SCS_CID2)  Position */
#define ARM_SYSCTRL_SCS_CID2_SCS_CID2_Msk   (_U_(0xFF) << ARM_SYSCTRL_SCS_CID2_SCS_CID2_Pos)  /**< (ARM_SYSCTRL_SCS_CID2)  Mask */
#define ARM_SYSCTRL_SCS_CID2_SCS_CID2(value) (ARM_SYSCTRL_SCS_CID2_SCS_CID2_Msk & ((value) << ARM_SYSCTRL_SCS_CID2_SCS_CID2_Pos))
#define ARM_SYSCTRL_SCS_CID2_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_CID2) Register MASK  (Use ARM_SYSCTRL_SCS_CID2_Msk instead)  */
#define ARM_SYSCTRL_SCS_CID2_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_CID2) Register Mask  */


/* -------- ARM_SYSCTRL_SCS_CID3 : (ARM_SYSCTRL Offset: 0xffc) (R/ 8) Component ID Register 3 -------- */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef union { 
  struct {
    uint8_t  SCS_CID3:8;                /**< bit:   0..7                                           */
  } bit;                                /**< Structure used for bit  access */
  uint8_t  reg;                         /**< Type used for register access */
} ARM_SYSCTRL_SCS_CID3_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#define ARM_SYSCTRL_SCS_CID3_OFFSET         (0xFFC)                                       /**<  (ARM_SYSCTRL_SCS_CID3) Component ID Register 3  Offset */
#define ARM_SYSCTRL_SCS_CID3_RESETVALUE     _U_(0xB1)                                     /**<  (ARM_SYSCTRL_SCS_CID3) Component ID Register 3  Reset Value */

#define ARM_SYSCTRL_SCS_CID3_SCS_CID3_Pos   0                                              /**< (ARM_SYSCTRL_SCS_CID3)  Position */
#define ARM_SYSCTRL_SCS_CID3_SCS_CID3_Msk   (_U_(0xFF) << ARM_SYSCTRL_SCS_CID3_SCS_CID3_Pos)  /**< (ARM_SYSCTRL_SCS_CID3)  Mask */
#define ARM_SYSCTRL_SCS_CID3_SCS_CID3(value) (ARM_SYSCTRL_SCS_CID3_SCS_CID3_Msk & ((value) << ARM_SYSCTRL_SCS_CID3_SCS_CID3_Pos))
#define ARM_SYSCTRL_SCS_CID3_MASK           _U_(0xFF)                                      /**< \deprecated (ARM_SYSCTRL_SCS_CID3) Register MASK  (Use ARM_SYSCTRL_SCS_CID3_Msk instead)  */
#define ARM_SYSCTRL_SCS_CID3_Msk            _U_(0xFF)                                      /**< (ARM_SYSCTRL_SCS_CID3) Register Mask  */


#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if COMPONENT_TYPEDEF_STYLE == 'R'
/** \brief ARM_SYSCTRL hardware registers */
typedef struct {  /* ARM System Control */
  RoReg8  Reserved1[0x8];
  RoReg   ACTLR;          /**< (ARM_SYSCTRL Offset: 0x08) Auxiliary Control Register (not implemented) */
  RoReg8  Reserved2[0x4];
  RwReg   SYST_CSR;       /**< (ARM_SYSCTRL Offset: 0x10) SysTick Control and Status Register */
  RwReg   SYST_RVR;       /**< (ARM_SYSCTRL Offset: 0x14) SysTick Reload Value Register */
  RwReg   SYST_CVR;       /**< (ARM_SYSCTRL Offset: 0x18) SysTick Current Value Register (Any Write Clears to 0) */
  RoReg   SYST_CALIB;     /**< (ARM_SYSCTRL Offset: 0x1C) SysTick Calibration Value Register */
  RoReg8  Reserved3[0xE0];
  RwReg   NVIC_ISER;      /**< (ARM_SYSCTRL Offset: 0x100) Interrupt Set-Enable Register */
  RoReg8  Reserved4[0x7C];
  RwReg   NVIC_ICER;      /**< (ARM_SYSCTRL Offset: 0x180) Interrupt Clear Enable Register */
  RoReg8  Reserved5[0x7C];
  RwReg   NVIC_ISPR;      /**< (ARM_SYSCTRL Offset: 0x200) Interrupt Set-Pending Register */
  RoReg8  Reserved6[0x7C];
  RwReg   NVIC_ICPR;      /**< (ARM_SYSCTRL Offset: 0x280) Interrupt Clear-Pending Register */
  RoReg8  Reserved7[0x17C];
  RwReg   NVIC_IPR0;      /**< (ARM_SYSCTRL Offset: 0x400) Interrupt Priority Register 0 */
  RwReg   NVIC_IPR1;      /**< (ARM_SYSCTRL Offset: 0x404) Interrupt Priority Register 1 */
  RwReg   NVIC_IPR2;      /**< (ARM_SYSCTRL Offset: 0x408) Interrupt Priority Register 2 */
  RwReg   NVIC_IPR3;      /**< (ARM_SYSCTRL Offset: 0x40C) Interrupt Priority Register 3 */
  RwReg   NVIC_IPR4;      /**< (ARM_SYSCTRL Offset: 0x410) Interrupt Priority Register 4 */
  RwReg   NVIC_IPR5;      /**< (ARM_SYSCTRL Offset: 0x414) Interrupt Priority Register 5 */
  RwReg   NVIC_IPR6;      /**< (ARM_SYSCTRL Offset: 0x418) Interrupt Priority Register 6 */
  RwReg   NVIC_IPR7;      /**< (ARM_SYSCTRL Offset: 0x41C) Interrupt Priority Register 7 */
  RoReg8  Reserved8[0x8E0];
  RoReg   CPUID;          /**< (ARM_SYSCTRL Offset: 0xD00) CPU Identification Register */
  RoReg   ICSR;           /**< (ARM_SYSCTRL Offset: 0xD04) Interrupt Control State Register */
  RoReg8  Reserved9[0x4];
  RoReg   AIRCR;          /**< (ARM_SYSCTRL Offset: 0xD0C) Application Interrupt and Reset Control Register */
  RwReg8  SCR;            /**< (ARM_SYSCTRL Offset: 0xD10) System Control Register */
  RoReg8  Reserved10[0x3];
  RoReg16 CCR;            /**< (ARM_SYSCTRL Offset: 0xD14) Configuration and Control Register */
  RoReg8  Reserved11[0x6];
  RwReg   SHPR2;          /**< (ARM_SYSCTRL Offset: 0xD1C) System Handler Priority Register 2 */
  RwReg   SHPR3;          /**< (ARM_SYSCTRL Offset: 0xD20) System Handler Priority Register 3 */
  RwReg16 SHCSR;          /**< (ARM_SYSCTRL Offset: 0xD24) System Handler Control and State Register */
  RoReg8  Reserved12[0xA];
  RwReg8  DFSR;           /**< (ARM_SYSCTRL Offset: 0xD30) Debug Fault Status Register */
  RoReg8  Reserved13[0xBF];
  RwReg   DHCSR;          /**< (ARM_SYSCTRL Offset: 0xDF0) Debug Halting Control and Status Register */
  RwReg   DCRSR;          /**< (ARM_SYSCTRL Offset: 0xDF4) Debug Core Register Selector Register */
  RwReg   DCRDR;          /**< (ARM_SYSCTRL Offset: 0xDF8) Debug Core Register Data Register */
  RwReg   DEMCR;          /**< (ARM_SYSCTRL Offset: 0xDFC) Debug Exception and Monitor Control Register */
  RoReg8  Reserved14[0x1D0];
  RoReg8  SCS_PID4;       /**< (ARM_SYSCTRL Offset: 0xFD0) Peripheral ID Register 4 */
  RoReg8  Reserved15[0xF];
  RoReg8  SCS_PID0;       /**< (ARM_SYSCTRL Offset: 0xFE0) Peripheral ID Register 0 */
  RoReg8  Reserved16[0x3];
  RoReg8  SCS_PID1;       /**< (ARM_SYSCTRL Offset: 0xFE4) Peripheral ID Register 1 */
  RoReg8  Reserved17[0x3];
  RoReg8  SCS_PID2;       /**< (ARM_SYSCTRL Offset: 0xFE8) Peripheral ID Register 2 */
  RoReg8  Reserved18[0x3];
  RoReg8  SCS_PID3;       /**< (ARM_SYSCTRL Offset: 0xFEC) Peripheral ID Register 3 */
  RoReg8  Reserved19[0x3];
  RoReg8  SCS_CID0;       /**< (ARM_SYSCTRL Offset: 0xFF0) Component ID Register 0 */
  RoReg8  Reserved20[0x3];
  RoReg8  SCS_CID1;       /**< (ARM_SYSCTRL Offset: 0xFF4) Component ID Register 1 */
  RoReg8  Reserved21[0x3];
  RoReg8  SCS_CID2;       /**< (ARM_SYSCTRL Offset: 0xFF8) Component ID Register 2 */
  RoReg8  Reserved22[0x3];
  RoReg8  SCS_CID3;       /**< (ARM_SYSCTRL Offset: 0xFFC) Component ID Register 3 */
} ArmSysctrl;

#elif COMPONENT_TYPEDEF_STYLE == 'N'
/** \brief ARM_SYSCTRL hardware registers */
typedef struct {  /* ARM System Control */
  __I  uint32_t                       Reserved1[2];
  __I  ARM_SYSCTRL_ACTLR_Type         ACTLR;          /**< Offset: 0x08 (R/   32) Auxiliary Control Register (not implemented) */
  __I  uint32_t                       Reserved2[1];
  __IO ARM_SYSCTRL_SYST_CSR_Type      SYST_CSR;       /**< Offset: 0x10 (R/W  32) SysTick Control and Status Register */
  __IO ARM_SYSCTRL_SYST_RVR_Type      SYST_RVR;       /**< Offset: 0x14 (R/W  32) SysTick Reload Value Register */
  __IO ARM_SYSCTRL_SYST_CVR_Type      SYST_CVR;       /**< Offset: 0x18 (R/W  32) SysTick Current Value Register (Any Write Clears to 0) */
  __I  ARM_SYSCTRL_SYST_CALIB_Type    SYST_CALIB;     /**< Offset: 0x1C (R/   32) SysTick Calibration Value Register */
  __I  uint32_t                       Reserved3[56];
  __IO ARM_SYSCTRL_NVIC_ISER_Type     NVIC_ISER;      /**< Offset: 0x100 (R/W  32) Interrupt Set-Enable Register */
  __I  uint32_t                       Reserved4[31];
  __IO ARM_SYSCTRL_NVIC_ICER_Type     NVIC_ICER;      /**< Offset: 0x180 (R/W  32) Interrupt Clear Enable Register */
  __I  uint32_t                       Reserved5[31];
  __IO ARM_SYSCTRL_NVIC_ISPR_Type     NVIC_ISPR;      /**< Offset: 0x200 (R/W  32) Interrupt Set-Pending Register */
  __I  uint32_t                       Reserved6[31];
  __IO ARM_SYSCTRL_NVIC_ICPR_Type     NVIC_ICPR;      /**< Offset: 0x280 (R/W  32) Interrupt Clear-Pending Register */
  __I  uint32_t                       Reserved7[95];
  __IO ARM_SYSCTRL_NVIC_IPR0_Type     NVIC_IPR0;      /**< Offset: 0x400 (R/W  32) Interrupt Priority Register 0 */
  __IO ARM_SYSCTRL_NVIC_IPR1_Type     NVIC_IPR1;      /**< Offset: 0x404 (R/W  32) Interrupt Priority Register 1 */
  __IO ARM_SYSCTRL_NVIC_IPR2_Type     NVIC_IPR2;      /**< Offset: 0x408 (R/W  32) Interrupt Priority Register 2 */
  __IO ARM_SYSCTRL_NVIC_IPR3_Type     NVIC_IPR3;      /**< Offset: 0x40C (R/W  32) Interrupt Priority Register 3 */
  __IO ARM_SYSCTRL_NVIC_IPR4_Type     NVIC_IPR4;      /**< Offset: 0x410 (R/W  32) Interrupt Priority Register 4 */
  __IO ARM_SYSCTRL_NVIC_IPR5_Type     NVIC_IPR5;      /**< Offset: 0x414 (R/W  32) Interrupt Priority Register 5 */
  __IO ARM_SYSCTRL_NVIC_IPR6_Type     NVIC_IPR6;      /**< Offset: 0x418 (R/W  32) Interrupt Priority Register 6 */
  __IO ARM_SYSCTRL_NVIC_IPR7_Type     NVIC_IPR7;      /**< Offset: 0x41C (R/W  32) Interrupt Priority Register 7 */
  __I  uint32_t                       Reserved8[568];
  __I  ARM_SYSCTRL_CPUID_Type         CPUID;          /**< Offset: 0xD00 (R/   32) CPU Identification Register */
  __I  ARM_SYSCTRL_ICSR_Type          ICSR;           /**< Offset: 0xD04 (R/   32) Interrupt Control State Register */
  __I  uint32_t                       Reserved9[1];
  __I  ARM_SYSCTRL_AIRCR_Type         AIRCR;          /**< Offset: 0xD0C (R/   32) Application Interrupt and Reset Control Register */
  __IO ARM_SYSCTRL_SCR_Type           SCR;            /**< Offset: 0xD10 (R/W   8) System Control Register */
  __I  uint8_t                        Reserved10[3];
  __I  ARM_SYSCTRL_CCR_Type           CCR;            /**< Offset: 0xD14 (R/   16) Configuration and Control Register */
  __I  uint8_t                        Reserved11[6];
  __IO ARM_SYSCTRL_SHPR2_Type         SHPR2;          /**< Offset: 0xD1C (R/W  32) System Handler Priority Register 2 */
  __IO ARM_SYSCTRL_SHPR3_Type         SHPR3;          /**< Offset: 0xD20 (R/W  32) System Handler Priority Register 3 */
  __IO ARM_SYSCTRL_SHCSR_Type         SHCSR;          /**< Offset: 0xD24 (R/W  16) System Handler Control and State Register */
  __I  uint8_t                        Reserved12[10];
  __IO ARM_SYSCTRL_DFSR_Type          DFSR;           /**< Offset: 0xD30 (R/W   8) Debug Fault Status Register */
  __I  uint8_t                        Reserved13[191];
  __IO ARM_SYSCTRL_DHCSR_Type         DHCSR;          /**< Offset: 0xDF0 (R/W  32) Debug Halting Control and Status Register */
  __IO ARM_SYSCTRL_DCRSR_Type         DCRSR;          /**< Offset: 0xDF4 (R/W  32) Debug Core Register Selector Register */
  __IO ARM_SYSCTRL_DCRDR_Type         DCRDR;          /**< Offset: 0xDF8 (R/W  32) Debug Core Register Data Register */
  __IO ARM_SYSCTRL_DEMCR_Type         DEMCR;          /**< Offset: 0xDFC (R/W  32) Debug Exception and Monitor Control Register */
  __I  uint32_t                       Reserved14[116];
  __I  ARM_SYSCTRL_SCS_PID4_Type      SCS_PID4;       /**< Offset: 0xFD0 (R/    8) Peripheral ID Register 4 */
  __I  uint8_t                        Reserved15[15];
  __I  ARM_SYSCTRL_SCS_PID0_Type      SCS_PID0;       /**< Offset: 0xFE0 (R/    8) Peripheral ID Register 0 */
  __I  uint8_t                        Reserved16[3];
  __I  ARM_SYSCTRL_SCS_PID1_Type      SCS_PID1;       /**< Offset: 0xFE4 (R/    8) Peripheral ID Register 1 */
  __I  uint8_t                        Reserved17[3];
  __I  ARM_SYSCTRL_SCS_PID2_Type      SCS_PID2;       /**< Offset: 0xFE8 (R/    8) Peripheral ID Register 2 */
  __I  uint8_t                        Reserved18[3];
  __I  ARM_SYSCTRL_SCS_PID3_Type      SCS_PID3;       /**< Offset: 0xFEC (R/    8) Peripheral ID Register 3 */
  __I  uint8_t                        Reserved19[3];
  __I  ARM_SYSCTRL_SCS_CID0_Type      SCS_CID0;       /**< Offset: 0xFF0 (R/    8) Component ID Register 0 */
  __I  uint8_t                        Reserved20[3];
  __I  ARM_SYSCTRL_SCS_CID1_Type      SCS_CID1;       /**< Offset: 0xFF4 (R/    8) Component ID Register 1 */
  __I  uint8_t                        Reserved21[3];
  __I  ARM_SYSCTRL_SCS_CID2_Type      SCS_CID2;       /**< Offset: 0xFF8 (R/    8) Component ID Register 2 */
  __I  uint8_t                        Reserved22[3];
  __I  ARM_SYSCTRL_SCS_CID3_Type      SCS_CID3;       /**< Offset: 0xFFC (R/    8) Component ID Register 3 */
} ArmSysctrl;

#else /* COMPONENT_TYPEDEF_STYLE */
#error Unknown component typedef style
#endif /* COMPONENT_TYPEDEF_STYLE */

#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of ARM System Control */

#endif /* _SAMB11_ARM_SYSCTRL_COMPONENT_H_ */
