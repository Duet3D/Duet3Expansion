/**
 * \file
 *
 * \brief Header file for ATBTLC1000WLCSP
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
#ifndef _BTLC1000WLCSP_H_
#define _BTLC1000WLCSP_H_

/** \addtogroup BTLC1000WLCSP_definitions BTLC1000WLCSP definitions
  This file defines all structures and symbols for BTLC1000WLCSP:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - PIO definitions
 *  @{
 */

#ifdef __cplusplus
 extern "C" {
#endif

/** \defgroup Atmel_glob_defs Atmel Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.

    \remark
    CMSIS core has a syntax that differs from this using i.e. __I, __O, or __IO followed by 'uint<size>_t' respective types.
    Default the header files will follow the CMSIS core syntax.
 *  @{
 */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#include <stdint.h>

/* IO definitions (access restrictions to peripheral registers) */
#ifndef __cplusplus
typedef volatile const uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile const uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile const uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#else
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#endif
typedef volatile       uint32_t WoReg;   /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t WoReg16; /**< Write only 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t WoReg8;  /**< Write only  8-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */

#define CAST(type, value) ((type *)(value)) /**< Pointer Type Conversion Macro for C/C++ */
#define REG_ACCESS(type, address) (*(type*)(address)) /**< C code: Register value */
#else /* Assembler */
#define CAST(type, value) (value) /**< Pointer Type Conversion Macro for Assembler */
#define REG_ACCESS(type, address) (address) /**< Assembly code: Register address */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !defined(SKIP_INTEGER_LITERALS)

#if defined(_U_) || defined(_L_) || defined(_UL_)
  #error "Integer Literals macros already defined elsewhere"
#endif

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/* Macros that deal with adding suffixes to integer literal constants for C/C++ */
#define _U_(x) x ## U    /**< C code: Unsigned integer literal constant value */
#define _L_(x) x ## L    /**< C code: Long integer literal constant value */
#define _UL_(x) x ## UL  /**< C code: Unsigned Long integer literal constant value */

#else /* Assembler */

#define _U_(x) x    /**< Assembler: Unsigned integer literal constant value */
#define _L_(x) x    /**< Assembler: Long integer literal constant value */
#define _UL_(x) x   /**< Assembler: Unsigned Long integer literal constant value */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#endif /* SKIP_INTEGER_LITERALS */
/** @}  end of Atmel Global Defines */

/** \addtogroup BTLC1000WLCSP_cmsis CMSIS Definitions
 *  @{
 */
/* ************************************************************************** */
/*   CMSIS DEFINITIONS FOR BTLC1000WLCSP */
/* ************************************************************************** */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** Interrupt Number Definition */
typedef enum IRQn
{
/******  CORTEX-M0 Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn       = -14, /**< 2   Non Maskable Interrupt               */
  HardFault_IRQn            = -13, /**< 3   Hard Fault Interrupt                 */
  SVCall_IRQn               = -5 , /**< 11  SV Call Interrupt                    */
  PendSV_IRQn               = -2 , /**< 14  Pend SV Interrupt                    */
  SysTick_IRQn              = -1 , /**< 15  System Tick Interrupt                */
/******  BTLC1000WLCSP specific Interrupt Numbers ***********************************/
  UART0_RX_IRQn             = 0  , /**< 0   BTLC1000WLCSP UART Controller (UART0) */
  UART0_TX_IRQn             = 1  , /**< 1   BTLC1000WLCSP UART Controller (UART0) */
  UART1_RX_IRQn             = 2  , /**< 2   BTLC1000WLCSP UART Controller (UART1) */
  UART1_TX_IRQn             = 3  , /**< 3   BTLC1000WLCSP UART Controller (UART1) */
  SPI0_RX_IRQn              = 4  , /**< 4   BTLC1000WLCSP SPI Master/Slave Controller (SPI0) */
  SPI0_TX_IRQn              = 5  , /**< 5   BTLC1000WLCSP SPI Master/Slave Controller (SPI0) */
  I2C0_RX_IRQn              = 8  , /**< 8   BTLC1000WLCSP I2C Master/Slave Controller (I2C0) */
  I2C0_TX_IRQn              = 9  , /**< 9   BTLC1000WLCSP I2C Master/Slave Controller (I2C0) */
  I2C1_RX_IRQn              = 10 , /**< 10  BTLC1000WLCSP I2C Master/Slave Controller (I2C1) */
  I2C1_TX_IRQn              = 11 , /**< 11  BTLC1000WLCSP I2C Master/Slave Controller (I2C1) */
  WDT0_IRQn                 = 12 , /**< 12  BTLC1000WLCSP Watchdog0 Timer (WDT0) */
  WDT1_IRQn                 = 13 , /**< 13  BTLC1000WLCSP Watchdog0 Timer (WDT1) */
  DUALTIMER0_IRQn           = 14 , /**< 14  BTLC1000WLCSP ARM General Purpose Dual Timer (DUALTIMER0) */
  SPI_FLASH0_IRQn           = 18 , /**< 18  BTLC1000WLCSP SPI Flash Controller (SPI_FLASH0) */
  GPIO0_IRQn                = 23 , /**< 23  BTLC1000WLCSP GPIO Controller (GPIO0) */
  GPIO1_IRQn                = 24 , /**< 24  BTLC1000WLCSP GPIO Controller (GPIO1) */
  GPIO2_IRQn                = 25 , /**< 25  BTLC1000WLCSP GPIO Controller (GPIO2) */
  TIMER0_IRQn               = 26 , /**< 26  BTLC1000WLCSP ARM General Purpose Timer (TIMER0) */
  AON_SLEEP_TIMER0_IRQn     = 27 , /**< 27  BTLC1000WLCSP Always On Sleep Timer (AON_SLEEP_TIMER0) */

  PERIPH_COUNT_IRQn        = 28  /**< Number of peripheral IDs */
} IRQn_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;
  /* Cortex-M handlers */
  void* pvReservedC15;
  void* pfnNonMaskableInt_Handler;               /* -14 Non Maskable Interrupt               */
  void* pfnHardFault_Handler;                    /* -13 Hard Fault Interrupt                 */
  void* pvReservedC12;
  void* pvReservedC11;
  void* pvReservedC10;
  void* pvReservedC9;
  void* pvReservedC8;
  void* pvReservedC7;
  void* pvReservedC6;
  void* pfnSVCall_Handler;                       /*  -5 SV Call Interrupt                    */
  void* pvReservedC4;
  void* pvReservedC3;
  void* pfnPendSV_Handler;                       /*  -2 Pend SV Interrupt                    */
  void* pfnSysTick_Handler;                      /*  -1 System Tick Interrupt                */

  /* Peripheral handlers */
  void* pfnUART0_RX_Handler;                     /* 0   BTLC1000WLCSP UART Controller (UART0) */
  void* pfnUART0_TX_Handler;                     /* 1   BTLC1000WLCSP UART Controller (UART0) */
  void* pfnUART1_RX_Handler;                     /* 2   BTLC1000WLCSP UART Controller (UART1) */
  void* pfnUART1_TX_Handler;                     /* 3   BTLC1000WLCSP UART Controller (UART1) */
  void* pfnSPI0_RX_Handler;                      /* 4   BTLC1000WLCSP SPI Master/Slave Controller (SPI0) */
  void* pfnSPI0_TX_Handler;                      /* 5   BTLC1000WLCSP SPI Master/Slave Controller (SPI0) */
  void* pvReserved6;
  void* pvReserved7;
  void* pfnI2C0_RX_Handler;                      /* 8   BTLC1000WLCSP I2C Master/Slave Controller (I2C0) */
  void* pfnI2C0_TX_Handler;                      /* 9   BTLC1000WLCSP I2C Master/Slave Controller (I2C0) */
  void* pfnI2C1_RX_Handler;                      /* 10  BTLC1000WLCSP I2C Master/Slave Controller (I2C1) */
  void* pfnI2C1_TX_Handler;                      /* 11  BTLC1000WLCSP I2C Master/Slave Controller (I2C1) */
  void* pfnWDT0_Handler;                         /* 12  BTLC1000WLCSP Watchdog0 Timer (WDT0) */
  void* pfnWDT1_Handler;                         /* 13  BTLC1000WLCSP Watchdog0 Timer (WDT1) */
  void* pfnDUALTIMER0_Handler;                   /* 14  BTLC1000WLCSP ARM General Purpose Dual Timer (DUALTIMER0) */
  void* pvReserved15;
  void* pvReserved16;
  void* pvReserved17;
  void* pfnSPI_FLASH0_Handler;                   /* 18  BTLC1000WLCSP SPI Flash Controller (SPI_FLASH0) */
  void* pvReserved19;
  void* pvReserved20;
  void* pvReserved21;
  void* pvReserved22;
  void* pfnGPIO0_Handler;                        /* 23  BTLC1000WLCSP GPIO Controller (GPIO0) */
  void* pfnGPIO1_Handler;                        /* 24  BTLC1000WLCSP GPIO Controller (GPIO1) */
  void* pfnGPIO2_Handler;                        /* 25  BTLC1000WLCSP GPIO Controller (GPIO2) */
  void* pfnTIMER0_Handler;                       /* 26  BTLC1000WLCSP ARM General Purpose Timer (TIMER0) */
  void* pfnAON_SLEEP_TIMER0_Handler;             /* 27  BTLC1000WLCSP Always On Sleep Timer (AON_SLEEP_TIMER0) */
} DeviceVectors;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if !defined DONT_USE_PREDEFINED_CORE_HANDLERS

/* CORTEX-M0 core handlers */
void NonMaskableInt_Handler        ( void );
void HardFault_Handler             ( void );
void SVCall_Handler                ( void );
void PendSV_Handler                ( void );
void SysTick_Handler               ( void );
#endif /* DONT_USE_PREDEFINED_CORE_HANDLERS */

#if !defined DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS

/* Peripherals handlers */
void AON_SLEEP_TIMER0_Handler      ( void );
void DUALTIMER0_Handler            ( void );
void GPIO0_Handler                 ( void );
void GPIO1_Handler                 ( void );
void GPIO2_Handler                 ( void );
void I2C0_RX_Handler               ( void );
void I2C0_TX_Handler               ( void );
void I2C1_RX_Handler               ( void );
void I2C1_TX_Handler               ( void );
void SPI0_RX_Handler               ( void );
void SPI0_TX_Handler               ( void );
void SPI_FLASH0_Handler            ( void );
void TIMER0_Handler                ( void );
void UART0_RX_Handler              ( void );
void UART0_TX_Handler              ( void );
void UART1_RX_Handler              ( void );
void UART1_TX_Handler              ( void );
void WDT0_Handler                  ( void );
void WDT1_Handler                  ( void );
#endif /* DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */


/*
 * \brief Configuration of the CORTEX-M0 Processor and Core Peripherals
 */

#define PIO_API                     samb
#define LITTLE_ENDIAN                  1
#define __FPU_PRESENT                  0
#define __CM0_REV                   0x00
#define __NVIC_PRIO_BITS               2
#define __Vendor_SysTickConfig         0

/*
 * \brief CMSIS includes
 */
#include <core_cm0.h>
#if !defined DONT_USE_CMSIS_INIT
#include "system_samb11.h"
#endif /* DONT_USE_CMSIS_INIT */

/** @}  end of BTLC1000WLCSP_cmsis CMSIS Definitions */

/** \defgroup BTLC1000WLCSP_api Peripheral Software API
 *  @{
 */

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR BTLC1000WLCSP */
/* ************************************************************************** */
#include "component/timer.h"
#include "component/dualtimer.h"
#include "component/prov_dma_ctrl.h"
#include "component/i2c.h"
#include "component/uart.h"
#include "component/spi.h"
#include "component/wdt.h"
#include "component/efuse_misc_regs.h"
#include "component/lpmcu_misc_regs.h"
#include "component/lp_clk_cal_regs.h"
#include "component/aon_sleep_timer.h"
#include "component/aon_pwr_seq.h"
#include "component/aon_gp_regs.h"
#include "component/gpio.h"
#include "component/spi_flash.h"
#include "component/arm_sysctrl.h"
#include "component/arm_bpu.h"
#include "component/arm_dwt.h"
#include "component/arm_rom.h"
/** @}  end of Peripheral Software API */

/** \defgroup BTLC1000WLCSP_reg Registers Access Definitions
 *  @{
 */

/* ************************************************************************** */
/*   REGISTER ACCESS DEFINITIONS FOR BTLC1000WLCSP */
/* ************************************************************************** */
#include "instance/timer0.h"
#include "instance/dualtimer0.h"
#include "instance/prov_dma_ctrl0.h"
#include "instance/i2c0.h"
#include "instance/i2c1.h"
#include "instance/uart0.h"
#include "instance/uart1.h"
#include "instance/spi0.h"
#include "instance/wdt0.h"
#include "instance/wdt1.h"
#include "instance/efuse_misc_regs0.h"
#include "instance/lpmcu_misc_regs0.h"
#include "instance/lp_clk_cal_regs0.h"
#include "instance/aon_sleep_timer0.h"
#include "instance/aon_pwr_seq0.h"
#include "instance/aon_gp_regs0.h"
#include "instance/gpio0.h"
#include "instance/gpio1.h"
#include "instance/gpio2.h"
#include "instance/spi_flash0.h"
#include "instance/arm_sysctrl0.h"
#include "instance/arm_bpu0.h"
#include "instance/arm_dwt0.h"
#include "instance/arm_rom0.h"
/** @}  end of Registers Access Definitions */

/** \addtogroup BTLC1000WLCSP_id Peripheral Ids Definitions
 *  @{
 */

/* ************************************************************************** */
/*  PERIPHERAL ID DEFINITIONS FOR BTLC1000WLCSP */
/* ************************************************************************** */

/** @}   */

/** \addtogroup BTLC1000WLCSP_base Peripheral Base Address Definitions
 *  @{
 */

/* ************************************************************************** */
/*   BASE ADDRESS DEFINITIONS FOR BTLC1000WLCSP */
/* ************************************************************************** */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#define TIMER0                 (0x40000000)                   /**< \brief (TIMER0    ) Base Address */
#define DUALTIMER0             (0x40001000)                   /**< \brief (DUALTIMER0) Base Address */
#define PROV_DMA_CTRL0         (0x40002000)                   /**< \brief (PROV_DMA_CTRL0) Base Address */
#define I2C0                   (0x40003000)                   /**< \brief (I2C0      ) Base Address */
#define I2C1                   (0x40003400)                   /**< \brief (I2C1      ) Base Address */
#define UART0                  (0x40004000)                   /**< \brief (UART0     ) Base Address */
#define UART1                  (0x40005000)                   /**< \brief (UART1     ) Base Address */
#define SPI0                   (0x40006000)                   /**< \brief (SPI0      ) Base Address */
#define WDT0                   (0x40008000)                   /**< \brief (WDT0      ) Base Address */
#define WDT1                   (0x40009000)                   /**< \brief (WDT1      ) Base Address */
#define EFUSE_MISC_REGS0       (0x4000A000)                   /**< \brief (EFUSE_MISC_REGS0) Base Address */
#define LPMCU_MISC_REGS0       (0x4000B000)                   /**< \brief (LPMCU_MISC_REGS0) Base Address */
#define LP_CLK_CAL_REGS0       (0x4000C000)                   /**< \brief (LP_CLK_CAL_REGS0) Base Address */
#define AON_SLEEP_TIMER0       (0x4000D000)                   /**< \brief (AON_SLEEP_TIMER0) Base Address */
#define AON_PWR_SEQ0           (0x4000E000)                   /**< \brief (AON_PWR_SEQ0) Base Address */
#define AON_GP_REGS0           (0x4000F000)                   /**< \brief (AON_GP_REGS0) Base Address */
#define GPIO0                  (0x40010000)                   /**< \brief (GPIO0     ) Base Address */
#define GPIO1                  (0x40011000)                   /**< \brief (GPIO1     ) Base Address */
#define GPIO2                  (0x40013000)                   /**< \brief (GPIO2     ) Base Address */
#define SPI_FLASH0             (0x40012000)                   /**< \brief (SPI_FLASH0) Base Address */
#define ARM_SYSCTRL0           (0xE000E000)                   /**< \brief (ARM_SYSCTRL0) Base Address */
#define ARM_BPU0               (0xE0002000)                   /**< \brief (ARM_BPU0  ) Base Address */
#define ARM_DWT0               (0xE0001000)                   /**< \brief (ARM_DWT0  ) Base Address */
#define ARM_ROM0               (0xE00FF000)                   /**< \brief (ARM_ROM0  ) Base Address */

#else /* For C/C++ compiler */

#define TIMER0                 ((Timer *)0x40000000U)         /**< \brief (TIMER0    ) Base Address */
#define TIMER_INST_NUM         1                              /**< \brief (TIMER     ) Number of instances */
#define TIMER_INSTS            { TIMER0 }                     /**< \brief (TIMER     ) Instances List */

#define DUALTIMER0             ((Dualtimer *)0x40001000U)     /**< \brief (DUALTIMER0) Base Address */
#define DUALTIMER_INST_NUM     1                              /**< \brief (DUALTIMER ) Number of instances */
#define DUALTIMER_INSTS        { DUALTIMER0 }                 /**< \brief (DUALTIMER ) Instances List */

#define PROV_DMA_CTRL0         ((ProvDmaCtrl *)0x40002000U)   /**< \brief (PROV_DMA_CTRL0) Base Address */
#define PROV_DMA_CTRL_INST_NUM 1                              /**< \brief (PROV_DMA_CTRL) Number of instances */
#define PROV_DMA_CTRL_INSTS    { PROV_DMA_CTRL0 }             /**< \brief (PROV_DMA_CTRL) Instances List */

#define I2C0                   ((I2c *)0x40003000U)           /**< \brief (I2C0      ) Base Address */
#define I2C1                   ((I2c *)0x40003400U)           /**< \brief (I2C1      ) Base Address */
#define I2C_INST_NUM           2                              /**< \brief (I2C       ) Number of instances */
#define I2C_INSTS              { I2C0, I2C1 }                 /**< \brief (I2C       ) Instances List */

#define UART0                  ((Uart *)0x40004000U)          /**< \brief (UART0     ) Base Address */
#define UART1                  ((Uart *)0x40005000U)          /**< \brief (UART1     ) Base Address */
#define UART_INST_NUM          2                              /**< \brief (UART      ) Number of instances */
#define UART_INSTS             { UART0, UART1 }               /**< \brief (UART      ) Instances List */

#define SPI0                   ((Spi *)0x40006000U)           /**< \brief (SPI0      ) Base Address */
#define SPI_INST_NUM           1                              /**< \brief (SPI       ) Number of instances */
#define SPI_INSTS              { SPI0 }                       /**< \brief (SPI       ) Instances List */

#define WDT0                   ((Wdt *)0x40008000U)           /**< \brief (WDT0      ) Base Address */
#define WDT1                   ((Wdt *)0x40009000U)           /**< \brief (WDT1      ) Base Address */
#define WDT_INST_NUM           2                              /**< \brief (WDT       ) Number of instances */
#define WDT_INSTS              { WDT0, WDT1 }                 /**< \brief (WDT       ) Instances List */

#define EFUSE_MISC_REGS0       ((EfuseMiscRegs *)0x4000A000U) /**< \brief (EFUSE_MISC_REGS0) Base Address */
#define EFUSE_MISC_REGS_INST_NUM 1                              /**< \brief (EFUSE_MISC_REGS) Number of instances */
#define EFUSE_MISC_REGS_INSTS  { EFUSE_MISC_REGS0 }           /**< \brief (EFUSE_MISC_REGS) Instances List */

#define LPMCU_MISC_REGS0       ((LpmcuMiscRegs *)0x4000B000U) /**< \brief (LPMCU_MISC_REGS0) Base Address */
#define LPMCU_MISC_REGS_INST_NUM 1                              /**< \brief (LPMCU_MISC_REGS) Number of instances */
#define LPMCU_MISC_REGS_INSTS  { LPMCU_MISC_REGS0 }           /**< \brief (LPMCU_MISC_REGS) Instances List */

#define LP_CLK_CAL_REGS0       ((LpClkCalRegs *)0x4000C000U)  /**< \brief (LP_CLK_CAL_REGS0) Base Address */
#define LP_CLK_CAL_REGS_INST_NUM 1                              /**< \brief (LP_CLK_CAL_REGS) Number of instances */
#define LP_CLK_CAL_REGS_INSTS  { LP_CLK_CAL_REGS0 }           /**< \brief (LP_CLK_CAL_REGS) Instances List */

#define AON_SLEEP_TIMER0       ((AonSleepTimer *)0x4000D000U) /**< \brief (AON_SLEEP_TIMER0) Base Address */
#define AON_SLEEP_TIMER_INST_NUM 1                              /**< \brief (AON_SLEEP_TIMER) Number of instances */
#define AON_SLEEP_TIMER_INSTS  { AON_SLEEP_TIMER0 }           /**< \brief (AON_SLEEP_TIMER) Instances List */

#define AON_PWR_SEQ0           ((AonPwrSeq *)0x4000E000U)     /**< \brief (AON_PWR_SEQ0) Base Address */
#define AON_PWR_SEQ_INST_NUM   1                              /**< \brief (AON_PWR_SEQ) Number of instances */
#define AON_PWR_SEQ_INSTS      { AON_PWR_SEQ0 }               /**< \brief (AON_PWR_SEQ) Instances List */

#define AON_GP_REGS0           ((AonGpRegs *)0x4000F000U)     /**< \brief (AON_GP_REGS0) Base Address */
#define AON_GP_REGS_INST_NUM   1                              /**< \brief (AON_GP_REGS) Number of instances */
#define AON_GP_REGS_INSTS      { AON_GP_REGS0 }               /**< \brief (AON_GP_REGS) Instances List */

#define GPIO0                  ((Gpio *)0x40010000U)          /**< \brief (GPIO0     ) Base Address */
#define GPIO1                  ((Gpio *)0x40011000U)          /**< \brief (GPIO1     ) Base Address */
#define GPIO2                  ((Gpio *)0x40013000U)          /**< \brief (GPIO2     ) Base Address */
#define GPIO_INST_NUM          3                              /**< \brief (GPIO      ) Number of instances */
#define GPIO_INSTS             { GPIO0, GPIO1, GPIO2 }        /**< \brief (GPIO      ) Instances List */

#define SPI_FLASH0             ((SpiFlash *)0x40012000U)      /**< \brief (SPI_FLASH0) Base Address */
#define SPI_FLASH_INST_NUM     1                              /**< \brief (SPI_FLASH ) Number of instances */
#define SPI_FLASH_INSTS        { SPI_FLASH0 }                 /**< \brief (SPI_FLASH ) Instances List */

#define ARM_SYSCTRL0           ((ArmSysctrl *)0xE000E000U)    /**< \brief (ARM_SYSCTRL0) Base Address */
#define ARM_SYSCTRL_INST_NUM   1                              /**< \brief (ARM_SYSCTRL) Number of instances */
#define ARM_SYSCTRL_INSTS      { ARM_SYSCTRL0 }               /**< \brief (ARM_SYSCTRL) Instances List */

#define ARM_BPU0               ((ArmBpu *)0xE0002000U)        /**< \brief (ARM_BPU0  ) Base Address */
#define ARM_BPU_INST_NUM       1                              /**< \brief (ARM_BPU   ) Number of instances */
#define ARM_BPU_INSTS          { ARM_BPU0 }                   /**< \brief (ARM_BPU   ) Instances List */

#define ARM_DWT0               ((ArmDwt *)0xE0001000U)        /**< \brief (ARM_DWT0  ) Base Address */
#define ARM_DWT_INST_NUM       1                              /**< \brief (ARM_DWT   ) Number of instances */
#define ARM_DWT_INSTS          { ARM_DWT0 }                   /**< \brief (ARM_DWT   ) Instances List */

#define ARM_ROM0               ((ArmRom *)0xE00FF000U)        /**< \brief (ARM_ROM0  ) Base Address */
#define ARM_ROM_INST_NUM       1                              /**< \brief (ARM_ROM   ) Number of instances */
#define ARM_ROM_INSTS          { ARM_ROM0 }                   /**< \brief (ARM_ROM   ) Instances List */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of Peripheral Base Address Definitions */

/** \addtogroup BTLC1000WLCSP_pio Peripheral Pio Definitions
 *  @{
 */

/* ************************************************************************** */
/*   PIO DEFINITIONS FOR BTLC1000WLCSP*/
/* ************************************************************************** */
#include "pio/btlc1000wlcsp.h"
/** @}  end of Peripheral Pio Definitions */

/* ************************************************************************** */
/*   MEMORY MAPPING DEFINITIONS FOR BTLC1000WLCSP*/
/* ************************************************************************** */

#define BOOTROM_SIZE             _U_(0x00020000)       /*  128kB Memory segment type: rom */
#define IDRAM_SIZE               _U_(0x00020000)       /*  128kB Memory segment type: ram */
#define BLERAM_SIZE              _U_(0x00002000)       /*    8kB Memory segment type: ram */
#define APB_SIZE                 _U_(0x00020000)       /*  128kB Memory segment type: io */

#define BOOTROM_ADDR             _U_(0x00000000)       /**< BOOTROM base address (type: rom)*/
#define IDRAM_ADDR               _U_(0x10000000)       /**< IDRAM base address (type: ram)*/
#define BLERAM_ADDR              _U_(0x10040000)       /**< BLERAM base address (type: ram)*/
#define APB_ADDR                 _U_(0x40000000)       /**< APB base address (type: io)*/

/* ************************************************************************** */
/**  DEVICE SIGNATURES FOR BTLC1000WLCSP */
/* ************************************************************************** */
#define LPMCU_CHIP_ID_REV_ID     _UL_(0X002000B0)

/* ************************************************************************** */
/**  ELECTRICAL DEFINITIONS FOR BTLC1000WLCSP */
/* ************************************************************************** */

#ifdef __cplusplus
}
#endif

/** @}  end of BTLC1000WLCSP definitions */


#endif /* _BTLC1000WLCSP_H_ */
