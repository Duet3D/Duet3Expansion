/**
 * \file
 *
 * \brief IAR startup file for ATSAME70J21
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

#include "same70j21.h"

typedef void (*intfunc) (void);
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

void __iar_program_start(void);
int __low_level_init(void);

/* Reset handler */
void Reset_Handler(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M7 core handlers */
#pragma weak NonMaskableInt_Handler=Dummy_Handler
#pragma weak HardFault_Handler=Dummy_Handler
#pragma weak MemoryManagement_Handler=Dummy_Handler
#pragma weak BusFault_Handler=Dummy_Handler
#pragma weak UsageFault_Handler=Dummy_Handler
#pragma weak SVCall_Handler=Dummy_Handler
#pragma weak DebugMonitor_Handler=Dummy_Handler
#pragma weak PendSV_Handler=Dummy_Handler
#pragma weak SysTick_Handler=Dummy_Handler

/* Peripherals handlers */
#pragma weak SUPC_Handler=Dummy_Handler
#pragma weak RSTC_Handler=Dummy_Handler
#pragma weak RTC_Handler=Dummy_Handler
#pragma weak RTT_Handler=Dummy_Handler
#pragma weak WDT_Handler=Dummy_Handler
#pragma weak PMC_Handler=Dummy_Handler
#pragma weak EFC_Handler=Dummy_Handler
#pragma weak UART0_Handler=Dummy_Handler
#pragma weak UART1_Handler=Dummy_Handler
#pragma weak PIOA_Handler=Dummy_Handler
#pragma weak PIOB_Handler=Dummy_Handler
#pragma weak USART0_Handler=Dummy_Handler
#pragma weak USART1_Handler=Dummy_Handler
#pragma weak PIOD_Handler=Dummy_Handler
#pragma weak TWIHS0_Handler=Dummy_Handler
#pragma weak TWIHS1_Handler=Dummy_Handler
#pragma weak SSC_Handler=Dummy_Handler
#pragma weak TC0_Handler=Dummy_Handler
#pragma weak TC1_Handler=Dummy_Handler
#pragma weak TC2_Handler=Dummy_Handler
#pragma weak AFEC0_Handler=Dummy_Handler
#pragma weak DACC_Handler=Dummy_Handler
#pragma weak PWM0_Handler=Dummy_Handler
#pragma weak ICM_Handler=Dummy_Handler
#pragma weak ACC_Handler=Dummy_Handler
#pragma weak USBHS_Handler=Dummy_Handler
#pragma weak MCAN0_INT0_Handler=Dummy_Handler
#pragma weak MCAN0_INT1_Handler=Dummy_Handler
#pragma weak GMAC_Handler=Dummy_Handler
#pragma weak AFEC1_Handler=Dummy_Handler
#pragma weak QSPI_Handler=Dummy_Handler
#pragma weak UART2_Handler=Dummy_Handler
#pragma weak TC9_Handler=Dummy_Handler
#pragma weak TC10_Handler=Dummy_Handler
#pragma weak TC11_Handler=Dummy_Handler
#pragma weak AES_Handler=Dummy_Handler
#pragma weak TRNG_Handler=Dummy_Handler
#pragma weak XDMAC_Handler=Dummy_Handler
#pragma weak ISI_Handler=Dummy_Handler
#pragma weak PWM1_Handler=Dummy_Handler
#pragma weak FPU_Handler=Dummy_Handler
#pragma weak RSWDT_Handler=Dummy_Handler
#pragma weak CCW_Handler=Dummy_Handler
#pragma weak CCF_Handler=Dummy_Handler
#pragma weak GMAC_Q1_Handler=Dummy_Handler
#pragma weak GMAC_Q2_Handler=Dummy_Handler
#pragma weak IXC_Handler=Dummy_Handler

/* Exception Table */
#pragma language = extended
#pragma segment = "CSTACK"

/* The name "__vector_table" has special meaning for C-SPY: */
/* it is where the SP start value is found, and the NVIC vector */
/* table register (VTOR) is initialized to this address if != 0 */

#pragma section = ".intvec"
#pragma location = ".intvec"
const DeviceVectors __vector_table = {
        (void*) __sfe("CSTACK"),

        .pfnReset_Handler              = (void*) Reset_Handler,
        .pfnNonMaskableInt_Handler     = (void*) NonMaskableInt_Handler,
        .pfnHardFault_Handler          = (void*) HardFault_Handler,
        .pfnMemoryManagement_Handler   = (void*) MemoryManagement_Handler,
        .pfnBusFault_Handler           = (void*) BusFault_Handler,
        .pfnUsageFault_Handler         = (void*) UsageFault_Handler,
        .pvReservedC9                  = (void*) (0UL), /* Reserved */
        .pvReservedC8                  = (void*) (0UL), /* Reserved */
        .pvReservedC7                  = (void*) (0UL), /* Reserved */
        .pvReservedC6                  = (void*) (0UL), /* Reserved */
        .pfnSVCall_Handler             = (void*) SVCall_Handler,
        .pfnDebugMonitor_Handler       = (void*) DebugMonitor_Handler,
        .pvReservedC3                  = (void*) (0UL), /* Reserved */
        .pfnPendSV_Handler             = (void*) PendSV_Handler,
        .pfnSysTick_Handler            = (void*) SysTick_Handler,

        /* Configurable interrupts */
        .pfnSUPC_Handler               = (void*) SUPC_Handler,   /* 0  Supply Controller */
        .pfnRSTC_Handler               = (void*) RSTC_Handler,   /* 1  Reset Controller */
        .pfnRTC_Handler                = (void*) RTC_Handler,    /* 2  Real-time Clock */
        .pfnRTT_Handler                = (void*) RTT_Handler,    /* 3  Real-time Timer */
        .pfnWDT_Handler                = (void*) WDT_Handler,    /* 4  Watchdog Timer */
        .pfnPMC_Handler                = (void*) PMC_Handler,    /* 5  Power Management Controller */
        .pfnEFC_Handler                = (void*) EFC_Handler,    /* 6  Embedded Flash Controller */
        .pfnUART0_Handler              = (void*) UART0_Handler,  /* 7  Universal Asynchronous Receiver Transmitter */
        .pfnUART1_Handler              = (void*) UART1_Handler,  /* 8  Universal Asynchronous Receiver Transmitter */
        .pvReserved9                   = (void*) (0UL),          /* 9  Reserved */
        .pfnPIOA_Handler               = (void*) PIOA_Handler,   /* 10 Parallel Input/Output Controller */
        .pfnPIOB_Handler               = (void*) PIOB_Handler,   /* 11 Parallel Input/Output Controller */
        .pvReserved12                  = (void*) (0UL),          /* 12 Reserved */
        .pfnUSART0_Handler             = (void*) USART0_Handler, /* 13 Universal Synchronous Asynchronous Receiver Transmitter */
        .pfnUSART1_Handler             = (void*) USART1_Handler, /* 14 Universal Synchronous Asynchronous Receiver Transmitter */
        .pvReserved15                  = (void*) (0UL),          /* 15 Reserved */
        .pfnPIOD_Handler               = (void*) PIOD_Handler,   /* 16 Parallel Input/Output Controller */
        .pvReserved17                  = (void*) (0UL),          /* 17 Reserved */
        .pvReserved18                  = (void*) (0UL),          /* 18 Reserved */
        .pfnTWIHS0_Handler             = (void*) TWIHS0_Handler, /* 19 Two-wire Interface High Speed */
        .pfnTWIHS1_Handler             = (void*) TWIHS1_Handler, /* 20 Two-wire Interface High Speed */
        .pvReserved21                  = (void*) (0UL),          /* 21 Reserved */
        .pfnSSC_Handler                = (void*) SSC_Handler,    /* 22 Synchronous Serial Controller */
        .pfnTC0_Handler                = (void*) TC0_Handler,    /* 23 Timer Counter */
        .pfnTC1_Handler                = (void*) TC1_Handler,    /* 24 Timer Counter */
        .pfnTC2_Handler                = (void*) TC2_Handler,    /* 25 Timer Counter */
        .pvReserved26                  = (void*) (0UL),          /* 26 Reserved */
        .pvReserved27                  = (void*) (0UL),          /* 27 Reserved */
        .pvReserved28                  = (void*) (0UL),          /* 28 Reserved */
        .pfnAFEC0_Handler              = (void*) AFEC0_Handler,  /* 29 Analog Front-End Controller */
        .pfnDACC_Handler               = (void*) DACC_Handler,   /* 30 Digital-to-Analog Converter Controller */
        .pfnPWM0_Handler               = (void*) PWM0_Handler,   /* 31 Pulse Width Modulation Controller */
        .pfnICM_Handler                = (void*) ICM_Handler,    /* 32 Integrity Check Monitor */
        .pfnACC_Handler                = (void*) ACC_Handler,    /* 33 Analog Comparator Controller */
        .pfnUSBHS_Handler              = (void*) USBHS_Handler,  /* 34 USB High-Speed Interface */
        .pfnMCAN0_INT0_Handler         = (void*) MCAN0_INT0_Handler, /* 35 Controller Area Network */
        .pfnMCAN0_INT1_Handler         = (void*) MCAN0_INT1_Handler, /* 36 Controller Area Network */
        .pvReserved37                  = (void*) (0UL),          /* 37 Reserved */
        .pvReserved38                  = (void*) (0UL),          /* 38 Reserved */
        .pfnGMAC_Handler               = (void*) GMAC_Handler,   /* 39 Gigabit Ethernet MAC */
        .pfnAFEC1_Handler              = (void*) AFEC1_Handler,  /* 40 Analog Front-End Controller */
        .pvReserved41                  = (void*) (0UL),          /* 41 Reserved */
        .pvReserved42                  = (void*) (0UL),          /* 42 Reserved */
        .pfnQSPI_Handler               = (void*) QSPI_Handler,   /* 43 Quad Serial Peripheral Interface */
        .pfnUART2_Handler              = (void*) UART2_Handler,  /* 44 Universal Asynchronous Receiver Transmitter */
        .pvReserved45                  = (void*) (0UL),          /* 45 Reserved */
        .pvReserved46                  = (void*) (0UL),          /* 46 Reserved */
        .pvReserved47                  = (void*) (0UL),          /* 47 Reserved */
        .pvReserved48                  = (void*) (0UL),          /* 48 Reserved */
        .pvReserved49                  = (void*) (0UL),          /* 49 Reserved */
        .pfnTC9_Handler                = (void*) TC9_Handler,    /* 50 Timer Counter */
        .pfnTC10_Handler               = (void*) TC10_Handler,   /* 51 Timer Counter */
        .pfnTC11_Handler               = (void*) TC11_Handler,   /* 52 Timer Counter */
        .pvReserved53                  = (void*) (0UL),          /* 53 Reserved */
        .pvReserved54                  = (void*) (0UL),          /* 54 Reserved */
        .pvReserved55                  = (void*) (0UL),          /* 55 Reserved */
        .pfnAES_Handler                = (void*) AES_Handler,    /* 56 Advanced Encryption Standard */
        .pfnTRNG_Handler               = (void*) TRNG_Handler,   /* 57 True Random Number Generator */
        .pfnXDMAC_Handler              = (void*) XDMAC_Handler,  /* 58 Extensible DMA Controller */
        .pfnISI_Handler                = (void*) ISI_Handler,    /* 59 Image Sensor Interface */
        .pfnPWM1_Handler               = (void*) PWM1_Handler,   /* 60 Pulse Width Modulation Controller */
        .pfnFPU_Handler                = (void*) FPU_Handler,    /* 61 Floating Point Unit Registers */
        .pvReserved62                  = (void*) (0UL),          /* 62 Reserved */
        .pfnRSWDT_Handler              = (void*) RSWDT_Handler,  /* 63 Reinforced Safety Watchdog Timer */
        .pfnCCW_Handler                = (void*) CCW_Handler,    /* 64 System Control Registers */
        .pfnCCF_Handler                = (void*) CCF_Handler,    /* 65 System Control Registers */
        .pfnGMAC_Q1_Handler            = (void*) GMAC_Q1_Handler, /* 66 Gigabit Ethernet MAC */
        .pfnGMAC_Q2_Handler            = (void*) GMAC_Q2_Handler, /* 67 Gigabit Ethernet MAC */
        .pfnIXC_Handler                = (void*) IXC_Handler     /* 68 Floating Point Unit Registers */
};

/**------------------------------------------------------------------------------
 * This is the code that gets called on processor reset. To initialize the
 * device.
 *------------------------------------------------------------------------------*/
int __low_level_init(void)
{
        uint32_t *pSrc = __section_begin(".intvec");

        SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

        return 1; /* if return 0, the data sections will not be initialized */
}

/**------------------------------------------------------------------------------
 * This is the code that gets called on processor reset. To initialize the
 * device.
 *------------------------------------------------------------------------------*/
void Reset_Handler(void)
{
        __iar_program_start();
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
        while (1) {
        }
}
