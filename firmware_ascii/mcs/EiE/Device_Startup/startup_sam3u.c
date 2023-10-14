/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) %copyright_year%, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

#include "sam3u.h"
#include "core_cm3.h"

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

/** \cond DOXYGEN_SHOULD_SKIP_THIS */
int main(void);
/** \endcond */

void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M3 core handlers */
void NMI_Handler        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void MemManage_Handler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void BusFault_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UsageFault_Handler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DebugMon_Handler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler     ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Peripherals handlers */
void SUPC_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RSTC_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTT_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PMC_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EFC0_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_EFC1_INSTANCE_
void EFC1_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_EFC1_INSTANCE_ */
void UART_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PIOA_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PIOB_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_PIOC_INSTANCE_
void PIOC_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_PIOC_INSTANCE_ */
void USART0_IrqHandler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART1_IrqHandler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART2_IrqHandler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef _SAM3U_USART3_INSTANCE_
void USART3_IrqHandler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif /* _SAM3U_USART3_INSTANCE_ */
void HSMCI_IrqHandler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TWI0_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TWI1_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SSC_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC0_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC1_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC2_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC12B_IrqHandler ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_IrqHandler    ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_IrqHandler   ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UDPHS_IrqHandler  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table = {

        /* Configure Initial Stack Pointer, using linker-generated symbols */
        .pvStack = (void*) (&_estack),

        .pfnReset_Handler      = (void*) Reset_Handler,
        .pfnNMI_Handler        = (void*) NMI_Handler,
        .pfnHardFault_Handler  = (void*) HardFault_Handler,
        .pfnMemManage_Handler  = (void*) MemManage_Handler,
        .pfnBusFault_Handler   = (void*) BusFault_Handler,
        .pfnUsageFault_Handler = (void*) UsageFault_Handler,
        .pfnReserved1_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved2_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved3_Handler  = (void*) (0UL),          /* Reserved */
        .pfnReserved4_Handler  = (void*) (0UL),          /* Reserved */
        .pfnSVC_Handler        = (void*) SVC_Handler,
        .pfnDebugMon_Handler   = (void*) DebugMon_Handler,
        .pfnReserved5_Handler  = (void*) (0UL),          /* Reserved */
        .pfnPendSV_Handler     = (void*) PendSV_Handler,
        .pfnSysTick_Handler    = (void*) SysTick_Handler,

        /* Configurable interrupts */
        .pfnSUPC_Handler   = (void*) SUPC_IrqHandler,   /* 0  Supply Controller */
        .pfnRSTC_Handler   = (void*) RSTC_IrqHandler,   /* 1  Reset Controller */
        .pfnRTC_Handler    = (void*) RTC_IrqHandler,    /* 2  Real Time Clock */
        .pfnRTT_Handler    = (void*) RTT_IrqHandler,    /* 3  Real Time Timer */
        .pfnWDT_Handler    = (void*) WDT_IrqHandler,    /* 4  Watchdog Timer */
        .pfnPMC_Handler    = (void*) PMC_IrqHandler,    /* 5  Power Management Controller */
        .pfnEFC0_Handler   = (void*) EFC0_IrqHandler,   /* 6  Enhanced Embedded Flash Controller 0 */
#ifdef _SAM3U_EFC1_INSTANCE_
        .pfnEFC1_Handler   = (void*) EFC1_IrqHandler,   /* 7  Enhanced Embedded Flash Controller 1 */
#else
        .pvReserved7       = (void*) (0UL),          /* 7  Reserved */
#endif /* _SAM3U_EFC1_INSTANCE_ */
        .pfnUART_Handler   = (void*) UART_IrqHandler,   /* 8  Universal Asynchronous Receiver Transmitter */
        .pvReserved9       = (void*) (0UL),          /* 9  Reserved */
        .pfnPIOA_Handler   = (void*) PIOA_IrqHandler,   /* 10 Parallel I/O Controller A, */
        .pfnPIOB_Handler   = (void*) PIOB_IrqHandler,   /* 11 Parallel I/O Controller B */
#ifdef _SAM3U_PIOC_INSTANCE_
        .pfnPIOC_Handler   = (void*) PIOC_IrqHandler,   /* 12 Parallel I/O Controller C */
#else
        .pvReserved12      = (void*) (0UL),          /* 12 Reserved */
#endif /* _SAM3U_PIOC_INSTANCE_ */
        .pfnUSART0_Handler = (void*) USART0_IrqHandler, /* 13 USART 0 */
        .pfnUSART1_Handler = (void*) USART1_IrqHandler, /* 14 USART 1 */
        .pfnUSART2_Handler = (void*) USART2_IrqHandler, /* 15 USART 2 */
#ifdef _SAM3U_USART3_INSTANCE_
        .pfnUSART3_Handler = (void*) USART3_IrqHandler, /* 16 USART 3 */
#else
        .pvReserved16      = (void*) (0UL),          /* 16 Reserved */
#endif /* _SAM3U_USART3_INSTANCE_ */
        .pfnHSMCI_Handler  = (void*) HSMCI_IrqHandler,  /* 17 High Speed Multimedia Card Interface */
        .pfnTWI0_Handler   = (void*) TWI0_IrqHandler,   /* 18 Two-Wire Interface 0 */
        .pfnTWI1_Handler   = (void*) TWI1_IrqHandler,   /* 19 Two-Wire Interface 1 */
        .pfnSPI_Handler    = (void*) SPI_IrqHandler,    /* 20 Serial Peripheral Interface */
        .pfnSSC_Handler    = (void*) SSC_IrqHandler,    /* 21 Synchronous Serial Controller */
        .pfnTC0_Handler    = (void*) TC0_IrqHandler,    /* 22 Timer Counter 0 */
        .pfnTC1_Handler    = (void*) TC1_IrqHandler,    /* 23 Timer Counter 1 */
        .pfnTC2_Handler    = (void*) TC2_IrqHandler,    /* 24 Timer Counter 2 */
        .pfnPWM_Handler    = (void*) PWM_IrqHandler,    /* 25 Pulse Width Modulation Controller */
        .pfnADC12B_Handler = (void*) ADC12B_IrqHandler, /* 26 12-bit ADC Controller */
        .pfnADC_Handler    = (void*) ADC_IrqHandler,    /* 27 10-bit ADC Controller */
        .pfnDMAC_Handler   = (void*) DMAC_IrqHandler,   /* 28 DMA Controller */
        .pfnUDPHS_Handler  = (void*) UDPHS_IrqHandler   /* 29 USB Device High Speed */
};

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler(void)
{
        uint32_t *pSrc, *pDest;

        /* Initialize the relocate segment */
        pSrc = &_etext;
        pDest = &_srelocate;

        if (pSrc != pDest) {
                for (; pDest < &_erelocate;) {
                        *pDest++ = *pSrc++;
                }
        }

        /* Clear the zero segment */
        for (pDest = &_szero; pDest < &_ezero;) {
                *pDest++ = 0;
        }

        /* Set the vector table base address */
        pSrc = (uint32_t *) & _sfixed;
        SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

        /* Initialize the C library */
        __libc_init_array();

        /* Branch to main function */
        main();

        /* Infinite loop */
        while (1);
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
        while (1) {
        }
}

extern int  _sheap;

/**
 * \brief _sbrk implementation for compatibility
 */
unsigned int _sbrk( int incr )
{
	static unsigned char *heap = 0;
	unsigned char *prev_heap;
	
	if (heap == 0) {
		heap = (unsigned char *) &(_sheap);
	}
	prev_heap = heap;
	heap += incr;

	return (unsigned int) prev_heap;
}
