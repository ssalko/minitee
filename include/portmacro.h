/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portLONG
#define portBASE_TYPE	long

#if( configUSE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_RATE_MS			( ( portTickType ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			8
#define portHEAP_END				( ( (char * )&_data + (unsigned long)configTOTAL_HEAP_SIZE ) )
/*-----------------------------------------------------------*/

/* Critical section management. */
extern void vTaskEnterCritical();
extern void vTaskExitCritical();
#define portCRITICAL_NESTING_IN_TCB			1
#define portENTER_CRITICAL()				vTaskEnterCritical()
#define portEXIT_CRITICAL()					vTaskExitCritical()

//extern portBASE_TYPE xPortSetInterruptMask(void);
//extern void vPortClearInterruptMask(portBASE_TYPE);
#define portSET_INTERRUPT_MASK()				0 //xPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK(x)				(x) //vPortClearInterruptMask(x)
#if 0
#define portSET_INTERRUPT_MASK_FROM_ISR()		portSET_INTERRUPT_MASK()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	portCLEAR_INTERRUPT_MASK(x)
#endif
#define portDISABLE_INTERRUPTS()				((void)portSET_INTERRUPT_MASK())
#define portENABLE_INTERRUPTS()					portCLEAR_INTERRUPT_MASK(configLOWEST_INTERRUPT_PRIORITY)

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

//#define portNOP() __asm volatile( "NOP" )
#define portNOP()

/* The System Control Registers. */
#define portSYSCTRL_ZERO_REG				( ( volatile unsigned long * ) 0x1C020000 )		/* realview-pbx-a9 */
#define portSYSCTRL_ONE_REG					( ( volatile unsigned long * ) 0x1001A000 )		/* realview-pbx-a9 */
#define portSYSCTRL_ZERO_TIMER0_ENABLE		( 0x00018000 )

/* Peripheral Base. */
#if configPLATFORM == 0
	#define portGIC_PRIVATE_BASE					( 0x08010000UL )	/* vexpress-a15 */
	#define portGIC_DISTRIBUTOR_BASE				( 0x08000000UL )	/* vexpress-a15 */
	#define portGIC2_BASE							( portGIC_PRIVATE_BASE )	/* vexpress-a15 */
	#define portGIC2_DISTRIBUTOR_BASE				( portGIC_DISTRIBUTOR_BASE )	/* vexpress-a15 */
	#define portEXCEPTION_VECTORS_BASE				( 0x0e000000UL )
	#define portMAX_VECTORS							( 64UL )
#endif

/* GIC Processor Registers. */
#define portGIC_ICCICR(x)					( ((unsigned long)(x)) + 0x00UL )
#define portGIC_ICCPMR(x)					( ((unsigned long)(x)) + 0x04UL )
#define portGIC_ICCBPR(x)					( ((unsigned long)(x)) + 0x08UL )
#define portGIC_ICCIAR(x)					( ((unsigned long)(x)) + 0x0CUL )
#define portGIC_ICCEOIR(x)					( ((unsigned long)(x)) + 0x10UL )
#define portGIC_ICCRPR(x)					( ((unsigned long)(x)) + 0x14UL )
#define portGIC_ICCHPIR(x)					( ((unsigned long)(x)) + 0x18UL )

/* GIC Distributor Registers. */
#define portGIC_ICDDCR(x)					( ((unsigned long)(x)) + 0x00UL )
#define portGIC_ICDICTR(x)					( ((unsigned long)(x)) + 0x04UL )
#define portGIC_ICDISER_BASE(x)				( ((unsigned long)(x)) + 0x100UL )
#define portGIC_ICDICER_BASE(x)				( ((unsigned long)(x)) + 0x180UL )
#define portGIC_ICDISPR_BASE(x)				( ((unsigned long)(x)) + 0x200UL )
#define portGIC_ICDICPR_BASE(x)				( ((unsigned long)(x)) + 0x280UL )
#define portGIC_ICDABR_BASE(x)				( ((unsigned long)(x)) + 0x300UL )
#define portGIC_ICDIPR_BASE(x)				( ((unsigned long)(x)) + 0x400UL )
#define portGIC_ICDIPTR_BASE(x)				( ((unsigned long)(x)) + 0x800UL )
#define portGIC_ICDICR_BASE(x)				( ((unsigned long)(x)) + 0xC00UL )
#define portGIC_ICDPPIS(x)					( ((unsigned long)(x)) + 0xD00UL )
#define portGIC_ICDSPIS_BASE(x)				( ((unsigned long)(x)) + 0xD04UL )
#define portGIC_ICDSGIR(x)					( ((unsigned long)(x)) + 0xF00UL )
#define portGIC_ICDCPENDGIR(x)				( ((unsigned long)(x)) + 0xF10UL )
#define portGIC_ICDSPENDGIR(x)				( ((unsigned long)(x)) + 0xF20UL )

#define __writel(v, a)  (*(volatile unsigned int *)(a) = (v))
#define __readl(a)      (*(volatile unsigned int *)(a))

/* Macros to access the GIC. */
#define portGIC_WRITE(address,value)		( *( ( unsigned long * volatile )( address ) ) = ( value ) )
#define portGIC_SET(address,value)			( *( ( unsigned long * volatile )( address ) ) |= ( value ) )
#define portGIC_READ(address)				( *( ( unsigned long * volatile )( address ) ) )
#define portGIC_CLEAR(address,value)		( *( ( unsigned long * volatile )( address ) ) &= ~( value ) )

#define REG_READ(address)				    ( *( ( unsigned long * volatile )( address ) ) )

/* GIC Values. */
#define portGIC_CONTROL_ENABLE_INTERRUPTS	( 0x01UL )
#define portGIC_CONTROL_DISABLE_INTERRUPTS	( 0x00UL )
#define portGIC_PRIORITY_MASK_MASK			( 0xF0UL )
#define portGIC_SPURIOUS_VECTOR				( 1023 )
#define portGIC_CPU_INTERRUPT_SOURCE_MASK	( 0x1C00UL )
#define portGIC_VECTOR_MASK					( 0x3FFUL )

#if 0
/* Private Timers: Generic Timer */
#define portSYSTICK_BASE				( 0x1C110000 )
#define portSYSTICK_LOAD				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x00 ) )
#define portSYSTICK_VALUE				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x04 ) )
#define portSYSTICK_CONTROL				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x08 ) )
#define portSYSTICK_INTERRUPT_STATUS	( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x0C ) )
#define portSYSTICK_CTRL_ENABLE_PERIODIC_INTERRUPTS			  ( 0x00000007 )
#define portSYSTICK_PRESCALE			( 99 )		/* realview-pbx-a9 */
#define portSYSTICK_VECTOR_ID			( 34 )  /* SP804 TIMER0 IRQ NUMBER */

#define TIMER_CTRL_ONESHOT  (1 << 0)
#define TIMER_CTRL_32BIT    (1 << 1)
#define TIMER_CTRL_DIV1     (0 << 2)
#define TIMER_CTRL_DIV16    (1 << 2)
#define TIMER_CTRL_DIV256   (2 << 2)
#define TIMER_CTRL_IE       (1 << 5)
#define TIMER_CTRL_PERIODIC (1 << 6)
#define TIMER_CTRL_ENABLE   (1 << 7)
#endif
/* Private Timers: Generic Timer */
#define portSYSTICK_BASE				( 0x1C110000 )
#define portSYSTICK_LOAD				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x00 ) )
#define portSYSTICK_VALUE				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x04 ) )
#define portSYSTICK_CONTROL				( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x08 ) )
#define portSYSTICK_INTERRUPT_STATUS	( ( volatile unsigned long * ) ( portSYSTICK_BASE + 0x0C ) )
#define portSYSTICK_CTRL_ENABLE_PERIODIC_INTERRUPTS			  ( 0x00000007 )
#define portSYSTICK_PRESCALE			( 99 )		/* realview-pbx-a9 */
//Secure Physical Timer event (ID 29 <= 16 + 13)
//#define portSYSTICK_VECTOR_ID			( 29 )
#define portSYSTICK_VECTOR_ID_NS		( 30 )

#define portSYSTICK_VECTOR_ID		    ( 34 )

#define GENERIC_TIMER_CTRL_ENABLE       (1 << 0)
#define GENERIC_TIMER_CTRL_IMASK        (1 << 1)
#define GENERIC_TIMER_CTRL_ISTATUS      (1 << 2)

/* 100Mhz -> 1 count == 10ns at RTSM_VE_CA15, fast model */
#define SCHED_TICK  1000
#define CFG_CNTFRQ  100000000
#define USEC        1000000
#define COUNT_PER_USEC (CFG_CNTFRQ/USEC)

#define read_cntfrq()           ({ unsigned int rval; asm volatile(\
                                " mrc     p15, 0, %0, c14, c0, 0\n\t" \
                                : "=r" (rval) : : "memory", "cc"); rval; })

#define write_cntfrq(val)       asm volatile(\
                                " mcr     p15, 0, %0, c14, c0, 0\n\t" \
                                : : "r" ((val)) : "memory", "cc")

#define read_cntpct()           ({ unsigned int v1, v2; asm volatile(\
                                " mrrc     p15, 0, %0, %1, c14\n\t" \
                                : "=r" (v1), "=r" (v2) : : "memory", "cc"); \
                                (((unsigned long long)v2 << 32) + (unsigned long long)v1); })

#define read_cntkctl()          ({ unsigned int rval; asm volatile(\
                                " mrc     p15, 0, %0, c14, c1, 0\n\t" \
                                : "=r" (rval) : : "memory", "cc"); rval; })

#define write_cntkctl(val)      asm volatile(\
                                " mcr     p15, 0, %0, c14, c1, 0\n\t" \
                                : : "r" ((val)) : "memory", "cc")

#define read_cntp_tval()        ({ unsigned int rval; asm volatile(\
                                " mrc     p15, 0, %0, c14, c2, 0\n\t" \
                                : "=r" (rval) : : "memory", "cc"); rval; })

#define write_cntp_tval(val)    asm volatile(\
                                " mcr     p15, 0, %0, c14, c2, 0\n\t" \
                                : : "r" ((val)) : "memory", "cc")

#define read_cntp_ctl()         ({ unsigned int rval; asm volatile(\
                                " mrc     p15, 0, %0, c14, c2, 1\n\t" \
                                : "=r" (rval) : : "memory", "cc"); rval; })

#define write_cntp_ctl(val)     asm volatile(\
                                " mcr     p15, 0, %0, c14, c2, 1\n\t" \
                                : : "r" ((val)) : "memory", "cc")



/* SGI for Yielding Task. */
#define portSGI_YIELD_VECTOR_ID			( 1 )
#define portSGI_YIELD( xCPUID )			( ( 0 << 24 ) | ( ( 1 << 16 ) << ( xCPUID ) ) | portSGI_YIELD_VECTOR_ID )
#define portYIELD()		( ( portGIC_READ( portGIC_ICDISPR_BASE( portGIC_DISTRIBUTOR_BASE ) ) & portSGI_YIELD_VECTOR_ID ) == 0UL ? portGIC_WRITE( portGIC_ICDSGIR( portGIC_DISTRIBUTOR_BASE ), portSGI_YIELD( 0 ) ) : (void)portGIC_DISTRIBUTOR_BASE )
//#define portYIELD()		( ( portGIC_READ( portGIC_ICDSPENDGIR( portGIC_DISTRIBUTOR_BASE ) ) & portSGI_YIELD_VECTOR_ID ) == 0UL ? portGIC_WRITE( portGIC_ICDSGIR( portGIC_DISTRIBUTOR_BASE ), portSGI_YIELD( 0 ) ) : (void)portGIC_DISTRIBUTOR_BASE )
# if 0
static inline void portYIELD(void)
{
    if( ( portGIC_READ( portGIC_ICDISPR_BASE( portGIC_DISTRIBUTOR_BASE ) ) & portSGI_YIELD_VECTOR_ID ) == 0UL)
    {
        portGIC_WRITE( portGIC_ICDSGIR( portGIC_DISTRIBUTOR_BASE ), portSGI_YIELD( 0 ) );
        __asm__ __volatile__ ( "nop" ); /* Allow the yield SGI time to propagate. */
    }
}
#endif
#define portSGI_CLEAR_YIELD( pxDistributorBase, xCPUID )	portGIC_WRITE( portGIC_ICDCPENDGIR( pxDistributorBase ), portSGI_YIELD_VECTOR_ID )
#define portEND_SWITCHING_ISR( xSwitchRequired ) ( (xSwitchRequired) ? portYIELD() : (void)xSwitchRequired )

/* Processor Mode Definitions (CPSR) */
#define portPROCESSOR_MODE_MASK	( ~(0x1FUL) )
#define portUSER_MODE			( 0x10UL )
#define portFIQ_MODE			( 0x11UL )
#define portIRQ_MODE			( 0x12UL )
#define portSVC_MODE			( 0x13UL )
#define portMON_MODE			( 0x16UL )
#define portABT_MODE			( 0x17UL )
#define portUND_MODE			( 0x1BUL )
#define portSYS_MODE			( 0x1FUL )

/* Stack Size definitions for each of the modes. */
#define portFIQ_STACK_SIZE		( 256 )
#define portIRQ_STACK_SIZE		( 256 )
#define portABORT_STACK_SIZE	( 256 )
#define portSVC_STACK_SIZE		( 256 )
#define portMON_STACK_SIZE		( 512 )

/* Number of interrupts in one register */
#define NUM_INTS_PER_REG    32

/* Offsets from gic.gicc_base */
#define GICC_CTLR       (0x000)
#define GICC_PMR        (0x004)
#define GICC_IAR        (0x00C)
#define GICC_EOIR       (0x010)

#define GICC_CTLR_ENABLEGRP0    (1 << 0)
#define GICC_CTLR_ENABLEGRP1    (1 << 1)
#define GICC_CTLR_FIQEN     (1 << 3)

/* Offsets from gic.gicd_base */
#define GICD_CTLR       (0x000)
#define GICD_TYPER      (0x004)
#define GICD_IGROUPR(n)     (0x080 + (n) * 4)
#define GICD_ISENABLER(n)   (0x100 + (n) * 4)
#define GICD_ICENABLER(n)   (0x180 + (n) * 4)
#define GICD_ISPENDR(n)     (0x200 + (n) * 4)
#define GICD_ICPENDR(n)     (0x280 + (n) * 4)
#define GICD_IPRIORITYR(n)  (0x400 + (n) * 4)
#define GICD_ITARGETSR(n)   (0x800 + (n) * 4)
#define GICD_SGIR       (0xF00)

#define GICD_CTLR_ENABLEGRP0    (1 << 0)
#define GICD_CTLR_ENABLEGRP1    (1 << 1)

typedef unsigned int uint32_t;
typedef unsigned long vaddr_t;

/* Processor Mode Definitions (CPSR) */
#define PROCESSOR_MODE_MASK	( ~(0x1FUL) )
#define MODE_USR			( 0x10UL )
#define MODE_FIQ			( 0x11UL )
#define MODE_IRQ			( 0x12UL )
#define MODE_SVC			( 0x13UL )
#define MODE_MON			( 0x16UL )
#define MODE_ABT			( 0x17UL )
#define MODE_UND			( 0x1BUL )
#define MODE_SYS			( 0x1FUL )

/* Stack Size definitions for each of the modes. */
#define portFIQ_STACK_SIZE		( 256 )
#define portIRQ_STACK_SIZE		( 256 )
#define portABORT_STACK_SIZE	( 256 )
#define portSVC_STACK_SIZE		( 256 )
#define portMON_STACK_SIZE		( 512 )

#define SCR_NS (1<<0)
#define SCR_FIQ (1<<2)

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
