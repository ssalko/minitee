/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"

unsigned long sec_ctx[100];
unsigned long nsec_ctx[100];

/* Declare some stack space for each mode. */
portSTACK_TYPE puxFIQStack[ portFIQ_STACK_SIZE ];
portSTACK_TYPE puxIRQStack[ portIRQ_STACK_SIZE ];
portSTACK_TYPE puxAbortStack[ portABORT_STACK_SIZE ];
portSTACK_TYPE puxSVCStack[ portSVC_STACK_SIZE ];
portSTACK_TYPE puxMONStack[ portMON_STACK_SIZE ];
portSTACK_TYPE *fiq_sp_ptr = &(puxFIQStack[ portFIQ_STACK_SIZE - 1 ] );
portSTACK_TYPE *irq_sp_ptr = &(puxIRQStack[ portIRQ_STACK_SIZE - 1 ] );
portSTACK_TYPE *abt_sp_ptr = &(puxAbortStack[ portABORT_STACK_SIZE - 1 ] );
portSTACK_TYPE *svc_sp_ptr = &(puxSVCStack[ portSVC_STACK_SIZE - 1 ] );
portSTACK_TYPE *mon_sp_ptr = &(puxMONStack[ portMON_STACK_SIZE - 1 ] );

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x0000001F )

extern unsigned portBASE_TYPE * volatile pxCurrentTCB;

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
portSTACK_TYPE *pxOriginalStack = pxTopOfStack;
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) 0;	/* LR */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxOriginalStack;	/* SP */
	pxTopOfStack -= 12;		/* R1 through R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters;	/* R0 */

	return pxTopOfStack;
}

void vPortSVCHandler( void )
{
	__asm volatile(
			" ldr r9, pxCurrentTCBConst2	\n"		/* Load the pxCurrentTCB pointer address. */
			" ldr r8, [r9]					\n"		/* Load the pxCurrentTCB address. */
			" ldr lr, [r8]					\n"		/* Load the Task Stack Pointer into LR. */
//			" ldmia lr, {r0-lr}^		 	\n"		/* Load the Task's registers. */
			" add lr, lr, #60			 	\n"		/* Re-adjust the stack for the Task Context */
			" nop						 	\n"
			" rfeia lr				 		\n"		/* Return from exception by loading the PC and CPSR from Task Stack. */
			" nop						 	\n"
			"								\n"
			"	.align 2					\n"
			" pxCurrentTCBConst2: .word pxCurrentTCB	\n"
			);
}

void vPortStartFirstTask( void )
{
	__asm volatile(
					" mov SP, %[svcsp]			\n" /* Set-up the supervisor stack. */
					" svc 0 					\n" /* Use the supervisor call to be in an exception. */
					" nop						\n"
					: : [pxTCB] "r" (pxCurrentTCB), [svcsp] "r" (svc_sp_ptr) :
				);
}

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
//	prvSetupTimerInterrupt();

	/* Install the interrupt handler. */
	//vPortInstallInterruptHandler( (void (*)(void *))vPortYieldFromISR, NULL, portSGI_YIELD_VECTOR_ID, pdTRUE, /* configMAX_SYSCALL_INTERRUPT_PRIORITY */ configKERNEL_INTERRUPT_PRIORITY, 1 );

#if 0
	/* Finally, allow the GIC to pass interrupts to the processor. */
	portGIC_WRITE( portGIC_ICDDCR(portGIC_DISTRIBUTOR_BASE), 0x01UL );
	portGIC_WRITE( portGIC_ICCBPR(portGIC_PRIVATE_BASE), 0x00UL );
	portGIC_WRITE( portGIC_ICCPMR(portGIC_PRIVATE_BASE), configLOWEST_INTERRUPT_PRIORITY );
	portGIC_WRITE( portGIC_ICCICR(portGIC_PRIVATE_BASE), 0x01UL );
#endif

	/* Start the first task. */
	vPortStartFirstTask();

	/* Should not get here! */
	return 0;
}

void vApplicationMallocFailedHook( void )
{
    __asm volatile ("b .");
}

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the CM3 port will require this function as there
	is nothing to return to.  */
}

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

static inline void write32(uint32_t val, vaddr_t addr)
{
    *(volatile uint32_t *)addr = val;
}

static inline uint32_t read32(vaddr_t addr)
{
    return *(volatile uint32_t *)addr;
}

static void gic_init(void)
{
    size_t n;

//	n = 0;
	//{
    for (n = 0; n <= 8; n++) { //256 /* TODO */ / NUM_INTS_PER_REG; n++) {
        /* Disable interrupts */
        write32(0xffffffff, portGIC_DISTRIBUTOR_BASE + GICD_ICENABLER(n));

        /* Make interrupts non-pending */
        write32(0xffffffff, portGIC_DISTRIBUTOR_BASE + GICD_ICPENDR(n));

        /* Mark interrupts non-secure */
        if (n == 0) {
            /* per-CPU inerrupts config:
                         * ID0-ID7(SGI)   for Non-secure interrupts
                         * ID8-ID15(SGI)  for Secure interrupts.
                         * All PPI config as Non-secure interrupts.
             */
            write32(0xffff00ff, portGIC_DISTRIBUTOR_BASE + GICD_IGROUPR(n));
        } else {
            write32(0x00000002, portGIC_DISTRIBUTOR_BASE + GICD_IGROUPR(n));
        }
    }

    /* Set the priority mask to permit Non-secure interrupts, and to
     * allow the Non-secure world to adjust the priority mask itself
     */
    write32(0x80, portGIC_PRIVATE_BASE + GICC_PMR);

    /* Enable GIC */
    write32(GICC_CTLR_ENABLEGRP0 | GICC_CTLR_ENABLEGRP1 | GICC_CTLR_FIQEN,
        portGIC_PRIVATE_BASE + GICC_CTLR);

    write32(read32(portGIC_DISTRIBUTOR_BASE + GICD_CTLR) | GICD_CTLR_ENABLEGRP0 |
        GICD_CTLR_ENABLEGRP1, portGIC_DISTRIBUTOR_BASE + GICD_CTLR);
}

#define	SP804_TIMER1_LOAD	0x00
#define	SP804_TIMER1_VALUE	0x04
#define	SP804_TIMER1_CONTROL	0x08
#define	TIMER_CONTROL_EN	(1 << 7)
#define	TIMER_CONTROL_FREERUN	(0 << 6)
#define	TIMER_CONTROL_PERIODIC	(1 << 6)
#define	TIMER_CONTROL_INTREN	(1 << 5)
#define	TIMER_CONTROL_DIV1	(0 << 2)
#define	TIMER_CONTROL_DIV16	(1 << 2)
#define	TIMER_CONTROL_DIV256	(2 << 2)
#define	TIMER_CONTROL_32BIT	(1 << 1)
#define	TIMER_CONTROL_ONESHOT	(1 << 0)
#define	SP804_TIMER1_INTCLR	0x0C
#define	SP804_TIMER1_RIS	0x10
#define	SP804_TIMER1_MIS	0x14
#define	SP804_TIMER1_BGLOAD	0x18
#define	SP804_TIMER2_LOAD	0x20
#define	SP804_TIMER2_VALUE	0x24
#define	SP804_TIMER2_CONTROL	0x28
#define	SP804_TIMER2_INTCLR	0x2C
#define	SP804_TIMER2_RIS	0x30
#define	SP804_TIMER2_MIS	0x34
#define	SP804_TIMER2_BGLOAD	0x38

#define	SP804_PERIPH_ID0	0xFE0
#define	SP804_PERIPH_ID1	0xFE4
#define	SP804_PERIPH_ID2	0xFE8
#define	SP804_PERIPH_ID3	0xFEC
#define	SP804_PRIMECELL_ID0	0xFF0
#define	SP804_PRIMECELL_ID1	0xFF4
#define	SP804_PRIMECELL_ID2	0xFF8
#define	SP804_PRIMECELL_ID3	0xFFC

#define	DEFAULT_FREQUENCY	1000000
/*
 * QEMU seems to have problem with full frequency
 */
#define	DEFAULT_DIVISOR		16
#define	DEFAULT_CONTROL_DIV	TIMER_CONTROL_DIV16

void fiq_print() {
	vSerialPutString(0, "jeee fiq\n", 9);
}

static int start_timer() {
	uint32_t reg = 0;

    write32(1 << (48 - 32), portGIC_DISTRIBUTOR_BASE + GICD_ISENABLER(2));

	write32(0xfffff, 0x20001000 + SP804_TIMER1_LOAD);
	reg = TIMER_CONTROL_32BIT | TIMER_CONTROL_INTREN |
		    TIMER_CONTROL_PERIODIC | DEFAULT_CONTROL_DIV |
		    TIMER_CONTROL_EN;
	write32(reg, 0x20001000 + SP804_TIMER1_CONTROL);
}

static void mode_switch(void) {
	__asm volatile ("nop\nnop\n"
					"smc #0 \n"
					"nop");
}

static portTASK_FUNCTION( vMainTask, pvParameters )
{
	portTickType xTimeToWait;

    /* Just to stop compiler warnings. */
    ( void ) pvParameters;

    for( ;; )
    {
		mode_switch();

		vSerialPutString(0, "testing2...\n", 12);

        /* We have posted all the characters in the string - wait before
        re-sending.  Wait a pseudo-random time as this will provide a better
        test. */
        xTimeToWait = xTaskGetTickCount() + 3;

        /* Make sure we don't wait too long... */
        xTimeToWait %= 0x96;

        /* ...but we do want to wait. */
        if( xTimeToWait < 0x32 )
        {
            xTimeToWait = 0x32;
        }

        vTaskDelay( xTimeToWait );
    }
}

int main(void) {
	gic_init();
	xSerialPortInitMinimal( 9600, 1000 );

	vSerialPutString(0, "Booting minitee...\n", 19);

//	start_timer();
#if 0
    xTaskCreate( vMainTask, ( signed char * ) "maintask", 1024, NULL, 1, ( xTaskHandle * ) NULL );

    /* Start the scheduler. */
    vTaskStartScheduler();
#endif

	mode_switch();

    for( ;; )
    {
		vSerialPutString(0, "testing...\n", 11);
		mode_switch();
	}

	return 0;
}
