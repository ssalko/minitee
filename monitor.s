#include <portmacro.h>

.balign 4
.code 32

.global smc_handler

.extern nsec_spsr_addr
.extern nsec_lr_addr

.equ MODE_MASK,    0x1f
.equ MODE_USR, 0x10
.equ MODE_FIQ, 0x11
.equ MODE_IRQ, 0x12
.equ MODE_SVC, 0x13
.equ MODE_MON, 0x16
.equ MODE_ABT, 0x17
.equ MODE_UND, 0x1b
.equ MODE_SYS, 0x1f

.equ SCR_NS, (1<<0)
.equ SCR_FIQ, (1<<2)


save_context:
    cps #MODE_SYS
    stm r0!, {sp, lr}

    cps #MODE_IRQ
    mrs r2, spsr
    stm r0!, {r2, sp, lr}

    cps #MODE_FIQ
    mrs r2, spsr
    stm r0!, {r2, sp, lr}

    cps #MODE_SVC
    mrs r2, spsr
    stm r0!, {r2, sp, lr}

    cps #MODE_ABT
    mrs r2, spsr
    stm r0!, {r2, sp, lr}

    cps #MODE_UND
    mrs r2, spsr
    stm r0!, {r2, sp, lr}

    cps #MODE_MON
    bx  lr

restore_context:
    cps #MODE_SYS
    ldm r0!, {sp, lr}

    cps #MODE_IRQ
    ldm r0!, {r2, sp, lr}
    msr spsr_fsxc, r2

    cps #MODE_FIQ
    ldm r0!, {r2, sp, lr}
    msr spsr_fsxc, r2

    cps #MODE_SVC
    ldm r0!, {r2, sp, lr}
    msr spsr_fsxc, r2

    cps #MODE_ABT
    ldm r0!, {r2, sp, lr}
    msr spsr_fsxc, r2

    cps #MODE_UND
    ldm r0!, {r2, sp, lr}
    msr spsr_fsxc, r2

    cps #MODE_MON
    bx  lr

smc_handler:
	srsdb sp!, #22

	ldr r2, [sp] // lr
	ldr r3, [sp, #4] // cpsr
	
	clrex

	mrc p15, 0, r4, c1, c1, 0

	// Test ns bit
	tst r4, #(1<<0)
	bne from_nsec

from_sec:
	ldr r1, =sec_ctx
	stm r1!, {r2, r3}
	//stm r1!, {r4-r12}

	ldr r1, =nsec_ctx
	ldm r1!, {r2, r3}
	//ldm r1!, {r4-r12}
	mov r8, r2

	ldr r0, =sec_ctx
	add r0, r0, #100
	bl save_context

	ldr r0, =nsec_ctx
	add r0, r0, #100
	bl restore_context

	orr r4, r4, #(SCR_NS | SCR_FIQ)
	mcr p15, 0, r4, c1, c1, 0

	mov r2, r8
	cmp r2, #0
	beq first_time

	str r2, [sp] // lr
	str r3, [sp, #4] // cpsr
	
	//pop {r0-r3}
	rfefd sp!

first_time:
	ldr r2, =nsec_lr_addr
	ldr r2, [r2]
	ldr r3, =nsec_spsr_addr
	ldr r3, [r3]

	str r2, [sp] // lr
	str r3, [sp, #4] // cpsr
	
	//pop {r0-r3}
	rfefd sp!

from_nsec:
	ldr r1, =nsec_ctx
	stm r1!, {r2, r3}
	//stm r1!, {r4-r12}

	bic r4, r4, #(SCR_NS | SCR_FIQ)
	mcr p15, 0, r4, c1, c1, 0

	ldr r1, =sec_ctx
	ldm r1!, {r2, r3}
	//ldm r1!, {r4-r12}
	mov r8, r2

	ldr r0, =nsec_ctx
	add r0, r0, #100
	bl save_context

	ldr r0, =sec_ctx
	add r0, r0, #100
	bl restore_context

	mov r2, r8
	str r2, [sp] // lr
	str r3, [sp, #4] // cpsr
	
	//pop {r0-r3}
	rfefd sp!


