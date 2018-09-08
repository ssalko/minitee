#include <portmacro.h>

.balign 4
.code 32

.extern smc_handler
.extern __stack
.extern main
.extern sec_ctx
.extern nsec_ctx

.equ MODE_MASK,    0x1f
.equ MODE_USR, 0x10
.equ MODE_FIQ, 0x11
.equ MODE_IRQ, 0x12
.equ MODE_SVC, 0x13
.equ MODE_MON, 0x16
.equ MODE_ABT, 0x17
.equ MODE_UND, 0x1b
.equ MODE_SYS, 0x1f

.global nsec_spsr_addr
.global nsec_lr_addr

.global vectors_start

.section .isr_vector, "x"

vectors_start:
	b reset_handler
	b .
	b swi_handler
	b .
	b .
	nop
	b irq_handler
	b fiq_handler

monitor_vectors:
	b .
	b .
	b smc_handler
	b .
	b .
	nop 
	b irq_handler
	b fiq_handler

reset_handler:
	ldr r0, =nsec_lr_addr
	str lr, [r0]

	ldr r0, =nsec_spsr_addr
	mrs r4, cpsr
	str r4, [r0]

	ldr r0, =__bss_start
	ldr r1, =__bss_end
bss_clear:
	mov r2, #0
	str r2, [r0]
	add r0, r0, #4
	cmp r0, r1
	bne bss_clear

	cps #MODE_FIQ
	nop
	ldr sp, =fiq_sp_ptr
	nop
	cps #MODE_IRQ
	nop
	ldr sp, =irq_sp_ptr
	nop
	cps #MODE_MON
	nop
	ldr sp, =mon_sp_ptr
	nop
	cps #MODE_ABT
	nop
	ldr sp, =abt_sp_ptr
	nop
	cps #MODE_UND
	nop
	ldr sp, =abt_sp_ptr
	nop
	cps #MODE_SVC
	nop
	ldr sp, =svc_sp_ptr
	nop

	ldr r0, =vectors_start
	mcr p15, 0, r0, c12, c0, 0

	mrc p15, 0, r0, c1, c0, 0
	bic r0, r0, #(1 << 13)
	mcr p15, 0, r0, c1, c0, 0

	ldr r1, =monitor_vectors
	cps #MODE_MON
	nop
	mcr p15, 0, r1, c12, c0, 1
	cps #MODE_SVC
	
	b main
	b .

swi_handler:
	b vPortSVCHandler

irq_handler:
	b .

fiq_handler:
	smc #0
	b .

.section .data

nsec_lr_addr:
	.word nsec_lr
nsec_spsr_addr:
	.word nsec_spsr

nsec_lr:
	.word
nsec_spsr:
	.word

.end

