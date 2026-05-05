// Copyright (c) 2025 Syswonder
// hvisor is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//     http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR
// FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
//
// Syswonder Website:
//      https://www.syswonder.org
//
// Authors:
//

.macro	ventry	label
	.align	7
	b	\label
.endm

.macro handle_vmexit exit_reason
	.align	7
	/* Fill the union registers. Should comply with NUM_USR_REGS */
	/*now sp =>per_cpu stack top*/
	stp	x29, x30, [sp, #-16]!
	stp	x27, x28, [sp, #-16]!
	stp	x25, x26, [sp, #-16]!
	stp	x23, x24, [sp, #-16]!
	stp	x21, x22, [sp, #-16]!
	stp	x19, x20, [sp, #-16]!
	stp	x17, x18, [sp, #-16]!
	stp	x15, x16, [sp, #-16]!
	stp	x13, x14, [sp, #-16]!
	stp	x11, x12, [sp, #-16]!
	stp	x9, x10, [sp, #-16]!
	stp	x7, x8, [sp, #-16]!
	stp	x5, x6, [sp, #-16]!
	stp	x3, x4, [sp, #-16]!
	stp	x1, x2, [sp, #-16]!

	mov	x1, #\exit_reason
	stp	x1, x0, [sp, #-16]!

	mov	x29, xzr	/* reset fp,lr */
	mov	x30, xzr
	mov	x0, sp		/*x0 guest regs*/
	bl	{0}
	b	.
.endm
EXIT_REASON_EL2_ABORT	=0x0
EXIT_REASON_EL2_IRQ		=0x1
EXIT_REASON_EL1_ABORT	=0x2
EXIT_REASON_EL1_IRQ		=0x3
EXIT_REASON_EL1_AARCH32_ABORT =0x4
EXIT_REASON_EL1_AARCH32_IRQ	  =0x5

.global _hyp_trap_vector
	.align 11
_hyp_trap_vector:
	ventry	.
	ventry	.
	ventry	.
	ventry	.

	handle_vmexit EXIT_REASON_EL2_ABORT
	.align	7				// EL2h IRQ — lightweight path, NOT handle_vmexit
	b	_el2_irq_handler
	ventry	.
	ventry	.

	handle_vmexit EXIT_REASON_EL1_ABORT
	handle_vmexit EXIT_REASON_EL1_IRQ
	ventry	.
	ventry	.

	handle_vmexit EXIT_REASON_EL1_AARCH32_ABORT
	handle_vmexit EXIT_REASON_EL1_AARCH32_IRQ
	ventry	.
	ventry	.

// ============================================================
// EL2h IRQ lightweight handler
//
// Called when an IRQ fires while the CPU executes at EL2
// (idle loop, schedule(), etc.). Saves caller-saved registers,
// calls el2_irq_handler() (GIC processing), then eret back.
// Does NOT go through arch_handle_exit or vmreturn.
// ============================================================
.global _el2_irq_handler
_el2_irq_handler:
	sub	sp, sp, #0xa0
	stp	x0,  x1,  [sp, #0x00]
	stp	x2,  x3,  [sp, #0x10]
	stp	x4,  x5,  [sp, #0x20]
	stp	x6,  x7,  [sp, #0x30]
	stp	x8,  x9,  [sp, #0x40]
	stp	x10, x11, [sp, #0x50]
	stp	x12, x13, [sp, #0x60]
	stp	x14, x15, [sp, #0x70]
	stp	x16, x17, [sp, #0x80]
	stp	x18, x29, [sp, #0x90]
	str	x30, [sp, #-8]!

	bl	{1}			// el2_irq_handler()

	ldr	x30, [sp], #8
	ldp	x0,  x1,  [sp, #0x00]
	ldp	x2,  x3,  [sp, #0x10]
	ldp	x4,  x5,  [sp, #0x20]
	ldp	x6,  x7,  [sp, #0x30]
	ldp	x8,  x9,  [sp, #0x40]
	ldp	x10, x11, [sp, #0x50]
	ldp	x12, x13, [sp, #0x60]
	ldp	x14, x15, [sp, #0x70]
	ldp	x16, x17, [sp, #0x80]
	ldp	x18, x29, [sp, #0x90]
	add	sp, sp, #0xa0

	eret