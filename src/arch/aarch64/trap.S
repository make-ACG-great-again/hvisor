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

// handle_vmexit: save guest registers directly into the current vCPU's TrapFrame.
//
// SP is set to the TrapFrame address by vmreturn (mov sp, x0) before eret.
// So when a trap fires, SP already points to the TrapFrame base.
//
// TrapFrame layout (SP = base):
//   [sp+0x000] x0       (u64)
//   [sp+0x008] x1       (u64)
//   ...
//   [sp+0x0f0] x30      (u64)
//   [sp+0x0f8] elr_el2  (u64)
//   [sp+0x100] spsr_el2 (u64)
//
// Calling convention to arch_handle_exit:
//   x0 = sp  (pointer to TrapFrame = &mut TrapFrame)
//   x1 = exit_reason
.macro	handle_vmexit exit_reason
	.align	7
	stp	x0,  x1,  [sp]
	stp	x2,  x3,  [sp, #0x10]
	stp	x4,  x5,  [sp, #0x20]
	stp	x6,  x7,  [sp, #0x30]
	stp	x8,  x9,  [sp, #0x40]
	stp	x10, x11, [sp, #0x50]
	stp	x12, x13, [sp, #0x60]
	stp	x14, x15, [sp, #0x70]
	stp	x16, x17, [sp, #0x80]
	stp	x18, x19, [sp, #0x90]
	stp	x20, x21, [sp, #0xa0]
	stp	x22, x23, [sp, #0xb0]
	stp	x24, x25, [sp, #0xc0]
	stp	x26, x27, [sp, #0xd0]
	stp	x28, x29, [sp, #0xe0]
	str	x30,      [sp, #0xf0]
	mrs	x1, ELR_EL2
	str	x1,       [sp, #0xf8]
	mrs	x1, SPSR_EL2
	str	x1,       [sp, #0x100]
	mov	x0, sp
	mov	x1, #\exit_reason
	mov	x29, xzr
	mov	x30, xzr
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
	ventry	_el2_irq_handler	// EL2h IRQ: lightweight handler (EL2 code only)
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
// EL2h IRQ handler — only reached when IRQ interrupts EL2 code.
// EL1 IRQs are routed via handle_vmexit EXIT_REASON_EL1_IRQ above.
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

	bl	{1}			// el2_irq_handler() — handles GIC, no return value

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
