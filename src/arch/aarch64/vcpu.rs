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

//! AArch64 vCPU architecture state: per-vCPU TrapFrame + EL1 system registers + GIC state.
//!
//! Each VCpu has an `ArchVCpu` that holds:
//! - `stack`: a private 128 KiB stack with a `TrapFrame` at the top.
//!   trap.S saves guest x0..x30 + ELR_EL2 + SPSR_EL2 to pCPU stack on each exit;
//!   arch_handle_exit() copies them into the vCPU's TrapFrame immediately.
//!   vmreturn(trapframe_ptr) restores from TrapFrame and erets.
//! - `el1_regs`: all EL1 system registers (SCTLR, TTBR, TCR, timer regs, etc.)
//! - `gic_state`: GIC virtualization registers (ICH_LR*, ICH_VMCR, etc.)

use crate::arch::sysreg::{read_sysreg, write_sysreg};
use alloc::boxed::Box;
use alloc::sync::Arc;
use core::cell::UnsafeCell;
use crate::consts::VCPU_STACK_SIZE;

// ========================
// EL1 System Registers
// ========================

/// Saved EL1 system register state for a VCpu.
/// These must be saved/restored on every VCpu context switch.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct El1SysRegs {
    pub sctlr_el1: u64,
    pub ttbr0_el1: u64,
    pub ttbr1_el1: u64,
    pub tcr_el1: u64,
    pub esr_el1: u64,
    pub far_el1: u64,
    pub mair_el1: u64,
    pub amair_el1: u64,
    pub vbar_el1: u64,
    pub contextidr_el1: u64,
    pub cpacr_el1: u64,
    pub csselr_el1: u64,
    pub sp_el0: u64,
    pub sp_el1: u64,
    pub spsr_el1: u64,
    pub elr_el1: u64,
    pub afsr0_el1: u64,
    pub afsr1_el1: u64,
    pub par_el1: u64,
    pub tpidr_el0: u64,
    pub tpidr_el1: u64,
    pub tpidrro_el0: u64,
    // Timer registers (trapped via EL2)
    pub cntvoff_el2: u64,
    pub cntp_ctl_el0: u64,
    pub cntp_cval_el0: u64,
    pub cntv_ctl_el0: u64,
    pub cntv_cval_el0: u64,
    pub cntkctl_el1: u64,
}

impl Default for El1SysRegs {
    fn default() -> Self {
        Self::reset()
    }
}

impl El1SysRegs {
    /// Initialize with ARMv8 reset values.
    pub fn reset() -> Self {
        Self {
            // SCTLR_EL1 reset: bits 11, 20, 22-23, 28-29 set (EOS, TSCXT, EIS, LSMAOE, nTLSMD)
            sctlr_el1: (1 << 11) | (1 << 20) | (3 << 22) | (3 << 28),
            ttbr0_el1: 0,
            ttbr1_el1: 0,
            tcr_el1: 0,
            esr_el1: 0,
            far_el1: 0,
            mair_el1: 0,
            amair_el1: 0,
            vbar_el1: 0,
            contextidr_el1: 0,
            cpacr_el1: 0,
            csselr_el1: 0,
            sp_el0: 0,
            sp_el1: 0,
            spsr_el1: 0,
            elr_el1: 0,
            afsr0_el1: 0,
            afsr1_el1: 0,
            par_el1: 0,
            tpidr_el0: 0,
            tpidr_el1: 0,
            tpidrro_el0: 0,
            cntvoff_el2: 0,
            cntp_ctl_el0: 0,
            cntp_cval_el0: 0,
            cntv_ctl_el0: 0,
            cntv_cval_el0: 0,
            cntkctl_el1: 0,
        }
    }

    /// Save all EL1 system registers from hardware into this struct.
    pub fn save_from_hardware(&mut self) {
        self.sctlr_el1      = read_sysreg!(SCTLR_EL1);
        self.ttbr0_el1      = read_sysreg!(TTBR0_EL1);
        self.ttbr1_el1      = read_sysreg!(TTBR1_EL1);
        self.tcr_el1        = read_sysreg!(TCR_EL1);
        self.esr_el1        = read_sysreg!(ESR_EL1);
        self.far_el1        = read_sysreg!(FAR_EL1);
        self.mair_el1       = read_sysreg!(MAIR_EL1);
        self.amair_el1      = read_sysreg!(AMAIR_EL1);
        self.vbar_el1       = read_sysreg!(VBAR_EL1);
        self.contextidr_el1 = read_sysreg!(CONTEXTIDR_EL1);
        self.cpacr_el1      = read_sysreg!(CPACR_EL1);
        self.csselr_el1     = read_sysreg!(CSSELR_EL1);
        self.sp_el0         = read_sysreg!(SP_EL0);
        self.sp_el1         = read_sysreg!(SP_EL1);
        self.spsr_el1       = read_sysreg!(SPSR_EL1);
        self.elr_el1        = read_sysreg!(ELR_EL1);
        self.afsr0_el1      = read_sysreg!(AFSR0_EL1);
        self.afsr1_el1      = read_sysreg!(AFSR1_EL1);
        self.par_el1        = read_sysreg!(PAR_EL1);
        self.tpidr_el0      = read_sysreg!(TPIDR_EL0);
        self.tpidr_el1      = read_sysreg!(TPIDR_EL1);
        self.tpidrro_el0    = read_sysreg!(TPIDRRO_EL0);
        self.cntvoff_el2    = read_sysreg!(CNTVOFF_EL2);
        self.cntp_ctl_el0   = read_sysreg!(CNTP_CTL_EL0);
        self.cntp_cval_el0  = read_sysreg!(CNTP_CVAL_EL0);
        self.cntv_cval_el0  = read_sysreg!(CNTV_CVAL_EL0);
        // Save CNTV_CTL with IMASK policy:
        // - ISTATUS=1 (timer expired): set IMASK=1 to suppress the physical IRQ 27 signal
        //   while this vCPU is off-CPU, preventing it from interfering with other vCPUs.
        // - ISTATUS=0 (timer handled or not yet expired): clear IMASK=0 so the guest's
        //   intended timer delivery mode is preserved.  IMASK may have been set by the
        //   hypervisor (vcpu_vmreturn / truly_alone path) as a temporary measure; once
        //   ISTATUS=0 the guest has handled the interrupt, so we restore IMASK=0.
        let cntv_ctl = read_sysreg!(CNTV_CTL_EL0);
        let timer_enabled = (cntv_ctl & 1) != 0;
        if timer_enabled {
            // Always mask hardware CNTV delivery while this vCPU is off-CPU.
            // Even if ISTATUS=0 now, the counter keeps running and the timer may
            // expire before this vCPU is scheduled back in, which would fire a
            // spurious physical IRQ 27 with no vCPU to receive it.
            // Timer expiry is checked by check_blocked_timers() via CNTPCT comparison.
            let masked = cntv_ctl | 2; // set IMASK
            write_sysreg!(CNTV_CTL_EL0, masked);
            self.cntv_ctl_el0 = masked;
        } else {
            self.cntv_ctl_el0 = cntv_ctl;
        }
        self.cntkctl_el1    = read_sysreg!(CNTKCTL_EL1);
    }

    /// Restore all EL1 system registers from this struct to hardware.
    pub fn restore_to_hardware(&self) {
        write_sysreg!(SCTLR_EL1,      self.sctlr_el1);
        write_sysreg!(TTBR0_EL1,      self.ttbr0_el1);
        write_sysreg!(TTBR1_EL1,      self.ttbr1_el1);
        write_sysreg!(TCR_EL1,        self.tcr_el1);
        write_sysreg!(ESR_EL1,        self.esr_el1);
        write_sysreg!(FAR_EL1,        self.far_el1);
        write_sysreg!(MAIR_EL1,       self.mair_el1);
        write_sysreg!(AMAIR_EL1,      self.amair_el1);
        write_sysreg!(VBAR_EL1,       self.vbar_el1);
        write_sysreg!(CONTEXTIDR_EL1, self.contextidr_el1);
        write_sysreg!(CPACR_EL1,      self.cpacr_el1);
        write_sysreg!(CSSELR_EL1,     self.csselr_el1);
        write_sysreg!(SP_EL0,         self.sp_el0);
        write_sysreg!(SP_EL1,         self.sp_el1);
        write_sysreg!(SPSR_EL1,       self.spsr_el1);
        write_sysreg!(ELR_EL1,        self.elr_el1);
        write_sysreg!(AFSR0_EL1,      self.afsr0_el1);
        write_sysreg!(AFSR1_EL1,      self.afsr1_el1);
        write_sysreg!(PAR_EL1,        self.par_el1);
        write_sysreg!(TPIDR_EL0,      self.tpidr_el0);
        write_sysreg!(TPIDR_EL1,      self.tpidr_el1);
        write_sysreg!(TPIDRRO_EL0,    self.tpidrro_el0);
        write_sysreg!(CNTVOFF_EL2,    self.cntvoff_el2);
        write_sysreg!(CNTP_CTL_EL0,   self.cntp_ctl_el0);
        write_sysreg!(CNTP_CVAL_EL0,  self.cntp_cval_el0);
        write_sysreg!(CNTV_CVAL_EL0,  self.cntv_cval_el0);
        // Restore virtual timer control.
        // If already expired (ISTATUS=1): mask hardware delivery (IMASK=1) to prevent
        // an immediate physical IRQ 27 on restore. sched_tick_handler Step 3 or
        // check_blocked_timers will inject IRQ 27 via the software pending path.
        // If ISTATUS=0: restore with IMASK=0 (save_from_hardware already cleared it)
        // so the guest's normal hardware timer delivery path works correctly.
        let cntv_ctl = self.cntv_ctl_el0;
        let timer_enabled = (cntv_ctl & 1) != 0;
        let timer_expired = (cntv_ctl & 4) != 0; // ISTATUS
        if timer_enabled && timer_expired {
            write_sysreg!(CNTV_CTL_EL0, cntv_ctl | 2); // set IMASK
        } else {
            write_sysreg!(CNTV_CTL_EL0, cntv_ctl & !2u64); // ensure IMASK=0
        }
        write_sysreg!(CNTKCTL_EL1, self.cntkctl_el1);
        write_sysreg!(PMCR_EL0, 0);
    }
}

// ========================
// GIC Virtualization State
// ========================

/// Maximum number of GIC List Registers.
pub const MAX_GIC_LRS: usize = 16;

/// Per-VCpu virtual GICR (Redistributor) shadow state for SGI/PPI registers.
/// In overcommit mode, multiple VCPUs share one physical GICR.
/// This struct holds the software shadow so each VCpu has its own SGI/PPI config.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct VirtualGicrState {
    pub isenabler: u32,
    pub ipriorityr: [u32; 8],
    pub icfgr: [u32; 2],
    pub ispendr: u32,
    pub isactiver: u32,
    pub igroupr: u32,
    pub initialized: bool,
}

impl Default for VirtualGicrState {
    fn default() -> Self {
        Self {
            isenabler: 0,
            ipriorityr: [0; 8],
            icfgr: [0; 2],
            ispendr: 0,
            isactiver: 0,
            igroupr: 0,
            initialized: false,
        }
    }
}

/// Saved GIC virtualization register state for a VCpu.
#[repr(C)]
#[derive(Debug)]
pub struct GicState {
    pub ich_lr: [u64; MAX_GIC_LRS],
    pub ich_vmcr: u64,
    pub ich_hcr: u64,
    pub ich_ap1r: [u64; 4],
    /// Number of LRs available on this hardware (detected from ICH_VTR_EL2).
    pub lr_count: usize,
    /// Number of preemption priority bits (from ICH_VTR_EL2.PREbits+1).
    pub pri_bits: usize,
    /// Per-VCpu virtual GICR SGI/PPI state (shadow of physical GICR).
    pub vgicr: UnsafeCell<VirtualGicrState>,
}

// Safety: VirtualGicrState is only mutated when the VCpu is NOT running.
unsafe impl Send for GicState {}
unsafe impl Sync for GicState {}

impl Clone for GicState {
    fn clone(&self) -> Self {
        Self {
            ich_lr:   self.ich_lr,
            ich_vmcr: self.ich_vmcr,
            ich_hcr:  self.ich_hcr,
            ich_ap1r: self.ich_ap1r,
            lr_count: self.lr_count,
            pri_bits: self.pri_bits,
            vgicr:    UnsafeCell::new(unsafe { (*self.vgicr.get()).clone() }),
        }
    }
}

impl Default for GicState {
    fn default() -> Self {
        Self {
            ich_lr:   [0; MAX_GIC_LRS],
            ich_vmcr: 0,
            ich_hcr:  0,
            ich_ap1r: [0; 4],
            lr_count: 0,
            pri_bits: 5,
            vgicr:    UnsafeCell::new(VirtualGicrState::default()),
        }
    }
}

impl GicState {
    /// Create a new GicState with lr_count/pri_bits detected from hardware.
    /// ICH_HCR_EL2.En=1 so the virtual GIC interface is active on first schedule-in.
    pub fn new_with_lr_count(lr_count: usize) -> Self {
        // ICH_VMCR_EL2: VPMR=0xff (allow all priorities), VENG1=1 (Group1 enabled)
        let ich_vmcr_default: u64 = (0xff_u64 << 24) | (1 << 1);
        let pri_bits = detect_gic_pri_bits();
        Self {
            lr_count,
            pri_bits,
            ich_hcr: 1, // En=1
            ich_vmcr: ich_vmcr_default,
            ..Default::default()
        }
    }

    /// Save GIC virtualization registers from hardware.
    pub fn save_from_hardware(&mut self) {
        self.ich_vmcr    = read_sysreg!(ICH_VMCR_EL2);
        self.ich_hcr     = read_sysreg!(ICH_HCR_EL2);
        self.ich_ap1r[0] = read_sysreg!(ICH_AP1R0_EL2);
        if self.pri_bits >= 6 { self.ich_ap1r[1] = read_sysreg!(ICH_AP1R1_EL2); }
        if self.pri_bits >= 7 {
            self.ich_ap1r[2] = read_sysreg!(ICH_AP1R2_EL2);
            self.ich_ap1r[3] = read_sysreg!(ICH_AP1R3_EL2);
        }
        if self.lr_count > 0  { self.ich_lr[0]  = read_sysreg!(ICH_LR0_EL2);  }
        if self.lr_count > 1  { self.ich_lr[1]  = read_sysreg!(ICH_LR1_EL2);  }
        if self.lr_count > 2  { self.ich_lr[2]  = read_sysreg!(ICH_LR2_EL2);  }
        if self.lr_count > 3  { self.ich_lr[3]  = read_sysreg!(ICH_LR3_EL2);  }
        if self.lr_count > 4  { self.ich_lr[4]  = read_sysreg!(ICH_LR4_EL2);  }
        if self.lr_count > 5  { self.ich_lr[5]  = read_sysreg!(ICH_LR5_EL2);  }
        if self.lr_count > 6  { self.ich_lr[6]  = read_sysreg!(ICH_LR6_EL2);  }
        if self.lr_count > 7  { self.ich_lr[7]  = read_sysreg!(ICH_LR7_EL2);  }
        if self.lr_count > 8  { self.ich_lr[8]  = read_sysreg!(ICH_LR8_EL2);  }
        if self.lr_count > 9  { self.ich_lr[9]  = read_sysreg!(ICH_LR9_EL2);  }
        if self.lr_count > 10 { self.ich_lr[10] = read_sysreg!(ICH_LR10_EL2); }
        if self.lr_count > 11 { self.ich_lr[11] = read_sysreg!(ICH_LR11_EL2); }
        if self.lr_count > 12 { self.ich_lr[12] = read_sysreg!(ICH_LR12_EL2); }
        if self.lr_count > 13 { self.ich_lr[13] = read_sysreg!(ICH_LR13_EL2); }
        if self.lr_count > 14 { self.ich_lr[14] = read_sysreg!(ICH_LR14_EL2); }
        if self.lr_count > 15 { self.ich_lr[15] = read_sysreg!(ICH_LR15_EL2); }
    }

    /// Restore GIC virtualization registers to hardware.
    pub fn restore_to_hardware(&self) {
        write_sysreg!(ICH_VMCR_EL2,   self.ich_vmcr);
        write_sysreg!(ICH_HCR_EL2,    self.ich_hcr);
        write_sysreg!(ICH_AP1R0_EL2,  self.ich_ap1r[0]);
        if self.pri_bits >= 6 { write_sysreg!(ICH_AP1R1_EL2, self.ich_ap1r[1]); }
        if self.pri_bits >= 7 {
            write_sysreg!(ICH_AP1R2_EL2, self.ich_ap1r[2]);
            write_sysreg!(ICH_AP1R3_EL2, self.ich_ap1r[3]);
        }
        if self.lr_count > 0  { write_sysreg!(ICH_LR0_EL2,  self.ich_lr[0]);  }
        if self.lr_count > 1  { write_sysreg!(ICH_LR1_EL2,  self.ich_lr[1]);  }
        if self.lr_count > 2  { write_sysreg!(ICH_LR2_EL2,  self.ich_lr[2]);  }
        if self.lr_count > 3  { write_sysreg!(ICH_LR3_EL2,  self.ich_lr[3]);  }
        if self.lr_count > 4  { write_sysreg!(ICH_LR4_EL2,  self.ich_lr[4]);  }
        if self.lr_count > 5  { write_sysreg!(ICH_LR5_EL2,  self.ich_lr[5]);  }
        if self.lr_count > 6  { write_sysreg!(ICH_LR6_EL2,  self.ich_lr[6]);  }
        if self.lr_count > 7  { write_sysreg!(ICH_LR7_EL2,  self.ich_lr[7]);  }
        if self.lr_count > 8  { write_sysreg!(ICH_LR8_EL2,  self.ich_lr[8]);  }
        if self.lr_count > 9  { write_sysreg!(ICH_LR9_EL2,  self.ich_lr[9]);  }
        if self.lr_count > 10 { write_sysreg!(ICH_LR10_EL2, self.ich_lr[10]); }
        if self.lr_count > 11 { write_sysreg!(ICH_LR11_EL2, self.ich_lr[11]); }
        if self.lr_count > 12 { write_sysreg!(ICH_LR12_EL2, self.ich_lr[12]); }
        if self.lr_count > 13 { write_sysreg!(ICH_LR13_EL2, self.ich_lr[13]); }
        if self.lr_count > 14 { write_sysreg!(ICH_LR14_EL2, self.ich_lr[14]); }
        if self.lr_count > 15 { write_sysreg!(ICH_LR15_EL2, self.ich_lr[15]); }
    }
}

/// Detect number of GIC List Registers from ICH_VTR_EL2.
pub fn detect_gic_lr_count() -> usize {
    let vtr = read_sysreg!(ICH_VTR_EL2);
    // ICH_VTR_EL2[4:0] = ListRegs - 1
    ((vtr & 0x1f) + 1) as usize
}

/// Detect number of priority bits from ICH_VTR_EL2.PREbits[28:26].
pub fn detect_gic_pri_bits() -> usize {
    let vtr = read_sysreg!(ICH_VTR_EL2);
    (((vtr >> 26) & 0x7) + 1) as usize
}

// ========================
// ArchVCpu
// ========================

// ========================
// Per-vCPU stack
// ========================

#[repr(C, align(4096))]
struct VCpuStack {
    _st: [u8; VCPU_STACK_SIZE],
}

impl VCpuStack {
    fn new_boxed() -> Box<Self> {
        unsafe {
            let layout = alloc::alloc::Layout::new::<Self>();
            let ptr = alloc::alloc::alloc_zeroed(layout) as *mut Self;
            if ptr.is_null() {
                alloc::alloc::handle_alloc_error(layout);
            }
            Box::from_raw(ptr)
        }
    }

    fn upper_bound(&self) -> *const u8 {
        unsafe { (self._st.as_ptr()).add(VCPU_STACK_SIZE) }
    }
}

// ========================
// TrapFrame
// ========================

/// Per-vCPU guest register frame, lives at the top of the vCPU's private stack.
///
/// Layout (matches trap.S save/restore order):
///   offset 0x000: x[0..30]  — x0..x30  (31 × u64)
///   offset 0x0f8: elr        — ELR_EL2  (guest PC)
///   offset 0x100: spsr       — SPSR_EL2 (guest PSTATE)
///   offset 0x108: _pad       — padding to reach 272 bytes (16-byte aligned)
/// Total: 34 × 8 = 272 bytes. SP must be 16-byte aligned on AArch64.
#[repr(C)]
#[derive(Default)]
pub struct TrapFrame {
    pub x: [u64; 31],   // x0..x30
    pub elr: u64,       // ELR_EL2: guest PC at trap
    pub spsr: u64,      // SPSR_EL2: guest PSTATE at trap
    _pad: u64,          // padding: sizeof(TrapFrame) = 272 = 16×17 (SP alignment)
}

// ========================
// ArchVCpu
// ========================

/// Per-VCpu architecture state for AArch64.
///
/// Each vCPU has its own private stack (`stack`) with a `TrapFrame` at the top.
/// `vmreturn(trapframe_ptr)` restores x0..x30, ELR_EL2, SPSR_EL2 and erets to guest.
/// Context switches operate directly on the trapframe — no memcpy to/from pCPU stack.
#[repr(C)]
pub struct ArchVCpu {
    /// Private stack for this vCPU. TrapFrame lives at stack top.
    stack: Box<VCpuStack>,
    /// Saved EL1 system registers (SCTLR, TTBR, TCR, timer, etc.) + ELR/SPSR.
    pub el1_regs: El1SysRegs,
    /// Saved GIC virtualization state (ICH_LR*, ICH_VMCR, vGICR shadow).
    pub gic_state: GicState,
}

impl ArchVCpu {
    pub fn new() -> Self {
        let lr_count = safe_detect_gic_lr_count();
        Self {
            stack:     VCpuStack::new_boxed(),
            el1_regs:  El1SysRegs::reset(),
            gic_state: GicState::new_with_lr_count(lr_count),
        }
    }

    /// Returns a mutable reference to the TrapFrame at the top of this vCPU's stack.
    pub fn trapframe(&self) -> &mut TrapFrame {
        unsafe {
            let ptr = self.stack.upper_bound() as usize - core::mem::size_of::<TrapFrame>();
            &mut *(ptr as *mut TrapFrame)
        }
    }

    /// Returns the raw pointer to the TrapFrame, suitable for passing to `vmreturn`.
    pub fn trapframe_ptr(&self) -> usize {
        self.stack.upper_bound() as usize - core::mem::size_of::<TrapFrame>()
    }

    /// Reset EL1 regs to ARMv8 architectural reset values (for PSCI CPU_ON hotplug).
    /// Only call when the VCpu is in Stopped state — not thread-safe.
    pub fn reset_el1_regs(&self) {
        unsafe {
            core::ptr::write(
                core::ptr::addr_of!(self.el1_regs) as *mut El1SysRegs,
                El1SysRegs::reset(),
            );
        }
    }

    /// Reset GIC state to initial values (for PSCI CPU_ON hotplug).
    pub fn reset_gic_state(&self) {
        unsafe {
            core::ptr::write(
                core::ptr::addr_of!(self.gic_state) as *mut GicState,
                GicState::new_with_lr_count(safe_detect_gic_lr_count()),
            );
        }
    }
}

/// Safely detect GIC LR count. Falls back to 4 if hardware is not ready.
fn safe_detect_gic_lr_count() -> usize {
    detect_gic_lr_count()
}

pub type ArchVCpuType = ArchVCpu;

// ========================
// arch_wakeup_vcpu
// ========================

/// Transition a vCPU from Stopped → Ready and deliver it to its affinity pCPU.
/// Used by PSCI CPU_ON emulation.
pub fn arch_wakeup_vcpu(vcpu: Arc<crate::vcpu::VCpu>) -> isize {
    use crate::vcpu::VCpuState;

    if vcpu.transition(VCpuState::Stopped, VCpuState::Ready).is_err() {
        error!(
            "PSCI: vcpu {} is not in Stopped state (current: {:?})",
            vcpu.id,
            vcpu.state()
        );
        return -4; // PSCI_ALREADY_ON
    }

    let from_pcpu = crate::cpu_data::this_cpu_data().id;
    let target_pcpu = vcpu.get_pcpu_affinity();
    info!(
        "PSCI: wakeup vcpu {} -> pCPU {} (from pCPU {})",
        vcpu.id, target_pcpu, from_pcpu
    );

    if target_pcpu != from_pcpu {
        crate::vcpu::deliver_vcpu_to_pcpu(target_pcpu, vcpu);
    } else {
        crate::vcpu::enqueue_vcpu_on_affinity_pcpu(vcpu);
    }

    0
}

// ========================
// GICR shadow write-back (called from vcpu_switch_in in scheduler.rs)
// ========================

/// Write the saved vGICR shadow registers back to the physical GICR.
/// Restores per-VCpu SGI/PPI configuration (IGROUPR0, ISENABLER0, IPRIORITYR, ICFGR1).
pub fn restore_vgicr(vcpu: &crate::vcpu::VCpu) {
    use crate::device::irqchip::gicv3::{gicr, host_gicr_base};

    let pcpu_id = vcpu.get_pcpu_affinity();
    let sgi_base = host_gicr_base(pcpu_id) + gicr::GICR_SGI_BASE;
    let vgicr = unsafe { &*vcpu.arch.gic_state.vgicr.get() };
    // Always ensure IRQ 26 (CNTHP, EL2 physical timer) is enabled on this
    // pCPU's GICR regardless of whether the vCPU has initialized its shadow.
    unsafe {
        let isenabler_ptr = (sgi_base + gicr::GICR_ISENABLER) as *mut u32;
        isenabler_ptr.write_volatile(1u32 << 26);
        let ipriorityr26 = (sgi_base + gicr::GICR_IPRIORITYR + 26) as *mut u8;
        ipriorityr26.write_volatile(0xa0);
    }

    if !vgicr.initialized {
        return;
    }
    unsafe {
        use crate::hypercall::SGI_IPI_ID;
        use crate::device::irqchip::gicv3::MAINTENACE_INTERRUPT;

        // Restore IGROUPR (guest-controlled; IRQ 26 group is managed by Secure firmware).
        let igroupr = (sgi_base + gicr::GICR_IGROUPR) as *mut u32;
        igroupr.write_volatile(vgicr.igroupr);

        // Clear all guest-controlled PPI enables first, preserving hypervisor IRQs.
        // IRQ 26 (EL2 physical timer), IRQ 25 (maintenance), SGI_IPI_ID are preserved.
        let hv_mask: u32 = (1u32 << 26) | (1u32 << MAINTENACE_INTERRUPT) | (1u32 << SGI_IPI_ID);
        let icenabler = (sgi_base + gicr::GICR_ICENABLER) as *mut u32;
        icenabler.write_volatile(!hv_mask);

        // Restore guest enables, force IRQ 26 set.
        let isenabler = (sgi_base + gicr::GICR_ISENABLER) as *mut u32;
        isenabler.write_volatile(vgicr.isenabler | (1 << 26));

        for i in 0..8 {
            let reg = (sgi_base + gicr::GICR_IPRIORITYR + i * 4) as *mut u32;
            reg.write_volatile(vgicr.ipriorityr[i]);
        }
        // Force IRQ 26 priority to 0xa0 after full restore.
        let ipriorityr26 = (sgi_base + gicr::GICR_IPRIORITYR + 26) as *mut u8;
        ipriorityr26.write_volatile(0xa0);

        // ICFGR0 (SGIs) is read-only, only write ICFGR1 (PPIs)
        let icfgr1 = (sgi_base + gicr::GICR_ICFGR + 4) as *mut u32;
        icfgr1.write_volatile(vgicr.icfgr[1]);
    }
}
