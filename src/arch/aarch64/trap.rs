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
use aarch64_cpu::registers::*;
use core::arch::global_asm;

use crate::arch::vcpu::TrapFrame;
use crate::arch::sysreg::smc_call;
use crate::zone::zone_error;
use crate::{
    arch::{
        cpu::mpidr_to_cpuid,
        sysreg::{read_sysreg, write_sysreg},
    },
    cpu_data::{get_cpu_data, this_cpu_data, this_zone},
    device::irqchip::gic_handle_irq,
    event::{send_event, IPI_EVENT_SHUTDOWN, IPI_EVENT_WAKEUP},
    hypercall::{HyperCall, SGI_IPI_ID},
    memory::{mmio_handle_access, MMIOAccess},
    vcpu::VCpuState,
    zone::{is_this_root_zone, remove_zone, GuestMpidr},
};

global_asm!(
    include_str!("./trap.S"),
    sym arch_handle_exit,
    sym el2_irq_handler
);

/// Lightweight EL2 IRQ handler called from _el2_irq_handler in trap.S.
///
/// Invoked when an IRQ arrives while the CPU is at EL2 (idle loop or first entry).
/// Processes the interrupt via GIC and returns — trap.S then eret back to EL2 code.
/// Must NOT call vmreturn or schedule.
#[no_mangle]
#[cfg(feature = "vcpu_debug_trace")]
pub static EL2_IRQ_COUNTER: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "vcpu_debug_trace")]
pub static EL1_IRQ_COUNTER: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Called from _el2_irq_handler in trap.S (EL2 WFI / EL2 context only).
///
/// This handler is reached only when the IRQ interrupted EL2 code (SPSR_EL2.M = EL2h).
/// When an IRQ interrupts EL1 (guest), trap.S routes it directly to arch_handle_exit
/// as EXIT_REASON_EL1_IRQ, so schedule() runs there in the normal vmexit path.
#[no_mangle]
extern "C" fn el2_irq_handler() {
    #[cfg(feature = "vcpu_debug_trace")]
    {
        use core::sync::atomic::Ordering;
        let n = EL2_IRQ_COUNTER.fetch_add(1, Ordering::Relaxed);
        if n % 10000 == 0 {
            use crate::arch::sysreg::read_sysreg;
            let elr = read_sysreg!(ELR_EL2);
            let spsr = read_sysreg!(SPSR_EL2);
            let sp: u64;
            unsafe { core::arch::asm!("mov {}, sp", out(reg) sp, options(nostack, preserves_flags)) };
            let cur = crate::cpu_data::this_cpu_data().scheduler.current.as_ref().map(|v| v.id);
            let sched_calls = crate::scheduler::SCHEDULE_CALL_COUNTER.load(Ordering::Relaxed);
            info!("[EL2-IRQ] #{} elr={:#x} spsr={:#x} sp={:#x} sched_cur={:?} sched_calls={}", n, elr, spsr, sp, cur, sched_calls);
        }
    }
    crate::device::irqchip::gic_handle_irq();
}

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]
pub mod ExceptionType {
    pub const EXIT_REASON_EL2_ABORT: u64 = 0x0;
    pub const EXIT_REASON_EL2_IRQ: u64 = 0x1;
    pub const EXIT_REASON_EL1_ABORT: u64 = 0x2;
    pub const EXIT_REASON_EL1_IRQ: u64 = 0x3;
    pub const EXIT_REASON_EL1_AARCH32_ABORT: u64 = 0x4;
    pub const EXIT_REASON_EL1_AARCH32_IRQ: u64 = 0x5;
}
const SMC_TYPE_MASK: u64 = 0x3F000000;
#[allow(non_snake_case)]
pub mod SmcType {
    pub const ARCH_SC: u64 = 0x0;
    pub const SIP_SC: u64 = 0x02000000;
    pub const STANDARD_SC: u64 = 0x04000000;
    pub const TOS_SC_START: u64 = 0x32000000;
    pub const TOS_SC_END: u64 = 0x3F000000;
}

const PSCI_VERSION_1_1: u64 = 0x10001;
const PSCI_TOS_NOT_PRESENT_MP: u64 = 2;
const ARM_SMCCC_VERSION_1_1: u64 = 0x10001;

#[allow(unused)]
const ARM_SMCCC_NOT_SUPPORTED: i64 = -1;

extern "C" {
    fn _hyp_trap_vector();
}

pub fn install_trap_vector() {
    // Set the trap vector.
    VBAR_EL2.set(_hyp_trap_vector as _);
}

// ----------------------------------------------

#[allow(non_snake_case)]
pub mod PsciFnId {
    pub const PSCI_VERSION: u64 = 0x84000000;
    pub const PSCI_CPU_SUSPEND_32: u64 = 0x84000001;
    pub const PSCI_CPU_OFF_32: u64 = 0x84000002;
    pub const PSCI_CPU_ON_32: u64 = 0x84000003;
    pub const PSCI_AFFINITY_INFO_32: u64 = 0x84000004;
    pub const PSCI_MIG_INFO_TYPE: u64 = 0x84000006;
    pub const PSCI_SYSTEM_OFF: u64 = 0x84000008;
    pub const PSCI_FEATURES: u64 = 0x8400000a;

    pub const PSCI_CPU_SUSPEND_64: u64 = 0xc4000001;
    pub const PSCI_CPU_OFF_64: u64 = 0xc4000002;
    pub const PSCI_CPU_ON_64: u64 = 0xc4000003;
    pub const PSCI_AFFINITY_INFO_64: u64 = 0xc4000004;
}
#[allow(non_snake_case)]
pub mod SMCccFnId {
    pub const SMCCC_VERSION: u64 = 0x80000000;
    pub const SMCCC_ARCH_FEATURES: u64 = 0x80000001;
}

#[allow(unused)]
pub enum TrapReturn {
    TrapHandled = 1,
    TrapUnhandled = 0,
    TrapForbidden = -1,
}

/// EL2 trap entry point called from trap.S handle_vmexit.
///
/// trap.S writes guest registers directly into the vCPU TrapFrame (SP = TrapFrame base),
/// then calls us with x0=&TrapFrame, x1=exit_reason.
/// We dispatch the exit, then vmreturn from TrapFrame (which also resets SP=TrapFrame).
pub fn arch_handle_exit(regs: &mut TrapFrame, exit_reason: u64) -> ! {
    let mpidr = MPIDR_EL1.get();
    let _cpu_id = mpidr_to_cpuid(mpidr);
    trace!("cpu exit, exit_reason:{:#x?}", exit_reason);

    match exit_reason {
        ExceptionType::EXIT_REASON_EL1_IRQ | ExceptionType::EXIT_REASON_EL1_AARCH32_IRQ => {
            irqchip_handle_irq1();
            {
                #[cfg(feature = "vcpu_debug_trace")]
                {
                    use core::sync::atomic::{AtomicU64, Ordering};
                    static AHE_GIC_RET: AtomicU64 = AtomicU64::new(0);
                    let n = AHE_GIC_RET.fetch_add(1, Ordering::Relaxed);
                    if n % 10000 == 0 {
                        info!("[AHE-GIC-RET] #{} gic returned ok", n);
                    }
                }
            }
        }
        ExceptionType::EXIT_REASON_EL1_ABORT | ExceptionType::EXIT_REASON_EL1_AARCH32_ABORT => {
            arch_handle_trap_el1(regs)
        }
        ExceptionType::EXIT_REASON_EL2_ABORT => {
            let cpu = this_cpu_data();
            println!(
                "EL2 ABORT on pcpu={} current_vcpu={:?} ELR={:#x} ESR={:#x}",
                cpu.id,
                cpu.current_vcpu.as_ref().map(|v| v.id),
                read_sysreg!(ELR_EL2),
                read_sysreg!(ESR_EL2),
            );
            arch_handle_trap_el2(regs)
        }
        ExceptionType::EXIT_REASON_EL2_IRQ => irqchip_handle_irq2(),
        _ => arch_dump_exit(exit_reason),
    }

    let cpu = this_cpu_data();
    #[cfg(feature = "vcpu_debug_trace")]
    {
        use core::sync::atomic::{AtomicU64, Ordering};
        static AHE_COUNT: AtomicU64 = AtomicU64::new(0);
        let n = AHE_COUNT.fetch_add(1, Ordering::Relaxed);
        if n % 10000 == 0 {
            info!("[AHE] #{} exit_reason={} need_resched={} sched_calls={}",
                n, exit_reason,
                cpu.need_resched.load(Ordering::Relaxed),
                crate::scheduler::SCHEDULE_CALL_COUNTER.load(Ordering::Relaxed));
        }
    }
    if cpu.need_resched.load(core::sync::atomic::Ordering::Acquire) {
        crate::scheduler::schedule();
    }

    vcpu_vmreturn()
}

/// Final step before returning to guest: drain pending IRQs and eret.
///
/// This is the single drain+inject point for every EL2 exit, regardless of
/// whether schedule() ran a context switch.  Keeping injection here (rather
/// than also in vcpu_switch_in) avoids the double-inject race where:
///   schedule() → vcpu_switch_in() injects IRQ 27 HW=0
///   sched_tick_handler pushes another IRQ 27 into pending_virqs
///   vcpu_vmreturn() injects a second HW=0 LR → GIC conflict
///
/// IRQ 27 filter: if CNTV is enabled, unmasked, and already expired (ISTATUS=1),
/// the physical IRQ 27 will arrive as HW=1 via the EL1-IRQ path on its own.
/// Injecting a HW=0 LR on top creates a Pending HW=0/HW=1 conflict causing
/// Active-state leaks and warn floods. Drop the HW=0 entry in that case.
///
/// If current_vcpu is None (e.g. after IPI_EVENT_ZONE_SHUTDOWN cleared it),
/// re-enter the scheduler (el2_idle_loop) instead.
#[cfg(feature = "vcpu_debug_trace")]
pub static VMRETURN_CALL_COUNTER: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

fn vcpu_vmreturn() -> ! {
    use crate::arch::sysreg::{read_sysreg, write_sysreg};
    #[cfg(feature = "vcpu_debug_trace")]
    {
        use core::sync::atomic::Ordering;
        static VVR_ENTRY: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
        let vvr_n = VVR_ENTRY.fetch_add(1, Ordering::Relaxed);
        if vvr_n % 10000 == 0 {
            info!("[VVR] #{} entered vcpu_vmreturn sched_calls={}", vvr_n,
                crate::scheduler::SCHEDULE_CALL_COUNTER.load(Ordering::Relaxed));
        }
    }
    loop {
        #[cfg(feature = "vcpu_debug_trace")]
        {
            let loop_n = VMRETURN_CALL_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            if loop_n % 10000 == 0 {
                let cpu = this_cpu_data();
                info!("[VVR-LOOP] #{} current_vcpu={:?} sched_calls={}", loop_n,
                    cpu.current_vcpu.as_ref().map(|v| v.id),
                    crate::scheduler::SCHEDULE_CALL_COUNTER.load(core::sync::atomic::Ordering::Relaxed));
            }
        }
        if let Some(ref vcpu) = this_cpu_data().current_vcpu {
            // Compute once: will physical IRQ 27 arrive naturally (HW=1)?
            let irq27_will_arrive_physically = {
                let cntv_ctl: u64 = read_sysreg!(CNTV_CTL_EL0);
                let enabled = (cntv_ctl & 1) != 0;
                let masked  = (cntv_ctl & 2) != 0; // IMASK
                let expired = (cntv_ctl & 4) != 0; // ISTATUS
                enabled && !masked && expired
            };

            let pending = vcpu.drain_pending_irqs();
            for pirq in pending {
                if pirq.irq_id == 27 && !pirq.is_hardware {
                    if irq27_will_arrive_physically {
                        // Physical IRQ 27 (HW=1) is already pending/asserted and unmasked.
                        // Injecting HW=0 on top causes a Pending HW=0/HW=1 conflict.
                        // Drop the SW entry and let hardware deliver it naturally.
                        trace!("[VMRET] dropped pending IRQ 27 HW=0: physical HW=1 will arrive naturally");
                        continue;
                    }
                    // Injecting HW=0 for IRQ 27: set IMASK=1 first to suppress the
                    // physical IRQ 27 signal while the HW=0 LR is pending in GIC.
                    // Without IMASK=1 the physical IRQ stays asserted and re-fires
                    // as EL1-IRQ before the guest has consumed the HW=0 LR, causing
                    // the "HW=0 Pending conflict" warn flood in gic_handle_irq.
                    // IMASK will be cleared when the vCPU is next switched out and
                    // save_from_hardware observes ISTATUS=0 (timer handled by guest).
                    let cntv_ctl = read_sysreg!(CNTV_CTL_EL0);
                    if (cntv_ctl & 1) != 0 {
                        write_sysreg!(CNTV_CTL_EL0, cntv_ctl | 2); // set IMASK
                    }
                }
                crate::device::irqchip::inject_irq(pirq.irq_id, pirq.is_hardware);
            }

            let trapframe_ptr = vcpu.arch.trapframe_ptr();
            #[cfg(feature = "vcpu_debug_trace")]
            {
                use core::sync::atomic::{AtomicU64, Ordering};
                static VMRET_COUNT: AtomicU64 = AtomicU64::new(0);
                let n = VMRET_COUNT.fetch_add(1, Ordering::Relaxed);
                {
                    let tf = vcpu.arch.trapframe();
                    if n % 10000 == 0 {
                        info!("[VMRET] #{} vcpu={} elr={:#x} spsr={:#x}",
                            n, vcpu.id, tf.elr, tf.spsr);
                    }
                }
            }
            // Update TPIDR_EL2 so _el2h_irq_entry knows a guest is running in EL1.
            // The value (trapframe_ptr) is also used as SP in the EL1-from-IRQ path.
            write_sysreg!(TPIDR_EL2, trapframe_ptr as u64);
            unsafe { vmreturn(trapframe_ptr) }
        } else {
            // No current vCPU — re-enter scheduler (el2_idle_loop).
            // This happens after IPI_EVENT_ZONE_SHUTDOWN clears current_vcpu.
            #[cfg(feature = "vcpu_debug_trace")]
            warn!("[VVR-ELSE] current_vcpu=None, calling schedule() sched_calls={}",
                crate::scheduler::SCHEDULE_CALL_COUNTER.load(core::sync::atomic::Ordering::Relaxed));
            crate::scheduler::schedule();
            // schedule() picked a new vCPU: loop back to drain+vmreturn it.
        }
    }
}

fn irqchip_handle_irq1() {
    #[cfg(feature = "vcpu_debug_trace")]
    {
        use core::sync::atomic::Ordering;
        let n = EL1_IRQ_COUNTER.fetch_add(1, Ordering::Relaxed);
        if n % 10000 == 0 && n > 0 {
            trace!("[EL1-IRQ] irqchip_handle_irq1 called {} times (EL1 exit)", n);
        }
    }
    gic_handle_irq();
}

fn irqchip_handle_irq2() {
    error!("irq not handle from el2");
    loop {}
}

fn arch_handle_trap_el1(regs: &mut TrapFrame) {
    let mut _ret = TrapReturn::TrapUnhandled;

    trace!(
        "arch_handle_trap ec={:#x?} elr={:#x?}",
        ESR_EL2.read(ESR_EL2::EC),
        ESR_EL2.read(ESR_EL2::ISS)
    );

    match ESR_EL2.read_as_enum(ESR_EL2::EC) {
        Some(ESR_EL2::EC::Value::HVC64) => handle_hvc(regs),
        Some(ESR_EL2::EC::Value::SMC64) => handle_smc(regs),
        Some(ESR_EL2::EC::Value::TrappedMsrMrs) => handle_sysreg(regs),
        Some(ESR_EL2::EC::Value::TrappedWFIorWFE) => handle_wfi_trap(regs),
        Some(ESR_EL2::EC::Value::DataAbortLowerEL) => handle_dabt(regs),
        Some(ESR_EL2::EC::Value::InstrAbortLowerEL) => handle_iabt(regs),
        _ => {
            error!(
                "Unsupported Exception EC:{:#x?}!",
                ESR_EL2.read(ESR_EL2::EC)
            );
            error!("esr_el2: iss {:#x?}", ESR_EL2.read(ESR_EL2::ISS));
            loop {}
            // ret = TrapReturn::TrapUnhandled;
        }
    }
}

fn arch_handle_trap_el2(_regs: &mut TrapFrame) {
    let elr = ELR_EL2.get();
    let esr = ESR_EL2.get();
    let far = FAR_EL2.get();
    match ESR_EL2.read_as_enum(ESR_EL2::EC) {
        Some(ESR_EL2::EC::Value::HVC64) => {
            println!("EL2 Exception: HVC64 call, ELR_EL2: {:#x?}", ELR_EL2.get());
        }
        Some(ESR_EL2::EC::Value::SMC64) => {
            println!("EL2 Exception: SMC64 call, ELR_EL2: {:#x?}", ELR_EL2.get());
        }
        Some(ESR_EL2::EC::Value::DataAbortCurrentEL) => {
            println!(
                "EL2 Exception: Data Abort, ELR_EL2: {:#x?}, ESR_EL2: {:#x?}, FAR_EL2: {:#x?}",
                elr, esr, far
            );
            loop {}
        }
        Some(ESR_EL2::EC::Value::InstrAbortCurrentEL) => {
            println!(
                "EL2 Exception: Instruction Abort, ELR_EL2: {:#x?}, ESR_EL2: {:#x?},FAR_EL2: {:#x?}",
                elr, esr, far
            );
        }
        _ => {
            println!(
                "Unhandled EL2 Exception: EC={:#x} ELR={:#x} ESR={:#x} FAR={:#x}",
                ESR_EL2.read(ESR_EL2::EC), elr, esr, far
            );
        }
    }
    loop {}
}

fn arch_dump_el2_state() {
    println!("  SPSR_EL2={:#x} ELR_EL2={:#x}", read_sysreg!(SPSR_EL2), read_sysreg!(ELR_EL2));
    println!("  ESR_EL2={:#x} FAR_EL2={:#x} HPFAR_EL2={:#x}", read_sysreg!(ESR_EL2), read_sysreg!(FAR_EL2), read_sysreg!(HPFAR_EL2));
}

fn handle_iabt(_regs: &mut TrapFrame) {
    let iss = ESR_EL2.read(ESR_EL2::ISS);
    let op = iss >> 6 & 0x1;
    let hpfar = read_sysreg!(HPFAR_EL2);
    let far = read_sysreg!(FAR_EL2);
    let address = (far & 0xfff) | (hpfar << 8);
    error!(
        "Failed to fetch instruction (op={}) at {:#x?}, ELR_EL2={:#x?}!",
        op,
        address,
        ELR_EL2.get()
    );
    loop {}
    // TODO: finish iabt handle
    // arch_skip_instruction(frame);
}

fn handle_wfi_trap(regs: &mut TrapFrame) {
    use crate::arch::sysreg::read_sysreg;
    use crate::vcpu::VCpuState;

    let iss = ESR_EL2.read(ESR_EL2::ISS);
    let is_wfe = (iss & 1) != 0;

    // WFE: just skip, no yield.
    if is_wfe {
        arch_skip_instruction(regs);
        return;
    }

    let cpu = this_cpu_data();
    let vcpu = match cpu.current_vcpu.as_ref() {
        Some(v) => v.clone(),
        None => { arch_skip_instruction(regs); return; }
    };

    // If pending IRQs already exist, WFI condition is satisfied — just skip.
    if vcpu.has_pending_irqs() {
        arch_skip_instruction(regs);
        return;
    }

    // If any GIC LR is pending (not all free), skip WFI so guest can handle it.
    let elrsr = read_sysreg!(ICH_ELRSR_EL2);
    let vtr   = read_sysreg!(ICH_VTR_EL2);
    let lr_count = ((vtr & 0xf) + 1) as u64;
    let all_empty_mask = (1u64 << lr_count) - 1;
    if (elrsr & all_empty_mask) != all_empty_mask {
        arch_skip_instruction(regs);
        return;
    }

    let truly_alone = cpu.scheduler.no_other_vcpus();

    let vcpu_id = vcpu.id;
    let pcpu_id = cpu.id;

    #[cfg(feature = "vcpu_debug_trace")]
    {
        use core::sync::atomic::{AtomicU64, Ordering};
        static WFI_COUNT: AtomicU64 = AtomicU64::new(0);
        let n = WFI_COUNT.fetch_add(1, Ordering::Relaxed);
        if n % 10000 == 0 {
            info!("[WFI] #{} vcpu={} truly_alone={} rq={} blocked={}",
                n, vcpu_id, truly_alone,
                cpu.scheduler.len(), cpu.scheduler.blocked_vcpu_count());
        }
    }

    let cntv_ctl: u64 = read_sysreg!(CNTV_CTL_EL0);
    let timer_already_expired = (cntv_ctl & 0x5) == 0x5; // ENABLE=1, ISTATUS=1

    if truly_alone && timer_already_expired {
        // Timer already fired, vCPU is alone on pCPU (no switch-out ever ran).
        // Set IMASK=1 before injecting HW=0 to suppress the physical IRQ 27 signal.
        // Without IMASK=1, the physical CNTV stays asserted (ISTATUS=1, IMASK=0)
        // and fires as EL1-IRQ immediately after eret, before the guest has consumed
        // the HW=0 LR — causing the "HW=0 Pending conflict" warn in gic_handle_irq.
        write_sysreg!(CNTV_CTL_EL0, cntv_ctl | 2); // set IMASK=1
        drop(vcpu);
        crate::device::irqchip::inject_irq(27, false);
        arch_skip_instruction(regs);
    } else if truly_alone {
        // Timer not yet expired — real EL2 WFI until vCPU timer fires.
        // Arm EL2 timer precisely at the vCPU's CNTV expiry (physical = cval + cntvoff)
        // so the WFI wakes exactly when the guest timer is due, not after a full 10ms tick.
        let cntv_cval = read_sysreg!(CNTV_CVAL_EL0);
        let cntvoff   = read_sysreg!(CNTVOFF_EL2);
        let target_pct = cntv_cval.wrapping_add(cntvoff);
        drop(vcpu);
        crate::arch::timer::el2_timer_arm_at(target_pct);
        unsafe { core::arch::asm!("msr daifclr, #0xf") };
        aarch64_cpu::asm::wfi();
        unsafe { core::arch::asm!("msr daifset, #0xf") };
        crate::vcpu::drain_incoming_vcpus();
        if !cpu.scheduler.is_empty() {
            cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
        }
        arch_skip_instruction(regs);
    } else {
        // Other vCPUs exist (Ready or Blocked) — block self and yield pCPU.
        // schedule()'s slow path will call block_vcpu() after vcpu_switch_out()
        // saves hardware timer state. If rq is empty after blocking, schedule()
        // enters el2_idle_loop() which does real EL2 WFI until a blocked vCPU's
        // timer fires or a cross-pCPU SGI arrives.
        trace!("[WFI] vcpu={} pcpu={} → block + resched", vcpu_id, pcpu_id);
        let _ = vcpu.transition(VCpuState::Running, VCpuState::Blocked);
        arch_skip_instruction(regs);
        cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
    }
}

fn handle_dabt(regs: &mut TrapFrame) {
    let iss = ESR_EL2.read(ESR_EL2::ISS);
    let is_write = (iss >> 6 & 0x1) != 0;
    let srt = iss >> 16 & 0x1f;
    let sse = (iss >> 21 & 0x1) != 0;
    let sas = iss >> 22 & 0x3;

    let size = 1 << sas;
    let hpfar = read_sysreg!(HPFAR_EL2);
    let far = read_sysreg!(FAR_EL2);
    let address = (far & 0xfff) | (hpfar << 8);

    let mut mmio_access = MMIOAccess {
        address: address as _,
        size,
        is_write,
        value: if is_write && srt != 31 {
            regs.x[srt as usize] as _
        } else {
            0
        },
    };

    trace!("handle_dabt: {:#x?}", mmio_access);

    match mmio_handle_access(&mut mmio_access) {
        Ok(_) => {
            if !is_write && srt != 31 {
                if sse {
                    mmio_access.value =
                        ((mmio_access.value << (32 - 8 * size)) as i32) as usize >> (32 - 8 * size);
                }
                regs.x[srt as usize] = mmio_access.value as _;
            }
        }
        Err(e) => {
            error!("mmio_handle_access: {:#x?}", e);
            zone_error();
        }
    }
    //TODO finish dabt handle
    arch_skip_instruction(regs);
}

fn handle_sysreg(regs: &mut TrapFrame) {
    trace!("esr_el2: iss {:#x?}", ESR_EL2.read(ESR_EL2::ISS));
    let rt = (ESR_EL2.get() >> 5) & 0x1f;
    let val = regs.x[rt as usize];
    let sgi_id = ((val >> 24) & 0xf) as usize;
    handle_guest_sgi(val, sgi_id);
    arch_skip_instruction(regs);
}

/// Virtualise a guest write to ICC_SGI1R_EL1 / ICC_SGI0R_EL1 / ICC_ASGI1R_EL1.
///
/// Hardware passthrough is unsafe: `val` encodes guest MPIDRs (virtual) in its
/// affinity fields. When the target vCPU is Blocked (WFI), a physical SGI lands
/// at EL2 and is consumed by _el2_irq_handler — never reaching the guest GIC LR.
/// We must emulate delivery entirely in software.
fn handle_guest_sgi(val: u64, sgi_id: usize) {
    let irm         = (val >> 40) & 1;
    let aff3        = (val >> 44) & 0xf;
    let aff2        = (val >> 32) & 0xff;
    let aff1        = (val >> 16) & 0xff;
    let target_list =  val        & 0xffff;

    let cpu = this_cpu_data();
    let zone = match cpu.current_vcpu.as_ref() {
        Some(v) => v.zone.clone(),
        None => return,
    };
    let current_vcpu_id = cpu.current_vcpu.as_ref().map(|v| v.id);
    let zone_r = zone.read();

    // Collect targets first (under read lock), then deliver (lock-free).
    let targets: alloc::vec::Vec<_> = if irm == 1 {
        zone_r.vcpus()
            .iter()
            .filter(|(id, _)| Some(**id) != current_vcpu_id)
            .map(|(_, v)| v.clone())
            .collect()
    } else {
        let mut v = alloc::vec::Vec::new();
        for aff0 in 0..16u64 {
            if (target_list & (1 << aff0)) == 0 { continue; }
            let mpidr = GuestMpidr::new((aff3 << 32) | (aff2 << 16) | (aff1 << 8) | aff0);
            if let Some(vcpu) = zone_r.get_vcpu_by_guest_mpidr(mpidr) {
                v.push(vcpu);
            }
        }
        v
    };
    drop(zone_r);

    for vcpu in &targets {
        deliver_sgi_to_vcpu(vcpu, sgi_id, cpu);
    }
}

fn deliver_sgi_to_vcpu(vcpu: &alloc::sync::Arc<crate::vcpu::VCpu>, sgi_id: usize, cpu: &mut crate::cpu_data::PerCpu) {
    use crate::vcpu::VCpuState;

    if vcpu.id == 1 {
        trace!("[SGI] sgi={} -> vcpu={} state={:?} pcpu={}",
            sgi_id, vcpu.id, vcpu.state(), cpu.id);
    }

    // Target is the currently running vCPU on this pCPU — inject directly.
    if let Some(ref cur) = cpu.current_vcpu {
        if cur.id == vcpu.id && vcpu.state() == VCpuState::Running {
            crate::device::irqchip::inject_irq(sgi_id, false);
            return;
        }
    }

    match vcpu.state() {
        VCpuState::Blocked => {
            let target_pcpu  = vcpu.get_pcpu_affinity();
            let current_pcpu = cpu.id;
            if target_pcpu == current_pcpu {
                vcpu.push_pending_irq(sgi_id, false);
                match vcpu.transition(VCpuState::Blocked, VCpuState::Ready) {
                    Ok(()) => {
                        cpu.scheduler.remove_blocked(vcpu.id);
                        cpu.scheduler.enqueue(vcpu.clone());
                    }
                    Err(()) => { /* already Ready/Running — resched is sufficient */ }
                }
                cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
            } else {
                // Cross-pCPU: push IRQ directly into the vCPU's Mutex-protected pending_virqs,
                // then send IPI_EVENT_RESCHED so the target pCPU wakes the vCPU from blocked_vcpus.
                vcpu.push_pending_irq(sgi_id, false);
                crate::event::send_event(target_pcpu, crate::hypercall::SGI_IPI_ID as _, crate::event::IPI_EVENT_RESCHED);
            }
        }
        VCpuState::Ready => {
            // Already in runqueue — push IRQ so it is drained on next switch-in.
            vcpu.push_pending_irq(sgi_id, false);
            let target_pcpu  = vcpu.get_pcpu_affinity();
            let current_pcpu = cpu.id;
            if target_pcpu == current_pcpu {
                // Same pCPU: need_resched will trigger schedule() at next EL2 exit.
                cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
            } else {
                // Cross-pCPU: notify the target pCPU so it reschedules promptly.
                // Without this IPI the target pCPU may not learn about the new
                // pending IRQ until its next tick (up to 10 ms), causing SGI-based
                // synchronisation barriers in the guest to stall.
                let target_cpu_data = crate::cpu_data::get_cpu_data(target_pcpu);
                target_cpu_data.need_resched.store(true, core::sync::atomic::Ordering::Release);
                crate::event::send_event(
                    target_pcpu,
                    crate::hypercall::SGI_IPI_ID as _,
                    crate::event::IPI_EVENT_RESCHED,
                );
            }
        }
        VCpuState::Running => {
            // Running on another pCPU — send IPI_EVENT_RESCHED so the target pCPU
            // sets need_resched and drains pending_virqs on its next EL2 exit.
            let target_pcpu = vcpu.get_pcpu_affinity();
            vcpu.push_pending_irq(sgi_id, false);
            let target_cpu_data = crate::cpu_data::get_cpu_data(target_pcpu);
            target_cpu_data.need_resched.store(true, core::sync::atomic::Ordering::Release);
            crate::event::send_event(
                target_pcpu,
                crate::hypercall::SGI_IPI_ID as _,
                crate::event::IPI_EVENT_RESCHED,
            );
        }
        VCpuState::Stopped => {}
    }
}

fn handle_hvc(regs: &mut TrapFrame) {
    /*
    if ESR_EL2.read(ESR_EL2::ISS) != 0x4a48 {
        return;
    }
    */
    let (code, arg0, arg1) = (regs.x[0], regs.x[1], regs.x[2]);
    let cpu_data = this_cpu_data();

    trace!(
        "HVC from CPU{},code:{:#x?},arg0:{:#x?},arg1:{:#x?}",
        cpu_data.id,
        code,
        arg0,
        arg1
    );
    let result = match HyperCall::new(cpu_data).hypercall(code as _, arg0, arg1) {
        Ok(ret) => ret as _,
        Err(e) => {
            error!("hypercall error: {:#?}", e);
            e.code()
        }
    };
    debug!("HVC result = {}", result);
    regs.x[0] = result as _;
}

fn handle_smc(regs: &mut TrapFrame) {
    let (code, arg0, arg1, arg2) = (regs.x[0], regs.x[1], regs.x[2], regs.x[3]);
    //info!(
    //    "SMC from CPU{}, func_id:{:#x?}, arg0:{:#x?}, arg1:{:#x?}, arg2:{:#x?}",
    //    cpu_data.id, code, arg0, arg1, arg2
    //);
    let result = match code & SMC_TYPE_MASK {
        SmcType::ARCH_SC => handle_arch_smc(regs, code, arg0, arg1, arg2),
        SmcType::STANDARD_SC => handle_psci_smc(regs, code, arg0, arg1, arg2),
        SmcType::TOS_SC_START..=SmcType::TOS_SC_END | SmcType::SIP_SC => {
            let ret = smc_call(code, &regs.x[1..18]);
            regs.x[0] = ret[0];
            regs.x[1] = ret[1];
            regs.x[2] = ret[2];
            regs.x[3] = ret[3];
            ret[0]
        }
        _ => {
            warn!("unsupported smc {:#x?}", code);
            0
        }
    };
    regs.x[0] = result;

    arch_skip_instruction(regs); //skip the smc ins
}

fn psci_emulate_features_info(code: u64) -> u64 {
    match code {
        PsciFnId::PSCI_VERSION
        | PsciFnId::PSCI_CPU_SUSPEND_32
        | PsciFnId::PSCI_CPU_SUSPEND_64
        | PsciFnId::PSCI_CPU_OFF_32
        | PsciFnId::PSCI_CPU_ON_32
        | PsciFnId::PSCI_CPU_ON_64
        | PsciFnId::PSCI_AFFINITY_INFO_32
        | PsciFnId::PSCI_AFFINITY_INFO_64
        | PsciFnId::PSCI_FEATURES
        | SMCccFnId::SMCCC_VERSION => 0,
        _ => !0,
    }
}

fn psci_emulate_cpu_on(regs: &mut TrapFrame) -> u64 {
    // regs.x[1] = target MPIDR (guest-visible virtual, NOT physical)
    // regs.x[2] = entry_point_address
    // regs.x[3] = context_id (passed to secondary in x0)
    let target_guest_mpidr = GuestMpidr::new(regs.x[1]);
    let entry_point = regs.x[2] as usize;
    let context_id  = regs.x[3];
    info!(
        "psci CPU_ON: guest_mpidr={:#x} entry={:#x} ctx={:#x}",
        target_guest_mpidr.0, entry_point, context_id
    );

    // Look up target vCPU by guest MPIDR in the current zone.
    // MUST use guest MPIDR lookup — mpidr_to_cpuid() resolves physical MPIDRs,
    // which are unrelated to the zone-local virtual MPIDR guest passes here.
    let zone = this_zone();
    let vcpu = match zone.read().get_vcpu_by_guest_mpidr(target_guest_mpidr) {
        Some(v) => v,
        None => {
            error!(
                "psci CPU_ON: no vCPU for guest MPIDR {:#x}",
                target_guest_mpidr.0
            );
            return u64::MAX; // PSCI_INVALID_PARAMETERS
        }
    };

    if vcpu.state() != VCpuState::Stopped {
        error!(
            "psci CPU_ON: vcpu {} not Stopped (state={:?})",
            vcpu.id, vcpu.state()
        );
        return u64::MAX - 3; // PSCI_ALREADY_ON
    }

    // Reset vCPU arch state to ARMv8 reset values for a clean secondary boot.
    vcpu.arch.reset_el1_regs();
    vcpu.arch.reset_gic_state();

    // Set entry point and context_id in the vCPU's TrapFrame.
    // vcpu_switch_in will load ELR_EL2/SPSR_EL2 from trapframe on first schedule-in.
    {
        let tf = vcpu.arch.trapframe();
        tf.x.fill(0);
        tf.x[0] = context_id;          // x0 = context_id (PSCI spec §5.4.2)
        tf.elr  = entry_point as u64;  // guest entry PC
        tf.spsr = 0x3c5;               // EL1h, D/A/I/F all masked
    }

    // Transition Stopped→Ready and deliver to its affinity pCPU.
    let ret = crate::arch::vcpu::arch_wakeup_vcpu(vcpu);
    if ret == 0 { 0 } else { u64::MAX - 3 }
}

fn handle_psci_smc(
    regs: &mut TrapFrame,
    code: u64,
    arg0: u64,
    _arg1: u64,
    _arg2: u64,
) -> u64 {
    match code {
        PsciFnId::PSCI_VERSION => PSCI_VERSION_1_1,
        PsciFnId::PSCI_CPU_SUSPEND_32 | PsciFnId::PSCI_CPU_SUSPEND_64 => {
            info!("[PSCI] CPU_SUSPEND vcpu={:?} pcpu={}", this_cpu_data().current_vcpu.as_ref().map(|v| v.id), this_cpu_data().id);
            // Block the current vCPU: transition Running→Blocked, then schedule().
            // The scheduler will save context and pick the next Ready vCPU (or idle).
            // When the vCPU is woken (timer expiry / SGI), schedule() will resume it
            // and el1_regs.restore_to_hardware() will restore ELR_EL2/SPSR_EL2
            // so guest resumes after the SMC instruction (arch_skip_instruction
            // has already advanced ELR_EL2 past the SMC by the time we get here).
            let cpu = this_cpu_data();
            if let Some(ref vcpu) = cpu.current_vcpu.clone() {
                let _ = vcpu.transition(VCpuState::Running, VCpuState::Blocked);
                cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
            }
            // Return 0 (PSCI_SUCCESS) — guest will see this in x0 when woken.
            0
        }
        PsciFnId::PSCI_CPU_OFF_32 | PsciFnId::PSCI_CPU_OFF_64 => {
            // Stop the current vCPU permanently: transition Running→Stopped, then schedule().
            // The scheduler will not re-enqueue it. Guest can re-start it via PSCI CPU_ON.
            let cpu = this_cpu_data();
            if let Some(ref vcpu) = cpu.current_vcpu.clone() {
                let _ = vcpu.transition(VCpuState::Running, VCpuState::Stopped);
                cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
            }
            // CPU_OFF never returns to the caller — need_resched will trigger
            // schedule() at the end of arch_handle_exit and pick another vCPU.
            0
        }
        PsciFnId::PSCI_AFFINITY_INFO_32 | PsciFnId::PSCI_AFFINITY_INFO_64 => {
            // arg0 = target affinity (guest MPIDR), arg1 = lowest_affinity_level
            // Return 0 = ON, 1 = OFF, 2 = ON_PENDING
            let target_guest_mpidr = GuestMpidr::new(arg0);
            let zone = this_zone();
            let vcpu_opt = zone.read().get_vcpu_by_guest_mpidr(target_guest_mpidr);
            match vcpu_opt {
                Some(vcpu) => match vcpu.state() {
                    VCpuState::Running | VCpuState::Ready | VCpuState::Blocked => 0, // ON
                    VCpuState::Stopped => 1, // OFF
                },
                None => 1, // unknown MPIDR → treat as OFF
            }
        }
        PsciFnId::PSCI_MIG_INFO_TYPE => PSCI_TOS_NOT_PRESENT_MP,
        PsciFnId::PSCI_FEATURES => psci_emulate_features_info(regs.x[1]),
        PsciFnId::PSCI_CPU_ON_32 | PsciFnId::PSCI_CPU_ON_64 => psci_emulate_cpu_on(regs),
        PsciFnId::PSCI_SYSTEM_OFF => {
            let zone = this_zone();
            let zone_id = zone.id();
            let is_root = is_this_root_zone();

            for cpu_id in zone.read().cpu_set().iter_except(this_cpu_data().id) {
                let target_cpu = get_cpu_data(cpu_id);
                let _lock = target_cpu.ctrl_lock.lock();
                target_cpu.zone = None;
                send_event(cpu_id, SGI_IPI_ID as _, IPI_EVENT_SHUTDOWN);
            }

            this_cpu_data().zone = None;
            drop(zone);
            remove_zone(zone_id);

            if is_root {
                psci::system_off().unwrap();
            }

            this_cpu_data().arch_cpu.idle();
        }

        _ => {
            warn!("unsupported smc standard service {:#x?}", code);
            0
        }
    }
}

fn handle_arch_smc(
    _regs: &mut TrapFrame,
    code: u64,
    _arg0: u64,
    _arg1: u64,
    _arg2: u64,
) -> u64 {
    match code {
        SMCccFnId::SMCCC_VERSION => ARM_SMCCC_VERSION_1_1,
        SMCccFnId::SMCCC_ARCH_FEATURES => !0,
        _ => {
            error!("unsupported ARM smc service");
            return !0;
        }
    }
}

fn arch_skip_instruction(regs: &mut TrapFrame) {
    let ins = match ESR_EL2.read(ESR_EL2::IL) {
        0 => 2, // 16-bit Thumb instruction
        1 => 4, // 32-bit AArch64 instruction
        _ => 0,
    };
    regs.elr += ins;
}

fn arch_dump_exit(reason: u64) {
    //TODO hypervisor coredump
    error!("Unsupported Exit:{:#x?}, elr={:#x?}", reason, ELR_EL2.get());
    loop {}
}

#[naked]
#[no_mangle]
/// Restore guest context from a TrapFrame and eret to guest.
///
/// x0 = pointer to TrapFrame. We set SP = x0 so that the next trap
/// writes directly into this TrapFrame (SP stays = TrapFrame base).
///
/// TrapFrame layout:
///   [+0x000] x0..x30  (31 × u64, 248 bytes)
///   [+0x0f8] elr       (u64) — loaded into ELR_EL2
///   [+0x100] spsr      (u64) — loaded into SPSR_EL2
pub unsafe extern "C" fn vmreturn(trapframe: usize) -> ! {
    core::arch::asm!(
        "
        mov  sp, x0
        ldp  x2,  x3,  [sp, #0x10]
        ldp  x4,  x5,  [sp, #0x20]
        ldp  x6,  x7,  [sp, #0x30]
        ldp  x8,  x9,  [sp, #0x40]
        ldp  x10, x11, [sp, #0x50]
        ldp  x12, x13, [sp, #0x60]
        ldp  x14, x15, [sp, #0x70]
        ldp  x16, x17, [sp, #0x80]
        ldp  x18, x19, [sp, #0x90]
        ldp  x20, x21, [sp, #0xa0]
        ldp  x22, x23, [sp, #0xb0]
        ldp  x24, x25, [sp, #0xc0]
        ldp  x26, x27, [sp, #0xd0]
        ldp  x28, x29, [sp, #0xe0]
        ldr  x30,      [sp, #0xf0]
        ldr  x1,       [sp, #0xf8]
        msr  elr_el2,  x1
        ldr  x1,       [sp, #0x100]
        msr  spsr_el2, x1
        ldp  x0,  x1,  [sp]
        eret
        ",
        options(noreturn),
    )
}
