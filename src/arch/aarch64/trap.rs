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

use super::cpu::GeneralRegisters;
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
extern "C" fn el2_irq_handler() {
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

/*From hyp_vec->handle_vmexit x0:guest regs x1:exit_reason sp =stack_top-32*8*/
pub fn arch_handle_exit(regs: &mut GeneralRegisters) -> ! {
    let mpidr = MPIDR_EL1.get();
    let _cpu_id = mpidr_to_cpuid(mpidr);
    trace!("cpu exit, exit_reson:{:#x?}", regs.exit_reason);
    match regs.exit_reason as u64 {
        ExceptionType::EXIT_REASON_EL1_IRQ | ExceptionType::EXIT_REASON_EL1_AARCH32_IRQ => {
            irqchip_handle_irq1()
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
        _ => arch_dump_exit(regs.exit_reason),
    }

    // Check if a vCPU context switch is needed.
    // `regs` points to the pCPU stack frame (stack_top - 256) — this is the
    // "current guest register snapshot" that vcpu_switch_out will copy from.
    let stack_regs_ptr = regs as *const _ as usize;
    let cpu = this_cpu_data();
    if cpu.need_resched.load(core::sync::atomic::Ordering::Acquire) {
        crate::scheduler::schedule(stack_regs_ptr);
    }

    // schedule() → vcpu_switch_in() already copied the selected vCPU's guest_regs
    // to the pCPU stack frame at stack_regs_ptr.  Always vmreturn from there so
    // SP_EL2 ends up at stack_top after eret (not pointing into the vCPU heap).
    unsafe { vmreturn(stack_regs_ptr) }
}

fn irqchip_handle_irq1() {
    trace!("irq from el1");
    gic_handle_irq();
}

fn irqchip_handle_irq2() {
    error!("irq not handle from el2");
    loop {}
}

fn arch_handle_trap_el1(regs: &mut GeneralRegisters) {
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

fn arch_handle_trap_el2(_regs: &mut GeneralRegisters) {
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

fn handle_iabt(_regs: &mut GeneralRegisters) {
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

fn handle_dabt(regs: &mut GeneralRegisters) {
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
            regs.usr[srt as usize] as _
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
                regs.usr[srt as usize] = mmio_access.value as _;
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

fn handle_sysreg(regs: &mut GeneralRegisters) {
    trace!("esr_el2: iss {:#x?}", ESR_EL2.read(ESR_EL2::ISS));
    let rt = (ESR_EL2.get() >> 5) & 0x1f;
    let val = regs.usr[rt as usize];
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
    use crate::cpu_data::PendingWake;

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
                // Cross-pCPU: push to target's pending_wake_ids, send IPI_EVENT_RESCHED.
                // The target pCPU's handler injects the IRQ and enqueues the vCPU.
                crate::cpu_data::get_cpu_data(target_pcpu)
                    .pending_wake_ids.lock()
                    .push_back(PendingWake { vcpu_id: vcpu.id, irq_id: sgi_id, is_hardware: false });
                crate::event::send_event(target_pcpu, crate::hypercall::SGI_IPI_ID as _, crate::event::IPI_EVENT_RESCHED);
            }
        }
        VCpuState::Ready => {
            // Already in runqueue — IRQ will be drained on next switch-in.
            vcpu.push_pending_irq(sgi_id, false);
            cpu.need_resched.store(true, core::sync::atomic::Ordering::Release);
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

fn handle_hvc(regs: &mut GeneralRegisters) {
    /*
    if ESR_EL2.read(ESR_EL2::ISS) != 0x4a48 {
        return;
    }
    */
    let (code, arg0, arg1) = (regs.usr[0], regs.usr[1], regs.usr[2]);
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
    regs.usr[0] = result as _;
}

fn handle_smc(regs: &mut GeneralRegisters) {
    let (code, arg0, arg1, arg2) = (regs.usr[0], regs.usr[1], regs.usr[2], regs.usr[3]);
    //info!(
    //    "SMC from CPU{}, func_id:{:#x?}, arg0:{:#x?}, arg1:{:#x?}, arg2:{:#x?}",
    //    cpu_data.id, code, arg0, arg1, arg2
    //);
    let result = match code & SMC_TYPE_MASK {
        SmcType::ARCH_SC => handle_arch_smc(regs, code, arg0, arg1, arg2),
        SmcType::STANDARD_SC => handle_psci_smc(regs, code, arg0, arg1, arg2),
        SmcType::TOS_SC_START..=SmcType::TOS_SC_END | SmcType::SIP_SC => {
            let ret = smc_call(code, &regs.usr[1..18]);
            regs.usr[0] = ret[0];
            regs.usr[1] = ret[1];
            regs.usr[2] = ret[2];
            regs.usr[3] = ret[3];
            ret[0]
        }
        _ => {
            warn!("unsupported smc {:#x?}", code);
            0
        }
    };
    regs.usr[0] = result;

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

fn psci_emulate_cpu_on(regs: &mut GeneralRegisters) -> u64 {
    // regs.usr[1] = target MPIDR (guest-visible virtual, NOT physical)
    // regs.usr[2] = entry_point_address
    // regs.usr[3] = context_id (passed to secondary in x0)
    let target_guest_mpidr = GuestMpidr::new(regs.usr[1]);
    let entry_point = regs.usr[2] as usize;
    let context_id  = regs.usr[3];
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

    // Set entry point (ELR_EL2) and context_id (x0) in the vCPU's saved state.
    // These will be restored by el1_regs.restore_to_hardware() on first switch-in.
    unsafe {
        let el1 = &mut *(core::ptr::addr_of!(vcpu.arch.el1_regs)
            as *mut crate::arch::vcpu::El1SysRegs);
        el1.elr_el2  = entry_point as u64;
        el1.spsr_el2 = 0x3c5; // EL1h, D/A/I/F all masked
    }
    unsafe {
        let gr = &mut *(core::ptr::addr_of!(vcpu.arch.guest_regs)
            as *mut crate::arch::cpu::GeneralRegisters);
        gr.usr.fill(0);
        gr.usr[0] = context_id; // x0 = context_id (PSCI spec §5.4.2)
    }

    // Transition Stopped→Ready and deliver to its affinity pCPU.
    let ret = crate::arch::vcpu::arch_wakeup_vcpu(vcpu);
    if ret == 0 { 0 } else { u64::MAX - 3 }
}

fn handle_psci_smc(
    regs: &mut GeneralRegisters,
    code: u64,
    arg0: u64,
    _arg1: u64,
    _arg2: u64,
) -> u64 {
    match code {
        PsciFnId::PSCI_VERSION => PSCI_VERSION_1_1,
        PsciFnId::PSCI_CPU_SUSPEND_32 | PsciFnId::PSCI_CPU_SUSPEND_64 => {
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
        PsciFnId::PSCI_FEATURES => psci_emulate_features_info(regs.usr[1]),
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
    _regs: &mut GeneralRegisters,
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

fn arch_skip_instruction(_regs: &mut GeneralRegisters) {
    //ELR_EL2: ret address
    let mut pc = ELR_EL2.get();
    //ESR_EL2::IL exception instruction length
    let ins = match ESR_EL2.read(ESR_EL2::IL) {
        0 => 2, //16 bit ins
        1 => 4, //32 bit ins
        _ => 0,
    };
    //skip ins
    pc = pc + ins;
    ELR_EL2.set(pc);
}

fn arch_dump_exit(reason: u64) {
    //TODO hypervisor coredump
    error!("Unsupported Exit:{:#x?}, elr={:#x?}", reason, ELR_EL2.get());
    loop {}
}

#[naked]
#[no_mangle]
pub unsafe extern "C" fn vmreturn(_gu_regs: usize) -> ! {
    core::arch::asm!(
        "
        /* x0: guest registers */
        mov	sp, x0
        ldp	x1, x0, [sp], #16	/* x1 is the exit_reason */
        ldp	x1, x2, [sp], #16
        ldp	x3, x4, [sp], #16
        ldp	x5, x6, [sp], #16
        ldp	x7, x8, [sp], #16
        ldp	x9, x10, [sp], #16
        ldp	x11, x12, [sp], #16
        ldp	x13, x14, [sp], #16
        ldp	x15, x16, [sp], #16
        ldp	x17, x18, [sp], #16
        ldp	x19, x20, [sp], #16
        ldp	x21, x22, [sp], #16
        ldp	x23, x24, [sp], #16
        ldp	x25, x26, [sp], #16
        ldp	x27, x28, [sp], #16
        ldp	x29, x30, [sp], #16
        /*now el2 sp point to per cpu stack top*/
        eret                            //ret to el2_entry hvc #0 now,depend on ELR_EL2
        
    ",
        options(noreturn),
    )
}
