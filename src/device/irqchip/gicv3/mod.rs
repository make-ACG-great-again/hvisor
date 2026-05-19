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

#![allow(dead_code)]
pub mod gicd;
pub mod gicr;
pub mod gits;
pub mod vgic;

use core::arch::asm;
use core::ptr::write_volatile;
use core::sync::atomic::AtomicU64;

use alloc::vec::Vec;
use gicr::init_lpi_prop;
use gits::gits_init;
use spin::{Lazy, Once};

use self::gicd::{enable_gic_are_ns, GICD_ICACTIVER, GICD_ICENABLER};
use self::gicr::enable_ipi;
use crate::arch::aarch64::sysreg::{read_sysreg, smc_arg1, write_sysreg};
use crate::arch::cpu::cpuid_to_mpidr_affinity;
use crate::arch::zone::GicConfig;
use crate::config::root_zone_config;
use crate::consts::{self, MAX_CPU_NUM};

use crate::device::irqchip::gicv3::gits::gits_reset;
use crate::event::check_events;
use crate::hypercall::SGI_IPI_ID;
use crate::zone::Zone;

const ICH_HCR_UIE: u64 = 1 << 1;
//TODO: add Distributor init
pub fn gicc_init() {
    //TODO: add Redistributor init
    let sdei_ver = unsafe { smc_arg1!(0xc4000020) }; //sdei_check();

    // Make ICC_EOIR1_EL1 provide priority drop functionality only. ICC_DIR_EL1 provides interrupt deactivation functionality.
    let _ctlr = read_sysreg!(icc_ctlr_el1);
    write_sysreg!(icc_ctlr_el1, 0x2);
    // Set Interrupt Controller Interrupt Priority Mask Register
    let pmr = read_sysreg!(icc_pmr_el1);
    write_sysreg!(icc_pmr_el1, 0xf0);
    // Enable group 1 irq
    let _igrpen = read_sysreg!(icc_igrpen1_el1);
    write_sysreg!(icc_igrpen1_el1, 0x1);

    gicv3_clear_pending_irqs();
    let _vtr = read_sysreg!(ich_vtr_el2);
    let vmcr = ((pmr & 0xff) << 24) | (1 << 1); //VPMR|VENG1
    write_sysreg!(ich_vmcr_el2, vmcr);
    write_sysreg!(ich_hcr_el2, 0x1); //enable virt cpu interface

    info!("gicc init done, sdei_ver = {}", sdei_ver);
}

fn gicv3_clear_pending_irqs() {
    let vtr = read_sysreg!(ich_vtr_el2) as usize;
    let lr_num: usize = (vtr & 0xf) + 1;
    for i in 0..lr_num {
        write_lr(i, 0) //clear lr
    }
    let num_priority_bits = (vtr >> 29) + 1;
    /* Clear active priority bits */
    if num_priority_bits >= 5 {
        write_sysreg!(ICH_AP1R0_EL2, 0); //Interrupt Controller Hyp Active Priorities Group 1 Register 0 No interrupt active
    }
    if num_priority_bits >= 6 {
        write_sysreg!(ICH_AP1R1_EL2, 0);
    }
    if num_priority_bits > 6 {
        write_sysreg!(ICH_AP1R2_EL2, 0);
        write_sysreg!(ICH_AP1R3_EL2, 0);
    }
}

static TIMER_INTERRUPT_COUNTER: AtomicU64 = AtomicU64::new(0);
// how often to print timer interrupt counter
const TIMER_INTERRUPT_PRINT_INTERVAL: u64 = 50;

pub fn gicv3_handle_irq_el1() {
    use core::sync::atomic::{AtomicU64, Ordering};
    static GIC_ENTRY_COUNT: AtomicU64 = AtomicU64::new(0);
    let gic_n = GIC_ENTRY_COUNT.fetch_add(1, Ordering::Relaxed);
    if gic_n % 10000 == 0 {
        info!("[GIC-EL1] enter #{}", gic_n);
    }
    let mut irq26_count = 0u32;
    let mut irq27_count = 0u32;
    let mut other_count = 0u32;
    let mut loop_iter = 0u32;
    static LOOP_LOG: AtomicU64 = AtomicU64::new(0);
    while let Some(irq_id) = pending_irq() {
        loop_iter += 1;
        {
            let n = LOOP_LOG.fetch_add(1, Ordering::Relaxed);
            if n % 5000 == 0 {
                info!("[GIC-LOOP] #{} gic_n={} iter={} irq={}", n, gic_n, loop_iter, irq_id);
            }
        }
        if irq_id < 8 {
            trace!("sgi get {}, try to handle...", irq_id);
            deactivate_irq(irq_id);
            let mut ipi_handled = false;
            if irq_id == SGI_IPI_ID as _ {
                ipi_handled = check_events();
            }
            if !ipi_handled {
                trace!("sgi get {}, inject", irq_id);
                schedule_inject_irq(irq_id, false);
            }
        } else if irq_id < 16 {
            warn!("skip sgi {}", irq_id);
            deactivate_irq(irq_id);
        } else {
            if irq_id == 26 {
                // EL2 physical timer (CNTHP) — scheduling tick, private to hypervisor.
                // Must NOT be injected into the guest.
                irq26_count += 1;
                #[cfg(target_arch = "aarch64")]
                {
                    use core::sync::atomic::{AtomicU64, Ordering};
                    static TICK_CALL: AtomicU64 = AtomicU64::new(0);
                    let t = TICK_CALL.fetch_add(1, Ordering::Relaxed);
                    if t % 10000 == 0 { info!("[TICK-CALL] before #{}", t); }
                    crate::arch::timer::sched_tick_handler();
                    if t % 10000 == 0 { info!("[TICK-CALL] after #{}", t); }
                }
                deactivate_irq(irq_id);
                continue;
            } else if irq_id == 27 {
                irq27_count += 1;
                // virtual timer interrupt
                TIMER_INTERRUPT_COUNTER.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
                if TIMER_INTERRUPT_COUNTER.load(core::sync::atomic::Ordering::SeqCst)
                    % TIMER_INTERRUPT_PRINT_INTERVAL
                    == 0
                {
                    trace!(
                        "Virtual timer interrupt, counter = {}",
                        TIMER_INTERRUPT_COUNTER.load(core::sync::atomic::Ordering::SeqCst)
                    );
                }
            } else if irq_id == 25 {
                other_count += 1;
                // maintenace interrupt
                handle_maintenace_interrupt();
            } else if irq_id > 31 {
                other_count += 1;
                //inject phy irq
                trace!("*** get spi_irq id = {}", irq_id);
            } else {
                other_count += 1;
                warn!("not konw irq id = {}", irq_id);
            }
            let lr_written = if irq_id != 25 {
                schedule_inject_irq(irq_id, true)
            } else {
                true
            };
            // EOImode=1: EOIR only drops priority, DIR deactivates.
            // Always EOIR first (priority drop), then DIR if needed.
            deactivate_irq(irq_id);
            // IRQ 27: if LR.HW=1 was written (lr_written=true), guest EOIR will
            // hardware-deactivate the physical IRQ automatically (VEOIM=0).
            // Physical IRQ stays Active until guest EOI — this prevents CNTV from
            // re-asserting Pending while we are still in the EL2 handler loop.
            // If no LR was written (lr_written=false), write DIR now to prevent
            // physical IRQ 27 staying permanently Active → RCU stall.
            if irq_id == 27 && !lr_written {
                write_sysreg!(icc_dir_el1, 27u64);
            }
        }
    }
    // Log IRQ counts if anything unusual (irq26 > 1, or total > 3).
    let total = irq26_count + irq27_count + other_count;
    if gic_n % 10000 == 0 || loop_iter > 5 {
        info!("[GIC-EL1] exit #{} iters={} irq26={} irq27={} other={}", gic_n, loop_iter, irq26_count, irq27_count, other_count);
    }
    if irq26_count > 1 || total > 3 {
        warn!("[IRQ-STAT] #{} irq26={} irq27={} other={} total={}",
            gic_n, irq26_count, irq27_count, other_count, total);
    }
}

fn pending_irq() -> Option<usize> {
    let iar = read_sysreg!(icc_iar1_el1) as usize;
    if iar == 0x3ff {
        None
    } else {
        Some(iar as _)
    }
}

fn deactivate_irq(irq_id: usize) {
    write_sysreg!(icc_eoir1_el1, irq_id as u64);
    // With EOImode=1, EOIR only drops priority. DIR deactivates the interrupt.
    // Must DIR for: SGIs (<16), maintenance (25), EL2 timer (26).
    // IRQ 27 (CNTV) is injected with LR.HW=1; guest EOIR handles deactivation automatically.
    if irq_id < 16 || irq_id == 25 || irq_id == 26 {
        write_sysreg!(icc_dir_el1, irq_id as u64);
    }
}

pub fn read_lr(id: usize) -> u64 {
    let id = id as u64;
    match id {
        //TODO get lr size from gic reg
        0 => read_sysreg!(ich_lr0_el2),
        1 => read_sysreg!(ich_lr1_el2),
        2 => read_sysreg!(ich_lr2_el2),
        3 => read_sysreg!(ich_lr3_el2),
        4 => read_sysreg!(ich_lr4_el2),
        5 => read_sysreg!(ich_lr5_el2),
        6 => read_sysreg!(ich_lr6_el2),
        7 => read_sysreg!(ich_lr7_el2),
        8 => read_sysreg!(ich_lr8_el2),
        9 => read_sysreg!(ich_lr9_el2),
        10 => read_sysreg!(ich_lr10_el2),
        11 => read_sysreg!(ich_lr11_el2),
        12 => read_sysreg!(ich_lr12_el2),
        13 => read_sysreg!(ich_lr13_el2),
        14 => read_sysreg!(ich_lr14_el2),
        15 => read_sysreg!(ich_lr15_el2),
        _ => {
            error!("lr over");
            loop {}
        }
    }
}

pub fn write_lr(id: usize, val: u64) {
    let id = id as u64;
    match id {
        0 => write_sysreg!(ich_lr0_el2, val),
        1 => write_sysreg!(ich_lr1_el2, val),
        2 => write_sysreg!(ich_lr2_el2, val),
        3 => write_sysreg!(ich_lr3_el2, val),
        4 => write_sysreg!(ich_lr4_el2, val),
        5 => write_sysreg!(ich_lr5_el2, val),
        6 => write_sysreg!(ich_lr6_el2, val),
        7 => write_sysreg!(ich_lr7_el2, val),
        8 => write_sysreg!(ich_lr8_el2, val),
        9 => write_sysreg!(ich_lr9_el2, val),
        10 => write_sysreg!(ich_lr10_el2, val),
        11 => write_sysreg!(ich_lr11_el2, val),
        12 => write_sysreg!(ich_lr12_el2, val),
        13 => write_sysreg!(ich_lr13_el2, val),
        14 => write_sysreg!(ich_lr14_el2, val),
        15 => write_sysreg!(ich_lr15_el2, val),
        _ => {
            error!("lr over");
            loop {}
        }
    }
}

pub const MAINTENACE_INTERRUPT: u64 = 25;

// Enable or disable an underflow maintenance interrupt.
fn enable_maintenace_interrupt(is_enable: bool) {
    trace!("enable_maintenace_interrupt, is_enable is {}", is_enable);
    let mut hcr = read_sysreg!(ich_hcr_el2);
    if is_enable {
        hcr |= ICH_HCR_UIE;
    } else {
        hcr &= !ICH_HCR_UIE;
    }
    write_sysreg!(ich_hcr_el2, hcr);
}

/// Maintenance interrupt handler.
///
/// LR slots have become free (UIE = underflow). Drain the current running
/// vCPU's per-vCPU pending_virqs queue and inject as many as possible into
/// the newly freed LR slots. If LRs fill up again, keep UIE enabled so we
/// get called again when more slots free up.
fn handle_maintenace_interrupt() {
    trace!("handle_maintenace_interrupt");
    use crate::cpu_data::this_cpu_data;
    let cpu = this_cpu_data();
    let vcpu = match cpu.scheduler.current.as_ref() {
        Some(v) => v.clone(),
        None => {
            enable_maintenace_interrupt(false);
            return;
        }
    };

    let pending = vcpu.drain_pending_irqs();
    let mut deferred: Vec<crate::vcpu::PendingIrq> = Vec::new();
    for pirq in pending {
        if inject_irq(pirq.irq_id, pirq.is_hardware) {
            trace!("inject pending irq {} in maintenance interrupt", pirq.irq_id);
        } else {
            // LR full again — put remaining back
            deferred.push(pirq);
        }
    }
    if deferred.is_empty() {
        enable_maintenace_interrupt(false);
    } else {
        for pirq in deferred {
            vcpu.push_pending_irq(pirq.irq_id, pirq.is_hardware);
        }
        enable_maintenace_interrupt(true);
    }
}

/// Schedule-aware IRQ injection.
///
/// Routes the IRQ to the correct target vCPU:
///   - For PPI/SGI (irq < 32): targets the current vCPU on this pCPU.
///   - For SPI (irq >= 32): checks irq_target_vcpu mapping (set by GICD_IROUTER
///     writes) to find the intended vCPU. If the target is the current vCPU,
///     inject directly. If it is a different vCPU (Blocked/Ready on the same
///     pCPU), push to that vCPU's pending queue and wake it. If it lives on
///     another pCPU, push IRQ into pending_virqs and send IPI_EVENT_RESCHED.
///
/// Returns true if an LR entry was written (caller may skip DIR for HW IRQs).
pub fn schedule_inject_irq(irq_id: usize, is_hardware: bool) -> bool {
    use crate::cpu_data::{get_cpu_data, this_cpu_data};
    use crate::vcpu::VCpuState;

    let cpu = this_cpu_data();

    // For SPI, look up the target vCPU from the IROUTER mapping.
    if irq_id >= 32 {
        if let Some(ref zone_arc) = cpu.zone {
            let zone = zone_arc.read();
            if let Some(target_vcpu_id) = zone.irq_target_vcpu.get(irq_id).copied().flatten() {
                let is_current = cpu
                    .current_vcpu
                    .as_ref()
                    .map(|v| v.id == target_vcpu_id)
                    .unwrap_or(false);

                if !is_current {
                    if let Some(target_vcpu) = zone.vcpus().get(&target_vcpu_id).cloned() {
                        let target_pcpu = target_vcpu.get_pcpu_affinity();
                        let current_pcpu = cpu.id;
                        drop(zone);

                        if target_pcpu == current_pcpu {
                            target_vcpu.push_pending_irq(irq_id, is_hardware);
                            if target_vcpu.state() == VCpuState::Blocked {
                                if target_vcpu
                                    .transition(VCpuState::Blocked, VCpuState::Ready)
                                    .is_ok()
                                {
                                    cpu.scheduler.remove_blocked(target_vcpu_id);
                                    cpu.scheduler.enqueue(target_vcpu);
                                    cpu.need_resched
                                        .store(true, core::sync::atomic::Ordering::Release);
                                }
                            }
                        } else {
                            // Cross-pCPU: push IRQ directly into vCPU's Mutex-protected
                            // pending_virqs, then IPI_EVENT_RESCHED so the target pCPU
                            // wakes the vCPU from its blocked_vcpus list.
                            target_vcpu.push_pending_irq(irq_id, is_hardware);
                            crate::event::send_event(
                                target_pcpu,
                                crate::hypercall::SGI_IPI_ID as _,
                                crate::event::IPI_EVENT_RESCHED,
                            );
                        }
                        // LR not written — caller must DIR for HW-mapped IRQs.
                        return false;
                    }
                }
            }
            // No specific target or target == current: fall through.
        }
    }

    // Default: inject into the current vCPU on this pCPU.
    if let Some(ref vcpu) = cpu.current_vcpu {
        if vcpu.state() == VCpuState::Running {
            return inject_irq(irq_id, is_hardware);
        }
        vcpu.push_pending_irq(irq_id, is_hardware);
        if vcpu.state() == VCpuState::Blocked {
            if vcpu
                .transition(VCpuState::Blocked, VCpuState::Ready)
                .is_ok()
            {
                cpu.scheduler.remove_blocked(vcpu.id);
                cpu.scheduler.enqueue(vcpu.clone());
                cpu.need_resched
                    .store(true, core::sync::atomic::Ordering::Release);
            }
        }
        return false;
    }

    warn!(
        "schedule_inject_irq: no current vCPU on CPU {}, dropping IRQ {}",
        cpu.id, irq_id
    );
    false
}

/// Inject virtual interrupt to vCPU, return whether it not needs to add pending queue.
pub fn inject_irq(irq_id: usize, is_hardware: bool) -> bool {
    // mask
    const LR_VIRTIRQ_MASK: usize = (1 << 32) - 1;

    let elsr: u64 = read_sysreg!(ich_elrsr_el2);
    let vtr = read_sysreg!(ich_vtr_el2) as usize;
    let lr_num: usize = (vtr & 0xf) + 1;
    let mut free_ir = -1 as isize;
    for i in 0..lr_num {
        // find a free list register
        if (1 << i) & elsr > 0 {
            if free_ir == -1 {
                free_ir = i as isize;
            }
            continue;
        }
        let lr_val = read_lr(i) as usize;
        // if a virtual interrupt is enabled and equals to the physical interrupt irq_id
        if (lr_val & LR_VIRTIRQ_MASK) == irq_id {
            trace!("virtual irq {} enables again", irq_id);
            return true;
        }
    }
    trace!("To Inject IRQ {}, find lr {}", irq_id, free_ir);

    if free_ir == -1 {
        trace!("all list registers are valid, add to per-vcpu pending queue");
        // LR slots all occupied — store in the current vCPU's per-vCPU pending
        // queue and enable the UIE maintenance interrupt. When LR slots free up,
        // handle_maintenace_interrupt() drains from this same queue and injects
        // into the correct vCPU (whichever is Running at that point — which will
        // always be the same vCPU since we only inject for the current runner).
        //
        // Note: if there is no current vCPU (schedule() is mid-flight with
        // current=None), we cannot queue the IRQ. For hardware-mapped IRQs the
        // caller is responsible for writing DIR after EOIR to prevent a
        // permanent Active leak (gicv3_handle_irq_el1 already does this via the
        // lr_written=false path). For software IRQs (is_hardware=false) the
        // signal is level-sensitive and will be re-delivered naturally.
        use crate::cpu_data::this_cpu_data;
        if let Some(vcpu) = this_cpu_data().scheduler.current.as_ref() {
            vcpu.push_pending_irq(irq_id, is_hardware);
        } else {
            warn!("inject_irq: LR full, no current vCPU, IRQ {} (hw={}) deferred to re-delivery", irq_id, is_hardware);
        }
        enable_maintenace_interrupt(true);
        return false;
    } else {
        let mut val = irq_id as u64; //v intid
        val |= 1 << 60; //group 1
        val |= 1 << 62; //state pending

        if !is_sgi(irq_id as _) && is_hardware {
            val |= 1 << 61; //map hardware
            val |= (irq_id as u64) << 32; //pINTID
        }
        write_lr(free_ir as usize, val);
        return true;
    }
}

pub static GIC: Once<Gic> = Once::new();
pub const PER_GICR_SIZE: usize = 0x20000;

// GICR register offsets and fields
const GICR_TYPER_AFFINITY_VALUE_SHIFT: usize = 32;
const GICR_TYPER_AFFINITY_VALUE_MASK: u64 = 0xFFFFFFFF << GICR_TYPER_AFFINITY_VALUE_SHIFT;

#[derive(Debug)]
pub struct Gic {
    pub gicd_base: usize,
    pub gicr_base: usize,
    pub gicd_size: usize,
    pub gicr_size: usize,
    pub gits_base: usize,
    pub gits_size: usize,
}

pub fn host_gicd_base() -> usize {
    GIC.get().unwrap().gicd_base
}

static CPU_GICR_BASE: Lazy<Vec<usize>> = Lazy::new(|| {
    let mut bases = vec![0; MAX_CPU_NUM];
    let gic = GIC.get().unwrap();
    let base = gic.gicr_base;
    let mut found_cpus = 0;

    // Scan through all GICR frames once
    let mut curr_base = base;

    for _ in 0..MAX_CPU_NUM {
        let typer =
            unsafe { core::ptr::read_volatile((curr_base + gicr::GICR_TYPER) as *const u64) };
        let affinity = (typer & GICR_TYPER_AFFINITY_VALUE_MASK) >> GICR_TYPER_AFFINITY_VALUE_SHIFT;

        // Find which CPU this GICR belongs to
        if let Some(cpu_id) = (0..MAX_CPU_NUM).position(|cpu_id| {
            let (aff3, aff2, aff1, aff0) = cpuid_to_mpidr_affinity(cpu_id as u64);
            let aff = (aff3 << 24) | (aff2 << 16) | (aff1 << 8) | aff0;
            aff == affinity
        }) {
            bases[cpu_id] = curr_base;
            found_cpus += 1;
        }
        curr_base += PER_GICR_SIZE;
    }

    if found_cpus != MAX_CPU_NUM {
        panic!(
            "Could not find GICR for all CPUs, only found {}",
            found_cpus
        );
    }
    info!("GICR bases: {:#x?}", bases);
    bases
});

pub fn host_gicr_base(id: usize) -> usize {
    assert!(id < consts::MAX_CPU_NUM);
    CPU_GICR_BASE[id]
}

pub fn host_gits_base() -> usize {
    GIC.get().unwrap().gits_base
}

pub fn host_gicd_size() -> usize {
    GIC.get().unwrap().gicd_size
}

pub fn host_gicr_size() -> usize {
    GIC.get().unwrap().gicr_size
}

pub fn host_gits_size() -> usize {
    GIC.get().unwrap().gits_size
}

pub fn is_spi(irqn: u32) -> bool {
    irqn > 31 && irqn < 1020
}

pub fn is_sgi(irqn: u32) -> bool {
    irqn < 16
}

pub fn enable_irqs() {
    unsafe { asm!("msr daifclr, #0xf") };
}

pub fn disable_irqs() {
    unsafe { asm!("msr daifset, #0xf") };
}

pub fn primary_init_early() {
    let root_config = root_zone_config();
    match root_config.arch_config.gic_config {
        GicConfig::Gicv2(_) => {
            panic!("GICv2 is not supported in this version of hvisor");
        }
        GicConfig::Gicv3(ref gicv3_config) => {
            info!("GICv3 detected");
            GIC.call_once(|| Gic {
                gicd_base: gicv3_config.gicd_base,
                gicr_base: gicv3_config.gicr_base,
                gicd_size: gicv3_config.gicd_size,
                gicr_size: gicv3_config.gicr_size,
                gits_base: gicv3_config.gits_base,
                gits_size: gicv3_config.gits_size,
            });
            info!(
                "GIC Distributor base: {:#x}, size: {:#x}",
                GIC.get().unwrap().gicd_base,
                GIC.get().unwrap().gicd_size
            );
            info!(
                "GIC Redistributor base: {:#x}, size: {:#x}",
                GIC.get().unwrap().gicr_base,
                GIC.get().unwrap().gicr_size
            );
            info!(
                "GIC ITS base: {:#x}, size: {:#x}",
                GIC.get().unwrap().gits_base,
                GIC.get().unwrap().gits_size
            );
        }
    }
    init_lpi_prop();

    if host_gits_base() != 0 && host_gits_size() != 0 {
        gits_init();
    }

    // Force CPU_GICR_BASE Lazy initialization here, while running single-threaded
    // with IRQs disabled (primary_init_early runs before primary_init_late/enable_irqs).
    // Without this, the first access to CPU_GICR_BASE happens in el2_timer_init on each
    // pCPU concurrently with IRQs already enabled — if an IRQ fires during Lazy init and
    // the handler also calls host_gicr_base(), the spin::Lazy spinlock deadlocks silently.
    let _ = &*CPU_GICR_BASE;

    debug!("gic = {:#x?}", GIC.get().unwrap());
}

pub fn primary_init_late() {
    enable_gic_are_ns();
    enable_irqs();
}

pub fn percpu_init() {
    gicc_init();
    enable_ipi();
}

impl Zone {
    pub fn arch_irqchip_reset(&self) {
        let gicd_base = host_gicd_base();
        let zone = self.read();
        for (idx, &mask) in zone.irq_bitmap().iter().enumerate() {
            if idx == 0 {
                continue;
            }
            unsafe {
                write_volatile((gicd_base + GICD_ICENABLER + idx * 4) as *mut u32, mask);
                write_volatile((gicd_base + GICD_ICACTIVER + idx * 4) as *mut u32, mask);
            }
        }
        if host_gits_size() != 0 {
            gits_reset(self.id());
        }
    }
}
