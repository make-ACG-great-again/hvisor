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

//! Per-CPU VCpu Scheduler.
//!
//! Each physical CPU has its own `PerCpuScheduler` with independent run queues.
//! Scheduling policy: 4-level priority queues with round-robin within each level.

use crate::vcpu::{VCpu, VCpuState};
use alloc::collections::VecDeque;
use alloc::sync::Arc;

/// Number of priority levels (0=highest, 3=lowest).
pub const NUM_PRIORITIES: usize = 4;

/// Default time slice in ticks.
pub const DEFAULT_TIME_SLICE: usize = 1;

/// Starvation prevention: boost after this many rounds without execution.
const STARVATION_THRESHOLD: usize = 10;

// ========================
// PerCpuScheduler
// ========================

/// Per-CPU scheduler with multi-level priority run queues.
pub struct PerCpuScheduler {
    /// 4-level priority run queues. Index 0 = highest priority.
    run_queue: [VecDeque<Arc<VCpu>>; NUM_PRIORITIES],
    /// Currently running VCpu on this pCPU (None if idle).
    pub current: Option<Arc<VCpu>>,
    /// Remaining time slice ticks for the current VCpu.
    pub time_slice_remaining: usize,
    /// Counter for starvation prevention.
    starvation_counter: usize,
    /// Blocked VCpus waiting for an event (WFI).
    blocked_vcpus: VecDeque<BlockedVCpuEntry>,
}

/// Entry in the per-CPU blocked VCpu list.
pub struct BlockedVCpuEntry {
    pub vcpu: Arc<VCpu>,
    /// Saved CNTV_CVAL_EL0: virtual timer compare value.
    pub cntv_cval: u64,
    /// Saved CNTV_CTL_EL0: bit 0 = ENABLE, bit 1 = IMASK.
    pub cntv_ctl: u64,
    /// Saved CNTVOFF_EL2: virtual counter offset.
    pub cntvoff: u64,
    /// Physical counter value when this VCpu entered blocked state.
    pub blocked_at_cnt: u64,
}

impl PerCpuScheduler {
    pub fn new() -> Self {
        Self {
            run_queue: [
                VecDeque::new(),
                VecDeque::new(),
                VecDeque::new(),
                VecDeque::new(),
            ],
            current: None,
            time_slice_remaining: DEFAULT_TIME_SLICE,
            starvation_counter: 0,
            blocked_vcpus: VecDeque::new(),
        }
    }

    /// Enqueue a VCpu into the correct priority queue.
    pub fn enqueue(&mut self, vcpu: Arc<VCpu>) {
        let prio = (vcpu.priority as usize).min(NUM_PRIORITIES - 1);
        self.run_queue[prio].push_back(vcpu);
    }

    /// Select the next VCpu from the highest-priority non-empty queue.
    pub fn pick_next(&mut self) -> Option<Arc<VCpu>> {
        self.starvation_counter += 1;
        if self.starvation_counter >= STARVATION_THRESHOLD {
            self.boost_starving_vcpus();
            self.starvation_counter = 0;
        }

        for prio in 0..NUM_PRIORITIES {
            if let Some(vcpu) = self.run_queue[prio].pop_front() {
                if prio == NUM_PRIORITIES - 1 {
                    self.starvation_counter = 0;
                }
                return Some(vcpu);
            }
        }
        None
    }

    /// Boost all priority-3 VCpus to priority-2 to prevent starvation.
    fn boost_starving_vcpus(&mut self) {
        let lowest  = NUM_PRIORITIES - 1; // 3
        let boost_to = NUM_PRIORITIES - 2; // 2
        while let Some(vcpu) = self.run_queue[lowest].pop_front() {
            self.run_queue[boost_to].push_back(vcpu);
        }
    }

    /// Check if the run queues are all empty (does NOT include blocked_vcpus).
    pub fn is_empty(&self) -> bool {
        self.run_queue.iter().all(|q| q.is_empty())
    }

    /// True if there are NO other VCpus on this pCPU (neither runqueue nor blocked).
    pub fn no_other_vcpus(&self) -> bool {
        self.is_empty() && !self.has_blocked_vcpus()
    }

    /// Total number of VCpus in all run queues.
    pub fn len(&self) -> usize {
        self.run_queue.iter().map(|q| q.len()).sum()
    }

    /// Remove all VCpus belonging to the given zone from run queues and blocked list.
    pub fn clear_zone(&mut self, zone_id: usize) {
        for q in self.run_queue.iter_mut() {
            q.retain(|v| v.zone.id() != zone_id);
        }
        self.blocked_vcpus.retain(|entry| entry.vcpu.zone.id() != zone_id);
        if let Some(ref cur) = self.current {
            if cur.zone.id() == zone_id {
                self.current = None;
            }
        }
    }

    /// Remove ALL VCpus (for pCPU reclaim).
    pub fn clear_all(&mut self) {
        info!(
            "[SCH] clear_all: dropping {} run-queue + {} blocked vcpus",
            self.len(),
            self.blocked_vcpus.len()
        );
        for q in self.run_queue.iter_mut() {
            q.clear();
        }
        self.blocked_vcpus.clear();
        self.current = None;
    }

    /// Add a VCpu to the blocked list with its saved virtual timer state.
    pub fn block_vcpu(&mut self, vcpu: Arc<VCpu>, cntv_cval: u64, cntv_ctl: u64, cntvoff: u64) {
        let now = crate::arch::timer::current_cntpct();
        self.blocked_vcpus.push_back(BlockedVCpuEntry {
            vcpu,
            cntv_cval,
            cntv_ctl,
            cntvoff,
            blocked_at_cnt: now,
        });
    }

    /// Check all blocked VCpus for expired virtual timers.
    /// Returns the number of VCpus woken up.
    pub fn check_blocked_timers(&mut self, current_cnt: u64) -> usize {
        let mut woken = 0;
        let mut i = 0;
        while i < self.blocked_vcpus.len() {
            let entry = &self.blocked_vcpus[i];
            let ctl    = entry.cntv_ctl;
            let cval   = entry.cntv_cval;
            let cntvoff = entry.cntvoff;

            // Timer is active if ENABLE (bit 0) = 1.
            // NOTE: Do NOT check IMASK here — hvisor may have set IMASK=1 in
            // restore_to_hardware() to prevent IRQ storms on saved expired timers.
            let timer_enabled = (ctl & 1) != 0;
            let virtual_cnt   = current_cnt.wrapping_sub(cntvoff);
            let timer_expired = timer_enabled && virtual_cnt >= cval;

            let blocked_at = entry.blocked_at_cnt;
            let one_tick   = crate::arch::timer::tick_period_cnt();
            let timed_out  = current_cnt >= blocked_at.wrapping_add(one_tick);
            let ten_ticks  = one_tick.wrapping_mul(10);
            let absolute_timeout = current_cnt >= blocked_at.wrapping_add(ten_ticks);
            let no_timer_timeout = (timed_out && entry.vcpu.has_pending_irqs())
                || absolute_timeout;

            if timer_expired {
                let entry = self.blocked_vcpus.remove(i).unwrap();
                entry.vcpu.push_pending_irq(27, false);
                if entry.vcpu.transition(VCpuState::Blocked, VCpuState::Ready).is_ok() {
                    self.enqueue(entry.vcpu);
                    woken += 1;
                }
                // Don't increment i — next element shifted into position i
            } else if no_timer_timeout {
                let entry = self.blocked_vcpus.remove(i).unwrap();
                trace!("[WAKE-NOTIMER] vcpu={} woken after no-timer timeout", entry.vcpu.id);
                if entry.vcpu.transition(VCpuState::Blocked, VCpuState::Ready).is_ok() {
                    self.enqueue(entry.vcpu);
                    woken += 1;
                }
            } else {
                i += 1;
            }
        }
        woken
    }

    /// Check virtual timers for all Ready VCpus in the run queue (1:N overcommit).
    #[cfg(target_arch = "aarch64")]
    pub fn check_ready_timers(&self, current_cnt: u64) {
        for prio in 0..NUM_PRIORITIES {
            for vcpu in &self.run_queue[prio] {
                let cntv_ctl  = vcpu.arch.el1_regs.cntv_ctl_el0;
                let cntv_cval = vcpu.arch.el1_regs.cntv_cval_el0;
                let cntvoff   = vcpu.arch.el1_regs.cntvoff_el2;
                let enabled   = (cntv_ctl & 1) != 0;
                let masked    = (cntv_ctl & 2) != 0; // IMASK
                if !enabled || masked {
                    continue;
                }
                // Convert virtual deadline to physical: CNTPCT = CNTV_CVAL + CNTVOFF
                let phys_deadline = cntv_cval.wrapping_add(cntvoff);
                if current_cnt >= phys_deadline && !vcpu.has_pending_irqs() {
                    vcpu.push_pending_irq(27, false);
                }
            }
        }
    }

    /// Remove a specific VCpu from the blocked list.
    pub fn remove_blocked(&mut self, vcpu_id: usize) -> bool {
        if let Some(pos) = self.blocked_vcpus.iter().position(|e| e.vcpu.id == vcpu_id) {
            self.blocked_vcpus.remove(pos);
            true
        } else {
            false
        }
    }

    /// Find a blocked VCpu by id without removing it.
    pub fn find_blocked(&self, vcpu_id: usize) -> Option<Arc<VCpu>> {
        self.blocked_vcpus
            .iter()
            .find(|e| e.vcpu.id == vcpu_id)
            .map(|e| e.vcpu.clone())
    }

    /// Check if there are any blocked VCpus.
    pub fn has_blocked_vcpus(&self) -> bool {
        !self.blocked_vcpus.is_empty()
    }

    /// Return the earliest blocked VCpu timer expiry (as physical CNTPCT).
    pub fn earliest_blocked_timer_cntpct(&self) -> Option<u64> {
        self.blocked_vcpus
            .iter()
            .filter(|e| (e.cntv_ctl & 1) != 0) // ENABLE only, ignore IMASK
            .map(|e| e.cntv_cval.wrapping_add(e.cntvoff)) // physical = virtual + offset
            .min()
    }
}

// ========================
// Context switch
// ========================

/// Save outgoing VCpu's full context.
/// General registers are already in the vCPU's TrapFrame (saved by trap.S on entry).
/// Here we only save EL1 system registers + GIC virtualization state.
#[cfg(target_arch = "aarch64")]
pub fn vcpu_switch_out(vcpu: &VCpu) {
    // DSB ISH: ensure all guest EL1 stores are visible before saving context.
    unsafe { core::arch::asm!("dsb ish", options(nostack, preserves_flags)) };

    unsafe {
        let arch = &vcpu.arch as *const _ as *mut crate::arch::vcpu::ArchVCpu;
        (*arch).el1_regs.save_from_hardware();
        (*arch).gic_state.save_from_hardware();
    }
}

/// Restore incoming VCpu's full context.
/// Restores EL1 registers + GIC state + vGICR + VMPIDR_EL2 + pending IRQs.
/// General registers live in the vCPU's TrapFrame; vmreturn(trapframe_ptr) restores them.
#[cfg(target_arch = "aarch64")]
pub fn vcpu_switch_in(vcpu: &VCpu, prev_zone_id: Option<usize>) {
    use crate::arch::vcpu::restore_vgicr;
    use crate::arch::sysreg::write_sysreg;

    // Restore EL1 system registers (does NOT restore ELR_EL2/SPSR_EL2 — those come from TrapFrame)
    vcpu.arch.el1_regs.restore_to_hardware();
    vcpu.arch.gic_state.restore_to_hardware();

    // Restore per-VCpu virtual GICR shadow (IGROUPR0, ISENABLER0, IPRIORITYR, ICFGR1)
    restore_vgicr(vcpu);

    // ISB: flush pipeline so restored sysregs take effect before any EL1 instruction.
    unsafe { core::arch::asm!("isb", options(nostack, preserves_flags)) };

    // Set VMPIDR_EL2 to the VCpu's virtual MPIDR so guest Linux sees the correct MPIDR.
    {
        use crate::arch::sysreg::read_sysreg;
        let phys_mpidr: u64 = read_sysreg!(MPIDR_EL1);
        let vcpu_base = vcpu.zone.vcpu_base();
        let local_idx = vcpu.id.saturating_sub(vcpu_base);
        let virtual_mpidr = (phys_mpidr & !0xff) | (local_idx as u64 & 0xff);
        write_sysreg!(VMPIDR_EL2, virtual_mpidr);
    }

    // Same-zone optimization: skip VTTBR_EL2 switch if zone matches.
    let curr_zone_id = vcpu.zone.id();
    let need_gpm_switch = match prev_zone_id {
        Some(prev_id) => prev_id != curr_zone_id,
        None => true,
    };
    if need_gpm_switch {
        // activate_vmm() sets VTCR_EL2 and HCR_EL2.VM — required before Stage-2
        // translation can work. Must be called before activate_gpm() (VTTBR_EL2).
        crate::cpu_data::this_cpu_data().arch_cpu.activate_vmm();
        vcpu.activate_gpm();
    }

    // Drain pending_virqs and inject into GIC LRs
    let pending = vcpu.drain_pending_irqs();
    for pirq in pending {
        crate::device::irqchip::inject_irq(pirq.irq_id, pirq.is_hardware);
    }
}

// ========================
// schedule()
// ========================

/// Core scheduling function.
///
/// Called when `need_resched` is true or from IPI handlers.
/// General registers are already in each vCPU's TrapFrame (saved by trap.S on entry).
/// After schedule() returns, the caller calls `vmreturn(vcpu.arch.trapframe_ptr())`.
#[cfg(target_arch = "aarch64")]
pub fn schedule() {
    use crate::cpu_data::this_cpu_data;
    use crate::vcpu::drain_incoming_vcpus;
    use core::sync::atomic::Ordering;

    // Drain incoming VCPUs (PSCI CPU_ON cross-pCPU delivery) before picking.
    drain_incoming_vcpus();

    let cpu = this_cpu_data();
    cpu.need_resched.store(false, Ordering::Release);

    // Fast path: if current VCpu is still Running and truly no other VCpu exists,
    // skip the switch cycle and just reset time slice.
    if let Some(ref current) = cpu.scheduler.current {
        if current.state() == VCpuState::Running && cpu.scheduler.no_other_vcpus() {
            cpu.scheduler.time_slice_remaining = DEFAULT_TIME_SLICE;
            crate::arch::timer::el2_timer_rearm();
            return;
        }
        // Fast path missed — log why (trace only, warn was causing UART timing issues).
        trace!("[SCH-SLOWPATH] pcpu={} vcpu={} state={:?} rq={} blocked={}",
            cpu.id, current.id, current.state(),
            cpu.scheduler.len(), cpu.scheduler.has_blocked_vcpus());
    } else {
        trace!("[SCH-SLOWPATH] pcpu={} no current vcpu rq={} blocked={}",
            cpu.id, cpu.scheduler.len(), cpu.scheduler.has_blocked_vcpus());
    }

    let prev_vcpu = cpu.scheduler.current.take();
    let prev_zone_id: Option<usize>;

    // Save outgoing VCpu
    if let Some(ref prev) = prev_vcpu {
        vcpu_switch_out(prev);
        prev_zone_id = Some(prev.zone.id());

        let state = prev.state();
        match state {
            VCpuState::Running => {
                match prev.transition(VCpuState::Running, VCpuState::Ready) {
                    Ok(()) => {
                        cpu.scheduler.enqueue(prev.clone());
                    }
                    Err(()) => {
                        let actual = prev.state();
                        warn!("[SCH] prev transition fail vcpu={} Running->Ready actual={:?}", prev.id, actual);
                        match actual {
                            VCpuState::Blocked => {
                                let cntv_cval = prev.arch.el1_regs.cntv_cval_el0;
                                let cntv_ctl  = prev.arch.el1_regs.cntv_ctl_el0;
                                let cntvoff   = prev.arch.el1_regs.cntvoff_el2;
                                cpu.scheduler.block_vcpu(prev.clone(), cntv_cval, cntv_ctl, cntvoff);
                            }
                            VCpuState::Stopped => { /* CPU_OFF — drop */ }
                            VCpuState::Ready => {
                                cpu.scheduler.enqueue(prev.clone());
                            }
                            _ => {
                                warn!("[SCH] vcpu {} in unexpected state {:?}, dropping", prev.id, actual);
                            }
                        }
                    }
                }
            }
            VCpuState::Blocked => {
                let actual = prev.state();
                if actual == VCpuState::Blocked {
                    let cntv_cval = prev.arch.el1_regs.cntv_cval_el0;
                    let cntv_ctl  = prev.arch.el1_regs.cntv_ctl_el0;
                    let cntvoff   = prev.arch.el1_regs.cntvoff_el2;
                    cpu.scheduler.block_vcpu(prev.clone(), cntv_cval, cntv_ctl, cntvoff);
                }
                // If already Ready (woken concurrently), waker owns enqueue — don't enqueue again.
            }
            VCpuState::Stopped => {
                // CPU_OFF — don't re-enqueue
            }
            VCpuState::Ready => {
                warn!("[SCH] prev vcpu {} already in Ready state, re-enqueueing", prev.id);
                cpu.scheduler.enqueue(prev.clone());
            }
        }
    } else {
        prev_zone_id = None;
    }

    // Pick-and-transition loop: retry if CAS Ready→Running fails.
    loop {
        let next_vcpu = pick_next_or_idle(prev_zone_id);

        if let Some(next_vcpu) = next_vcpu {
            let cpu = this_cpu_data();
            if next_vcpu.transition(VCpuState::Ready, VCpuState::Running).is_err() {
                warn!("[SCH] next transition fail vcpu={} Ready->Running, retrying", next_vcpu.id);
                continue;
            }

            cpu.scheduler.time_slice_remaining = DEFAULT_TIME_SLICE;
            cpu.scheduler.current = Some(next_vcpu.clone());

            vcpu_switch_in(&next_vcpu, prev_zone_id);

            crate::arch::timer::el2_timer_rearm();

            // Update percpu current_vcpu
            cpu.current_vcpu = Some(next_vcpu);
            break;
        } else {
            break; // pick_next_or_idle returned None (went through idle) — exit loop.
        }
    }
}

/// Try to pick the next VCpu. If none is available, enter the EL2 idle loop.
#[cfg(target_arch = "aarch64")]
fn pick_next_or_idle(prev_zone_id: Option<usize>) -> Option<Arc<VCpu>> {
    use crate::cpu_data::this_cpu_data;

    let cpu = this_cpu_data();
    if let Some(vcpu) = cpu.scheduler.pick_next() {
        return Some(vcpu);
    }

    trace!("CPU {}: entering idle loop (no Ready VCpu)", cpu.id);
    el2_idle_loop()
}

/// EL2 idle loop — the pCPU sleeps here until a VCpu becomes Ready.
///
/// Executes at EL2 with IRQs enabled. EL2 IRQ vector fires on physical IRQ,
/// calls `gic_handle_irq()` then erets back here.
#[cfg(target_arch = "aarch64")]
fn el2_idle_loop() -> Option<Arc<VCpu>> {
    use aarch64_cpu::asm::wfi;
    use crate::cpu_data::this_cpu_data;
    use core::sync::atomic::Ordering;

    loop {
        let cpu = this_cpu_data();
        match cpu.scheduler.earliest_blocked_timer_cntpct() {
            Some(target) => {
                crate::arch::timer::el2_timer_arm_at(target);
            }
            None => {
                if cpu.scheduler.has_blocked_vcpus() {
                    crate::arch::timer::el2_timer_rearm();
                } else {
                    crate::arch::timer::el2_timer_disable();
                }
            }
        }

        // Enable IRQs so physical interrupts are delivered via _el2_irq_handler.
        unsafe { core::arch::asm!("msr daifclr, #0xf") };

        wfi(); // Real hardware WFI

        // Disable IRQs before touching scheduler state.
        unsafe { core::arch::asm!("msr daifset, #0xf") };

        // Unconditionally drain incoming VCPUs after WFI.
        // The SGI that woke us may not have been acknowledged via ICC_IAR1_EL1
        // in _el2_irq_handler (spurious read at EL2), so incoming_vcpus may still
        // have entries even though no check_events() drain ran.
        crate::vcpu::drain_incoming_vcpus();

        let cpu = this_cpu_data();
        if let Some(vcpu) = cpu.scheduler.pick_next() {
            return Some(vcpu);
        }

        if cpu.need_resched.load(Ordering::Acquire) {
            cpu.need_resched.store(false, Ordering::Release);
            if let Some(vcpu) = cpu.scheduler.pick_next() {
                return Some(vcpu);
            }
        }
    }
}

/// Initialize the scheduler subsystem (no-op for per-CPU schedulers).
pub fn init() {
    // Per-CPU schedulers are initialized in PerCpu::new()
}
