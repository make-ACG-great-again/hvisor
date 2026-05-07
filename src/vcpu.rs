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

//! Virtual CPU abstraction and lifecycle management.
//!
//! Each `VCpu` represents a guest virtual processor that can be scheduled
//! 1:N on physical pCPUs. VCpus are owned by a `Zone` and bound (by affinity)
//! to a specific pCPU's `PerCpuScheduler`.

use crate::arch::vcpu::ArchVCpu;
use crate::cpu_data::this_cpu_data;
use crate::zone::Zone;
use alloc::collections::VecDeque;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicU8, AtomicUsize, Ordering};
use spin::Mutex;

// ========================
// VCpu ID pool
// ========================

static VCPU_ID: AtomicUsize = AtomicUsize::new(0);

lazy_static::lazy_static! {
    static ref VCPU_ID_POOL: Mutex<VecDeque<usize>> = Mutex::new(VecDeque::new());
}

fn free_vcpu_id() -> usize {
    if let Some(id) = VCPU_ID_POOL.lock().pop_front() {
        id
    } else {
        VCPU_ID.fetch_add(1, Ordering::SeqCst)
    }
}

pub fn reclaim_vcpu_id(id: usize) {
    VCPU_ID_POOL.lock().push_back(id);
}

// ========================
// VCpuState
// ========================

/// VCpu lifecycle states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VCpuState {
    /// Initial state or after PSCI CPU_OFF. Not in any run queue.
    Stopped = 0,
    /// In a pCPU's run queue, waiting to be scheduled.
    Ready = 1,
    /// Currently executing on a pCPU.
    Running = 2,
    /// Blocked by WFI/CPU_SUSPEND. Not in any run queue, awaiting interrupt wakeup.
    Blocked = 3,
}

impl VCpuState {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Stopped),
            1 => Some(Self::Ready),
            2 => Some(Self::Running),
            3 => Some(Self::Blocked),
            _ => None,
        }
    }

    /// Check whether a transition from `self` to `to` is valid.
    pub fn can_transition_to(self, to: VCpuState) -> bool {
        matches!(
            (self, to),
            (VCpuState::Stopped, VCpuState::Ready)
                | (VCpuState::Ready, VCpuState::Running)
                | (VCpuState::Running, VCpuState::Ready)
                | (VCpuState::Running, VCpuState::Blocked)
                | (VCpuState::Running, VCpuState::Stopped)
                | (VCpuState::Blocked, VCpuState::Ready)
                | (VCpuState::Blocked, VCpuState::Stopped) // zone shutdown
                | (VCpuState::Ready, VCpuState::Stopped)   // zone shutdown
        )
    }
}

// ========================
// PendingIrq
// ========================

/// A virtual interrupt pending delivery to a VCpu.
#[derive(Debug, Clone, Copy)]
pub struct PendingIrq {
    pub irq_id: usize,
    pub is_hardware: bool,
}

/// Maximum number of pending IRQs per VCpu.
const MAX_PENDING_VIRQS: usize = 64;

// ========================
// VCpu
// ========================

pub struct VCpu {
    pub id: usize,
    pub zone: Arc<Zone>,
    pub arch: ArchVCpu,

    state: AtomicU8,

    /// Scheduling priority (0=highest, 3=lowest, default=2).
    pub priority: u8,

    /// pCPU affinity: which pCPU this vCPU is bound to.
    /// Set once during zone_create, never changed after that.
    pub pcpu_affinity: Mutex<Option<usize>>,

    /// Pending virtual IRQs to be injected into GIC LRs on next switch-in.
    pending_virqs: Mutex<VecDeque<PendingIrq>>,

    /// Diagnostic: number of EL2 exits (traps) taken by this VCpu.
    pub exit_count: AtomicU64,
}

impl VCpu {
    pub fn new(zone: Arc<Zone>) -> Self {
        Self {
            id: free_vcpu_id(),
            zone,
            arch: ArchVCpu::new(),
            state: AtomicU8::new(VCpuState::Stopped as u8),
            priority: 2,
            pcpu_affinity: Mutex::new(None),
            pending_virqs: Mutex::new(VecDeque::new()),
            exit_count: AtomicU64::new(0),
        }
    }

    pub fn activate_gpm(&self) {
        unsafe {
            self.zone.read().gpm().activate();
        }
    }

    pub fn zone_id(&self) -> usize {
        self.zone.id()
    }

    // --- State query ---

    /// Returns the current state of this VCpu.
    pub fn state(&self) -> VCpuState {
        VCpuState::from_u8(self.state.load(Ordering::Acquire))
            .expect("Invalid VCpuState value")
    }

    // --- State transition (CAS) ---

    /// Attempt to transition from `from` to `to` using compare-and-swap.
    pub fn transition(&self, from: VCpuState, to: VCpuState) -> Result<(), ()> {
        if !from.can_transition_to(to) {
            warn!(
                "VCpu {}: invalid state transition {:?} -> {:?}",
                self.id, from, to
            );
            return Err(());
        }

        self.state
            .compare_exchange(from as u8, to as u8, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|actual| {
                let actual_state = VCpuState::from_u8(actual);
                warn!(
                    "VCpu {}: CAS failed for {:?} -> {:?}, actual state = {:?}",
                    self.id, from, to, actual_state
                );
            })
    }

    /// Force-set state to Stopped (for zone shutdown).
    pub fn force_stop(&self) {
        self.state.store(VCpuState::Stopped as u8, Ordering::Release);
    }

    // --- Pending IRQ management ---

    /// Add a pending virtual IRQ.
    /// SGIs (0-15) and IRQ 27 (CNTV virtual timer) are deduplicated — they are
    /// level-triggered or idempotent, so multiple pending copies have the same
    /// effect as one and would otherwise flood the queue.
    /// Drops oldest entry if queue is full.
    pub fn push_pending_irq(&self, irq_id: usize, is_hardware: bool) {
        let mut queue = self.pending_virqs.lock();
        // Deduplicate virtual timer (27) only — it is level-triggered so multiple
        // pending copies are redundant. SGIs must NOT be deduplicated: each SGI
        // represents an independent IPI (e.g., TLB shootdown), and dropping one
        // can cause memory consistency failures in the guest.
        if irq_id == 27 {
            if let Some(pos) = queue.iter().position(|p| p.irq_id == 27) {
                queue[pos] = PendingIrq { irq_id, is_hardware };
                return;
            }
        }
        if queue.len() >= MAX_PENDING_VIRQS {
            let dropped = queue.pop_front();
            warn!(
                "VCpu {}: pending_virqs full, dropped {:?}",
                self.id, dropped
            );
        }
        queue.push_back(PendingIrq { irq_id, is_hardware });
    }

    /// Remove and return all pending IRQs.
    pub fn drain_pending_irqs(&self) -> Vec<PendingIrq> {
        let mut queue = self.pending_virqs.lock();
        queue.drain(..).collect()
    }

    /// Check if there are pending IRQs without draining.
    pub fn has_pending_irqs(&self) -> bool {
        !self.pending_virqs.lock().is_empty()
    }

    // --- pCPU affinity helpers ---

    /// Set the pCPU affinity for this VCpu. Called once during zone_create().
    pub fn set_pcpu_affinity(&self, pcpu_id: usize) {
        *self.pcpu_affinity.lock() = Some(pcpu_id);
    }

    /// Get the pCPU affinity for this VCpu.
    /// Panics if affinity was never set (programming error).
    pub fn get_pcpu_affinity(&self) -> usize {
        self.pcpu_affinity
            .lock()
            .expect("VCpu::get_pcpu_affinity called but affinity was never set")
    }
}

// ========================
// Global helpers
// ========================

/// Get the current VCpu running on this pCPU.
pub fn current_vcpu() -> Arc<VCpu> {
    this_cpu_data().current_vcpu.clone().unwrap()
}

/// Set the current VCpu running on this pCPU.
pub fn set_current_vcpu(vcpu: Arc<VCpu>) {
    this_cpu_data().current_vcpu = Some(vcpu);
}

/// Enqueue a VCpu on its affinity pCPU's scheduler and send IPI if cross-pCPU.
///
/// Prerequisites: The VCpu must already be in Ready state.
pub fn enqueue_vcpu_on_affinity_pcpu(vcpu: Arc<VCpu>) {
    use crate::cpu_data::{get_cpu_data, this_cpu_data};

    let target_pcpu = vcpu.get_pcpu_affinity();
    let current_pcpu = this_cpu_data().id;
    let vcpu_id = vcpu.id;

    let target_cpu_data = get_cpu_data(target_pcpu);
    target_cpu_data.scheduler.remove_blocked(vcpu_id);
    target_cpu_data.scheduler.enqueue(vcpu);
    target_cpu_data.need_resched.store(true, Ordering::Release);

    if target_pcpu != current_pcpu {
        crate::event::send_event(
            target_pcpu,
            crate::hypercall::SGI_IPI_ID as _,
            crate::event::IPI_EVENT_RESCHED,
        );
    }
}

/// Deliver a newly-initialized VCpu to a target pCPU via its incoming_vcpus queue.
/// Used by PSCI CPU_ON for cross-pCPU first-boot delivery.
pub fn deliver_vcpu_to_pcpu(target_pcpu: usize, vcpu: Arc<VCpu>) {
    use crate::cpu_data::get_cpu_data;
    let target_cpu_data = get_cpu_data(target_pcpu);
    target_cpu_data.incoming_vcpus.lock().push_back(vcpu);
    crate::event::send_event(
        target_pcpu,
        crate::hypercall::SGI_IPI_ID as _,
        crate::event::IPI_EVENT_INCOMING_VCPU,
    );
}

/// Drain this pCPU's incoming_vcpus queue into the local scheduler.
/// Called by IPI_EVENT_INCOMING_VCPU handler — always local, race-free.
pub fn drain_incoming_vcpus() {
    let cpu = this_cpu_data();
    let pending: Vec<Arc<VCpu>> = {
        let mut q = cpu.incoming_vcpus.lock();
        q.drain(..).collect()
    };
    for vcpu in pending {
        let vcpu_id = vcpu.id;
        cpu.scheduler.remove_blocked(vcpu_id);
        cpu.scheduler.enqueue(vcpu);
    }
}

/// Drain this pCPU's pending_wake_ids queue.
/// For each PendingWake: inject the IRQ into the VCpu's pending_virqs,
/// remove from blocked list, and enqueue.
/// Called by IPI_EVENT_RESCHED handler — always local, race-free.
pub fn drain_pending_wake_ids() {
    let cpu = this_cpu_data();
    let wakes: Vec<crate::cpu_data::PendingWake> = {
        let mut q = cpu.pending_wake_ids.lock();
        q.drain(..).collect()
    };
    for wake in wakes {
        if let Some(vcpu) = cpu.scheduler.find_blocked(wake.vcpu_id) {
            vcpu.push_pending_irq(wake.irq_id, wake.is_hardware);
            if vcpu.transition(VCpuState::Blocked, VCpuState::Ready).is_ok() {
                cpu.scheduler.remove_blocked(wake.vcpu_id);
                cpu.scheduler.enqueue(vcpu);
            }
        }
    }
}
