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
#![allow(unused)]
use crate::{
    arch::ipi::{arch_check_events, arch_prepare_send_event, arch_send_event},
    consts::{
        IPI_EVENT_CLEAR_INJECT_IRQ, IPI_EVENT_SEND_IPI, IPI_EVENT_UPDATE_HART_LINE, MAX_CPU_NUM,
    },
    cpu_data::this_cpu_data,
    device::{irqchip::inject_irq, virtio_trampoline::handle_virtio_irq},
    platform::IRQ_WAKEUP_VIRTIO_DEVICE,
};
use alloc::{collections::VecDeque, vec::Vec};
use spin::Mutex;

pub const IPI_EVENT_WAKEUP: usize = 0;
pub const IPI_EVENT_SHUTDOWN: usize = 1;
pub const IPI_EVENT_VIRTIO_INJECT_IRQ: usize = 2;
pub const IPI_EVENT_WAKEUP_VIRTIO_DEVICE: usize = 3;
/// Tell target pCPU to wake blocked vCPUs with pending IRQs + schedule().
pub const IPI_EVENT_RESCHED: usize = 7;
/// New vCPU arrived in incoming_vcpus queue; drain and schedule.
pub const IPI_EVENT_INCOMING_VCPU: usize = 8;
/// Zone is being destroyed: pCPU must clear its scheduler and re-enter idle loop.
pub const IPI_EVENT_ZONE_SHUTDOWN: usize = 9;

#[percpu::def_percpu]
static PERCPU_EVENTS: Mutex<VecDeque<usize>> = Mutex::new(VecDeque::new());

// The caller ensures the cpu_id is valid
#[inline(always)]
fn get_percpu_events(cpu: usize) -> &'static Mutex<VecDeque<usize>> {
    unsafe { PERCPU_EVENTS.remote_ref_raw(cpu) }
}

fn add_event(cpu: usize, event_id: usize) -> Option<()> {
    if cpu >= MAX_CPU_NUM {
        return None;
    }
    let mut e = get_percpu_events(cpu).lock();
    if event_id == IPI_EVENT_SHUTDOWN {
        // If the event is shutdown, we need to clear all previous events, because shutdown will make cpu idle and won't process any events.
        e.clear();
    }
    e.push_back(event_id);
    Some(())
}

pub fn fetch_event(cpu: usize) -> Option<usize> {
    if cpu >= MAX_CPU_NUM {
        return None;
    }
    get_percpu_events(cpu).lock().pop_front()
}

pub fn dump_events() {
    for cpu in 0..MAX_CPU_NUM {
        let events = get_percpu_events(cpu).lock();
        if !events.is_empty() {
            debug!("cpu {} events: {:?}", cpu, *events);
        }
    }
}

pub fn dump_cpu_events(cpu: usize) -> Vec<usize> {
    if cpu >= MAX_CPU_NUM {
        return Vec::new();
    }
    get_percpu_events(cpu).lock().iter().cloned().collect()
}

pub fn clear_events(cpu: usize) {
    if cpu >= MAX_CPU_NUM {
        return;
    }
    get_percpu_events(cpu).lock().clear();
}

pub fn check_events() -> bool {
    let cpu_data = this_cpu_data();
    let mut handled = false;
    let mut drained: u32 = 0;
    loop {
        let event = fetch_event(cpu_data.id);
        match event {
            None => break,
            Some(IPI_EVENT_WAKEUP) => {
                cpu_data.arch_cpu.run();
            }
            Some(IPI_EVENT_SHUTDOWN) => {
                cpu_data.arch_cpu.idle();
            }
            Some(IPI_EVENT_VIRTIO_INJECT_IRQ) => {
                handle_virtio_irq();
                handled = true;
            }
            Some(IPI_EVENT_WAKEUP_VIRTIO_DEVICE) => {
                #[cfg(all(feature = "gicv3", target_arch = "aarch64"))]
                crate::device::irqchip::gicv3::schedule_inject_irq(
                    IRQ_WAKEUP_VIRTIO_DEVICE,
                    false,
                );
                #[cfg(not(all(feature = "gicv3", target_arch = "aarch64")))]
                inject_irq(IRQ_WAKEUP_VIRTIO_DEVICE, false);
                handled = true;
            }
            Some(IPI_EVENT_ZONE_SHUTDOWN) => {
                // Zone is being destroyed. Clear all vCPUs from this pCPU's scheduler
                // (safe here — we are in EL2, guest is not running).
                // Also wipe physical timer state so a stale CNTV/CNTP comparator
                // does not keep asserting IRQ 26/27 to a pCPU that has no
                // current_vcpu (would otherwise produce an IRQ storm).
                info!("[SHUTDOWN] pcpu{} received IPI_EVENT_ZONE_SHUTDOWN, calling clear_all+idle", cpu_data.id);
                cpu_data.scheduler.clear_all();
                cpu_data.current_vcpu = None;
                #[cfg(target_arch = "aarch64")]
                {
                    use crate::arch::sysreg::write_sysreg;
                    write_sysreg!(CNTV_CTL_EL0, 0u64);
                    write_sysreg!(CNTV_CVAL_EL0, 0u64);
                    write_sysreg!(CNTVOFF_EL2, 0u64);
                    write_sysreg!(CNTP_CTL_EL0, 0u64);
                    write_sysreg!(CNTP_CVAL_EL0, 0u64);
                }
                cpu_data.arch_cpu.idle();
            }
            Some(IPI_EVENT_RESCHED) => {
                // Wake any locally-blocked vCPUs that now have pending IRQs.
                let woken = cpu_data.scheduler.drain_pending_irq_wakeups();
                if woken > 0 {
                    cpu_data
                        .need_resched
                        .store(true, core::sync::atomic::Ordering::Release);
                }
                handled = true;
            }
            Some(IPI_EVENT_INCOMING_VCPU) => {
                crate::vcpu::drain_incoming_vcpus();
                cpu_data
                    .need_resched
                    .store(true, core::sync::atomic::Ordering::Release);
                handled = true;
            }
            Some(ev @ IPI_EVENT_CLEAR_INJECT_IRQ)
            | Some(ev @ IPI_EVENT_UPDATE_HART_LINE)
            | Some(ev @ IPI_EVENT_SEND_IPI) => {
                arch_check_events(Some(ev));
                handled = true;
            }
            Some(_) => {
                // Unknown event id — drop it silently to avoid stalling the queue.
            }
        }
        drained = drained.saturating_add(1);
    }
    // [EVT-BURST] rate-limited trace: emit on new max-per-pCPU and every Nth burst.
    #[cfg(feature = "vcpu_debug_trace")]
    if drained > 1 {
        use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
        const MAX_PCPUS: usize = MAX_CPU_NUM;
        static MAX_BURST: [AtomicU32; MAX_PCPUS] = {
            const Z: AtomicU32 = AtomicU32::new(0);
            [Z; MAX_PCPUS]
        };
        static BURST_COUNT: [AtomicU64; MAX_PCPUS] = {
            const Z: AtomicU64 = AtomicU64::new(0);
            [Z; MAX_PCPUS]
        };
        let id = cpu_data.id.min(MAX_PCPUS - 1);
        let cur_max = MAX_BURST[id].load(Ordering::Relaxed);
        let new_max = drained > cur_max;
        if new_max {
            MAX_BURST[id].store(drained, Ordering::Relaxed);
        }
        let n = BURST_COUNT[id].fetch_add(1, Ordering::Relaxed) + 1;
        if new_max || n % 4096 == 0 {
            trace!(
                "[EVT-BURST] pcpu={} drained={} max={} total_bursts={}",
                cpu_data.id,
                drained,
                MAX_BURST[id].load(Ordering::Relaxed),
                n
            );
        }
    }
    let _ = drained;
    handled
}

pub fn send_event(cpu_id: usize, ipi_int_id: usize, event_id: usize) {
    // #[cfg(target_arch = "loongarch64")]
    // {
    //     // block until the previous event is processed, which means
    //     // the target queue is empty
    //     while !fetch_event(cpu_id).is_none() {}
    //     debug!(
    //         "loongarch64:: send_event: cpu_id: {}, ipi_int_id: {}, event_id: {}",
    //         cpu_id, ipi_int_id, event_id
    //     );
    // }
    /// Some arch need do something before send event.
    /// Currently, we are not passing parameters, and we will modify the function signature later as needed.
    arch_prepare_send_event(cpu_id, ipi_int_id, event_id);
    add_event(cpu_id, event_id);
    arch_send_event(cpu_id as _, ipi_int_id as _);
}
