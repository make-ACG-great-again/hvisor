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

/// EL2 Physical Timer (CNTHP) for scheduling ticks.
///
/// Uses CNTHP_TVAL_EL2 and CNTHP_CTL_EL2 to generate periodic EL2 timer interrupts
/// (IRQ ID 26) that drive the scheduler's time slice mechanism.

use crate::arch::sysreg::{read_sysreg, write_sysreg};

/// Default scheduling tick period in microseconds (10ms).
pub const SCHED_TICK_PERIOD_US: u64 = 10_000;

/// EL2 physical timer interrupt ID (CNTHP, PPI #26).
pub const EL2_TIMER_IRQ: usize = 26;

/// Cached tick value in counter ticks (computed once from CNTFRQ_EL0).
static mut TICK_TVAL: u64 = 0;

/// Read the current physical counter value (CNTPCT_EL0).
#[inline(always)]
pub fn current_cntpct() -> u64 {
    read_sysreg!(CNTPCT_EL0)
}

/// Return the tick period in counter ticks (same value as CNTHP_TVAL).
/// Used by the scheduler to compute "one tick from now" for blocked VCPU wakeup.
#[inline(always)]
pub fn tick_period_cnt() -> u64 {
    unsafe { TICK_TVAL }
}

/// Initialize the EL2 physical timer.
///
/// Reads CNTFRQ_EL0 to compute the counter value corresponding to `tick_period_us`,
/// then arms the timer.
pub fn el2_timer_init(tick_period_us: u64) {
    let freq = read_sysreg!(CNTFRQ_EL0);
    let tval = freq * tick_period_us / 1_000_000;
    unsafe {
        TICK_TVAL = tval;
    }

    info!(
        "EL2 timer init: freq={} Hz, tick_period={}us, tval={}",
        freq, tick_period_us, tval
    );

    // Enable PPI 26 (EL2 Physical Timer) in the GIC Redistributor.
    {
        use crate::device::irqchip::gicv3::host_gicr_base;
        let cpu_id = crate::cpu_data::this_cpu_data().id;
        let gicr_sgi_base = host_gicr_base(cpu_id) + 0x10000; // SGI_BASE offset
        let isenabler0 = gicr_sgi_base + 0x100; // GICR_ISENABLER0
        unsafe {
            // Set bit 26 to enable PPI 26 (EL2 Physical Timer)
            core::ptr::write_volatile(isenabler0 as *mut u32, 1u32 << 26);
        }
        // Set priority for IRQ 26
        let ipriorityr6 = gicr_sgi_base + 0x400 + 26; // byte-addressable
        unsafe {
            core::ptr::write_volatile(ipriorityr6 as *mut u8, 0xa0); // priority 0xa0
        }
    }

    // Set timer value and enable
    write_sysreg!(CNTHP_TVAL_EL2, tval);
    // CNTHP_CTL_EL2: ENABLE=1, IMASK=0
    write_sysreg!(CNTHP_CTL_EL2, 1u64);
}

/// Re-arm the EL2 timer for the next tick.
pub fn el2_timer_rearm() {
    let tval = unsafe { TICK_TVAL };
    write_sysreg!(CNTHP_TVAL_EL2, tval);
    write_sysreg!(CNTHP_CTL_EL2, 1u64);
}

/// Disable the EL2 timer (e.g., when entering idle with no blocked VCPUs).
pub fn el2_timer_disable() {
    write_sysreg!(CNTHP_CTL_EL2, 0u64);
}

/// Arm the EL2 timer to fire at a specific physical counter value (CNTPCT).
/// Used by the idle loop to sleep exactly until the earliest blocked VCPU's
/// virtual timer expires.
pub fn el2_timer_arm_at(target_cntpct: u64) {
    let now = read_sysreg!(CNTPCT_EL0);
    let tval = unsafe { TICK_TVAL };

    let delta = if target_cntpct > now {
        (target_cntpct - now).min(tval)
    } else {
        1
    };

    write_sysreg!(CNTHP_TVAL_EL2, delta);
    write_sysreg!(CNTHP_CTL_EL2, 1u64);
}

/// Scheduling tick handler.
///
/// Called when IRQ 26 (CNTHP) fires. Responsibilities:
/// 1. Virtual timer proxy for blocked/ready VCPUs.
/// 2. Decrement current VCPU's time slice; if expired, set `need_resched`.
pub fn sched_tick_handler() {
    use crate::cpu_data::this_cpu_data;
    use core::sync::atomic::{AtomicU64, Ordering};

    // Diagnostic: print tick count every 100 ticks per pCPU to confirm EL2 timer is alive.
    static TICK_COUNT: [AtomicU64; 4] = [
        AtomicU64::new(0), AtomicU64::new(0),
        AtomicU64::new(0), AtomicU64::new(0),
    ];

    let cpu = this_cpu_data();

    let tick_n = TICK_COUNT[cpu.id.min(3)].fetch_add(1, Ordering::Relaxed);
    if tick_n % 10000 == 0 {
        info!(
            "[TICK] pcpu={} tick={} vcpu={:?} sched_cur={:?} rq={} blocked={} slice={}",
            cpu.id,
            tick_n,
            cpu.current_vcpu.as_ref().map(|v| v.id),
            cpu.scheduler.current.as_ref().map(|v| v.id),
            cpu.scheduler.len(),
            cpu.scheduler.blocked_vcpu_count(),
            cpu.scheduler.time_slice_remaining,
        );
    }

    let current_cnt = read_sysreg!(CNTPCT_EL0);

    // Step 1: Check blocked VCPUs' virtual timers
    let blocked_before = cpu.scheduler.blocked_vcpu_count();
    let woken = cpu.scheduler.check_blocked_timers(current_cnt);
    if woken > 0 {
        cpu.need_resched.store(true, Ordering::Release);
        trace!("[TICK] pcpu={} woke {} blocked vcpus (was {})", cpu.id, woken, blocked_before);
    } else if blocked_before > 0 {
        // Blocked vCPUs exist but none woken — log first entry for diagnosis
        if let Some((cval, ctl, cntvoff, blocked_at)) = cpu.scheduler.first_blocked_timer_info() {
            let virtual_cnt = current_cnt.wrapping_sub(cntvoff);
            trace!("[TICK] pcpu={} {} blocked vcpu(s) not woken: cval={:#x} virt_cnt={:#x} ctl={:#x} blocked_at={:#x} now={:#x}",
                cpu.id, blocked_before, cval, virtual_cnt, ctl, blocked_at, current_cnt);
        }
    }

    // Step 2: Check ready VCPUs' virtual timers (1:N overcommit)
    cpu.scheduler.check_ready_timers(current_cnt);

    // Step 3: Check the currently running vCPU's hardware CNTV ISTATUS.
    //
    // When restore_to_hardware sets IMASK=1 on an already-expired CNTV, the physical
    // IRQ 27 signal is suppressed. The running vCPU will never receive IRQ 27 via the
    // hardware EL1-IRQ path. We must inject it here via the software (HW=0) LR path.
    //
    // IMPORTANT: only inject when IMASK=1. If IMASK=0, the physical IRQ 27 is already
    // unmasked and will arrive via the normal EL1-IRQ path as HW=1. Injecting HW=0
    // on top of a pending HW=1 creates a conflict: guest EOI clears the virtual side
    // but leaves the physical Active state set, causing an IRQ 27 storm.
    {
        let cntv_ctl: u64 = read_sysreg!(CNTV_CTL_EL0);
        let timer_enabled = (cntv_ctl & 1) != 0;
        let timer_masked  = (cntv_ctl & 2) != 0; // IMASK: set by restore_to_hardware
        let timer_expired = (cntv_ctl & 4) != 0; // ISTATUS bit
        // Only proxy IRQ 27 when IMASK=1 suppresses the physical delivery.
        // Push to pending_virqs rather than calling inject_irq() directly:
        // vcpu_vmreturn() is the single inject point and also sets IMASK=1 before
        // writing the HW=0 LR, preventing the physical IRQ 27 from re-firing
        // before the guest consumes the LR (which caused the Pending-conflict warns).
        if timer_enabled && timer_masked && timer_expired {
            if let Some(ref vcpu) = cpu.scheduler.current {
                vcpu.push_pending_irq(27, false);
            }
        }
    }

    // Time slice management
    if cpu.scheduler.current.is_none() {
        el2_timer_rearm();
        return;
    }

    if cpu.scheduler.time_slice_remaining > 0 {
        cpu.scheduler.time_slice_remaining -= 1;
    }

    if cpu.scheduler.time_slice_remaining == 0 {
        cpu.need_resched.store(true, Ordering::Release);
    }

    el2_timer_rearm();
}
