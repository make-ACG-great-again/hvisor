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
use super::{gicd::GICD_LOCK, is_spi};
use crate::platform::BOARD_MPIDR_MAPPINGS;
use crate::{
    arch::zone::{GicConfig, HvArchZoneConfig},
    config::{BitmapWord, CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD, CONFIG_MAX_INTERRUPTS},
    cpu_data::{this_cpu_data, this_zone},
    device::irqchip::gicv3::{
        gicd::*, gicr::*, gits::*, host_gicd_base, host_gicr_base, host_gits_base,
        MAINTENACE_INTERRUPT, PER_GICR_SIZE,
    },
    error::HvResult,
    hypercall::SGI_IPI_ID,
    memory::{mmio_perform_access, MMIOAccess},
    zone::{this_zone_id, Zone},
};
pub fn reg_range(base: usize, n: usize, size: usize) -> core::ops::Range<usize> {
    base..(base + n * size)
}

impl Zone {
    pub fn vgicv3_mmio_init(&self, arch: &HvArchZoneConfig) {
        match arch.gic_config {
            GicConfig::Gicv2(_) => {
                panic!("vgicv3_mmio_init: GICv2 is not supported in this function");
            }
            GicConfig::Gicv3(ref gicv3_config) => {
                if gicv3_config.gicd_base == 0 || gicv3_config.gicr_base == 0 {
                    panic!("vgicv3_mmio_init: gicd_base or gicr_base is null");
                }

                // Collect vcpu_ids sorted by local index before taking the write lock.
                // GICR MMIO offset is zone-local (gicr_base + i * PER_GICR_SIZE) so the
                // guest enumerates them starting from 0; handler arg is global vcpu_id so
                // vgicv3_redist_handler can look up the bound pCPU via get_vcpu().
                let vcpu_base = self.vcpu_base();
                let num_vcpus = self.read().vcpu_count();

                let mut inner = self.write();
                inner.mmio_region_register(
                    gicv3_config.gicd_base,
                    gicv3_config.gicd_size,
                    vgicv3_dist_handler,
                    0,
                );
                inner.mmio_region_register(
                    gicv3_config.gits_base,
                    gicv3_config.gits_size,
                    vgicv3_its_handler,
                    0,
                );

                for i in 0..num_vcpus {
                    let vcpu_id = vcpu_base + i; // global vcpu_id — handler uses it for pcpu lookup
                    let gicr_base = gicv3_config.gicr_base + i * PER_GICR_SIZE; // zone-local offset
                    debug!(
                        "Registering GIC Redistributor region for vcpu {} (global_id={}) at {:#x}",
                        i, vcpu_id, gicr_base
                    );
                    inner.mmio_region_register(
                        gicr_base,
                        PER_GICR_SIZE,
                        vgicv3_redist_handler,
                        vcpu_id,
                    );
                }
            }
        }
    }

    pub fn irq_bitmap_init(&mut self, irqs_bitmap: &[BitmapWord]) {
        let mut inner = self.write();
        for i in 0..irqs_bitmap.len() {
            let word = irqs_bitmap[i];

            for j in 0..CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD {
                if ((word >> j) & 1) == 1 {
                    let irq_id = (i * CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD + j) as u32;
                    assert!(irq_id < (CONFIG_MAX_INTERRUPTS as u32));
                    let irq_index = irq_id / (CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD as u32);
                    let irq_bit = irq_id % (CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD as u32);
                    inner.irq_bitmap_mut()[irq_index as usize] |= 1 << irq_bit;
                }
            }
        }

        for (index, &word) in inner.irq_bitmap().iter().enumerate() {
            for bit_position in 0..CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD {
                if word & (1 << bit_position) != 0 {
                    let interrupt_number =
                        index * CONFIG_INTERRUPTS_BITMAP_BITS_PER_WORD + bit_position;
                    info!(
                        "Found interrupt in Zone {} irq_bitmap: {}",
                        self.id(),
                        interrupt_number
                    );
                }
            }
        }
    }
}

fn restrict_bitmask_access(
    mmio: &mut MMIOAccess,
    reg_index: usize,
    bits_per_irq: usize,
    is_poke: bool,
    gicd_base: usize,
) -> HvResult {
    let zone = this_zone();
    let zone_r = zone.read();
    let mut access_mask: usize = 0;
    /*
     * In order to avoid division, the number of bits per irq is limited
     * to powers of 2 for the moment.
     */
    let irqs_per_reg = 32 / bits_per_irq;
    let irq_bits = (1 << bits_per_irq) - 1;
    /* First, extract the first interrupt affected by this access */
    let first_irq = reg_index * irqs_per_reg;

    for irq in 0..irqs_per_reg {
        if zone_r.irq_in_zone((first_irq + irq) as _) {
            trace!("restrict visit irq {}", first_irq + irq);
            access_mask |= irq_bits << (irq * bits_per_irq);
        }
    }

    if !mmio.is_write {
        /* Restrict the read value */
        mmio_perform_access(gicd_base, mmio);
        mmio.value &= access_mask;
        return Ok(());
    }

    if !is_poke {
        /*
         * Modify the existing value of this register by first reading
         * it into mmio->value
         * Relies on a spinlock since we need two mmio accesses.
         */
        let access_val = mmio.value;

        let _lock = GICD_LOCK.lock();

        mmio.is_write = false;
        mmio_perform_access(gicd_base, mmio);

        mmio.is_write = true;
        mmio.value &= !access_mask;
        mmio.value |= access_val & access_mask;
        mmio_perform_access(gicd_base, mmio);

        // drop lock automatically here
    } else {
        mmio.value &= access_mask;
        mmio_perform_access(gicd_base, mmio);
    }
    Ok(())
}

/// Check if the given vcpu_id belongs to the current zone.
fn is_same_zone(vcpu_id: usize) -> bool {
    let zone = this_zone();
    let zone_lock = zone.read();
    zone_lock.get_vcpu(vcpu_id).is_some()
}

/// Handle SGI/PPI register access via the per-vCPU virtual GICR shadow state.
/// Called when the target vCPU is not currently running on its bound pCPU,
/// or to keep shadow in sync when it is running.

fn vgicr_shadow_access(mmio: &mut MMIOAccess, vcpu_id: usize, reg: usize) {
    let zone = this_zone();
    let zone_lock = zone.read();
    let vcpu = match zone_lock.get_vcpu(vcpu_id) {
        Some(v) => v,
        None => {
            warn!("vgicr_shadow_access: vcpu {} not found", vcpu_id);
            return;
        }
    };

    // Safety: vgicr is wrapped in UnsafeCell. Only mutated when vCPU is not
    // running on another pCPU; caller ensures this invariant.
    let vgicr = unsafe { &mut *vcpu.arch.gic_state.vgicr.get() };

    if mmio.is_write {
        vgicr.initialized = true;
    }

    match reg {
        r if r == GICR_SGI_BASE + GICR_IGROUPR => {
            if mmio.is_write {
                // Keep IRQ 26 (EL2 physical timer) in Group 1 regardless of
                // what the guest writes. Linux sets IGROUPR0=0 during init
                // which would move IRQ 26 to Group 0 and prevent delivery.
                vgicr.igroupr = (mmio.value as u32) | (1 << 26);
            } else {
                mmio.value = vgicr.igroupr as usize;
            }
        }
        r if r == GICR_SGI_BASE + GICR_ISENABLER => {
            if mmio.is_write { vgicr.isenabler |= mmio.value as u32; }
            else { mmio.value = vgicr.isenabler as usize; }
        }
        r if r == GICR_SGI_BASE + GICR_ICENABLER => {
            if mmio.is_write { vgicr.isenabler &= !(mmio.value as u32); }
            else { mmio.value = vgicr.isenabler as usize; }
        }
        r if r == GICR_SGI_BASE + GICR_ISPENDR => {
            if mmio.is_write { vgicr.ispendr |= mmio.value as u32; }
            else { mmio.value = vgicr.ispendr as usize; }
        }
        r if r == GICR_SGI_BASE + GICR_ICPENDR => {
            if mmio.is_write { vgicr.ispendr &= !(mmio.value as u32); }
            else { mmio.value = vgicr.ispendr as usize; }
        }
        r if r == GICR_SGI_BASE + GICR_ISACTIVER => {
            if mmio.is_write { vgicr.isactiver |= mmio.value as u32; }
            else { mmio.value = vgicr.isactiver as usize; }
        }
        r if r == GICR_SGI_BASE + GICR_ICACTIVER => {
            if mmio.is_write { vgicr.isactiver &= !(mmio.value as u32); }
            else { mmio.value = vgicr.isactiver as usize; }
        }
        r if reg_range(GICR_SGI_BASE + GICR_IPRIORITYR, 8, 4).contains(&r) => {
            let idx = (r - (GICR_SGI_BASE + GICR_IPRIORITYR)) / 4;
            if mmio.is_write { vgicr.ipriorityr[idx] = mmio.value as u32; }
            else { mmio.value = vgicr.ipriorityr[idx] as usize; }
        }
        r if reg_range(GICR_SGI_BASE + GICR_ICFGR, 2, 4).contains(&r) => {
            let idx = (r - (GICR_SGI_BASE + GICR_ICFGR)) / 4;
            if mmio.is_write { vgicr.icfgr[idx] = mmio.value as u32; }
            else { mmio.value = vgicr.icfgr[idx] as usize; }
        }
        GICR_WAKER => {
            // ProcessorSleep/ChildrenAsleep: shadow always reports awake
            if !mmio.is_write { mmio.value = 0; }
        }
        _ => {
            if !mmio.is_write { mmio.value = 0; }
        }
    }
}

pub fn vgicv3_redist_handler(mmio: &mut MMIOAccess, vcpu_id: usize) -> HvResult {
    trace!("[GICR] vcpu={} addr={:#x} is_write={} val={:#x}", vcpu_id, mmio.address, mmio.is_write, mmio.value);

    // Translate vcpu_id to the physical CPU it is bound to.
    let phys_cpu = {
        let zone = this_zone();
        let zone_lock = zone.read();
        match zone_lock.get_vcpu(vcpu_id) {
            Some(v) => v.get_pcpu_affinity(),
            None => {
                warn!("vgicv3_redist_handler: vcpu {} not found", vcpu_id);
                return HvResult::Ok(());
            }
        }
    };
    let gicr_base = host_gicr_base(phys_cpu);

    match mmio.address {
        GICR_CTLR => {
            if !is_same_zone(vcpu_id) {
                // Foreign redistributor — only allow reads, ignore writes.
                if !mmio.is_write {
                    mmio_perform_access(gicr_base, mmio);
                }
            } else {
                mmio_perform_access(gicr_base, mmio);
                if !mmio.is_write {
                    // Clear EnableLPIs (bit0) and RWP (bit1): guest must set up
                    // LPI tables itself; stale RWP from a prior zone's init would
                    // cause gicr_wait_for_rwp() to spin forever.
                    mmio.value &= !0x3;
                }
            }
            trace!("[GICR_CTLR] vcpu={} is_write={} val={:#x}", vcpu_id, mmio.is_write, mmio.value);
        }
        GICR_TYPER => {
            mmio_perform_access(gicr_base, mmio);
            let zone = this_zone();
            let zone_lock = zone.read();
            let num_vcpus = zone_lock.vcpu_count();
            let local_idx = vcpu_id.saturating_sub(zone_lock.vcpu_base());
            // Patch Processor_Number [23:8] and Affinity [63:32] to zone-local index.
            mmio.value &= !(0xFFFF << 8);
            mmio.value |= (local_idx & 0xFFFF) << 8;
            if mmio.size >= 8 {
                mmio.value &= !(0xFFFF_FFFF_usize << 32);
                mmio.value |= (local_idx & 0xFFFF_FFFF) << 32;
            }
            // Set LAST on the final vCPU so Linux stops enumerating redistributors.
            mmio.value &= !GICR_TYPER_LAST;
            if local_idx == num_vcpus - 1 {
                mmio.value |= GICR_TYPER_LAST;
            }
            trace!("[GICR_TYPER] vcpu={} local={} patched={:#x} last={}", vcpu_id, local_idx, mmio.value, local_idx == num_vcpus - 1);
        }
        // Linux may read GICR_TYPER as two 32-bit accesses; high word = Affinity.
        r if r == GICR_TYPER + 4 => {
            if !mmio.is_write {
                mmio_perform_access(gicr_base, mmio);
                let zone = this_zone();
                let zone_lock = zone.read();
                let local_idx = vcpu_id.saturating_sub(zone_lock.vcpu_base());
                mmio.value = local_idx & 0xFF;
            }
        }
        GICR_IIDR | 0xffd0..=0xfffc => {
            mmio_perform_access(gicr_base, mmio);
        }
        GICR_PENDBASER => {
            mmio_perform_access(gicr_base, mmio);
            if mmio.is_write { trace!("write pending tbl base: {:#x}", mmio.value); }
            else              { trace!("read  pending tbl base: {:#x}", mmio.value); }
        }
        GICR_PROPBASER => {
            if mmio.is_write { set_prop_baser(mmio.value); }
            else              { mmio.value = read_prop_baser(); }
        }
        GICR_SYNCR => { mmio.value = 0; }
        GICR_SETLPIR => { mmio_perform_access(gicr_base, mmio); }
        reg if reg == GICR_CLRLPIR || reg == GICR_INVALLR => {
            mmio_perform_access(gicr_base, mmio);
        }
        GICR_INVLPIR => {
            enable_one_lpi((mmio.value & 0xffffffff) - 8192);
        }
        // GICR_WAKER: route to shadow or hardware based on whether the vCPU
        // is currently running on its bound pCPU.
        GICR_WAKER => {
            let before = mmio.value;
            let zone = this_zone();
            let zone_lock = zone.read();
            let is_current_on_pcpu = if let Some(_vcpu) = zone_lock.get_vcpu(vcpu_id) {
                let cur = this_cpu_data();
                cur.id == phys_cpu && cur.current_vcpu.as_ref().map(|v| v.id) == Some(vcpu_id)
            } else {
                false
            };
            drop(zone_lock);
            if is_current_on_pcpu {
                mmio_perform_access(gicr_base, mmio);
            } else {
                vgicr_shadow_access(mmio, vcpu_id, GICR_WAKER);
            }
            trace!("[GICR_WAKER] vcpu={} phys_cpu={} is_current={} is_write={} in={:#x} out={:#x}",
                vcpu_id, phys_cpu, is_current_on_pcpu, mmio.is_write, before, mmio.value);
            return HvResult::Ok(());
        }
        // SGI/PPI region — per-vCPU virtual state.
        reg if reg == GICR_STATUSR
            || reg == GICR_SGI_BASE + GICR_IGROUPR
            || reg == GICR_SGI_BASE + GICR_ISENABLER
            || reg == GICR_SGI_BASE + GICR_ICENABLER
            || reg == GICR_SGI_BASE + GICR_ISPENDR
            || reg == GICR_SGI_BASE + GICR_ICPENDR
            || reg == GICR_SGI_BASE + GICR_ISACTIVER
            || reg == GICR_SGI_BASE + GICR_ICACTIVER
            || reg_range(GICR_SGI_BASE + GICR_IPRIORITYR, 8, 4).contains(&reg)
            || reg_range(GICR_SGI_BASE + GICR_ICFGR, 2, 4).contains(&reg) =>
        {
            if !is_same_zone(vcpu_id) {
                trace!("gicr: ignore access to foreign redistributor vcpu={}", vcpu_id);
                return HvResult::Ok(());
            }

            // Protect hypervisor-owned PPIs/SGIs from being disabled by guest.
            if reg == GICR_SGI_BASE + GICR_ICENABLER && mmio.is_write {
                mmio.value &= !(1 << MAINTENACE_INTERRUPT);
                mmio.value &= !(1 << SGI_IPI_ID);
                mmio.value &= !(1 << 26); // EL2 timer
            }

            // Determine if this vCPU is currently running on its bound pCPU.
            let zone = this_zone();
            let zone_lock = zone.read();
            let is_current_on_pcpu = if let Some(_vcpu) = zone_lock.get_vcpu(vcpu_id) {
                let cur = this_cpu_data();
                cur.id == phys_cpu && cur.current_vcpu.as_ref().map(|v| v.id) == Some(vcpu_id)
            } else {
                false
            };
            drop(zone_lock);

            if is_current_on_pcpu {
                // vCPU is running — update shadow first (applies all filters,
                // e.g. IGROUPR forces bit 26=1), then forward the filtered
                // value to hardware so the physical GICR matches the shadow.
                // Doing it this way ensures hvisor-owned PPIs are never
                // disturbed by guest writes even while the vCPU is live.
                vgicr_shadow_access(mmio, vcpu_id, reg);
                // For write accesses, forward the filtered value to hardware.
                // IGROUPR: read shadow (has IRQ26 bit forced) back into mmio.value.
                // ICENABLER: mmio.value is already filtered (bit26/25/IPI cleared above).
                // ISENABLER: mmio.value is the guest's set-bits; shadow OR'd bit26 in,
                //   read back so we also set bit26 in hardware if shadow added it.
                // Other regs: read back shadow to avoid forwarding partially-wrong values.
                if mmio.is_write && reg != GICR_SGI_BASE + GICR_ICENABLER {
                    mmio.is_write = false;
                    vgicr_shadow_access(mmio, vcpu_id, reg); // shadow → mmio.value
                    mmio.is_write = true;
                }
                // For ICENABLER writes: filter out hypervisor-owned IRQs so
                // guest cannot disable the EL2 timer (26), GIC maintenance (25),
                // or the hypervisor IPI SGI.
                if mmio.is_write && reg == GICR_SGI_BASE + GICR_ICENABLER {
                    // Prevent guest from disabling hypervisor-owned PPIs/SGIs:
                    // IRQ 26 (CNTHP, EL2 physical timer), IRQ 25 (maintenance), SGI_IPI_ID.
                    let hv_mask = (1u32 << 26) | (1u32 << MAINTENACE_INTERRUPT) | (1u32 << SGI_IPI_ID);
                    mmio.value &= !(hv_mask as usize);
                }
                mmio_perform_access(gicr_base, mmio);
            } else {
                // vCPU not running — shadow only, do not touch hardware.
                vgicr_shadow_access(mmio, vcpu_id, reg);
            }
        }
        _ => {
            info!("[GICR-UNHANDLED] vcpu={} addr={:#x} is_write={} val={:#x}",
                vcpu_id, mmio.address, mmio.is_write, mmio.value);
        }
    }
    HvResult::Ok(())
}

// The return value should be the register value to be read.
fn vgicv3_handle_irq_ops(mmio: &mut MMIOAccess, irq: u32) -> HvResult {
    use crate::zone::GuestMpidr;

    let zone = this_zone();
    let zone_r = zone.read();

    if !is_spi(irq) || !zone_r.irq_in_zone(irq) {
        debug!(
            "gicd-mmio: skip irq {} access, reg = {:#x?}",
            irq, mmio.address
        );
        return Ok(());
    }

    // Intercept GICD_IROUTER writes: record IRQ→VCPU mapping and translate
    // guest MPIDR to host pCPU MPIDR before forwarding to hardware.
    let reg = mmio.address;
    if mmio.is_write && reg >= GICD_IROUTER && reg < GICD_IROUTER + 1024 * 8 {
        let guest_mpidr_val = mmio.value as u64;

        // Resolve guest MPIDR to a VCPU id within this zone.
        // Bit 31 = Interrupt_Routing_Mode: 1 means "any PE", no specific target.
        let target_vcpu_id: Option<usize> = if (guest_mpidr_val & (1 << 31)) != 0 {
            None // ANY routing
        } else {
            let guest_mpidr = GuestMpidr(guest_mpidr_val & 0xFF_00FF_FFFF);
            zone_r
                .get_vcpu_by_guest_mpidr(guest_mpidr)
                .map(|v| v.id)
        };

        // Translate guest MPIDR → host pCPU affinity for the physical IROUTER.
        let host_mpidr: u64 = if let Some(tid) = target_vcpu_id {
            if let Some(vcpu) = zone_r.vcpus().get(&tid) {
                vcpu.get_pcpu_affinity() as u64
            } else {
                guest_mpidr_val
            }
        } else {
            guest_mpidr_val // ANY: passthrough as-is
        };

        drop(zone_r);
        zone.write().irq_target_vcpu[irq as usize] = target_vcpu_id;

        trace!(
            "IROUTER write: IRQ {} -> guest_mpidr {:#x} -> vcpu {:?} -> host_mpidr {:#x}",
            irq, guest_mpidr_val, target_vcpu_id, host_mpidr
        );

        let orig_value = mmio.value;
        mmio.value = host_mpidr as usize;
        mmio_perform_access(host_gicd_base(), mmio);
        mmio.value = orig_value;
    } else {
        mmio_perform_access(host_gicd_base(), mmio);
    }

    Ok(())
}

fn vgicv3_dist_misc_access(mmio: &mut MMIOAccess, gicd_base: usize) -> HvResult {
    let reg = mmio.address;
    if reg_range(GICDV3_PIDR0, 4, 4).contains(&reg)
        || reg_range(GICDV3_PIDR4, 4, 4).contains(&reg)
        || reg_range(GICDV3_CIDR0, 4, 4).contains(&reg)
        || reg == GICD_CTLR
        || reg == GICD_TYPER
        || reg == GICD_IIDR
        || reg == GICD_TYPER2
    {
        if !mmio.is_write {
            // ignore write
            mmio_perform_access(gicd_base, mmio);
        }
    } else {
        todo!("vgicv3_dist_misc_access: MMIO.Address = {:#x?}", reg)
    }

    Ok(())
}

pub fn vgicv3_dist_handler(mmio: &mut MMIOAccess, _arg: usize) -> HvResult {
    trace!("gicd mmio = {:#x?}", mmio);
    let gicd_base = host_gicd_base();
    let reg = mmio.address;

    match reg {
        reg if reg_range(GICD_IROUTER, 1024, 8).contains(&reg) => {
            vgicv3_handle_irq_ops(mmio, (reg - GICD_IROUTER) as u32 / 8)
        }
        reg if reg_range(GICD_ITARGETSR, 1024, 1).contains(&reg) => {
            vgicv3_handle_irq_ops(mmio, (reg - GICD_ITARGETSR) as u32)
        }
        reg if reg_range(GICD_ICENABLER, 32, 4).contains(&reg)
            || reg_range(GICD_ISENABLER, 32, 4).contains(&reg)
            || reg_range(GICD_ICPENDR, 32, 4).contains(&reg)
            || reg_range(GICD_ISPENDR, 32, 4).contains(&reg)
            || reg_range(GICD_ICACTIVER, 32, 4).contains(&reg)
            || reg_range(GICD_ISACTIVER, 32, 4).contains(&reg) =>
        {
            restrict_bitmask_access(mmio, (reg & 0x7f) / 4, 1, true, gicd_base)
        }
        reg if reg_range(GICD_IGROUPR, 32, 4).contains(&reg) => {
            restrict_bitmask_access(mmio, (reg & 0x7f) / 4, 1, false, gicd_base)
        }
        reg if reg_range(GICD_ICFGR, 64, 4).contains(&reg) => {
            restrict_bitmask_access(mmio, (reg & 0xff) / 4, 2, false, gicd_base)
        }
        reg if reg_range(GICD_IPRIORITYR, 255, 4).contains(&reg) => {
            restrict_bitmask_access(mmio, (reg & 0x3ff) / 4, 8, false, gicd_base)
        }
        reg if reg_range(GICD_IGRPMODR, 32, 4).contains(&reg) => {
            // GICD_IGRPMODR is not supported in hvisor because it is used for secure state.
            warn!(
                "GICD_IGRPMODR is not supported in hvisor, reg = {:#x?}",
                reg
            );
            Ok(())
        }
        _ => vgicv3_dist_misc_access(mmio, gicd_base),
    }
}

pub fn vgicv3_its_handler(mmio: &mut MMIOAccess, _arg: usize) -> HvResult {
    let gits_base = host_gits_base();
    let reg = mmio.address;
    let zone_id = this_zone_id();

    match reg {
        GITS_CTRL => {
            mmio_perform_access(gits_base, mmio);
        }
        GITS_CBASER => {
            if mmio.is_write {
                set_cbaser(mmio.value, zone_id);
            } else {
                mmio.value = read_cbaser(zone_id);
            }
        }
        // v_dt_addr + 0x10000000;
        GITS_BASER => {
            if mmio.is_write {
                set_dt_baser(mmio.value, zone_id);
                if zone_id == 0 {
                    let v_dt_addr = mmio.value & 0xfff_fff_fff_000usize;
                    let phys_dt_trans =
                        unsafe { this_zone().read().gpm().page_table_query(v_dt_addr) };
                    match phys_dt_trans {
                        Ok(p) => {
                            mmio.value &= !0xfff_fff_fff_000usize;
                            mmio.value |= p.0 as usize;
                        }
                        _ => {}
                    }
                    mmio_perform_access(gits_base, mmio);
                }
            } else {
                mmio.value = read_dt_baser(zone_id);
            }
        }
        GITS_COLLECTION_BASER => {
            if mmio.is_write {
                set_ct_baser(mmio.value, zone_id);
                if zone_id == 0 {
                    let v_ct_addr = mmio.value & 0xfff_fff_fff_000usize;
                    let phys_ct_trans =
                        unsafe { this_zone().read().gpm().page_table_query(v_ct_addr) };
                    match phys_ct_trans {
                        Ok(p) => {
                            mmio.value &= !0xfff_fff_fff_000usize;
                            mmio.value |= p.0 as usize;
                        }
                        _ => {}
                    }
                    mmio_perform_access(gits_base, mmio);
                }
            } else {
                mmio.value = read_ct_baser(zone_id);
            }
        }
        GITS_CWRITER => {
            if mmio.is_write {
                set_cwriter(mmio.value, zone_id);
            } else {
                mmio.value = read_cwriter(zone_id);
            }
        }
        GITS_CREADR => {
            mmio.value = read_creadr(zone_id);
        }
        GITS_TYPER => {
            mmio_perform_access(gits_base, mmio);
        }
        _ => {
            mmio_perform_access(gits_base, mmio);
            if mmio.is_write {
                debug!(
                    "write GITS offset: {:#x}, 0x{:016x}",
                    mmio.address, mmio.value
                );
            } else {
                debug!(
                    "read GITS offset: {:#x}, 0x{:016x}",
                    mmio.address, mmio.value
                );
            }
        }
    }
    Ok(())
}
