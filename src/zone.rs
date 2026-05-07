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
use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec::Vec;
// use psci::error::INVALID_ADDRESS;
use crate::consts::{INVALID_ADDRESS, MAX_CPU_NUM};
use crate::pci::pci_struct::VirtualRootComplex;
use spin::{RwLock, RwLockReadGuard, RwLockWriteGuard};

// ========================
// GuestMpidr
// ========================

/// Guest-visible virtual MPIDR value (zone-local, starts from 0).
///
/// This is distinct from the physical MPIDR (MPIDR_EL1) and the pCPU index.
/// PSCI CPU_ON passes a guest MPIDR — we must look it up via `guest_mpidr_to_vcpu`,
/// NOT via `mpidr_to_cpuid()` which resolves physical MPIDRs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct GuestMpidr(pub u64);

impl GuestMpidr {
    /// Create from a raw MPIDR value, masking to affinity fields only.
    pub fn new(raw: u64) -> Self {
        // Mask: Aff3[39:32], Aff2[23:16], Aff1[15:8], Aff0[7:0]
        Self(raw & 0x00_00FF_00FF_FF_FF)
    }
}

#[cfg(feature = "dwc_pcie")]
use crate::pci::{config_accessors::dwc_atu::AtuConfig, PciConfigAddress};
#[cfg(feature = "dwc_pcie")]
use alloc::collections::btree_map::BTreeMap;

use crate::arch::mm::new_s2_memory_set;
use crate::arch::s2pt::Stage2PageTable;
use crate::config::{HvZoneConfig, CONFIG_NAME_MAXLEN};

use crate::cpu_data::{get_cpu_data, this_zone, CpuSet};
use crate::error::HvResult;
use crate::memory::addr::GuestPhysAddr;
use crate::memory::{MMIOConfig, MMIOHandler, MMIORegion, MemorySet};
use core::panic;
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(feature = "dwc_pcie")]
#[derive(Debug)]
pub struct VirtualAtuConfigs {
    ecam_to_atu: BTreeMap<usize, AtuConfig>,
    io_base_to_ecam: BTreeMap<PciConfigAddress, usize>,
    cfg_base_to_ecam: BTreeMap<PciConfigAddress, usize>,
}

#[cfg(feature = "dwc_pcie")]
impl VirtualAtuConfigs {
    pub fn new() -> Self {
        Self {
            ecam_to_atu: BTreeMap::new(),
            io_base_to_ecam: BTreeMap::new(),
            cfg_base_to_ecam: BTreeMap::new(),
        }
    }

    pub fn get_atu_by_ecam(&self, ecam_base: usize) -> Option<&AtuConfig> {
        self.ecam_to_atu.get(&ecam_base)
    }

    pub fn get_atu_by_ecam_mut(&mut self, ecam_base: usize) -> Option<&mut AtuConfig> {
        self.ecam_to_atu.get_mut(&ecam_base)
    }

    pub fn insert_atu(&mut self, ecam_base: usize, atu: AtuConfig) -> Option<AtuConfig> {
        self.ecam_to_atu.insert(ecam_base, atu)
    }

    pub fn get_or_insert_atu<F>(&mut self, ecam_base: usize, f: F) -> &mut AtuConfig
    where
        F: FnOnce() -> AtuConfig,
    {
        self.ecam_to_atu.entry(ecam_base).or_insert_with(f)
    }

    pub fn get_atu_by_io_base(&self, io_base: PciConfigAddress) -> Option<&AtuConfig> {
        let ecam = self.io_base_to_ecam.get(&io_base);
        if let Some(ecam) = ecam {
            self.get_atu_by_ecam(*ecam)
        } else {
            None
        }
    }

    pub fn get_ecam_by_io_base(&self, io_base: PciConfigAddress) -> Option<usize> {
        self.io_base_to_ecam.get(&io_base).copied()
    }

    pub fn insert_io_base_mapping(&mut self, io_base: PciConfigAddress, ecam_base: usize) {
        self.io_base_to_ecam.insert(io_base, ecam_base);
    }

    pub fn get_atu_by_cfg_base(&self, cfg_base: PciConfigAddress) -> Option<&AtuConfig> {
        let ecam = self.cfg_base_to_ecam.get(&cfg_base);
        if let Some(ecam) = ecam {
            self.get_atu_by_ecam(*ecam)
        } else {
            None
        }
    }

    pub fn get_ecam_by_cfg_base(&self, cfg_base: PciConfigAddress) -> Option<usize> {
        self.cfg_base_to_ecam.get(&cfg_base).copied()
    }

    pub fn insert_cfg_base_mapping(&mut self, cfg_base: PciConfigAddress, ecam_base: usize) {
        self.cfg_base_to_ecam.insert(cfg_base, ecam_base);
    }
}

pub struct Zone {
    name: [u8; CONFIG_NAME_MAXLEN],
    id: usize,
    is_err: AtomicBool,
    inner: RwLock<ZoneInner>,
}

pub struct ZoneInner {
    mmio: Vec<MMIOConfig>,
    cpu_num: usize,
    cpu_set: CpuSet,
    irq_bitmap: [u32; 1024 / 32],
    gpm: MemorySet<Stage2PageTable>,
    iommu_pt: Option<MemorySet<Stage2PageTable>>,
    vpci_bus: VirtualRootComplex,
    #[cfg(feature = "dwc_pcie")]
    atu_configs: VirtualAtuConfigs,
    // --- vCPU fields ---
    /// Global vCPU ID of the first vCPU in this zone.
    /// Used to compute zone-local index for VMPIDR_EL2.
    vcpu_base: usize,
    /// All vCPUs belonging to this zone, keyed by global vcpu_id.
    vcpus: BTreeMap<usize, Arc<crate::vcpu::VCpu>>,
    /// Mapping from guest-visible MPIDR to global vcpu_id.
    /// Used by PSCI CPU_ON to find the target vCPU.
    guest_mpidr_to_vcpu: BTreeMap<GuestMpidr, usize>,
}

impl Zone {
    #[allow(dead_code)]
    pub fn new(zoneid: usize, name: &[u8]) -> Self {
        Self {
            name: name.try_into().unwrap(),
            id: zoneid,
            is_err: AtomicBool::new(false),
            inner: RwLock::new(ZoneInner::new()),
        }
    }

    pub fn read(&self) -> RwLockReadGuard<'_, ZoneInner> {
        self.inner.read()
    }

    pub fn write(&self) -> RwLockWriteGuard<'_, ZoneInner> {
        self.inner.write()
    }

    pub fn id(&self) -> usize {
        self.id
    }

    pub fn name(&self) -> [u8; CONFIG_NAME_MAXLEN] {
        self.name
    }

    pub fn is_err(&self) -> bool {
        self.is_err.load(Ordering::Acquire)
    }

    pub fn set_err(&self) {
        self.is_err.store(true, Ordering::Release);
    }

    pub fn cpu_set(&self) -> CpuSet {
        self.read().cpu_set()
    }

    /// Returns the global vCPU ID of the first vCPU in this zone.
    /// Used to compute zone-local index for VMPIDR_EL2.
    pub fn vcpu_base(&self) -> usize {
        self.read().vcpu_base()
    }
}

impl ZoneInner {
    pub fn new() -> Self {
        Self {
            gpm: new_s2_memory_set(),
            mmio: Vec::new(),
            cpu_num: 0,
            cpu_set: CpuSet::new(MAX_CPU_NUM as usize, 0),
            irq_bitmap: [0; 1024 / 32],
            iommu_pt: if cfg!(feature = "iommu") {
                Some(new_s2_memory_set())
            } else {
                None
            },
            vpci_bus: VirtualRootComplex::new(),
            #[cfg(feature = "dwc_pcie")]
            atu_configs: VirtualAtuConfigs::new(),
            vcpu_base: usize::MAX,
            vcpus: BTreeMap::new(),
            guest_mpidr_to_vcpu: BTreeMap::new(),
        }
    }

    // pub fn suspend(&self) {
    //     trace!("suspending cpu_set = {:#x?}", self.cpu_set);
    //     self.cpu_set.iter_except(this_cpu_id()).for_each(|cpu_id| {
    //         trace!("try to suspend cpu_id = {:#x?}", cpu_id);
    //         suspend_cpu(cpu_id);
    //     });
    //     info!("send sgi done!");
    // }

    // pub fn resume(&self) {
    //     trace!("resuming cpu_set = {:#x?}", self.cpu_set);
    //     self.cpu_set.iter_except(this_cpu_id()).for_each(|cpu_id| {
    //         trace!("try to resume cpu_id = {:#x?}", cpu_id);
    //         resume_cpu(cpu_id);
    //     });
    // }

    // pub fn owns_cpu(&self, id: usize) -> bool {
    //     self.cpu_set.contains_cpu(id)
    // }

    /// Register a mmio region and its handler.
    pub fn mmio_region_register(
        &mut self,
        start: GuestPhysAddr,
        size: usize,
        handler: MMIOHandler,
        arg: usize,
    ) {
        if let Some(mmio) = self.mmio.iter_mut().find(|mmio| mmio.region.start == start) {
            warn!("duplicated mmio region {:#x?}", mmio);
            if mmio.region.size != size {
                error!("duplicated mmio region size not match, PLEASE CHECK!!!");
            }
            mmio.handler = handler;
            mmio.arg = arg;
        } else {
            self.mmio.push(MMIOConfig {
                region: MMIORegion { start, size },
                handler,
                arg,
            })
        }
    }
    #[allow(dead_code)]
    /// Remove the mmio region beginning at `start`.
    pub fn mmio_region_remove(&mut self, start: GuestPhysAddr) {
        if let Some((idx, _)) = self
            .mmio
            .iter()
            .enumerate()
            .find(|(_, mmio)| mmio.region.start == start)
        {
            self.mmio.remove(idx);
        }
    }
    /// Find the mmio region contains (addr..addr+size).
    pub fn find_mmio_region(
        &self,
        addr: GuestPhysAddr,
        size: usize,
    ) -> Option<(MMIORegion, MMIOHandler, usize)> {
        self.mmio
            .iter()
            .find(|cfg| cfg.region.contains_region(addr, size))
            .map(|cfg| (cfg.region, cfg.handler, cfg.arg))
    }
    /// If irq_id belongs to this zone
    pub fn irq_in_zone(&self, irq_id: u32) -> bool {
        let idx = (irq_id / 32) as usize;
        let bit_pos = (irq_id % 32) as usize;
        (self.irq_bitmap[idx] & (1 << bit_pos)) != 0
    }

    pub fn cpu_set(&self) -> CpuSet {
        self.cpu_set
    }

    pub fn cpu_num(&self) -> usize {
        self.cpu_num
    }

    pub fn set_cpu_num(&mut self, cpu_num: usize) {
        self.cpu_num = cpu_num;
    }

    pub fn cpu_set_mut(&mut self) -> &mut CpuSet {
        &mut self.cpu_set
    }

    pub fn irq_bitmap(&self) -> &[u32; 1024 / 32] {
        &self.irq_bitmap
    }

    pub fn irq_bitmap_mut(&mut self) -> &mut [u32; 1024 / 32] {
        &mut self.irq_bitmap
    }

    // --- vCPU accessors ---

    pub fn vcpu_base(&self) -> usize {
        self.vcpu_base
    }

    pub fn vcpus(&self) -> &BTreeMap<usize, Arc<crate::vcpu::VCpu>> {
        &self.vcpus
    }

    /// Look up a vCPU by guest-visible MPIDR. Used by PSCI CPU_ON.
    pub fn get_vcpu_by_guest_mpidr(&self, mpidr: GuestMpidr) -> Option<Arc<crate::vcpu::VCpu>> {
        let vcpu_id = *self.guest_mpidr_to_vcpu.get(&mpidr)?;
        self.vcpus.get(&vcpu_id).cloned()
    }

    pub fn gpm(&self) -> &MemorySet<Stage2PageTable> {
        &self.gpm
    }

    pub fn gpm_mut(&mut self) -> &mut MemorySet<Stage2PageTable> {
        &mut self.gpm
    }

    pub fn iommu_pt(&self) -> Option<&MemorySet<Stage2PageTable>> {
        self.iommu_pt.as_ref()
    }

    pub fn iommu_pt_mut(&mut self) -> Option<&mut MemorySet<Stage2PageTable>> {
        self.iommu_pt.as_mut()
    }

    pub fn vpci_bus(&self) -> &VirtualRootComplex {
        &self.vpci_bus
    }

    pub fn vpci_bus_mut(&mut self) -> &mut VirtualRootComplex {
        &mut self.vpci_bus
    }

    #[cfg(feature = "dwc_pcie")]
    pub fn atu_configs(&self) -> &VirtualAtuConfigs {
        &self.atu_configs
    }

    #[cfg(feature = "dwc_pcie")]
    pub fn atu_configs_mut(&mut self) -> &mut VirtualAtuConfigs {
        &mut self.atu_configs
    }
}

static ZONE_LIST: RwLock<Vec<Arc<Zone>>> = RwLock::new(vec![]);

pub fn root_zone() -> Arc<Zone> {
    ZONE_LIST.read().get(0).cloned().unwrap()
}

pub fn is_this_root_zone() -> bool {
    Arc::ptr_eq(&this_zone(), &root_zone())
}

/// Add zone to CELL_LIST
pub fn add_zone(zone: Arc<Zone>) {
    ZONE_LIST.write().push(zone);
}

/// Remove zone from ZONE_LIST
pub fn remove_zone(zone_id: usize) {
    let mut zone_list = ZONE_LIST.write();
    let (idx, _) = zone_list
        .iter()
        .enumerate()
        .find(|(_, zone)| zone.id() == zone_id)
        .unwrap();
    let removed_zone = zone_list.remove(idx);
    assert_eq!(Arc::strong_count(&removed_zone), 1);
}

pub fn find_zone(zone_id: usize) -> Option<Arc<Zone>> {
    ZONE_LIST
        .read()
        .iter()
        .find(|zone| zone.id() == zone_id)
        .cloned()
}

pub fn all_zones_info() -> Vec<ZoneInfo> {
    let zone_list = ZONE_LIST.read();

    zone_list
        .iter()
        .map(|zone| ZoneInfo {
            zone_id: zone.id() as u32,
            cpus: zone.read().cpu_set().bitmap,
            name: zone.name(),
            is_err: zone.is_err() as u8,
        })
        .collect()
}

pub fn this_zone_id() -> usize {
    this_zone().id()
}

pub fn zone_create(config: &HvZoneConfig) -> HvResult<Arc<Zone>> {
    // we create the new zone here
    // TODO: create Zone with cpu_set
    let zone_id = config.zone_id as usize;

    if find_zone(zone_id).is_some() {
        return hv_result_err!(
            EINVAL,
            format!("Failed to create zone: zone_id {} already exists", zone_id)
        );
    }

    let mut zone = Zone::new(zone_id, &config.name);
    zone.pt_init(config.memory_regions())?;
    zone.mmio_init(&config.arch_config);

    #[cfg(feature = "pci")]
    {
        let _ = zone.virtual_pci_mmio_init(&config.pci_config, config.num_pci_bus as usize);
        let _ = zone.guest_pci_init(
            zone_id,
            &config.alloc_pci_devs,
            config.num_pci_devs,
            &config.pci_config,
            config.num_pci_bus as usize,
        );
    }

    // #[cfg(target_arch = "aarch64")]
    // zone.ivc_init(config.ivc_config());

    /* loongarch page table emergency */
    /* Kai: Maybe unnecessary but i can't boot vms on my 3A6000 PC without this function. */
    // #[cfg(target_arch = "loongarch64")]
    // zone.page_table_emergency(
    //     config.pci_config[0].ecam_base as _,
    //     config.pci_config[0].ecam_size as _,
    // )?;

    let mut cpu_num = 0;
    for cpu_id in config.cpus().iter() {
        if let Some(existing_zone) = get_cpu_data(*cpu_id as _).zone.clone() {
            return hv_result_err!(
                EBUSY,
                format!(
                    "Failed to create zone: cpu {} already belongs to zone {}",
                    cpu_id,
                    existing_zone.id()
                )
            );
        }
        zone.write().cpu_set_mut().set_bit(*cpu_id as _);
        cpu_num += 1;
    }
    zone.write().set_cpu_num(cpu_num);
    let cpu_set = zone.read().cpu_set();
    info!("zone cpu_set: {:#b}", cpu_set.bitmap);

    zone.arch_zone_pre_configuration(config)?;
    // #[cfg(target_arch = "aarch64")]
    // zone.ivc_init(config.ivc_config());

    #[cfg(all(feature = "iommu", target_arch = "aarch64"))]
    zone.iommu_pt_init(config.memory_regions(), &config.arch_config)?;

    /* loongarch page table emergency */
    /* Kai: Maybe unnecessary but i can't boot vms on my 3A6000 PC without this function. */
    // #[cfg(target_arch = "loongarch64")]
    // zone.page_table_emergency(
    //     config.pci_config.ecam_base as _,
    //     config.pci_config.ecam_size as _,
    // )?;

    /*zone.pci_init(
        &config.pci_config,
        config.num_pci_devs as _,
        &config.alloc_pci_devs,
    );*/

    zone.arch_zone_post_configuration(config)?;

    // Reset the zone arch-related resources, e.g. invalid data cache
    zone.arch_zone_reset(config)?;

    // Initialize the virtual interrupt controller, it needs zone.cpu_num
    zone.virqc_init(config);

    zone.irq_bitmap_init(config.interrupts_bitmap());

    let mut dtb_ipa = INVALID_ADDRESS as u64;
    for region in config.memory_regions() {
        // region contains config.dtb_load_paddr?
        if region.physical_start <= config.dtb_load_paddr
            && region.physical_start + region.size > config.dtb_load_paddr
        {
            dtb_ipa = region.virtual_start + config.dtb_load_paddr - region.physical_start;
        }
    }

    let new_zone_pointer = Arc::new(zone);
    {
        cpu_set.iter().for_each(|cpuid| {
            let cpu_data = get_cpu_data(cpuid);
            cpu_data.zone = Some(new_zone_pointer.clone());
            //chose boot cpu
            if cpuid == cpu_set.first_cpu().unwrap() {
                cpu_data.boot_cpu = true;
            }
            cpu_data.cpu_on_entry = config.entry_point as _;
            cpu_data.dtb_ipa = dtb_ipa as _;
            #[cfg(target_arch = "aarch64")]
            {
                cpu_data.arch_cpu.is_aarch32 = config.arch_config.is_aarch32 != 0;
            }
        });
    }

    // Create one vCPU per pCPU in cpu_set, set affinity, register guest MPIDR mapping,
    // and enqueue each vCPU onto its affinity pCPU's scheduler.
    #[cfg(target_arch = "aarch64")]
    {
        use crate::vcpu::{VCpu, VCpuState};

        let mut vcpu_base = usize::MAX;
        let mut local_idx: u64 = 0;

        for cpuid in cpu_set.iter() {
            let vcpu = Arc::new(VCpu::new(new_zone_pointer.clone()));

            // Record the first vCPU id as vcpu_base
            if vcpu_base == usize::MAX {
                vcpu_base = vcpu.id;
            }

            vcpu.set_pcpu_affinity(cpuid);

            // Guest MPIDR for this vCPU: zone-local index in Aff0 field
            let guest_mpidr = GuestMpidr::new(local_idx);

            // Register in zone's vCPU map
            {
                let mut inner = new_zone_pointer.write();
                inner.vcpus.insert(vcpu.id, vcpu.clone());
                inner.guest_mpidr_to_vcpu.insert(guest_mpidr, vcpu.id);
            }

            // Boot vCPU (local_idx == 0) starts in Ready state immediately.
            // Secondary vCPUs start Stopped; PSCI CPU_ON will wake them.
            if local_idx == 0 {
                // Set guest entry point (ELR_EL2) and initial x0 = dtb_ipa (Linux convention).
                info!("boot vcpu={} entry_point={:#x} dtb_ipa={:#x}", vcpu.id, config.entry_point, dtb_ipa);
                {
                    // Set entry point and dtb in the boot vCPU's TrapFrame.
                    let tf = vcpu.arch.trapframe();
                    tf.x.fill(0);
                    tf.x[0] = dtb_ipa as u64;       // x0 = DTB IPA
                    tf.elr  = config.entry_point as u64;
                    tf.spsr = 0x3c5;                 // EL1h, D/A/I/F masked
                    info!("boot vcpu trapframe: elr={:#x} spsr={:#x} x0={:#x}", tf.elr, tf.spsr, tf.x[0]);
                }
                let _ = vcpu.transition(VCpuState::Stopped, VCpuState::Ready);
                let cpu_data = get_cpu_data(cpuid);
                cpu_data.scheduler.enqueue(vcpu.clone());
            }

            local_idx += 1;
        }

        // Store vcpu_base in zone
        new_zone_pointer.write().vcpu_base = vcpu_base;

        info!(
            "zone {}: created {} vCPU(s), vcpu_base={}",
            zone_id,
            local_idx,
            vcpu_base
        );
    }

    Ok(new_zone_pointer)
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ZoneInfo {
    zone_id: u32,
    cpus: u64,
    name: [u8; CONFIG_NAME_MAXLEN],
    is_err: u8,
}
// Be careful about dead lock for zone.write()
pub fn zone_error() {
    if is_this_root_zone() {
        panic!("root zone has some error");
    }
    let zone = this_zone();
    let zone_id = zone.id();
    error!("zone {} has some error, please shut down it", zone_id);

    zone.set_err();
    drop(zone);
}

#[test_case]
fn test_add_and_remove_zone() {
    let zone_count = 50;
    let zone_count_before = ZONE_LIST.read().len();
    for i in 0..zone_count {
        let u8name_array = [i as u8; CONFIG_NAME_MAXLEN];
        let zone = Zone::new(i, &u8name_array);
        ZONE_LIST.write().push(Arc::new(zone));
    }
    for i in 0..zone_count {
        remove_zone(i);
    }
    assert_eq!(ZONE_LIST.read().len(), zone_count_before);
}
