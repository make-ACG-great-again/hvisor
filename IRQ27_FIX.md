# IRQ27 (CNTV Virtual Timer) 无限循环问题分析与修复

## 问题现象

hvisor 在 1pCPU:2vCPU 超分场景下，Linux 启动约 8 秒后系统挂死：
- `need_resched=true` 持续为真但 vCPU 从不切换
- scheduler 调用计数冻结
- 通过 `[GIC-LOOP]` 日志确认：`gicv3_handle_irq_el1` 的 while 循环内，`pending_irq()` 反复返回 `irq=27`，循环无法退出

---

## 根因分析

### GICv3 IRQ27 状态机

IRQ27（CNTV 虚拟定时器 PPI）是 level-triggered 中断。其物理状态机为：

```
Inactive → Pending（CNTV到期，ISTATUS=1）
         → Active（IAR读取）
         → Inactive（DIR写入）
         → Pending（若ISTATUS仍=1且IMASK=0，立刻重新Pending）
```

### 错误的 Hw0Conflict 机制

新版引入了 `find_lr_hw0` / `Hw0Conflict` 机制，试图检测"LR 里已有 HW=0 条目时物理中断又来了"的冲突场景，并在 `Hw0Conflict::Active` 时写 DIR。

**这是错误的**，原因如下：

- 旧版（hvisor_vcpu）的正确设计：`lr_written=true` 时**只做 EOIR（priority drop），不写 DIR**
- 物理 IRQ27 保持 Active 状态，Active 状态下 CNTV 不会重新 Pending
- guest 运行后其 EOI 触发物理 deactivate（LR.HW=1，VEOIM=0）
- 新版在 `Hw0Conflict::Active` 分支额外写了 DIR → 物理 Active 被清除 → CNTV ISTATUS=1 且 IMASK=0 → 立刻重新 Pending → IAR 再次返回 27 → **无限循环**

日志证据：
```
[IRQ27] #1000 lr_written=true hw0=1 cntv_ctl=0x5 iter=168
[IRQ27] #2000 lr_written=true hw0=1 cntv_ctl=0x5 iter=1169
...（同一次 gic_n 调用，iter 持续增长到数万）
```
`cntv_ctl=0x5`：bit0=ENABLE, bit2=ISTATUS=1，IMASK=0。

### Idle 场景下的额外问题

2pCPU:4vCPU 场景下，所有 vCPU 均 Blocked 时 pCPU 进入 idle。此时：

- `schedule_inject_irq` 无 current_vcpu → `lr_written=false` → 写 DIR
- CNTV 到期（ISTATUS=1）且 IMASK=0 → 立刻重新 Pending → **无限循环**

根本原因：vCPU switch-out 时未屏蔽物理 CNTV，vCPU 切出后物理计数器继续运行，
到期后产生 IRQ27，但此时无任何 vCPU 在运行。

---

## 修复方案

### 修复1：删除错误的 Hw0Conflict 机制，还原旧版逻辑

**文件**：`src/device/irqchip/gicv3/mod.rs`

还原为旧版简洁逻辑：
- `lr_written=true`：只做 EOIR，不写 DIR。物理 IRQ27 保持 Active，等 guest EOI 自动 deactivate。
- `lr_written=false`：写 DIR（防止永久 Active 泄漏），同时设 IMASK=1（防止 CNTV 立刻重 Pending 死循环）。

删除的内容：`find_lr_hw0()`、`Hw0Conflict` 枚举、`is_hardware_irq()`。

同时修正 IRQ26 处理顺序，改为与旧版一致（先 `sched_tick_handler()` 再 `deactivate_irq()`）：

```rust
// 修复前（错误顺序）
deactivate_irq(irq_id);
sched_tick_handler();

// 修复后（正确顺序，与旧版一致）
sched_tick_handler();
deactivate_irq(irq_id);
```

### 修复2：vCPU switch-out 时屏蔽物理 CNTV

**文件**：`src/arch/aarch64/vcpu.rs`，`save_from_hardware()`

旧版只在 ISTATUS=1 时设 IMASK，但 ISTATUS=0 时 vCPU 切出后计数器仍在运行，
可能在 idle 期间到期产生 IRQ27。

```rust
// 修复前：只有 ISTATUS=1 才设 IMASK
if timer_enabled && timer_expired {
    self.cntv_ctl_el0 = cntv_ctl | 2; // set IMASK
} else {
    self.cntv_ctl_el0 = cntv_ctl & !2u64; // 不设 IMASK，存在隐患
}

// 修复后：只要 ENABLE=1 就设 IMASK
if timer_enabled {
    let masked = cntv_ctl | 2; // set IMASK，彻底屏蔽物理 IRQ27
    write_sysreg!(CNTV_CTL_EL0, masked);
    self.cntv_ctl_el0 = masked;
} else {
    self.cntv_ctl_el0 = cntv_ctl;
}
```

`restore_to_hardware()` 保持不变：switch-in 时恢复保存值，ISTATUS=0 则清 IMASK，
硬件正常投递 IRQ27；ISTATUS=1 则保持 IMASK，由 `check_blocked_timers` 软件路径注入。

---

## 设计原则总结

| 场景 | 处理方式 |
|------|----------|
| `lr_written=true`（LR.HW=1 写入或 LR 已有同 IRQ 去重） | 只 EOIR，不 DIR。物理保持 Active，由 guest EOI 触发 deactivate |
| `lr_written=false`（无 current_vcpu，idle 场景） | EOIR + DIR + IMASK=1。软件路径注入，switch-in 时恢复 |
| vCPU switch-out | 无论 ISTATUS，ENABLE=1 就设 IMASK，防止 idle 期间 IRQ27 泄漏 |
| vCPU switch-in | restore_to_hardware 恢复保存值；ISTATUS=0 清 IMASK，正常硬件投递 |
