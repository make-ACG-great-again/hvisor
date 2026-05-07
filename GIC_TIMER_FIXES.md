# GICv3 + Virtual Timer Bug Fixes

## 问题现象

- 内核启动随机挂死（无输出，需重启）
- 内核启动过程中随机出现约 34 秒停顿，位置不固定

---

## 根因分析与修复

### Bug 1：IRQ 26 (CNTHP) 永远 Active

**原因**：`deactivate_irq` 在 EOImode=1 下只写 EOIR（priority drop），不写 DIR。IRQ 26 是 EL2 私有定时器，从不注入为 LR.HW=1，guest EOI 不会清除其物理 Active 状态，导致 EL2 timer 永远无法再次触发。

**修复**：在 `deactivate_irq` 中为 IRQ 26 补写 `ICC_DIR_EL1`。

---

### Bug 2：所有 LR 占满时 IRQ 27 (CNTV) Active 泄漏

**原因**：`inject_irq` 在 LR 全满时返回 `false`，将 IRQ 放入 `PENDING_VIRQS`。此时 EOIR 只做 priority drop，物理 IRQ 27 保持 Active，GIC 停止投递 → RCU stall → 挂死。

**修复**：`gicv3_handle_irq_el1` 中捕获 `lr_written` 返回值，当 `irq_id == 27 && !lr_written` 时补写 DIR。

---

### Bug 3：`CPU_GICR_BASE` Lazy 初始化竞争死锁

**原因**：`spin::Lazy` 使用自旋锁保护初始化。各 pCPU 并发在 `el2_timer_init` 中首次访问 `CPU_GICR_BASE`，此时 IRQ 已使能。若初始化过程中 IRQ 触发且 handler 也访问 `host_gicr_base()`，自旋锁死锁 → 无输出挂死。

**修复**：在 `primary_init_early`（单核、IRQ 未使能）中提前强制初始化 `CPU_GICR_BASE`：`let _ = &*CPU_GICR_BASE;`

---

### Bug 4：DIR 写入时序违反 GICv3 规范（34 秒停顿根因）

**原因**：修复 Bug 2 时，在 `inject_irq` 内部检测到 LR HW=0 冲突后直接写 DIR，但此时 EOIR 尚未执行。GICv3 规范要求 **EOIR 必须早于 DIR**（先 priority drop，再 deactivate）。顺序错误在部分 GIC 实现上导致 priority 永远不被释放，后续同优先级中断无法投递，随机造成驱动等待超时（约 30 秒）。

**修复**：
1. 在 `inject_irq` 之前用 `find_lr_hw0()` 记录冲突状态
2. 按正确顺序执行：`inject_irq` → `deactivate_irq`（EOIR）→ DIR（如需）
3. 新增 `find_lr_hw0()` / `is_hardware_irq()` 辅助函数

```
正确顺序：pending_irq() → inject_irq() → deactivate_irq()[EOIR] → DIR
错误顺序：pending_irq() → inject_irq()[DIR in here] → deactivate_irq()[EOIR]
```

---

### Bug 5：restore_to_hardware 去掉 IMASK 导致 34 秒停顿

**原因**：vCPU switch-in 时若 CNTV 已到期，原本应设 IMASK=1 抑制硬件信号，由 `sched_tick_handler` Step 3 软件注入 IRQ 27。去掉 IMASK 后改为依赖物理 IRQ 27 硬件投递，但在某些路径下物理信号被屏蔽，guest 收不到定时器中断，驱动超时。

**修复**：恢复 `restore_to_hardware` 中 IMASK=1 逻辑；恢复 `sched_tick_handler` Step 3（ISTATUS=1 时主动注入 `inject_irq(27, false)`）。

---

### Bug 6：handle_wfi_trap 注入 HW=0 导致 Active 泄漏

**原因**：`truly_alone && timer_already_expired` 分支调用 `inject_irq(27, false)` 写入 LR.HW=0，随后物理 IRQ 27（LR.HW=1）到来时发现 LR 已被占用，冲突导致物理 Active 永不清除。

**修复**：该分支只 skip WFI 不注入，由 Step 3 在下一个 EL2 tick（≤10ms）处理。

---

## 修改文件汇总

| 文件 | 修改内容 |
|------|----------|
| `src/device/irqchip/gicv3/mod.rs` | IRQ 26 DIR；LR HW=0 冲突检测移至调用方且顺序修正；`find_lr_hw0`/`is_hardware_irq` 辅助函数；`CPU_GICR_BASE` 提前初始化 |
| `src/arch/aarch64/timer.rs` | 恢复 Step 3：ISTATUS=1 时注入 IRQ 27 |
| `src/arch/aarch64/vcpu.rs` | 恢复 IMASK=1 on switch-in when CNTV expired |
| `src/arch/aarch64/trap.rs` | `truly_alone+expired` 只 skip 不注入 |
| `src/scheduler.rs` | `SCH-SLOWPATH` 日志从 `warn` 降为 `trace` |
