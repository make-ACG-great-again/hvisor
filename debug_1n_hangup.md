# 1:N vCPU 超配挂死问题排查

## 背景

zone0 配置：2 pCPU（pCPU0/1），4 vCPU（vCPU0-3），每个 pCPU 承载 2 个 vCPU（round-robin 绑定）。
时间片：`DEFAULT_TIME_SLICE = 1 tick = 10ms`，故每个 vCPU 每 20ms 才能运行一次。
Guest Linux HZ=250，期望每 4ms 一次时钟节拍。

症状：
- 内核初始化阶段可能卡死（secondary CPU boot 阶段）
- 正常启动后多进程场景挂死（RCU stall）

---

## 问题列表

### P0-A：`vcpu_vmreturn` 与 `vcpu_switch_in` 双重 drain+inject

**位置**：`src/arch/aarch64/trap.rs:163` (`vcpu_vmreturn`) 和 `src/scheduler.rs:390` (`vcpu_switch_in`)

**描述**：
`arch_handle_exit` 末尾的调用路径为：
```
arch_handle_exit
  → schedule()         // need_resched=true 时
      → vcpu_switch_in()
          → drain_pending_irqs()  // ← 第一次 drain + inject
  → vcpu_vmreturn()
      → drain_pending_irqs()     // ← 第二次 drain + inject（重复！）
```
两次 drain 之间，`sched_tick_handler` Step 3 或 `check_ready_timers` 可能再次向
`pending_virqs` 里推入 IRQ 27，导致第二次 drain 再注入一个 HW=0 的 IRQ 27 LR。
此时若物理 IRQ 27 也 pending（IMASK=0, ISTATUS=1），就会出现 HW=0/HW=1 LR 冲突。

**验证方法**：在 `vcpu_vmreturn` 的 drain 路径加 `trace!`，观察是否频繁出现非空 drain。

**修复方向**：`vcpu_vmreturn` 不应再 drain pending_irqs；只有 `vcpu_switch_in` 负责注入。
或在 `vcpu_vmreturn` 中判断是否刚经过 `schedule()`，避免重复。

---

### P0-B：内核初始化时 SGI 传递丢失导致 secondary boot 卡死

**位置**：`src/arch/aarch64/trap.rs:494` (`deliver_sgi_to_vcpu` → `VCpuState::Ready` 分支)

**描述**：
secondary CPU boot 过程中，guest Linux 大量使用 SGI 做核间同步（TLB shootdown、cache
同步、SMP 屏障等）。当发送方 vCPU（如 vCPU2）给接收方 vCPU（如 vCPU0，状态为 Ready）
发送 SGI 时：

```rust
VCpuState::Ready => {
    vcpu.push_pending_irq(sgi_id, false);
    cpu.need_resched.store(true, ...);
}
```

IRQ 被 push 到 pending_virqs，但 pCPU 当前在运行 vCPU2，不会立刻切换到 vCPU0。
如果 guest 的同步是**忙等待**（polling 内存变量，等待接收方处理 SGI 后修改标志位），
而接收方 vCPU0 还在 Ready 队列里等待调度，就会死锁：
- 发送方在自旋等待
- 接收方没被调度，永远不会处理 SGI

**验证方法**：在内核初始化卡死时观察是否有大量 SGI（IRQ 0-15）堆积在某个 vCPU 的 pending_virqs。

**修复方向**：收到 SGI 且目标 vCPU 为 Ready 状态时，应触发 IPI 通知目标 pCPU 尽快调度，
或直接提升目标 vCPU 的优先级（priority boost）。

---

### P1-A：`sched_tick_handler` Step 3 在 tick 路径直接 inject，与后续 vcpu_vmreturn drain 冲突

**位置**：`src/arch/aarch64/timer.rs:164` (Step 3) 和 `src/arch/aarch64/trap.rs:167` (vcpu_vmreturn)

**描述**：
tick handler Step 3 直接调用 `inject_irq(27, false)`，将 IRQ 27 HW=0 写入 GIC LR。
tick handler 返回后，控制流到达 `vcpu_vmreturn`，后者再次 drain pending_irqs 并注入。
若 pending_virqs 中此时也有 IRQ 27（由 `check_ready_timers` 或 `vcpu_switch_out` retract 放入），
会产生**两个 IRQ 27 LR entry**，导致 GIC 行为异常（guest 可能收到两次 CNTV 中断）。

**验证方法**：在 `inject_irq(27, ...)` 处加 trace，观察同一个 EL2 exit 周期内是否注入两次。

**修复方向**：Step 3 不直接 inject，改为 push 到 pending_virqs；统一由 vcpu_switch_in 或
vcpu_vmreturn 的单一路径注入，保证每次 EL2 exit 最多注入一次 IRQ 27。

---

### P1-B：blocked vCPU 唤醒延迟（最大 100ms）累积导致 RCU stall

**位置**：`src/scheduler.rs:190` (`check_blocked_timers` 中 `absolute_timeout`)

**描述**：
vCPU 执行 WFI 后进入 Blocked 状态，由 `check_blocked_timers` 在每次 tick（10ms）时检查是否应唤醒。
若 vCPU 的虚拟 timer 未到期（guest 真的在等待某个未来事件），唤醒依赖 `absolute_timeout`（10 ticks = 100ms）。

在 1:N 超配下，pCPU 每次 tick 只有 50% 概率被 blocked vCPU 占用（另一个 vCPU 可能在运行），
实际唤醒延迟可能接近 100ms。而 Linux RCU 需要所有 CPU 定期通过 quiescent state，
长时间（累积数秒）延迟会触发 RCU stall warning（默认 21s 超时，但多次累积会触发）。

**验证方法**：观察 `[TICK] blocked vcpu(s) not woken` 日志中 `blocked_at` 与 `now` 的差值，
确认延迟是否接近 10 ticks。

**修复方向**：
- 缩短 absolute_timeout（如 2-3 ticks）
- 或在 blocked vCPU 期间用 `el2_timer_arm_at` 精确定时唤醒，而非依赖 tick 轮询

---

### P1-C：时间片粒度过粗，guest tick 频率不足

**位置**：`src/scheduler.rs:30` (`DEFAULT_TIME_SLICE = 1`) 和 `src/arch/aarch64/timer.rs:25` (`SCHED_TICK_PERIOD_US = 10_000`)

**描述**：
当前配置：时间片 = 1 tick = 10ms。在 2 个 vCPU 共享一个 pCPU 的情况下，每个 vCPU
每 20ms 才能运行一次（50Hz 等效）。

Guest Linux HZ=250 意味着每 4ms 期望一次时钟节拍（CNTV 中断）。实际上每个 vCPU 每
20ms 才运行一次，guest 的 jiffies 只能以 50Hz 速率推进，导致：
- timer wheel 中的定时器超时被严重延误
- RCU grace period 推进变慢
- 任何依赖 `schedule_timeout`、`msleep` 的内核代码延迟 4-5x

这是 1:N 超配的固有开销，但可以通过减小时间片或加快 tick 频率来缓解。

**验证方法**：在 guest 中运行 `date` 并与宿主机时间对比，或观察 dmesg 中时间戳是否比实际慢。

**修复方向**：
- 将 `SCHED_TICK_PERIOD_US` 从 10ms 降至 4ms（与 HZ=250 对齐），代价是切换开销增大
- 或将 `DEFAULT_TIME_SLICE` 增大（如 5），减少切换频率，但每个 vCPU 单次运行更长

---

## 排查顺序建议

1. **P0-B**（SGI 丢失）：最可能导致内核初始化直接卡死，影响最早，先排查
2. **P0-A**（双重 drain）：影响所有 IRQ 注入路径，修复最简单，顺带修
3. **P1-A**（Step 3 直接 inject）：与 P0-A 相关，一并处理
4. **P1-B**（blocked 唤醒延迟）：需要日志数据支撑，等前三项修完后再看 RCU stall 是否消失
5. **P1-C**（时间片粒度）：调参，最后调优

---

## 当前代码状态备忘（已修复）

- P0-A：`vcpu_vmreturn` 为单一 drain+inject 点，`vcpu_switch_in` 不再注入
- P0-B：SGI 跨 pCPU 时发 IPI_EVENT_RESCHED，同 pCPU 时直接 need_resched
- P1-A：sched_tick_handler Step 3 改为 push_pending_irq，不直接 inject
- P1-B：absolute_timeout 缩短至 2 ticks
- IRQ 27 IMASK 泄漏：`save_from_hardware`/`restore_to_hardware` 现在在 ISTATUS=0 时清除 IMASK
- Blocked→Ready 竞态：`drain_pending_wake_ids` 新增对 Ready 队列的查找（fix SGI 丢失）

---

## P2-A：IMASK 泄漏导致 timer 永远走代理路径

**位置**：`src/arch/aarch64/vcpu.rs` `save_from_hardware` / `restore_to_hardware`

**描述**：
hypervisor 在注入 IRQ 27 HW=0 时设置 IMASK=1，是临时抑制物理 IRQ 27 的手段。
guest 处理定时器后 ISTATUS→0，但 IMASK 仍为 1。`save_from_hardware` 的 else 分支
保存了带 IMASK=1 的 `cntv_ctl_el0`；下次 `restore_to_hardware` 时 ISTATUS=0，写回
带 IMASK=1 的值，物理 timer 永远被抑制，只能依赖每 10ms 一次的 sched_tick Step 3 代理。

**修复**：ISTATUS=0 时清除 IMASK（`cntv_ctl & !2u64`），保证 guest 正常的硬件 timer 路径。

---

## P2-B：`drain_pending_wake_ids` Blocked→Ready 竞态导致 SGI 丢失

**位置**：`src/vcpu.rs` `drain_pending_wake_ids`

**描述**：
跨 pCPU SGI 将 PendingWake 推入目标 pCPU 的 `pending_wake_ids`，然后发 IPI。
若 IPI 到达前，目标 pCPU 的 tick 已将 vCPU 从 Blocked 唤醒（→ Ready），
`drain_pending_wake_ids` 调用 `find_blocked` 返回 None，SGI 被丢弃，
guest 的 SGI 同步 barrier 死锁。

**修复**：`find_blocked` 返回 None 时额外查找 Ready 队列，找到则直接 push_pending_irq。
