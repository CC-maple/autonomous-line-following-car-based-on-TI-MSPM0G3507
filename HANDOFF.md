# MSPM0 Refactor Handoff

本文档用于将当前 MSPM0G3507 自动行驶小车重构工程交接给下一位 agent。

## 1. 当前结论

源码重构阶段已完成，当前停在 CCS 目标构建和硬件验证之前。

当前不能合并到 main。原因不是已发现的源码测试失败，而是本机缺少项目要求的完整 MSPM0 SDK 2.01.00.03，CCS 尚未进入 C 源文件编译阶段。

当前工程目录：
E:/Ee/my_project_refactor

受保护的原始工程：
E:/Ee/my_project_08010527end_review

严禁修改受保护工程。用户明确要求“请勿删文件”。不要删除文件、不要运行 git clean，不要使用破坏性 reset/checkout。

## 2. Git 状态

- 当前检出分支：refactor/mode-lifecycle
- 交付分支：refactor/v2
- 两个分支应保持指向同一个最新提交；用 git rev-parse HEAD 和 git rev-parse refactor/v2 现场确认
- 最后一个固件代码提交：ac0b696d5fb7776b16b82b6c9e4b66d07bfa1d41（fix: define scheduler control macros before helpers）
- HANDOFF.md 是 ac0b696 之后的文档提交，不改变固件行为
- 源码和配置工作树在新增本文件前是干净的；文档提交后工作树也应保持干净
- 基线 tag：contest-2024-final，固定在 2b4e3f02cdae530cdb04a54f81350b874ecaaee5
- 原始工程 HEAD：2b4e3f02cdae530cdb04a54f81350b874ecaaee5
- 不要合并到 main，直到 CCS 构建和实车验证完成。

## 3. 已完成提交

按需求顺序完成了 7 个阶段，另有 1 个结构修正：

1. 6639565 fix: make absolute and distance checks deterministic
2. a5767ba fix: enforce motor failsafe outputs
3. 359f0a1 refactor: make signal alerts nonblocking
4. 933aab8 fix: keep grayscale samples consistent
5. 6e07cdb refactor: reset mode runtime lifecycle
6. 2a0eb10 fix: validate IMU heading and freshness
7. 4b72bbd refactor: serialize control output commits
8. ac0b696 fix: define scheduler control macros before helpers

不要回退这些提交，也不要把生成文件手工加入提交。

## 4. 主要实现状态

### 4.1 电机和控制调度

文件：Control/control_scheduler.h、Control/control.c

控制输出已经事务化：

- 每个 TIMG0 ZERO tick 开始初始化一个空事务。
- Load() 只登记候选电机命令。
- brake() 锁定当前 tick 的安全状态，并清除普通候选。
- brake 锁定后，后续 Load() 不能覆盖 brake。
- 多个普通 Load() 时最后一个候选生效。
- 空事务默认提交 brake。
- 一个 ZERO tick 只提交一次真实 Motor API 输出。
- 非 ZERO 中断的 default 路径不提交事务。
- 事务宏位于控制辅助函数定义之前；提交函数中的 (Load)(...) 和 (brake)() 用于调用真实 Motor API，避免宏递归。

已接受的行为变化：mode4-7 中原先“同一个 tick 先写 PWM、随后 brake”的瞬时输出现在合并为最终 brake；可能使动作少推进一个 20 ms 周期。必须实车确认。

另一个需产品确认的行为：begin == 0 且没有候选命令时，ZERO tick 默认提交 brake。

### 4.2 信号

文件：Control/signal_state.h

- 声光信号改为非阻塞状态机。
- SIGNAL_DURATION_TICKS = 25，约 500 ms（控制周期 20 ms）。
- active、finished-gap、queued 状态均 brake 并跳过模式控制。
- 请求在 active 期间排队，计数饱和到 UINT8_MAX。
- 结束后保留一个完整 gap tick，再启动队列中的下一次信号。
- 当前所有调用者使用固定 25 tick；未来若需要不同 duration，需扩展队列数据结构。
- Sign_LED_Bee() 预期在控制 ISR 路径调用；若增加并发非 ISR 调用，需要原子保护或临界区。

### 4.3 传感器快照

文件：Control/sensor_snapshot.h、Control/control.c

- 六路和八路灰度更新各自只调用一次 DL_GPIO_readPins()。
- 每次从同一个 raw GPIO 值解码数组和 sum。
- 六路写入 huidu_data[1..6]。
- 八路写入 mode4_huidu_data[0..7]。
- mode3_only_go_line() 不再重复采样。
- SysConfig 映射已核对：六路为 PA13/12/11/10/24/25，八路额外为 PA26、PA7。

### 4.4 生命周期

文件：Control/mode_lifecycle.h、Control/control.c

- 运行中切换 mode 会 brake、清理运行态、清零 begin，并要求重新启动。
- 信号队列非 idle 时不会触发运行态重置。
- 自然完成会走 EXIT 清理路径。
- begin=1 且未选择 mode（mode=0）会走 INVALID，制动并拒绝启动。
- Control_ResetRuntime() 清理 encoder、速度、PID、mode2/3/4 状态和两组灰度缓存/sum。
- mode4_angle_change 是按键调参值，不属于运行态，不应在普通 runtime reset 中清零。

### 4.5 IMU 和航向

文件：UART_Gyro/imu_protocol.h、UART_Gyro/uart_gyro.c、Control/control_math.h

- 解析固定 11 字节 0x55 0x53 + 8 payload + checksum 帧。
- 校验为前 10 字节 8 bit 累加和。
- 坏帧、错误帧类型和 UART overrun/break/parity/framing/timeout/noise 错误会复位 parser 并使样本失效。
- 有效样本年龄在控制 tick 中递增；初始状态无效。
- IMU_SAMPLE_MAX_AGE_TICKS = 10，按 20 ms 控制周期约 200 ms。
- 控制 ISR 在进入运动控制前执行 uart_gyro_is_fresh() 门控；失效/stale 时 brake 并停止当前运行。
- 控制逻辑不再读写 UART 的全局 Angle[2]；航向由 uart_gyro_heading_degrees() 提供。
- PID 和到达判断使用最短环绕角差，范围约定为 [-180, 180)。
- PID 正负号、IMU 输出频率、UART 噪声 IIDX 呈现方式仍需硬件确认。

## 5. 已通过的验证

在 E:/Ee/my_project_refactor 执行过：

- tests/run_absolute_and_distance_tests.ps1
- tests/run_motor_failsafe_tests.ps1
- tests/run_nonblocking_signal_tests.ps1
- tests/run_sensor_snapshot_tests.ps1
- tests/run_mode_lifecycle_tests.ps1
- tests/run_imu_heading_tests.ps1
- tests/run_control_scheduler_tests.ps1
- 严格控制源 mock 语法检查：GCC C11、-Wall -Wextra -Werror
- Arm GCC Cortex-M0+ 调度测试语法检查
- git diff --check
- python E:/Ee/.agents/skills/mspm0-ccs/scripts/check_syscfg.py E:/Ee/my_project_refactor

推荐重新运行全部 host 回归：

    powershell
    cd E:/Ee/my_project_refactor
    ./tests/run_control_scheduler_tests.ps1
    ./tests/run_absolute_and_distance_tests.ps1
    ./tests/run_motor_failsafe_tests.ps1
    ./tests/run_nonblocking_signal_tests.ps1
    ./tests/run_sensor_snapshot_tests.ps1
    ./tests/run_mode_lifecycle_tests.ps1
    ./tests/run_imu_heading_tests.ps1

不要删除测试脚本生成并保留在 %TEMP% 下的临时 binary/mock 文件。

## 6. CCS Theia 图形化编译

### 6.1 应该导入哪个目录

在 CCS Theia 中导入整个工程根目录：

E:/Ee/my_project_refactor

不要导入 Control、Motor 等子目录，也不要导入受保护原始工程。

根目录应包含：

- .project
- .cproject
- .ccsproject
- main.syscfg
- targetConfigs/MSPM0G3507.ccxml

图形化操作：

1. 启动 E:/Ee/ccstheia140 中的 CCS Theia。
2. 使用 Import Existing CCS/Eclipse Project。
3. 选择 E:/Ee/my_project_refactor。
4. 工程名称应为 my_project_08012135。
5. 选择 Debug configuration。
6. 先执行 Clean Project。
7. 再执行 Build Project。

### 6.2 项目工具链要求

来自 .cproject 的要求：

- Device：MSPM0G3507
- Core：Cortex-M0+
- Compiler：TICLANG_3.2.2.LTS
- TI Arm Clang 路径：E:/Ee/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe
- SysConfig：1.20.0
- MSPM0 SDK：2.01.00.03（CCS 中显示为 MSPM0-SDK v2.1.0.03）
- Debug probe 配置：TI XDS110
- Target config：targetConfigs/MSPM0G3507.ccxml

### 6.3 当前 CCS 构建阻塞

已经使用 CCS Theia 的 headless backend 成功导入工程，并执行过 Debug clean/full build。结果：

Product MSPM0-SDK v2.1.0.03 is not currently installed and no compatible version is available.

这发生在生成 makefile 前，尚未编译 C 源文件。

本机现状：

- 完整的旧 SDK：E:/Ee/Ti/mspm0_sdk_2_00_01_00，版本不匹配。
- 名为 E:/Ee/mspm0_sdk_2_01_00_03 的目录只有 examples，缺少 .metadata/product.json、source 和标准库资源，不能注册为完整 SDK。
- 不要把旧版 SDK 注册为本项目的正式构建依赖，也不要修改 .cproject 降级版本以绕过检查。

下一位 agent 应先安装或取得完整官方 MSPM0 SDK 2.01.00.03，在 CCS 产品发现设置中注册该 SDK，然后重新执行 Clean/Build。成功标志应包括：

- CCS 不再报 product missing。
- SysConfig 能够从 main.syscfg 生成 ti_msp_dl_config.c/.h。
- Debug 目录出现目标构建文件、.out、.map 等产物。
- 没有编译/链接 error。

生成文件必须由 CCS/SysConfig 生成，禁止手工编辑。

## 7. 目标构建后检查

完成 SDK 安装和构建后，按以下顺序检查：

1. 检查 Debug/ti_msp_dl_config.c 与 .h 是否由 SysConfig 生成。
2. 检查 UART 错误宏、UART IRQ 名称、TIMG0 IRQ 名称与 main.syscfg/生成头文件一致。
3. 检查 .out、.map 和 linker diagnostics。
4. 运行 git diff --check。
5. 确认生成输出没有被手工改写；如输出目录被 Git 跟踪，先停止并检查，不要覆盖提交历史。
6. 记录完整 build log。

## 8. XDS110 和实车验证

只有目标构建成功后才进行下载/调试。先检测 probe：

    powershell
    python E:/Ee/.agents/skills/mspm0-ccs/scripts/detect_probe.py
    python E:/Ee/.agents/skills/mspm0-ccs/scripts/check_syscfg.py E:/Ee/my_project_refactor --probe

不要在未确认 XDS110 与目标连接的情况下选择其他 probe。下载/调试可能让控制 ISR 停止，接实车时要先断开机械负载或确保车辆抬起。

实车至少验证：

- 无 IMU 有效帧时保持四路 brake。
- 坏校验帧、UART 错误和 stale 超时不会继续运动。
- 合法帧跨越 +/-180° 时无大角度反向。
- 模式切换、重复启动、自然完成后的运行态清理。
- 信号 active、finished-gap 和 queued 状态期间电机保持 brake。
- mode4-7 的事务化输出是否少一个预期中的 20 ms 推进周期。
- PID 符号、左右电机方向、GPIO 有效电平和灰度索引。
- idle 状态持续 brake 是否符合手推/调试需求。

## 9. 交接规则

- 任何修改前先读取当前文件，并确认工作树是否有用户变更。
- 只修改 my_project_refactor，不要改 my_project_08010527end_review。
- .syscfg 是 pinmux/peripheral/clock/interrupt 的唯一配置源。
- 不手工编辑 SysConfig 生成的 .c/.h、linker、Debug 输出和二进制文件。
- 不删除文件，不运行 git clean，不执行 destructive reset/checkout。
- 新改动应保持在对应阶段边界内，并增加针对性测试。
- 没有 CCS clean build 和真实车辆验证前，不要合并 main。
- 最终报告必须区分：host 测试通过、目标编译通过、probe 下载通过、实车验证通过；它们不是同一件事。

## 10. 交接完成判据

交接完成前，下一位 agent 应至少补齐：

- 完整 MSPM0 SDK 2.01.00.03 安装和 CCS 产品注册。
- CCS Theia Debug clean/full build 成功。
- 生成文件和 linker 输出核验。
- XDS110 下载/启动验证。
- IMU、传感器、信号、生命周期、调度器和四路电机的实车回归。
- 记录任何与旧车行为不同的调参或时序变化。

当前文档本身不改变固件行为。
