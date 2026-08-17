# MSPM0G3507 自动行驶小车

基于 TI MSPM0G3507、SysConfig 和 DriverLib 开发的自动行驶小车程序。系统通过编码器、六路灰度传感器和串口陀螺仪获取车辆状态，并控制四路电机完成行驶与停车。

来源于2024年电赛H题。

## 主要功能

- 四路直流电机 PWM 驱动与制动
- 四路编码器计数
- 8个并行灰度传感器检测与巡线
- 串口读取陀螺仪航向角
- 20 ms 定时控制与 PID 航向修正
- 按键选择不同的行驶模式
- OLED 显示角度、模式、灰度和速度信息

## 无 CCS 命令行构建

项目新增了 [`standalone-ticlang/`](standalone-ticlang/README.md) 构建入口，可以不安装 Code Composer Studio，直接使用以下工具完成 SysConfig 生成、编译和链接：

- MSPM0 SDK 2.01.00.03
- SysConfig 1.20.0+3587
- TI Arm Clang 3.2.2.LTS
- GNU Make

当前机器上的默认安装路径写在 `standalone-ticlang/Makefile` 中，也可以通过 Make 命令行变量覆盖。PowerShell 构建命令：

```powershell
& "D:\clion-stm32\MinGW\bin\mingw32-make.exe" `
  -C "E:\Ee\my_project_refactor\standalone-ticlang" all
```

构建成功后，固件和链接映射分别位于：

```text
standalone-ticlang/my_project_08012135.out
standalone-ticlang/my_project_08012135.map
```

SysConfig 生成文件、目标文件、`.out` 和 `.map` 已由 `standalone-ticlang/.gitignore` 排除，不应手工编辑或提交。当前独立构建已通过 SysConfig 严格验证、TI Arm Clang 编译链接和全部 7 组 host 回归测试；XDS110 下载启动及真实车辆行为仍待验证。

## 已知问题

本仓库主要用于保存 2024 年竞赛期间形成的代码，目前尚未完成全面修正和重新实车验收。

- 修复 `my_abs()` 在非正数输入下的未定义行为，并统一距离和角度计算的数据类型。
- 将声光提示改为非阻塞实现，缩短定时中断执行时间，避免影响编码器和陀螺仪数据接收。
- 完善模式切换和异常模式下的制动逻辑，避免旧的电机控制指令继续生效。
- 修正灰度传感器采样与求和的时序，保证控制逻辑使用同一周期的数据。
- 完善各行驶模式的状态复位、重复启动和终点停车逻辑，重点验证模式 3 和模式 4。
- 增加陀螺仪数据校验、通信超时、失联停车和 ±180° 航向角环绕处理。

## 发布说明

- 题目路线、完成时间、停车位置和多次运行稳定性尚未重新进行实车验证。
- 项目许可证、第三方 OLED/字体代码来源及题目 PDF 的再分发授权仍待确认。
