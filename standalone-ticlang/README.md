# Standalone TI Arm Clang build

This directory builds the firmware without Code Composer Studio. It uses the
installed MSPM0 SDK, SysConfig CLI, TI Arm Clang compiler, and GNU Make.

Default tool locations:

```text
C:/ti/mspm0_sdk_2_01_00_03
C:/ti/sysconfig_1.20.0/sysconfig_cli.bat
C:/ti/ti-cgt-armllvm_3.2.2.LTS
```

From PowerShell:

```powershell
& "D:\clion-stm32\MinGW\bin\mingw32-make.exe" `
  -C "E:\Ee\my_project_refactor\standalone-ticlang" all
```

The build retains generated SysConfig files, object files, the map file, and
`my_project_08012135.out` in this directory. Do not edit generated files.

Override a tool location on the make command line when needed, for example:

```powershell
& "D:\clion-stm32\MinGW\bin\mingw32-make.exe" `
  -C "E:\Ee\my_project_refactor\standalone-ticlang" `
  TICLANG_ARMCOMPILER="C:/custom/ti-cgt-armllvm_3.2.2.LTS" all
```
