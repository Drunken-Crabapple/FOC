# C Porting Notes

This repository is a C port of the original QDrive C++ FOC firmware.

## Scope

- PID controller converted from `class PID` to `PID_t` plus inline C functions.
- FOC core converted from `class FOC` to `FOC_t` plus C functions.
- QD4310 product wrapper converted from inheritance to `QD4310_t` containing `FOC_t`.
- Driver, encoder, current sensor, and storage abstractions converted from virtual classes to C function-pointer interfaces.
- Application tasks, shell commands, communication task, storage, and retarget layer converted from `.cpp` to `.c`.
- CMake changed to C/ASM only; C++ compiler and standard library are no longer linked.

## Porting Policy

The C port keeps the original C++ control behavior as the reference behavior.
The goal is to replace C++ language features with C equivalents without changing
validated FOC runtime logic.

The STM32CubeG4 dependency path is configurable through `STM32CUBE_G4_PATH` so
the project is not tied to the original author's Windows user directory.

## Build Requirements

- `cmake`
- `arm-none-eabi-gcc`
- STM32CubeG4 firmware package matching the CubeMX-generated project, default:
  `C:/Users/$ENV{USERNAME}/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.2`

If the package is installed elsewhere, configure with:

```powershell
cmake -S . -B build -DSTM32CUBE_G4_PATH="D:/path/to/STM32Cube_FW_G4_V1.6.2"
```
