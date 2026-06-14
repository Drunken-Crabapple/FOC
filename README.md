# FOC

C language port of the QDrive QD4310 FOC firmware.

The original firmware used C++ classes for PID, FOC, hardware adapters, storage,
tasks, shell commands, and communication. This repository keeps the same control
flow and STM32G431 hardware target, but rewrites the user layer in C.

See [docs/PORTING_NOTES.md](docs/PORTING_NOTES.md) for migration notes and build
requirements.
