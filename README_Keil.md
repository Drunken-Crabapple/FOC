# QDrive FOC Keil 工程

本目录由 Codex 从已验证的 `QDrive_FOC_Rebuild` C/CMake 工程迁移生成，目录布局参考 `实验1-1 跑马灯`：

- `Projects/MDK-ARM/FOC_QD4310_Keil.uvprojx`：Keil uVision 工程
- `Output/`：Keil 编译输出
- `Core/`、`Applications/`、`BSP/`、`USB_Device/`、`UserLib/`：原工程业务源码副本
- `Drivers/`、`Middlewares/`：STM32CubeG4 HAL、CMSIS、USB Device、FreeRTOS 依赖副本
- `FOC_QD4310_Keil.sct`：Keil scatter 文件，保留原 Flash/RAM/CCMRAM/配置存储布局

注意：工程使用 ARM Compiler 6，并引用 Keil Pack `Keil.STM32G4xx_DFP.2.2.0`。
