# YFOC STM32 Project / YFOC STM32 项目

[English](#english) | [中文](#chinese)

---

## English

### Project Overview
This is a STM32F103C8 based project for YFOC  implementation. The project utilizes various peripherals including CAN, I2C, UART, and timers for motor control applications.

### Hardware Requirements
- **MCU**: STM32F103C8T6 
- **Encoder**: AS5600 magnetic encoder (I2C interface)
- **Communication**: CAN bus for motor control commands
- **Debug**: UART for debugging and monitoring

### Key Features
- CAN bus communication for motor control
- AS5600 magnetic encoder integration for position feedback
- Real-time motor control algorithms
- DMA-based data transfer for efficient processing
- Configurable timer-based PWM generation

### Project Structure
```
YFOC__ST/
├── Core/
│   ├── Inc/          # Header files
│   └── Src/          # Source files
├── Drivers/          # STM32 HAL drivers
├── Debug/            # Build output files
└── CMakeLists.txt    # CMake build configuration
```

### Build Instructions
1. **Prerequisites**:
   - ARM GCC toolchain (`arm-none-eabi-gcc`)
   - CMake (version 3.30 or higher)
   - STM32CubeIDE or compatible development environment

2. **Build Steps**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Output Files**:
   - `YFOC__ST.elf` - Executable file
   - `YFOC__ST.hex` - Intel HEX format for flashing
   - `YFOC__ST.bin` - Binary format

### Configuration
The project is configured through STM32CubeMX (`.ioc` file) with the following peripherals:
- **CAN**: 1Mbps baud rate for motor communication
- **I2C1**: For AS5600 encoder communication
- **UART**: Debug output and monitoring
- **Timers**: PWM generation and timing control
- **DMA**: Efficient data transfer

### Usage
1. Flash the firmware to STM32F103C8
2. Connect AS5600 encoder via I2C
3. Configure CAN bus communication
4. Send control commands via CAN (ID: 0x101)
5. Monitor feedback via CAN (ID base: 0x111)


## Chinese

### 项目概述
这是一个基于STM32F103C8的YFOC（磁场定向控制）项目。该项目利用多种外设包括CAN、I2C、UART和定时器来实现电机控制应用。

### 硬件要求
- **主控芯片**: STM32F103C8T6 
- **编码器**: AS5600磁性编码器 (I2C接口)
- **通信**: CAN总线用于电机控制指令
- **调试**: UART用于调试和监控

### 主要特性
- CAN总线通信用于电机控制
- 集成AS5600磁性编码器进行位置反馈
- 实时电机控制算法
- 基于DMA的数据传输，提高处理效率
- 可配置的基于定时器的PWM生成

### 项目结构
```
YFOC__ST/
├── Core/
│   ├── Inc/          # 头文件
│   └── Src/          # 源文件
├── Drivers/          # STM32 HAL驱动
├── Debug/            # 构建输出文件
└── CMakeLists.txt    # CMake构建配置
```

### 构建说明
1. **前置要求**:
   - ARM GCC工具链 (`arm-none-eabi-gcc`)
   - CMake (版本3.30或更高)
   - STM32CubeIDE或兼容的开发环境

2. **构建步骤**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **输出文件**:
   - `YFOC__ST.elf` - 可执行文件
   - `YFOC__ST.hex` - Intel HEX格式用于烧录
   - `YFOC__ST.bin` - 二进制格式

### 配置说明
项目通过STM32CubeMX（`.ioc`文件）配置，包含以下外设：
- **CAN**: 1Mbps波特率用于电机通信
- **I2C1**: 用于AS5600编码器通信
- **UART**: 调试输出和监控
- **定时器**: PWM生成和时序控制
- **DMA**: 高效数据传输

### 使用方法
1. 将固件烧录到STM32F103C8
2. 通过I2C连接AS5600编码器
3. 配置CAN总线通信
4. 通过CAN发送控制指令 (ID: 0x101)
5. 通过CAN监控反馈 (ID基址: 0x111)

