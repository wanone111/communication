# Communication

该目录用于存放项目通信相关代码。目前串口通信代码位于 `Serial` 目录。

## 目录结构

- `Serial/Serial_32`：STM32 端串口收发与数据解析代码。
- `Serial/Serial_ROS2`：ROS2 端串口代码预留目录，当前相关源文件为空。

## Serial_32

`Serial_32` 面向 STM32F103C8T6，使用 STM32 HAL UART 接口。当前代码文件：

- `include/main.h`：基础头文件。
- `include/serial.h`：串口宏定义、全局变量声明和函数声明。
- `src/Serial.c`：UART1/UART2 接收状态机、`printf` 重定向和接收回调。

### UART 用途

- `USART1`：`RX B15`、`TX B14`，用于无人机与地面端通信。
- `USART2`：`RX A3`、`TX A2`，用于 STM32 与串口屏通信，同时作为 `printf` 输出串口。

### 帧格式

`USART1` 接收帧：

```text
0x0F 0xF0 LEN DATA... CHECKSUM
```

- `LEN`：数据区长度，最大受 `MAX_FRAME_LEN` 限制。
- `DATA`：按高字节在前、低字节在后的 `int16_t` 数据解析。
- `CHECKSUM`：`0x0F + 0xF0 + LEN + DATA...` 的低 8 位。

`USART2` 接收帧：

```text
0xFF 0x0F DATA 0xFE
```

### 已完成的优化

- 补齐 `serial.h`，集中定义帧头、帧尾、缓冲区长度和函数声明。
- 删除 `Serial.c` 开头会导致编译失败的裸文本。
- 修复串口 1 接收长度边界检查。
- 将接收索引改为 `uint16_t`，避免 256 字节帧索引溢出。
- 修复 `int16_t` 数据解析，支持负数。
- 修复 `fputc` 发送 `int` 地址的问题。
- 给 UART 回调增加空指针保护。

### 已验证内容

已使用临时 STM32 HAL 桩头文件进行 `gcc -fsyntax-only` 语法检查：

- 默认 `MAX_FRAME_LEN=256U`：通过。
- 覆盖 `MAX_FRAME_LEN=64U`：通过。

### 未验证内容

当前目录未包含完整 STM32CubeMX/STM32F1 HAL 工程、启动文件、链接脚本和芯片配置，因此尚未验证 STM32F103C8T6 真实固件构建与上板运行。

集成到 STM32F103C8T6 工程时，需要确保工程已正确引入 STM32F1 HAL，例如常见 CubeMX 工程中的 `stm32f1xx_hal.h`、`usart.c/.h` 和 `UART_HandleTypeDef huart1/huart2`。
