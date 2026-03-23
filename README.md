# Serial

## Serial_32

STM32 端串口通信代码，基于 HAL 库实现。

- 芯片：STM32F103C8T6
- 主要功能：
- USART1 与蓝牙模块通信（无人机与地面端链路）
- USART2 与串口屏通信
- 采用中断 + 状态机 + 双缓冲机制进行接收

### 目录结构

```text
Serial_32/
	include/
		main.h
		Serial.h
	src/
		Serial.c
```

### 模块设计

`Serial.c` 中维护两个全局上下文：

- `g_uart1_ctx`：USART1 接收上下文
- `g_uart2_ctx`：USART2 接收上下文

上下文结构体 `Serial_Context_t` 主要字段：

- `rx_byte`：中断单字节接收缓存
- `rx_buffer`：工作缓冲区（ISR 写入）
- `rx_buffer_ready`：就绪缓冲区（主循环读取）
- `rx_flag`：一包数据完成标志
- `rx_state`：状态机当前状态
- `rx_index`：接收索引
- `data_len`：负载长度
- `checksum_calc`：校验和累加器

### 协议定义

协议常量位于 `Serial.h`：

- UART1 帧头：`0x0F 0xF0`
- UART2 帧头：`0xFF 0x0F`
- UART2 帧尾：`0xFE`
- 最大帧长：`MAX_FRAME_LEN = 255`

#### UART1 协议（蓝牙链路）

帧格式：

```text
[HEAD1][HEAD2][LEN][DATA...][CHECKSUM]
```

- `HEAD1 = 0x0F`
- `HEAD2 = 0xF0`
- `LEN`：数据段字节数
- `CHECKSUM`：从 `HEAD1` 到 `DATA` 的逐字节和，取低 8 位

接收解析逻辑：

- 状态机按“帧头 -> 长度 -> 数据 -> 校验”推进
- 校验通过后，将工作缓冲区复制到 `rx_buffer_ready` 并置 `rx_flag=1`
- `Serial_UART1_Process()` 中按两字节一组解析数据（高字节在前）

#### UART2 协议（串口屏链路）

帧格式：

```text
[0xFF][0x0F][DATA][0xFE]
```

- 固定 4 字节结构
- 状态机在接收到尾字节后置位 `rx_flag=1`

### 对外接口

`Serial.h` 暴露接口如下：

- `void Serial_Init(void)`
- `void Serial_UART1_Process(void)`
- `uint8_t Serial_GetRxFlag1(void)`
- `uint8_t Serial_GetRxFlag2(void)`

说明：

- `Serial_Init()`：初始化上下文并开启 USART1/USART2 的单字节中断接收
- `Serial_GetRxFlag1/2()`：读取并清除接收完成标志
- `Serial_UART1_Process()`：在主循环中调用，处理 UART1 的就绪数据

### 中断与错误处理

- `HAL_UART_RxCpltCallback()`：根据 UART 实例分发到对应状态机，并重新挂载下一次中断接收
- `HAL_UART_ErrorCallback()`：清除 ORE/NE/FE/PE 错误标志并重新开启接收，避免中断链断裂

### 使用方法

1. 在工程初始化完成后调用 `Serial_Init()`。
2. 确保 `huart1`、`huart2` 已在 CubeMX 或工程代码中正确初始化。
3. 在主循环中轮询：

```c
while (1)
{
		Serial_UART1_Process();

		if (Serial_GetRxFlag2())
		{
				// 处理 UART2 就绪数据（从 g_uart2_ctx.rx_buffer_ready 或 rx_buffer 读取）
		}
}
```

4. 如需调试打印，可在 `Serial.h` 中打开 `SERIAL_DEBUG` 宏。

### 注意事项

- `fputc/fgetc` 当前重定向到 `huart2`，如需改为其他串口请同步修改。
- `Serial_UART1_Process()` 内部示例将解析值写入静态数组 `parsed_data[256]`，如需业务层使用，建议增加拷贝接口或回调。
- UART1 校验策略是“逐字节累加低 8 位”，上位机发包应保持一致。
- UART2 当前仅完成接收标志置位，业务解析可在主循环中补充。

## Serial_ROS2

参考cgc的[serial_driver_ros](https://github.com/BoomBoomFly/serial_driver_ros)