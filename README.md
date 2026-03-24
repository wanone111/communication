# Serial

## Serial_32

STM32 端串口通信代码，基于 HAL 库实现。

- 芯片：STM32F103C8T6
- 主要功能：
- USART1 与蓝牙模块通信（无人机与地面端链路）
- USART2 与串口屏通信
- 采用中断 + 状态机 + 包级环形队列进行接收

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
- `rx_buffer_ready`：就绪缓冲区（UART2 兼容保留）
- `rx_flag`：数据就绪提示标志
- `rx_state`：状态机当前状态
- `rx_index`：接收索引
- `data_len`：负载长度
- `checksum_calc`：校验和累加器

UART1 额外采用包级环形队列（默认深度 8）：

- ISR 校验通过后入队完整数据包
- 主循环出队并解析，避免处理慢时覆盖最新帧

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
- 校验通过后，将完整包写入 UART1 环形队列
- `Serial_UART1_Process()` 中按两字节一组解析为 `int16_t` 毫值（高字节在前）
- UART1 状态机内部采用“状态处理函数表”分发，扩展协议时只需新增/修改对应状态处理函数
- UART2 状态机也采用相同的函数表分发风格，便于两条链路统一维护

#### UART2 协议（串口屏链路）

帧格式：

```text
[0xFF][0x0F][DATA][0xFE]
```

- 固定 4 字节结构
- 状态机在接收到尾字节后置位 `rx_flag=1`
- UART2 就绪数据采用无锁双缓冲发布，主循环通过 `Serial_GetRxFlag2()` 获取稳定快照，无需关中断

### 对外接口

`Serial.h` 暴露接口如下：

- `void Serial_Init(void)`
- `void Serial_UART1_Process(void)`
- `uint8_t Serial_GetRxFlag1(void)`
- `uint8_t Serial_GetRxFlag2(void)`
- `uint8_t Serial_UART1_PopFrame(uint8_t *out_frame, uint8_t *out_len)`
- `uint8_t Serial_UART1_GetParsedMilli(int16_t *out_data, uint8_t max_count, uint8_t *out_count)`
- `void Serial_GetStats(Serial_Stats_t *out_stats)`
- `void Serial_TestFeedByte(uint8_t uart_id, uint8_t byte)`

说明：

- `Serial_Init()`：初始化上下文并开启 USART1/USART2 的单字节中断接收
- `Serial_GetRxFlag1()`：查询 UART1 队列是否有待处理数据
- `Serial_GetRxFlag2()`：读取并清除 UART2 接收完成标志
- `Serial_UART1_Process()`：在主循环中调用，消费 UART1 队列并完成解析
- `Serial_UART1_GetParsedMilli()`：仅在有新解析结果时返回 1，并在读取后清除“新数据”标志
- `Serial_GetStats()`：读取当前串口统计信息（成功帧、丢弃帧、校验错误等）
- `Serial_TestFeedByte()`：用于无硬件场景下向状态机注入测试字节（`SERIAL_UART1_ID` / `SERIAL_UART2_ID`）

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
	int16_t values[UART1_PARSED_MAX_COUNT];
	uint8_t value_count = 0U;

	if (Serial_UART1_GetParsedMilli(values, UART1_PARSED_MAX_COUNT, &value_count) == 1U)
	{
		// 处理 UART1 解析结果
	}

	if (Serial_GetRxFlag2() == 1U)
	{
		// 处理 UART2 就绪数据（从 g_uart2_ctx.rx_buffer_ready 读取）
	}
}
```

4. 如需调试打印，可在 `Serial.h` 中打开 `SERIAL_DEBUG` 宏。

### 注意事项

- `fputc/fgetc` 当前重定向到 `huart2`，如需改为其他串口请同步修改。
- `Serial_UART1_Process()` 只做驱动侧消费与基础解析，业务层建议通过 `Serial_UART1_GetParsedMilli()` 获取结果。
- UART1 校验策略是“逐字节累加低 8 位”，上位机发包应保持一致。
- UART2 协议保持兼容不变，但已增加格式错误和丢包统计。

### 轻量回归测试示例

```c
// UART2 测试帧: FF 0F 01 FE
Serial_TestFeedByte(SERIAL_UART2_ID, 0xFF);
Serial_TestFeedByte(SERIAL_UART2_ID, 0x0F);
Serial_TestFeedByte(SERIAL_UART2_ID, 0x01);
Serial_TestFeedByte(SERIAL_UART2_ID, 0xFE);

if (Serial_GetRxFlag2() == 1U)
{
	// 断言 g_uart2_ctx.rx_buffer_ready[2] == 0x01
}
```

## Serial_ROS2

参考cgc的[serial_driver_ros](https://github.com/BoomBoomFly/serial_driver_ros)