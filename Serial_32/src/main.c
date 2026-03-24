#include "Serial.h"

static int16_t s_uart1_values[UART1_PARSED_MAX_COUNT];
static uint8_t s_uart1_value_count;
static Serial_Stats_t s_stats_snapshot;

static void App_HandleUart1Data(void)
{
	if (Serial_UART1_GetParsedMilli(s_uart1_values, UART1_PARSED_MAX_COUNT, &s_uart1_value_count) == 1U)
	{
		uint8_t i;
		for (i = 0U; i < s_uart1_value_count; i++)
		{
			// s_uart1_values[i] 为定点毫值（例如 1234 表示 1.234）
		}
	}
}

static void App_HandleUart2Data(void)
{
	if (g_uart2_ctx.data_len >= 4U)
	{
		uint8_t cmd = g_uart2_ctx.rx_buffer_ready[2];
		(void)cmd;
		// 在这里根据 cmd 做业务处理
	}
}

void App_MainLoop(void)
{
	uint32_t last_stats_tick = 0U;

	while (1)
	{
		Serial_UART1_Process();
		App_HandleUart1Data();

		if (Serial_GetRxFlag2() == 1U)
		{
			App_HandleUart2Data();
		}

		if ((HAL_GetTick() - last_stats_tick) >= 1000U)
		{
			last_stats_tick = HAL_GetTick();
			Serial_GetStats(&s_stats_snapshot);
			// 可在此处上报/打印 s_stats_snapshot
		}
	}
}

