#include "Serial.h"

/**
  * UART1 RX A9 TX A10  与蓝牙模块连接，用于无人机与地面通信
  * UART2 RX A3  TX A2  与串口屏连接，用于32与串口屏通信
  */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// 全局上下文实例
Serial_Context_t g_uart1_ctx;
Serial_Context_t g_uart2_ctx;

typedef struct {
    uint8_t frame[MAX_FRAME_LEN];
    uint8_t frame_len;
} UART1_FrameNode_t;

typedef enum {
    UART1_STATE_WAIT_HEAD1 = 0,
    UART1_STATE_WAIT_HEAD2,
    UART1_STATE_WAIT_LEN,
    UART1_STATE_WAIT_PAYLOAD,
    UART1_STATE_WAIT_CHECKSUM,
    UART1_STATE_COUNT
} UART1_RxState_t;

typedef enum {
    UART2_STATE_WAIT_HEAD1 = 0,
    UART2_STATE_WAIT_HEAD2,
    UART2_STATE_WAIT_DATA,
    UART2_STATE_WAIT_TAIL,
    UART2_STATE_COUNT
} UART2_RxState_t;

static volatile UART1_FrameNode_t s_uart1_queue[UART1_FRAME_QUEUE_DEPTH];
static volatile uint8_t s_uart1_q_head;
static volatile uint8_t s_uart1_q_tail;
static volatile uint8_t s_uart1_q_count;

static volatile Serial_Stats_t s_serial_stats;

static volatile uint8_t s_uart2_ready_buffers[2][MAX_FRAME_LEN];
static volatile uint8_t s_uart2_ready_len[2];
static volatile uint8_t s_uart2_read_idx;
static volatile uint8_t s_uart2_write_idx;
static volatile uint8_t s_uart2_produced_seq;
static volatile uint8_t s_uart2_consumed_seq;

// 解析后的数据存储 (保持原有逻辑)
static int16_t parsed_data[UART1_PARSED_MAX_COUNT];
static uint8_t parsed_count;
static uint8_t parsed_ready;

typedef void (*UART1_StateHandler_t)(uint8_t byte);
typedef void (*UART2_StateHandler_t)(uint8_t byte);

static uint8_t UART1_QueuePush(const volatile uint8_t *frame, uint8_t frame_len);

static uint32_t Serial_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Serial_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Serial_ResetRxContext(Serial_Context_t *ctx)
{
    if (ctx != NULL)
    {
        ctx->rx_state = 0U;
        ctx->rx_index = 0U;
    }
}

static uint8_t Serial_ContextPushByte(Serial_Context_t *ctx, uint8_t byte)
{
    if ((ctx == NULL) || (ctx->rx_index >= MAX_FRAME_LEN))
    {
        return 0U;
    }

    ctx->rx_buffer[ctx->rx_index++] = byte;
    return 1U;
}

static void UART1_ResetRxState(void)
{
    Serial_ResetRxContext(&g_uart1_ctx);
}

static void UART2_ResetRxState(void)
{
    Serial_ResetRxContext(&g_uart2_ctx);
}

static uint8_t UART1_PushPayloadByte(uint8_t byte)
{
    if (Serial_ContextPushByte(&g_uart1_ctx, byte) == 0U)
    {
        s_serial_stats.uart1_len_err++;
        UART1_ResetRxState();
        return 0U;
    }

    g_uart1_ctx.checksum_calc += byte;
    return 1U;
}

static void UART1_HandleWaitHead1(uint8_t byte)
{
    if (byte == UART1_FRAME_HEAD1)
    {
        g_uart1_ctx.checksum_calc = byte;
        g_uart1_ctx.rx_state = UART1_STATE_WAIT_HEAD2;
    }
}

static void UART1_HandleWaitHead2(uint8_t byte)
{
    if (byte == UART1_FRAME_HEAD2)
    {
        g_uart1_ctx.checksum_calc += byte;
        g_uart1_ctx.rx_state = UART1_STATE_WAIT_LEN;
    }
    else
    {
        UART1_ResetRxState();
    }
}

static void UART1_HandleWaitLen(uint8_t byte)
{
    if ((byte > 0U) && (byte < MAX_FRAME_LEN))
    {
        g_uart1_ctx.rx_index = 0U;
        (void)Serial_ContextPushByte(&g_uart1_ctx, byte);
        g_uart1_ctx.data_len = byte;
        g_uart1_ctx.checksum_calc += byte;
        g_uart1_ctx.rx_state = UART1_STATE_WAIT_PAYLOAD;
    }
    else
    {
        s_serial_stats.uart1_len_err++;
        UART1_ResetRxState();
    }
}

static void UART1_HandleWaitPayload(uint8_t byte)
{
    if (UART1_PushPayloadByte(byte) == 0U)
    {
        return;
    }

    if (g_uart1_ctx.rx_index >= (1U + g_uart1_ctx.data_len))
    {
        g_uart1_ctx.rx_state = UART1_STATE_WAIT_CHECKSUM;
    }
}

static void UART1_HandleWaitChecksum(uint8_t byte)
{
    if (byte == (g_uart1_ctx.checksum_calc & 0xFFU))
    {
        if (UART1_QueuePush(g_uart1_ctx.rx_buffer, g_uart1_ctx.rx_index) == 1U)
        {
            s_serial_stats.uart1_rx_ok++;
        }
    }
    else
    {
        s_serial_stats.uart1_checksum_err++;
        #ifdef SERIAL_DEBUG
        printf("Checksum error, expect 0x%02X, got 0x%02X\r\n", (g_uart1_ctx.checksum_calc & 0xFFU), byte);
        #endif
    }

    UART1_ResetRxState();
}

static const UART1_StateHandler_t s_uart1_handlers[UART1_STATE_COUNT] = {
    UART1_HandleWaitHead1,
    UART1_HandleWaitHead2,
    UART1_HandleWaitLen,
    UART1_HandleWaitPayload,
    UART1_HandleWaitChecksum
};

static void UART2_StartFrame(void)
{
    UART2_ResetRxState();
    if (Serial_ContextPushByte(&g_uart2_ctx, UART2_FRAME_HEAD1) == 1U)
    {
        g_uart2_ctx.rx_state = UART2_STATE_WAIT_HEAD2;
    }
    else
    {
        s_serial_stats.uart2_rx_drop++;
        UART2_ResetRxState();
    }
}

static uint8_t UART2_PushFrameByte(uint8_t byte)
{
    if (Serial_ContextPushByte(&g_uart2_ctx, byte) == 0U)
    {
        s_serial_stats.uart2_rx_drop++;
        UART2_ResetRxState();
        return 0U;
    }
    return 1U;
}

static uint8_t UART2_CommitFrame(void)
{
    uint8_t i;
    uint8_t write_idx;

    if (g_uart2_ctx.rx_index >= MAX_FRAME_LEN)
    {
        s_serial_stats.uart2_rx_drop++;
        return 0U;
    }

    g_uart2_ctx.rx_buffer[g_uart2_ctx.rx_index++] = UART2_FRAME_TAIL;
    g_uart2_ctx.data_len = g_uart2_ctx.rx_index;

    write_idx = s_uart2_write_idx;
    for (i = 0U; i < g_uart2_ctx.data_len; i++)
    {
        s_uart2_ready_buffers[write_idx][i] = g_uart2_ctx.rx_buffer[i];
    }
    s_uart2_ready_len[write_idx] = g_uart2_ctx.data_len;

    // 发布顺序：先写完整缓冲，再切换索引和序号，主循环只读取已发布索引
    s_uart2_read_idx = write_idx;
    s_uart2_write_idx = (uint8_t)(write_idx ^ 0x01U);
    s_uart2_produced_seq++;
    g_uart2_ctx.rx_flag = 1U;
    s_serial_stats.uart2_rx_ok++;
    return 1U;
}

static void UART2_HandleWaitHead1(uint8_t byte)
{
    if (byte == UART2_FRAME_HEAD1)
    {
        UART2_StartFrame();
    }
}

static void UART2_HandleWaitHead2(uint8_t byte)
{
    if (byte == UART2_FRAME_HEAD2)
    {
        g_uart2_ctx.rx_state = UART2_STATE_WAIT_DATA;
        (void)UART2_PushFrameByte(byte);
    }
    else if (byte == UART2_FRAME_HEAD1)
    {
        // 允许在头2状态下快速重同步到新帧头
        UART2_StartFrame();
    }
    else
    {
        s_serial_stats.uart2_format_err++;
        UART2_ResetRxState();
    }
}

static void UART2_HandleWaitData(uint8_t byte)
{
    if (UART2_PushFrameByte(byte) == 0U)
    {
        return;
    }

    if (g_uart2_ctx.rx_index == 3U)
    {
        g_uart2_ctx.rx_state = UART2_STATE_WAIT_TAIL;
    }
}

static void UART2_HandleWaitTail(uint8_t byte)
{
    if (byte == UART2_FRAME_TAIL)
    {
        (void)UART2_CommitFrame();
    }
    else
    {
        s_serial_stats.uart2_format_err++;
    }

    UART2_ResetRxState();
}

static const UART2_StateHandler_t s_uart2_handlers[UART2_STATE_COUNT] = {
    UART2_HandleWaitHead1,
    UART2_HandleWaitHead2,
    UART2_HandleWaitData,
    UART2_HandleWaitTail
};

static uint8_t UART1_QueuePush(const volatile uint8_t *frame, uint8_t frame_len)
{
    uint8_t i;
    uint8_t slot;

    if ((frame == NULL) || (frame_len == 0U) || (frame_len > MAX_FRAME_LEN))
    {
        return 0;
    }

    if (s_uart1_q_count >= UART1_FRAME_QUEUE_DEPTH)
    {
        s_serial_stats.uart1_rx_drop++;
        return 0;
    }

    slot = s_uart1_q_head;
    s_uart1_queue[slot].frame_len = frame_len;
    for (i = 0; i < frame_len; i++)
    {
        s_uart1_queue[slot].frame[i] = frame[i];
    }

    s_uart1_q_head = (uint8_t)((s_uart1_q_head + 1U) % UART1_FRAME_QUEUE_DEPTH);
    s_uart1_q_count++;
    if (s_uart1_q_count > s_serial_stats.uart1_queue_peak)
    {
        s_serial_stats.uart1_queue_peak = s_uart1_q_count;
    }

    g_uart1_ctx.rx_flag = 1U;
    return 1;
}

/**
  * 函数功能: 串口初始化，开启接收中断
  */
void Serial_Init(void)
{
    // 初始化上下文
    memset(&g_uart1_ctx, 0, sizeof(g_uart1_ctx));
    memset(&g_uart2_ctx, 0, sizeof(g_uart2_ctx));
    memset((void *)s_uart1_queue, 0, sizeof(s_uart1_queue));
    memset((void *)&s_serial_stats, 0, sizeof(s_serial_stats));
    memset((void *)s_uart2_ready_buffers, 0, sizeof(s_uart2_ready_buffers));
    memset((void *)s_uart2_ready_len, 0, sizeof(s_uart2_ready_len));
    memset(parsed_data, 0, sizeof(parsed_data));
    parsed_count = 0U;
    parsed_ready = 0U;
    s_uart2_read_idx = 0U;
    s_uart2_write_idx = 0U;
    s_uart2_produced_seq = 0U;
    s_uart2_consumed_seq = 0U;
    s_uart1_q_head = 0U;
    s_uart1_q_tail = 0U;
    s_uart1_q_count = 0U;

    // 开启中断接收
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_ctx.rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_uart2_ctx.rx_byte, 1);
}

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx (这里假设是UART2)
  */
int fputc(int ch, FILE *stream)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 100);
  return ch;
}

/**
  * 函    数：获取串口1接收数据包标志位
  */
uint8_t Serial_GetRxFlag1(void)
{
    uint8_t has_data;
    uint32_t primask = Serial_EnterCritical();
    has_data = (s_uart1_q_count > 0U) ? 1U : 0U;
    g_uart1_ctx.rx_flag = has_data;
    Serial_ExitCritical(primask);
    return has_data;
}

/**
  * 函    数：获取串口2接收数据包标志位
  */
uint8_t Serial_GetRxFlag2(void)
{
    uint8_t produced;

    produced = s_uart2_produced_seq;
    if (produced != s_uart2_consumed_seq)
    {
        uint8_t i;
        uint8_t read_idx = s_uart2_read_idx;
        uint8_t len = s_uart2_ready_len[read_idx];

        for (i = 0U; i < len; i++)
        {
            g_uart2_ctx.rx_buffer_ready[i] = s_uart2_ready_buffers[read_idx][i];
        }
        g_uart2_ctx.data_len = len;

        s_uart2_consumed_seq = produced;
        g_uart2_ctx.rx_flag = (s_uart2_produced_seq != s_uart2_consumed_seq) ? 1U : 0U;
        return 1U;
    }

    g_uart2_ctx.rx_flag = 0U;
    return 0U;
}

uint8_t Serial_UART1_PopFrame(uint8_t *out_frame, uint8_t *out_len)
{
    uint8_t i;
    uint8_t local_len;
    uint8_t local_frame[MAX_FRAME_LEN];
    uint32_t primask;

    if ((out_frame == NULL) || (out_len == NULL))
    {
        return 0;
    }

    primask = Serial_EnterCritical();
    if (s_uart1_q_count == 0U)
    {
        g_uart1_ctx.rx_flag = 0U;
        Serial_ExitCritical(primask);
        return 0;
    }

    local_len = s_uart1_queue[s_uart1_q_tail].frame_len;
    for (i = 0; i < local_len; i++)
    {
        local_frame[i] = s_uart1_queue[s_uart1_q_tail].frame[i];
    }

    s_uart1_q_tail = (uint8_t)((s_uart1_q_tail + 1U) % UART1_FRAME_QUEUE_DEPTH);
    s_uart1_q_count--;
    g_uart1_ctx.rx_flag = (s_uart1_q_count > 0U) ? 1U : 0U;
    Serial_ExitCritical(primask);

    for (i = 0; i < local_len; i++)
    {
        out_frame[i] = local_frame[i];
    }
    *out_len = local_len;

    return 1;
}

uint8_t Serial_UART1_GetParsedMilli(int16_t *out_data, uint8_t max_count, uint8_t *out_count)
{
    uint8_t i;
    uint8_t copy_count;
    uint32_t primask;

    if ((out_data == NULL) || (out_count == NULL) || (max_count == 0U))
    {
        return 0;
    }

    primask = Serial_EnterCritical();
    if (parsed_ready == 0U)
    {
        *out_count = 0U;
        Serial_ExitCritical(primask);
        return 0U;
    }

    copy_count = (parsed_count < max_count) ? parsed_count : max_count;
    for (i = 0; i < copy_count; i++)
    {
        out_data[i] = parsed_data[i];
    }
    *out_count = copy_count;
    parsed_ready = 0U;
    Serial_ExitCritical(primask);

    return 1U;
}

void Serial_GetStats(Serial_Stats_t *out_stats)
{
    uint32_t primask;

    if (out_stats == NULL)
    {
        return;
    }

    primask = Serial_EnterCritical();
    memcpy(out_stats, (const void *)&s_serial_stats, sizeof(*out_stats));
    Serial_ExitCritical(primask);
}

/**
  * 函数功能: 串口1处理数据 (原 uart1_data)
  */
void Serial_UART1_Process(void)
{
    uint8_t frame[MAX_FRAME_LEN];
    uint8_t frame_len;

    if (parsed_ready == 1U)
    {
        return;
    }

    while (Serial_UART1_PopFrame(frame, &frame_len) == 1U)
    {
        uint8_t payload_len;
        uint8_t i;
        uint8_t count = 0U;

        if (frame_len == 0U)
        {
            continue;
        }

        payload_len = frame[0];
        if ((payload_len + 1U) != frame_len)
        {
            s_serial_stats.uart1_len_err++;
            continue;
        }

        if ((payload_len % 2U) != 0U)
        {
            s_serial_stats.uart1_len_err++;
            continue;
        }

        for (i = 1U; (i + 1U) <= payload_len && count < UART1_PARSED_MAX_COUNT; i += 2U)
        {
            int16_t raw = (int16_t)(((uint16_t)frame[i] << 8) | frame[i + 1U]);
            parsed_data[count++] = raw;
        }

        parsed_count = count;
        parsed_ready = 1U;

        #ifdef SERIAL_DEBUG
        printf("uart1 frame len=%u, parsed=%u\r\n", payload_len, count);
        #endif

        // 每次主循环最多产出一帧解析结果，避免未消费时被覆盖
        break;
    }
}

/**
  * 函数功能: 串口1接收状态机 (内部调用)
  */
static void UART1_RxStateMachine(uint8_t byte)
{
    uint8_t state = g_uart1_ctx.rx_state;

    if (state >= UART1_STATE_COUNT)
    {
        UART1_ResetRxState();
        return;
    }

    s_uart1_handlers[state](byte);
}

/**
  * 函数功能: 串口2接收状态机 (内部调用)
  */
static void UART2_RxStateMachine(uint8_t byte)
{
    uint8_t state = g_uart2_ctx.rx_state;

    if (state >= UART2_STATE_COUNT)
    {
        UART2_ResetRxState();
        return;
    }

    s_uart2_handlers[state](byte);
}

void Serial_TestFeedByte(uint8_t uart_id, uint8_t byte)
{
    if (uart_id == SERIAL_UART1_ID)
    {
        UART1_RxStateMachine(byte);
    }
    else if (uart_id == SERIAL_UART2_ID)
    {
        UART2_RxStateMachine(byte);
    }
}

/**
  * 函数功能: 串口接收中断回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        UART1_RxStateMachine(g_uart1_ctx.rx_byte);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_ctx.rx_byte, 1);
    }
    else if(huart->Instance == USART2)
    {
        UART2_RxStateMachine(g_uart2_ctx.rx_byte);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_uart2_ctx.rx_byte, 1);
    }
}

/**
  * 函数功能: 串口错误回调函数 (增强健壮性)
  * 说明: 处理Overrun等错误，防止接收中断锁死
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    s_serial_stats.uart_error_recover++;

    // 清除错误标志 (读取SR后读取DR可以清除大多数错误标志)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    // 重新开启接收，防止中断链断裂
    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_ctx.rx_byte, 1);
    }
    else if(huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_uart2_ctx.rx_byte, 1);
    }
}

