#ifndef SERIAL_H
#define SERIAL_H

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h> // Added for uint8_t, uint16_t
#include "stm32f1xx_hal.h" // Added for HAL types

// --- 协议常量定义 (消除魔术数字) ---
#define UART1_FRAME_HEAD1    0x0F
#define UART1_FRAME_HEAD2    0xF0
#define UART2_FRAME_HEAD1    0xFF
#define UART2_FRAME_HEAD2    0x0F
#define UART2_FRAME_TAIL     0xFE

#define MAX_FRAME_LEN        255
#define UART1_FRAME_QUEUE_DEPTH 8
#define UART1_PARSED_MAX_COUNT  128
#define SERIAL_UART1_ID       1U
#define SERIAL_UART2_ID       2U

// --- 数据结构封装 ---
typedef struct {
    volatile uint8_t  rx_byte;                   // 用于HAL库接收的单字节缓冲
    volatile uint8_t  rx_buffer[MAX_FRAME_LEN];  // 接收数据包缓冲区 (工作缓冲)
    volatile uint8_t  rx_buffer_ready[MAX_FRAME_LEN]; // 就绪数据包缓冲区 (双缓冲机制)
    volatile uint8_t  rx_flag;                   // 接收完成标志
    volatile uint8_t  rx_state;                  // 状态机状态
    volatile uint8_t  rx_index;                  // 缓冲区索引
    volatile uint8_t  data_len;                  // 有效数据长度
    volatile uint8_t  checksum_calc;             // 校验和计算过程变量
} Serial_Context_t;

typedef struct {
    uint32_t uart1_rx_ok;
    uint32_t uart1_rx_drop;
    uint32_t uart1_checksum_err;
    uint32_t uart1_len_err;
    uint32_t uart2_rx_ok;
    uint32_t uart2_rx_drop;
    uint32_t uart2_format_err;
    uint32_t uart_error_recover;
    uint8_t  uart1_queue_peak;
} Serial_Stats_t;

// #define SERIAL_DEBUG // 取消注释以开启调试打印

// --- 外部变量声明 ---
extern Serial_Context_t g_uart1_ctx;
extern Serial_Context_t g_uart2_ctx;

// --- 函数声明 ---
void Serial_Init(void);                 // 初始化
void Serial_UART1_Process(void);        // 数据处理
uint8_t Serial_GetRxFlag1(void);
uint8_t Serial_GetRxFlag2(void);
uint8_t Serial_UART1_PopFrame(uint8_t *out_frame, uint8_t *out_len);
uint8_t Serial_UART1_GetParsedMilli(int16_t *out_data, uint8_t max_count, uint8_t *out_count);
void Serial_GetStats(Serial_Stats_t *out_stats);
void Serial_TestFeedByte(uint8_t uart_id, uint8_t byte);

#endif /* SERIAL_H */