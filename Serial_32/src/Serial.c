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

// 解析后的数据存储 (保持原有逻辑)
static uint16_t parsed_data[256]; 

/**
  * 函数功能: 串口初始化，开启接收中断
  */
void Serial_Init(void)
{
    // 初始化上下文
    memset(&g_uart1_ctx, 0, sizeof(g_uart1_ctx));
    memset(&g_uart2_ctx, 0, sizeof(g_uart2_ctx));

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
    if (g_uart1_ctx.rx_flag == 1)
    {
        g_uart1_ctx.rx_flag = 0;
        return 1;
    }
    return 0;
}

/**
  * 函    数：获取串口2接收数据包标志位
  */
uint8_t Serial_GetRxFlag2(void)
{
    if (g_uart2_ctx.rx_flag == 1)
    {
        g_uart2_ctx.rx_flag = 0;
        return 1;
    }
    return 0;
}

/**
  * 函数功能: 串口1处理数据 (原 uart1_data)
  */
void Serial_UART1_Process(void)
{
    if(Serial_GetRxFlag1() == 1)
    {
        #ifdef SERIAL_DEBUG
        printf("orin: ");
        // 注意：这里使用 g_uart1_ctx.data_len
        for(uint8_t i = 0; i <= g_uart1_ctx.data_len; i++) 
        {
            printf("0x%02X ", g_uart1_ctx.rx_buffer_ready[i]);
        }
        printf("\r\n");
        #endif

        uint8_t count = 0;
        // 解析逻辑保持不变，但使用结构体成员
        for(uint8_t i = 1; i + 1 <= g_uart1_ctx.data_len; i += 2)
        {
            int32_t raw = (g_uart1_ctx.rx_buffer_ready[i] << 8) | g_uart1_ctx.rx_buffer_ready[i + 1];
            parsed_data[count++] = (float)raw / 1000.0f;
        }

        #ifdef SERIAL_DEBUG
        printf(" (process%d):\r\n", count);
        for(uint8_t i = 0; i < count; i++)
        {
            printf("data[%d] = %f\r\n", i, parsed_data[i]);
        }
        #endif
    }
}

/**
  * 函数功能: 串口1接收状态机 (内部调用)
  */
static void UART1_RxStateMachine(uint8_t byte)
{
    switch (g_uart1_ctx.rx_state)
    {
    case 0: // 帧头1
        if(byte == UART1_FRAME_HEAD1)
        {
            g_uart1_ctx.checksum_calc = byte;
            g_uart1_ctx.rx_state = 1;
        }
        break;
    case 1: // 帧头2
        if(byte == UART1_FRAME_HEAD2)
        {
            g_uart1_ctx.checksum_calc += byte;
            g_uart1_ctx.rx_state = 2;
        }
        else
        {
            g_uart1_ctx.rx_state = 0;
        }
        break;
    case 2: // 数据长度
        // 简单的长度检查，防止缓冲区溢出
        if(byte < MAX_FRAME_LEN) 
        {
            g_uart1_ctx.rx_index = 0;
            g_uart1_ctx.rx_buffer[g_uart1_ctx.rx_index++] = byte;
            g_uart1_ctx.data_len = byte;
            g_uart1_ctx.checksum_calc += byte;
            
            if(byte == 0) // 长度为0的情况
            {
                g_uart1_ctx.rx_state = 4;
            }
            else
            {
                g_uart1_ctx.rx_state = 3;
            }
        }
        else
        {
            g_uart1_ctx.rx_state = 0;
            g_uart1_ctx.rx_index = 0;
        }
        break;
    case 3: // 数据内容
        g_uart1_ctx.rx_buffer[g_uart1_ctx.rx_index++] = byte;
        g_uart1_ctx.checksum_calc += byte;
        if(g_uart1_ctx.rx_index >= (1 + g_uart1_ctx.data_len))
        {
            g_uart1_ctx.rx_state = 4;
        }
        break;
    case 4: // 校验和
        if(byte == (g_uart1_ctx.checksum_calc & 0xFF))
        {
            // 双缓冲机制：将完整包复制到就绪缓冲，释放工作缓冲给ISR继续使用
            memcpy((void*)g_uart1_ctx.rx_buffer_ready, (void*)g_uart1_ctx.rx_buffer, g_uart1_ctx.rx_index);
            g_uart1_ctx.rx_flag = 1;
        }
        else
        {
            #ifdef SERIAL_DEBUG
            printf("Checksum error, expect 0x%02X, got 0x%02X\r\n", (g_uart1_ctx.checksum_calc & 0xFF), byte);
            #endif
        }
        g_uart1_ctx.rx_state = 0;
        g_uart1_ctx.rx_index = 0;
        break;
    default:
        g_uart1_ctx.rx_state = 0;
        g_uart1_ctx.rx_index = 0;
        break;
    }
}

/**
  * 函数功能: 串口2接收状态机 (内部调用)
  */
static void UART2_RxStateMachine(uint8_t byte)
{
    // 使用临时缓冲区来匹配原逻辑的 rx2_buf[6]
    // 原逻辑：0xFF -> 0x0F -> Data -> 0xFE
    // 这里直接存入 g_uart2_ctx.rx_buffer
    
    switch (g_uart2_ctx.rx_state)
    {
        case 0:
            if(byte == UART2_FRAME_HEAD1)
            {
                g_uart2_ctx.rx_state = 1;
                g_uart2_ctx.rx_index = 0;
                g_uart2_ctx.rx_buffer[g_uart2_ctx.rx_index++] = byte;
            }
            break;
        case 1:
            if(byte == UART2_FRAME_HEAD2)
            {
                g_uart2_ctx.rx_state = 2;
                g_uart2_ctx.rx_buffer[g_uart2_ctx.rx_index++] = byte;
            }
            else
            {
                g_uart2_ctx.rx_state = 0;
                g_uart2_ctx.rx_index = 0;
            }
            break;
        case 2:
            if (g_uart2_ctx.rx_index < MAX_FRAME_LEN) {
                g_uart2_ctx.rx_buffer[g_uart2_ctx.rx_index++] = byte;
            } else {
                // 缓冲区溢出保护
                g_uart2_ctx.rx_state = 0;
                g_uart2_ctx.rx_index = 0;
            }
            
            if(g_uart2_ctx.rx_index == 3) // 接收完第3个字节 (Data)
            {
                g_uart2_ctx.rx_state = 3;
            }
            break;
        case 3:
            if(byte == UART2_FRAME_TAIL)
            {
                if (g_uart2_ctx.rx_index < MAX_FRAME_LEN) {
                    g_uart2_ctx.rx_buffer[g_uart2_ctx.rx_index++] = byte;
                    g_uart2_ctx.rx_flag = 1;
                }
            }
            g_uart2_ctx.rx_state = 0;
            g_uart2_ctx.rx_index = 0;
            break;
        default:
            g_uart2_ctx.rx_state = 0;
            g_uart2_ctx.rx_index = 0;
            break;
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
    uint32_t isrflags   = READ_REG(huart->Instance->SR);
    
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

