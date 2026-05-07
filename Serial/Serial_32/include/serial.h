#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_FRAME_LEN
#define MAX_FRAME_LEN 256U
#endif

#if (MAX_FRAME_LEN < 2U)
#error "MAX_FRAME_LEN must be at least 2"
#endif

#define SERIAL1_HEADER1 0x0FU
#define SERIAL1_HEADER2 0xF0U
#define SERIAL2_HEADER1 0xFFU
#define SERIAL2_HEADER2 0x0FU
#define SERIAL2_TAIL    0xFEU

#define SERIAL2_BUF_LEN 6U
#define SERIAL2_FRAME_LEN 4U
#define SERIAL1_MAX_DATA_COUNT ((MAX_FRAME_LEN - 1U) / 2U)

extern volatile uint8_t RxData1;
extern volatile uint8_t RxData2;
extern volatile uint8_t data_len;
extern volatile uint8_t Serial_RxFlag1;
extern volatile uint8_t Serial_RxFlag2;

extern uint8_t Serial_TxPacket1[MAX_FRAME_LEN];
extern uint8_t Serial_RxPacket1[MAX_FRAME_LEN];
extern uint8_t Serial_TxPacket2[SERIAL2_BUF_LEN];
extern uint8_t Serial_RxPacket2[SERIAL2_BUF_LEN];
extern float data[SERIAL1_MAX_DATA_COUNT];

uint8_t Serial_GetRxFlag1(void);
uint8_t Serial_GetRxFlag2(void);
void uart1_data(void);
void UART1_RxPacket(void);
void UART2_RxPacket(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H */
