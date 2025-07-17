#ifndef UART_H
#define UART_H
#include "stdint.h"
#include "stdbool.h"

#include "main.h"

#include "OpenFWDconf.h"
#include "../../../Communicate.h"

#define BUF_SIZE 8
struct UART_t{
    uint8_t rx_buf[BUF_SIZE][MSG_LEN], tx_buf[BUF_SIZE][MSG_LEN];
    int rx_head, rx_tail, tx_head, tx_tail;
    UART_HandleTypeDef *uart;
    bool send_inflight;
};
extern struct UART_t uart_1;
#ifdef UART2_ENABLED
extern struct UART_t uart_2;
#endif
void initlize_usart(void);
//should not be called in handlers! alloc should always followed by commit!
uint8_t* usart_alloc(struct UART_t *uart);
void usart_commit(struct UART_t *uart);
const uint8_t* usart_poll(struct UART_t *uart);
#endif
