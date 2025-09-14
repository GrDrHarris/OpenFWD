/*
 * uart.c
 *
 *  Created on: Sep 11, 2025
 *      Author: DrHarris
 */

#include "uart.h"

#include "main.h"

#include "string.h"

#include "mainLoop.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
struct UART_t uarts[] = {
		{.tx_head = 0, .tx_tail = 0, .rx_head = 0, .rx_tail = 0, .id = 1, .send_inflight = false, .uart = &huart1}
#ifdef UART2_ENABLED
		,{.tx_head = 0, .tx_tail = 0, .rx_head = 0, .rx_tail = 0, .id = 2, .send_inflight = false, .uart = &huart2}
#endif
};
struct UART_t *uart_root = &uarts[0];
extern uint8_t init_sync[];
void initlize_usart(void) {
    for(unsigned i = 0; i < sizeof(uarts) / sizeof(struct UART_t); i++) {
    	__HAL_UART_ENABLE_IT(uarts[i].uart, UART_IT_IDLE);
    	HAL_UART_Receive_DMA(uarts[i].uart, uarts[i].rx_buf[0], MSG_LEN);
    }
}
uint8_t* usart_alloc(struct UART_t *uart) {
	return uart->tx_buf[uart->tx_head % BUF_SIZE];
}
void usart_commit(struct UART_t *uart) {
	uint8_t len = 0;
	while(uart->tx_buf[uart->tx_head % BUF_SIZE][len] != END_BYTE)
		len++;
	uart->tx_len[uart->tx_head % BUF_SIZE] = len + 1;
    uart->tx_head++;

    if(!uart->send_inflight) {
        HAL_UART_Transmit_DMA(uart->uart, uart->tx_buf[uart->tx_tail % BUF_SIZE], uart->tx_len[uart->tx_tail % BUF_SIZE]);
        uart->send_inflight = true;
    }
}
const uint8_t* usart_poll(struct UART_t *uart) {
    //this function is called only because a task is scheduled,
    //so there is always data available
    const uint8_t* data = uart->rx_buf[uart->rx_tail % BUF_SIZE];
    uart->rx_tail++;
    return data;
}
void usart_delayed_send(struct UART_t* uart) {
	HAL_Delay(1);
    uart->tx_tail++;
    if(uart->tx_tail != uart->tx_head)
        HAL_UART_Transmit_DMA(uart->uart, uart->tx_buf[uart->tx_tail % BUF_SIZE], uart->tx_len[uart->tx_tail % BUF_SIZE]);
    else
        uart->send_inflight = false;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	for(unsigned i = 0; i < sizeof(uarts) / sizeof(struct UART_t); i++) {
	    if(uarts[i].uart == huart) {
	    	mainLoop_pushTask(i << 4 | UART_SEND);
	    	return;
	    }
	}
}
void UART_IDLE_Handler(struct UART_t *uart)
{
	UART_HandleTypeDef *huart = uart->uart;
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	HAL_DMA_Abort(huart->hdmarx);

	if(MSG_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx) > 0)
	{
		uart->rx_head++;
		mainLoop_pushTask((uart->id) << 4 | UART_RECV);
	}

	huart->hdmarx->Instance->CNDTR = MSG_LEN;
	//huart->hdmarx->State = HAL_DMA_STATE_READY;
	HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)uart->rx_buf[uart->rx_head % BUF_SIZE], MSG_LEN);
	//SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
}

