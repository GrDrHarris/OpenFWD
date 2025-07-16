#include "uart.h"

#include "stm32f1xx_hal_uart.h"

#include "string.h"

#include "mainLoop.h"
extern UART_HandleTypeDef huart1;
struct UART_t uart_1;
#ifdef UART2_ENABLED
extern UART_HandleTypeDef huart2;
struct UART_t uart_2;
#endif
void initlize_usart(void) {
    uart_1.tx_head = 0; uart_1.tx_tail = 0;
    uart_1.rx_head = 0; uart_1.rx_tail = 0;
    uart_1.uart = &huart1;
    uart_1.send_inflight = false;
    HAL_UART_Receive_DMA(uart_1.uart, uart_1.rx_buf[0], MSG_LEN);

#ifdef UART2_ENABLED
    uart_2.tx_head = 0; uart_2.tx_tail = 0;
    uart_2.rx_head = 0; uart_2.rx_tail = 0;
    uart_2.uart = &huart2;
    uart_2.send_inflight = false;
    HAL_UART_Receive_DMA(uart_2.uart, uart_2.rx_buf[0], MSG_LEN);
#endif
}
uint8_t* usart_alloc(struct UART_t *uart) {
    return uart->tx_buf[uart->tx_head % BUF_SIZE];
}
void usart_commit(struct UART_t *uart) {
    __disable_irq();
    uart->tx_head++;
    if(!uart->send_inflight) {
        HAL_UART_Transmit_DMA(uart->uart, uart->tx_buf[uart->tx_head % BUF_SIZE], MSG_LEN);
        uart->send_inflight = true;
    }
    __enable_irq();     
}
const uint8_t* usart_poll(struct UART_t *uart) {
    //this function is called only because a task is scheduled, 
    //so there is always data avaliable
    const uint8_t* data = uart->rx_buf[uart->rx_tail % BUF_SIZE];
    uart->rx_tail++;
    return data;
}
static void TxCpltCallback(struct UART_t* uart) {
    uart->tx_tail++;
    if(uart->tx_tail != uart->tx_head)
        HAL_UART_Transmit_DMA(uart->uart, uart->tx_buf[uart->tx_tail % BUF_SIZE], MSG_LEN);
    else
        uart->send_inflight = false;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef UART2_ENABLED
    if(huart->Instance == USART1)
#endif
        TxCpltCallback(&uart_1);
#ifdef UART2_ENABLED
    else
        TxCpltCallback(&uart_2);
#endif
}
static void RxCpltCallback(struct UART_t* uart) {
    uart->rx_head++;
    HAL_UART_Receive_DMA(uart->uart, uart->rx_buf[uart->rx_head % BUF_SIZE], MSG_LEN);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef UART2_ENABLED
    if(huart->Instance == USART1) {
#endif
        RxCpltCallback(&uart_1);
        mainLoop_pushTask(UART1_RECV);
#ifdef UART2_ENABLED
    } else {
        RxCpltCallback(&uart_2);
        mainLoop_pushTask(UART2_RECV);
    }
#endif
}
