#ifndef UART_H
#define UART_H
#include "stdint.h"
#include "stdbool.h"

#include "main.h"

#include "OpenFWDconf.h"

#define BUF_SIZE 8
#define MSG_LEN 16
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
#define ROTOR_PREFIX 0xF0
#define MOTOR_CURRENT 0x00 // MOT_ID(starts from 1) CURR_LSB CURR_MSB
#define TOTAL_POWER 0x01 // CURR_LSB CURR_MSB VOLT_LSB VOLT_MSB
#define MOTOR_PROTECT 0x02 // MOT_ID
#define RUDDER_PROTECT 0x03 // RUD_ID(starts from 1)
#define MOTOR_SET_MODE 0x04 // MOT_ID CTRL_MODE
#define MOTOR_SET_TRG 0x05 // MOT_ID TRG_LSB TRG_MSB
#define RUDDER_SET_TRG 0x06 // RUD_ID TRG_VAL(range 0-199)
#define LED_SET_PWM 0x07 // LED_ID(starts from 1) PWM_VAL
#define LED_SET_TICK 0x08 // LED_ID TICK_SLOT TICK_LSB TICK_MSB
#define LED_RESET_TICK 0x09 // LED_ID
#define BEEP_SET 0x0A // BEEP_EN FRAC_LSB FRAC_MSB
#endif
