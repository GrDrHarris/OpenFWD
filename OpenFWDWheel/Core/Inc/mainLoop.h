/*
 * mainLoop.h
 *
 *  Created on: Sep 11, 2025
 *      Author: DrHarris
 */

#ifndef INC_MAINLOOP_H_
#define INC_MAINLOOP_H_

#include "stdint.h"

#include "main.h"

#include "OpenFWDconf.h"
#define TASK_NUM 64
#define LOW_FREQ_SCHED 0
/*
 * Because the uart_id of UART_RECV is gotten from the uart object,
 * it is required to minus one when accessing the array.
 */
#define UART_RECV 1
#define UART_RECV_MASK 0xf
/*
 * Because the uart_id of UART_SEND is gotten from loop compare,
 * it is incorrect to minus one when accessing the array.
 */
#define UART_SEND 2
#define UART_SEND_MASK 0xf
struct MainLoop {
  uint32_t head, tail;
  uint8_t tasks[TASK_NUM];
};
extern struct MainLoop mainLoop;
void mainLoop_loop(void);
static inline void mainLoop_pushTask(uint8_t task) {
  uint32_t old_basepri = __get_BASEPRI();
  __set_BASEPRI(0);
  mainLoop.tasks[mainLoop.head % TASK_NUM] = task;
  mainLoop.head++;
  __set_BASEPRI(old_basepri);
}

#endif /* INC_MAINLOOP_H_ */
