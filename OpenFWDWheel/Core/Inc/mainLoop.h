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
#define UART1_RECV 1
#define UART2_RECV 2
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
