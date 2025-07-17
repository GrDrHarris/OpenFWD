#include "mainLoop.h"

#include "OpenFWDconf.h"
#include "stdint.h"
#include "string.h"

#include "main.h"
#include "uart.h"

struct MainLoop mainLoop;
extern I2C_HandleTypeDef hi2c1, hi2c2;
extern ADC_HandleTypeDef hadc1, hadc2;
struct MotorController {
    uint16_t angle, last_angle; // one of 4096 circle
    int16_t speed, last_speed; // 2pi / 4096 / 20ms rad/s
    int16_t target; // angle, speed or pwm_ratio
    int16_t integral;
    int16_t Kp, Ki, Kd, Div;
    uint8_t mode, id;
};
#define MODE_DIR_ANGLE 0
#define MODE_BID_ANGLE 1
#define MODE_SPEED 2
#define MODE_DIRECT 3
#define MODE_REV 4
#define AS5600_ADDR (0x36 << 1)
#ifdef MOT1_ENABLED
struct MotorController mot_ctrl1;
#endif
#ifdef MOT2_ENABLED
struct MotorController mot_ctrl2;
#endif
#ifdef REPORT_TOT_POWER
#define INA232_ADDR (0x40 << 1)
#define INA232_CONF_REG 0x00
#define INA232_CONF 0x4427
/*  R_shunt = 4m
    I_max = 20A
    I_LSB = 1000uA
 */
#define INA232_SHUNT_REG 0x05
#define INA232_SHUNT 12800
//VOLT times 1.6mV
static const uint8_t INA232_VOLT_REG = 0x02;
//CURR times 1.0mA
static const uint8_t INA232_CURR_REG = 0x04;
#endif
static void read_angle(I2C_HandleTypeDef* i2c, struct MotorController* ctrl) {
    ctrl->last_angle = ctrl->angle;
    uint8_t tmp[2];
    HAL_I2C_Mem_Read(i2c, AS5600_ADDR, 0x0C, I2C_MEMADD_SIZE_16BIT, tmp, 2, HAL_MAX_DELAY);
    ctrl->angle = ((tmp[0] & 0x000f) << 8) | tmp[1];
    ctrl->speed = ctrl->angle - ctrl->last_angle;
}

void set_pwm_width(uint8_t id, uint8_t width);
static void motor_tick(struct MotorController* ctrl, I2C_HandleTypeDef* i2c) {
    if(ctrl->mode != MODE_DIRECT)
        read_angle(i2c, ctrl);
    int16_t pwm; 
    switch(ctrl->mode) {
        case MODE_BID_ANGLE: {
            int16_t diff = ctrl->target - (int16_t)ctrl->angle;
            if(diff > 2048)
                diff = 4096 - diff;
            else if(diff < -2048)
                diff += 4096;
            ctrl->integral += diff;
            pwm = 
                (ctrl->Kp * diff + ctrl->Ki * ctrl->integral + ctrl->Kd * ctrl->speed) / ctrl->Div;
            break;
        }
        case MODE_DIR_ANGLE: {
            int16_t diff = ctrl->target - (int16_t)ctrl->angle;
            ctrl->integral += diff;
            pwm = 
                (ctrl->Kp * diff + ctrl->Ki * ctrl->integral + ctrl->Kd * ctrl->speed) / ctrl->Div;
            break;
        }
        case MODE_SPEED: {
            ctrl->last_speed = ctrl->speed;
            ctrl->speed = (int16_t)ctrl->last_angle - (int16_t)ctrl->angle;
            int16_t p = ctrl->target - ctrl->speed;
            ctrl->integral += p;
            int16_t d = ctrl->last_speed - ctrl->speed;
            pwm = 
                (ctrl->Kp * p + ctrl->Ki * ctrl->integral + ctrl->Kd * d) / ctrl->Div;
            break;
        }
        case MODE_DIRECT: {
            pwm = ctrl->target;
            break;
        }
        default:
            return;
    }
    uint8_t f, r, t;
    if(pwm < 0) {
        f = 0; r = -pwm > 200 ? 200 : -pwm;
    } else {
        f = pwm > 200 ? 200 : pwm; r = 0;
    }
    if(ctrl->mode & MODE_REV) {
        t = f; f = r; r = t;
    }
    set_pwm_width(ctrl->id, f);
    set_pwm_width(ctrl->id + 1, r);
}
#define MAX_LED_TIME 16
struct LED_Stat {
    int16_t time[16], tick;
    int8_t ptr, enable;
};
#define LED_DEF(LED_ID) struct LED_Stat LED##LED_ID;
#ifdef LED1_TICK_ENABLE
LED_DEF(1)
#endif
#ifdef LED2_TICK_ENABLE
LED_DEF(2)
#endif
#ifdef LED3_TICK_ENABLE
LED_DEF(3)
#endif
#ifdef LED4_TICK_ENABLE
LED_DEF(4)
#endif
#define LED_TICK(LED_ID) \
    if(LED##LED_ID.enable) { \
        LED##LED_ID.tick++; \
        if(LED##LED_ID.tick == LED##LED_ID.time[LED##LED_ID.ptr]) { \
            LED##LED_ID.ptr++; \
            if(LED##LED_ID.time[LED##LED_ID.ptr] == 0) \
                LED##LED_ID.ptr = 0; \
            if(LED##LED_ID.ptr & 1) \
                LED##LED_ID##_GPIO_Port->BRR = LED##LED_ID##_Pin; \
            else \
                LED##LED_ID##_GPIO_Port->BSRR = LED##LED_ID##_Pin; \
        } \
    }
#define CHECK_MOT_PROTECT(ID) \
    if(!(MOT##ID##_P_GPIO_Port->IDR & MOT##ID##_P_Pin)) { \
        uint8_t* p = usart_alloc(&uart_1); \
        *p = MOTOR_PROTECT; \
        *(p+1) = ID; \
        usart_commit(&uart_1); \
    }
#define CHECK_RUD_PROTECT(ID) \
    if(RUD##ID##_P_GPIO_Port->IDR & RUD##ID##_P_Pin) { \
        uint8_t* p = usart_alloc(&uart_1); \
        *p = RUDDER_PROTECT; \
        *(p+1) = ID; \
        usart_commit(&uart_1); \
    }
static void do_low_freq_event(void) {
#ifdef MOT1_ENABLED
    motor_tick(&mot_ctrl1, &hi2c1);
#ifdef REPORT_MOT_CURRENT
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t v = HAL_ADC_GetValue(&hadc1);
        uint8_t *p = usart_alloc(&uart_1);
        *p = MOTOR_CURRENT;
        *(p+1) = 1;
        *(uint16_t*)(p+2) = v;
        usart_commit(&uart_1);
    }
#endif
#endif
#ifdef MOT2_ENABLED
    motor_tick(&mot_ctrl2, &hi2c2);
    #ifdef REPORT_MOT_CURRENT
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
        uint16_t v = HAL_ADC_GetValue(&hadc2);
        uint8_t *p = usart_alloc(&uart_1);
        *p = MOTOR_CURRENT;
        *(p+1) = 2;
        *(uint16_t*)(p+2) = v;
        usart_commit(&uart_1);
    }
#endif
#endif
#ifdef REPORT_TOT_POWER
    uint8_t *p = usart_alloc(&uart_1);
    uint8_t buffer[2];
    *p = TOTAL_POWER;
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, &INA232_CURR_REG, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, INA232_ADDR, buffer, 2, HAL_MAX_DELAY);
    *(p+1) = buffer[0]; *(p+2) = buffer[1];
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, &INA232_VOLT_REG, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, INA232_ADDR, buffer, 2, HAL_MAX_DELAY);
    *(p+3) = buffer[0]; *(p+4) = buffer[1];
    usart_commit(&uart_1);
#endif
#ifdef LED1_TICK_ENABLE
    LED_TICK(1)
#endif
#ifdef LED2_TICK_ENABLE
    LED_TICK(2)
#endif
#ifdef LED3_TICK_ENABLE
    LED_TICK(3)
#endif
#ifdef LED4_TICK_ENABLE
    LED_TICK(4)
#endif
#ifdef REPORT_MOT_PROTECT
#ifdef MOT1_ENABLED
    CHECK_MOT_PROTECT(1);
#endif
#ifdef MOT2_ENABLED
    CHECK_MOT_PROTECT(2);
#endif
#endif
#ifdef REPORT_RUD_PROTECT
#ifdef RUD1_ENABLED
    CHECK_RUD_PROTECT(1);
#endif
#ifdef RUD2_ENABLED
    CHECK_RUD_PROTECT(2);
#endif
#ifdef RUD3_ENABLED
    CHECK_RUD_PROTECT(3);
#endif
#ifdef RUD4_ENABLED
    CHECK_RUD_PROTECT(4);
#endif
#ifdef RUD5_ENABLED
    CHECK_RUD_PROTECT(5);
#endif
#ifdef RUD6_ENABLED
    CHECK_RUD_PROTECT(6);
#endif
#endif
}
void set_beep(uint8_t en, uint16_t arr);
#ifdef UART2_ENABLED //for extension of more uarts
#define HAS_CHILD_UART
#endif
#ifdef HAS_CHILD_UART
struct UART_t* child_uarts[] = {&uart_2};
static void upload_command(const uint8_t* cmd, uint8_t uart_id) {
    uint8_t* p = usart_alloc(&uart_1);
    *p = ROTOR_PREFIX | uart_id;
    memcpy(p+1, cmd, MSG_LEN - 1);
    usart_commit(&uart_1);
}
#endif
static void parse_command(const uint8_t* cmd) {
    uint8_t op = *cmd;
#ifdef HAS_CHILD_UART
    if((op & ROTOR_PREFIX) == ROTOR_PREFIX) {
        struct UART_t* p = child_uarts[op & 0x0F - 2];
        uint8_t* data_p = usart_alloc(p);
        memcpy(data_p, cmd+1, MSG_LEN-1);
        usart_commit(p);
        return;
    }
#endif
#define CASE_AND_DO(ID, VAR, INVOKE) \
    case ID: INVOKE(VAR##ID, ID); break;
    switch(op) {
        case MOTOR_SET_MODE: {
            uint8_t id = *(cmd+1);
            uint8_t mode = *(cmd+2);
            switch(id) {
#define MOTOR_MODE_INVOKE(VAR, ID) VAR.mode = mode;
#ifdef MOT1_ENABLED
            CASE_AND_DO(1, mot_ctrl, MOTOR_MODE_INVOKE);
#endif
#ifdef MOT2_ENABLED
            CASE_AND_DO(2, mot_ctrl, MOTOR_MODE_INVOKE);
#endif
            }
            break;
        }
        case MOTOR_SET_TRG: {
            uint8_t id = *(cmd+1);
            uint16_t target = *(uint16_t*)(cmd+2);
            switch(id) {
#define MOTOR_TRG_INVOKE(VAR, ID) VAR.target = target;
#ifdef MOT1_ENABLED
            CASE_AND_DO(1, mot_ctrl, MOTOR_TRG_INVOKE);
#endif
#ifdef MOT2_ENABLED
            CASE_AND_DO(2, mot_ctrl, MOTOR_TRG_INVOKE);
#endif
            }
            break;
        }
        case RUDDER_SET_TRG: {
            uint8_t id = *(cmd+1);
            uint8_t target = *(cmd+2);
            set_pwm_width(id + 7, target + 50);
            break;
        }
        case LED_SET_PWM: {
            uint8_t id = *(cmd+1);
            uint8_t target = *(cmd+2);
            set_pwm_width(id + 3, target);
            break;
        }
        case LED_SET_TICK: {
            uint8_t id = *(cmd+1);
            uint8_t slot = *(cmd+2);
            uint16_t time = *(uint16_t*)(cmd+3);
#define LED_TICK_INVOKE(VAR, ID) VAR.time[slot] = time;
            switch(id) {
#ifdef LED1_TICK_ENABLE
                CASE_AND_DO(1, LED, LED_TICK_INVOKE);
#endif
#ifdef LED2_TICK_ENABLE
                CASE_AND_DO(2, LED, LED_TICK_INVOKE);
#endif
#ifdef LED3_TICK_ENABLE
                CASE_AND_DO(3, LED, LED_TICK_INVOKE);
#endif
#ifdef LED4_TICK_ENABLE
                CASE_AND_DO(4, LED, LED_TICK_INVOKE);
#endif
            }
            break;
        }
        case LED_RESET_TICK: {
            uint8_t id = *(cmd+1);
            switch(id) {
#define LED_RETICK_INVOKE(VAR, ID) \
        VAR.ptr = 0; VAR.tick = 0; LED##ID##_GPIO_Port->BRR = LED##ID##_Pin;
#ifdef LED1_TICK_ENABLE
                CASE_AND_DO(1, LED, LED_RETICK_INVOKE);
#endif
#ifdef LED2_TICK_ENABLE
                CASE_AND_DO(2, LED, LED_RETICK_INVOKE);
#endif
#ifdef LED3_TICK_ENABLE
                CASE_AND_DO(3, LED, LED_RETICK_INVOKE);
#endif
#ifdef LED4_TICK_ENABLE
                CASE_AND_DO(4, LED, LED_RETICK_INVOKE);
#endif
            }
            break;
        }
        case BEEP_SET: {
            uint8_t beep_en = *(cmd+1);
            uint16_t arr = *(uint16_t*)(cmd+2);
            set_beep(beep_en, arr);
        }
    }
}
static void do_task(uint8_t task) {
    switch(task) {
        case LOW_FREQ_SCHED:
            do_low_freq_event(); break;
        case UART1_RECV:
            parse_command(usart_poll(&uart_1)); break;
#ifdef UART2_ENABLED
        case UART2_RECV:
            upload_command(usart_poll(&uart_2), 2); break;
#endif
    }
}
void mainLoop_loop(void) {
#ifdef MOT1_ENABLED
    mot_ctrl1.id = 0;
#endif
#ifdef MOT2_ENABLED
    mot_ctrl2.id = 2;
#endif
#ifdef REPORT_TOT_POWER
    uint8_t buffer1[3] = {INA232_CONF_REG, (INA232_CONF >> 8) & 0xFF, INA232_CONF & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, buffer1, sizeof(buffer1), HAL_MAX_DELAY);
    uint8_t buffer2[3] = {INA232_SHUNT_REG, (INA232_SHUNT >> 8) & 0xFF, INA232_SHUNT & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, buffer2, sizeof(buffer2), HAL_MAX_DELAY);
#endif
    while(1) {
        if(mainLoop.head != mainLoop.tail) {
            do_task(mainLoop.tasks[mainLoop.tail % TASK_NUM]);
            mainLoop.tail++;
        }
    }
}
