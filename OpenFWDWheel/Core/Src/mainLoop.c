/*
 * mainLoop.c
 *
 *  Created on: Sep 11, 2025
 *      Author: DrHarris
 */

#include "mainLoop.h"

#include "OpenFWDconf.h"
#include "stdint.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_tim.h"
#include "string.h"

#include "main.h"
#include "uart.h"

struct MainLoop mainLoop;
extern I2C_HandleTypeDef hi2c1, hi2c2;
extern ADC_HandleTypeDef hadc1, hadc2;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
struct PinData {
    GPIO_TypeDef* port;
    unsigned pin;
};
struct TimerChannel {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint8_t default_width;
};
#define MODE_DIR_ANGLE 0
#define MODE_BID_ANGLE 1
#define MODE_SPEED 2
#define MODE_DIRECT 3
#define MODE_REV 4
struct MotorController {
    uint16_t angle, last_angle; // one of 4096 circle
    int16_t speed, last_speed; // 2pi / 4096 / 20ms rad/s
    int16_t target; // angle, speed or pwm_ratio
    int16_t integral;
    int16_t Kp, Ki, Kd, Div;
    uint8_t mode, sensorDisabled, reportAngle, id;
    struct TimerChannel f_channel, r_channel;
    struct PinData protect_pin;
    I2C_HandleTypeDef* i2c;
    ADC_HandleTypeDef* adc;
} motorControlers[2] = {
    {
        .angle = 0, .last_angle = 0, .speed = 0, .last_speed = 0, .target = 0, .integral = 0,
        .Kp = 10, .Ki = 1, .Kd = 0, .Div = 3,
        .mode = MODE_DIRECT, .sensorDisabled = MOT1_SENSOR_DISABLED, .reportAngle = true, .id = 1,
        .f_channel = {&htim3, TIM_CHANNEL_1, 0}, .r_channel = {&htim3, TIM_CHANNEL_2, 0},
        .protect_pin = {MOT1_P_GPIO_Port, MOT1_P_Pin}, .i2c = &hi2c1, .adc = &hadc1
    }, {
        .angle = 0, .last_angle = 0, .speed = 0, .last_speed = 0, .target = 0, .integral = 0,
        .Kp = 10, .Ki = 1, .Kd = 0, .Div = 3,
        .mode = MODE_DIRECT, .sensorDisabled = MOT2_SENSOR_DISABLED, .reportAngle = false, .id = 2,
        .f_channel = {&htim3, TIM_CHANNEL_3, 0}, .r_channel = {&htim3, TIM_CHANNEL_4, 0},
        .protect_pin = {MOT2_P_GPIO_Port, MOT2_P_Pin}, .i2c = &hi2c2, .adc = &hadc2
    }
};
struct RudderController {
    struct TimerChannel pwm;
    struct PinData protect_pin;
} rudderControllers[] = {
    {{&htim1, TIM_CHANNEL_1, 150}, {RUD1_P_GPIO_Port, RUD1_P_Pin}},
    {{&htim2, TIM_CHANNEL_2, 150}, {RUD2_P_GPIO_Port, RUD2_P_Pin}},
    {{&htim2, TIM_CHANNEL_1, 150}, {RUD3_P_GPIO_Port, RUD3_P_Pin}},
    {{&htim1, TIM_CHANNEL_2, 150}, {RUD4_P_GPIO_Port, RUD4_P_Pin}},
    {{&htim1, TIM_CHANNEL_3, 150}, {RUD5_P_GPIO_Port, RUD5_P_Pin}},
    {{&htim1, TIM_CHANNEL_4, 150}, {RUD6_P_GPIO_Port, RUD6_P_Pin}}
};
struct PinData LEDPins[] = {
    {LED1_GPIO_Port, LED1_Pin},
    {LED2_GPIO_Port, LED2_Pin},
    {LED3_GPIO_Port, LED3_Pin},
    {LED4_GPIO_Port, LED4_Pin}
};
#define AS5600_ADDR (0x36 << 1)
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
static uint8_t INA232_VOLT_REG = 0x02;
//CURR times 1.0mA
static uint8_t INA232_CURR_REG = 0x04;
#endif
static inline void set_pwm_width(struct TimerChannel* channel, uint8_t val) {
    __HAL_TIM_SET_COMPARE(channel->timer, channel->channel, val);
}

static inline void tim_start(struct TimerChannel* ch) {
	HAL_TIM_PWM_Start(ch->timer, ch->channel);
}

static inline void read_angle(struct MotorController* ctrl) {
    if(ctrl->sensorDisabled)
        return;
    ctrl->last_angle = ctrl->angle;
    uint8_t tmp[2] = {0, 0};
    uint8_t stat = HAL_I2C_Mem_Read(ctrl->i2c, AS5600_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, tmp, 1, HAL_MAX_DELAY);
    stat = HAL_I2C_Mem_Read(ctrl->i2c, AS5600_ADDR, 0x0D, I2C_MEMADD_SIZE_8BIT, tmp + 1, 1, HAL_MAX_DELAY);
    ctrl->angle = ((tmp[0] & 0x000f) << 8) | tmp[1];
    ctrl->speed = ctrl->angle - ctrl->last_angle;
}

static inline int16_t correct_sign(int16_t val) {
    if(val > 2048)
        return 4096 - val;
    if(val < -2048)
        return val + 4096;
    return val;
}

static inline void report_angle(struct MotorController* ctrl) {
    uint8_t *p = usart_alloc(uart_root);
    *p = MOTOR_ANGLE;
    *(p+1) = ctrl->id;
    *(uint16_t*)(p+2) = ctrl->angle;
    *(p+4) = END_BYTE;
    usart_commit(uart_root);
}

static inline int16_t calc_bid_angle(struct MotorController* ctrl) {
    int16_t diff = correct_sign(ctrl->target - (int16_t)ctrl->angle);
    ctrl->integral += diff;
    return (ctrl->Kp * diff + ctrl->Ki * ctrl->integral + ctrl->Kd * ctrl->speed) >> ctrl->Div;
}

static inline int16_t calc_dir_angle(struct MotorController* ctrl) {
    int16_t diff = ctrl->target - (int16_t)ctrl->angle;
    ctrl->integral += diff;
    return (ctrl->Kp * diff + ctrl->Ki * ctrl->integral + ctrl->Kd * ctrl->speed) >> ctrl->Div;
}

static inline int16_t calc_speed(struct MotorController* ctrl) {
    ctrl->last_speed = ctrl->speed;
    ctrl->speed = correct_sign(ctrl->last_angle - (int16_t)ctrl->angle);
    int16_t p = ctrl->target - ctrl->speed;
    ctrl->integral += p;
    int16_t d = ctrl->last_speed - ctrl->speed;
    return (ctrl->Kp * p + ctrl->Ki * ctrl->integral + ctrl->Kd * d) >> ctrl->Div;
}

static inline uint8_t clamp_200(int16_t v) {
    return v > 200 ? 200 : v;
}

static inline void motor_tick(struct MotorController* ctrl) {
    if(ctrl -> reportAngle || ctrl -> mode != MODE_DIRECT)
        read_angle(ctrl);
    if(ctrl -> reportAngle)
        report_angle(ctrl);
    int16_t pwm;
    switch(ctrl->mode) {
        case MODE_BID_ANGLE: pwm = calc_bid_angle(ctrl); break;
        case MODE_DIR_ANGLE: pwm = calc_dir_angle(ctrl); break;
        case MODE_SPEED:     pwm = calc_speed(ctrl); break;
        case MODE_DIRECT:    pwm = ctrl->target; break;
        default:             return;
    }
    uint8_t f, r, t;
    if(pwm < 0) {
        f = 0; r = clamp_200(-pwm);
    } else {
        f = clamp_200(pwm); r = 0;
    }
    if(ctrl->mode & MODE_REV) {
        t = f; f = r; r = t;
    }
    set_pwm_width(&(ctrl->f_channel), f);
    set_pwm_width(&(ctrl->r_channel), r);

#ifdef REPORT_MOT_CURRENT
    HAL_ADC_Start(ctrl->adc);
    if (HAL_ADC_PollForConversion(ctrl->adc, 10) == HAL_OK)
    {
        uint16_t v = HAL_ADC_GetValue(ctrl->adc);
        uint8_t *p = usart_alloc(uart_root);
        *p = MOTOR_CURRENT;
        *(p+1) = ctrl->id;
        *(uint16_t*)(p+2) = v;
        *(p+4) = END_BYTE;
        usart_commit(uart_root);
    }
#endif

#ifdef REPORT_MOT_PROTECT
    if(!(ctrl->protect_pin.port->IDR & ctrl->protect_pin.pin)) {
        uint8_t* p = usart_alloc(uart_root);
        *p = MOTOR_PROTECT;
        *(p+1) = ctrl->id;
        *(p+2) = END_BYTE;
        usart_commit(uart_root);
    }
#endif
}
#define MAX_LED_TIME 16
struct LED_Stat {
    int16_t time[16], tick;
    int8_t ptr, enable;
} LEDControlers[4];

static inline void led_tick(uint8_t id) {
    id--;
    if(LEDControlers[id].enable) {
        LEDControlers[id].tick++;
        if(LEDControlers[id].tick == LEDControlers[id].time[LEDControlers[id].ptr]) {
            LEDControlers[id].ptr++;
            LEDControlers[id].tick = 0;
            if(LEDControlers[id].time[LEDControlers[id].ptr] == 0)
                LEDControlers[id].ptr = 0;
            if(LEDControlers[id].ptr & 1)
                LEDPins[id].port->BRR = LEDPins[id].pin;
            else
                LEDPins[id].port->BSRR = LEDPins[id].pin;
        }
    }
}

static inline void led_reset(uint8_t id) {
    id--;
    LEDControlers[id].ptr = 0;
    LEDControlers[id].tick = 0;
    LEDPins[id].port->BSRR = LEDPins[id].pin;
}

static inline void led_on(uint8_t id) {
    id--;
    LEDControlers[id].enable = true;
    LEDControlers[id].ptr = 0;
    LEDControlers[id].tick = 0;
    LEDPins[id].port->BRR = LEDPins[id].pin;
}

static inline void led_off(uint8_t id) {
    id--;
    LEDControlers[id].enable = false;
    LEDControlers[id].ptr = 0;
    LEDControlers[id].tick = 0;
    LEDPins[id].port->BRR = LEDPins[id].pin;
}

static inline void led_pause(uint8_t id) {
    LEDControlers[id-1].enable = false;
}

#ifdef REPORT_TOT_POWER
static inline void report_total_power(void) {
    uint8_t *p = usart_alloc(uart_root);
    uint8_t buffer[2];
    *p = TOTAL_POWER;
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, &INA232_CURR_REG, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, INA232_ADDR, buffer, 2, HAL_MAX_DELAY);
    *(p+1) = buffer[0]; *(p+2) = buffer[1];
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, &INA232_VOLT_REG, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, INA232_ADDR, buffer, 2, HAL_MAX_DELAY);
    *(p+3) = buffer[0]; *(p+4) = buffer[1];
    *(p+5) = END_BYTE;
    usart_commit(uart_root);
}
#endif

static inline void check_rud_protect(uint8_t id) {
    id--;
    if(rudderControllers[id].protect_pin.port->IDR & rudderControllers[id].protect_pin.pin) {
        uint8_t* p = usart_alloc(uart_root);
        *p = RUDDER_PROTECT;
        *(p+1) = id + 1;
        *(p+2) = END_BYTE;
        usart_commit(uart_root);
    }
}

static void do_low_freq_event(void) {
#ifdef MOT1_ENABLED
    motor_tick(&motorControlers[0]);
#endif
#ifdef MOT2_ENABLED
    motor_tick(&motorControlers[1]);
#endif
#ifdef REPORT_TOT_POWER
    report_total_power();
#endif
#ifdef LED1_TICK_ENABLE
    led_tick(1);
#endif
#ifdef LED2_TICK_ENABLE
    led_tick(2);
#endif
#ifdef LED3_TICK_ENABLE
    led_tick(3);
#endif
#ifdef LED4_TICK_ENABLE
    led_tick(4);
#endif

#ifdef REPORT_RUD_PROTECT
#ifdef RUD1_ENABLED
    check_rud_protect(1);
#endif
#ifdef RUD2_ENABLED
    check_rud_protect(2);
#endif
#ifdef RUD3_ENABLED
    check_rud_protect(3);
#endif
#ifdef RUD4_ENABLED
    check_rud_protect(4);
#endif
#ifdef RUD5_ENABLED
    check_rud_protect(5);
#endif
#ifdef RUD6_ENABLED
    check_rud_protect(6);
#endif
#endif
}
void set_beep(uint8_t en, uint16_t arr);

#if defined(UART2_ENABLED) //for future extension
#define HAS_CHILD_UART
#endif
#ifdef HAS_CHILD_UART
static void upload_command(const uint8_t* cmd, uint8_t uart_id) {
    uint8_t* p = usart_alloc(uart_root);
    *p = ROTOR_PREFIX | uart_id;
    memcpy(p+1, cmd, MSG_LEN - 1);
    usart_commit(uart_root);
}
#endif
static void parse_command(const uint8_t* cmd) {
    uint8_t op = *cmd;
#ifdef HAS_CHILD_UART
    if((op & ROTOR_PREFIX_MASK) == ROTOR_PREFIX) {
        struct UART_t* p = &uarts[(op & 0x0F) - 1];
        uint8_t* data_p = usart_alloc(p);
        const uint8_t *src = cmd + 1;
        int len = -1;
        do{
        	len++;
        	*(data_p + len) = *(src + len);
        }while(*(src + len) != END_BYTE);
        usart_commit(p);
        return;
    }
#endif
    switch(op) {
        case MOTOR_SET_MODE: {
            uint8_t id = *(cmd+1);
            uint8_t mode = *(cmd+2);
            motorControlers[id-1].mode = mode;
            break;
        }
        case MOTOR_SET_TRG: {
            uint8_t id = *(cmd+1);
            uint16_t target = *(uint16_t*)(cmd+2);
            motorControlers[id-1].target = target;
            break;
        }
        case MOTOR_SET_KP: {
            uint8_t id = *(cmd+1);
            int16_t Kp = *(int16_t*)(cmd+2);
            motorControlers[id-1].Kp = Kp;
            break;
        }
        case MOTOR_SET_KI: {
            uint8_t id = *(cmd+1);
            int16_t Ki = *(int16_t*)(cmd+2);
            motorControlers[id-1].Ki = Ki;
            break;
        }
        case MOTOR_SET_KD: {
            uint8_t id = *(cmd+1);
            int16_t Kd = *(int16_t*)(cmd+2);
            motorControlers[id-1].Kp = Kd;
            break;
        }
        case MOTOR_SET_DIV: {
            uint8_t id = *(cmd+1);
            uint8_t Div = *(cmd+2);
            motorControlers[id-1].Div = Div;
            break;
        }
        case RUDDER_SET_TRG: {
            uint8_t id = *(cmd+1);
            uint8_t target = *(cmd+2);
            set_pwm_width(&(rudderControllers[id-1].pwm), target + 50);
            break;
        }
        case LED_SET_TICK: {
            uint8_t id = *(cmd+1);
            uint8_t slot = *(cmd+2);
            uint16_t time = *(uint16_t*)(cmd+3);
            LEDControlers[id-1].time[slot] = time;
            break;
        }
        case LED_RESET_TICK: {
            uint8_t id = *(cmd+1);
            led_reset(id);
            break;
        }
        case LED_ON: {
        	uint8_t id = *(cmd+1);
        	led_on(id);
        	break;
        }
        case LED_OFF: {
            uint8_t id = *(cmd+1);
            led_off(id);
            break;
        }
        case LED_PAUSE: {
            uint8_t id = *(cmd+1);
            led_pause(id);
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
	if((task & UART_RECV_MASK) == UART_RECV) {
		uint8_t uart_id = task >> 4;
		const uint8_t *cmd = usart_poll(&uarts[uart_id - 1]);
		if(uart_id == 1)
			parse_command(cmd);
#ifdef HAS_CHILD_UART
		else
			upload_command(cmd, uart_id);
#endif
		return;
	}

	if((task & UART_SEND_MASK) == UART_SEND) {
		uint8_t uart_id = task >> 4;
		usart_delayed_send(&uarts[uart_id]);
		return;
	}

    switch(task) {
        case LOW_FREQ_SCHED:
            do_low_freq_event(); break;
    }
}

#ifdef REPORT_TOT_POWER
static void init_power(void) {
    uint8_t buffer1[3] = {INA232_CONF_REG, (INA232_CONF >> 8) & 0xFF, INA232_CONF & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, buffer1, sizeof(buffer1), HAL_MAX_DELAY);
    uint8_t buffer2[3] = {INA232_SHUNT_REG, (INA232_SHUNT >> 8) & 0xFF, INA232_SHUNT & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c1, INA232_ADDR, buffer2, sizeof(buffer2), HAL_MAX_DELAY);
}
#endif

static void init_motor(void) {
    for(unsigned i = 0; i < sizeof(motorControlers) / sizeof(struct MotorController); i++) {
    	tim_start(&(motorControlers[i].f_channel));
        set_pwm_width(&(motorControlers[i].f_channel), motorControlers[i].f_channel.default_width);
        tim_start(&(motorControlers[i].r_channel));
        set_pwm_width(&(motorControlers[i].r_channel), motorControlers[i].r_channel.default_width);
    }
}

static void init_rudder(void) {
    for(unsigned i = 0; i < sizeof(rudderControllers) / sizeof(struct RudderController); i++) {
    	tim_start(&(rudderControllers[i].pwm));
        set_pwm_width(&(rudderControllers[i].pwm), rudderControllers[i].pwm.default_width);
    }
}
void mainLoop_loop(void) {
#ifdef REPORT_TOT_POWER
    init_power();
#endif
    init_motor();
    init_rudder();
    initlize_usart();
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
    while(1) {
        if(mainLoop.head != mainLoop.tail) {
            do_task(mainLoop.tasks[mainLoop.tail % TASK_NUM]);
            mainLoop.tail++;
        }
    }
}

