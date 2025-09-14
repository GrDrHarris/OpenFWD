#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#define MSG_LEN 16

#define ROTOR_PREFIX_MASK 0xF0
#define ROTOR_PREFIX 0xE0

/**
 * In order to tell the partener that the uart is ready,  
 * after boot (2 * MSG_LEN - 1) bytes of SYNC_BIT should be 
 * send through the uart. After sending the sync bytes, delay 5ms
 * for the partener's reaction.
 * After receiveing a message that are completely made up of 
 * SYNC_BYTE, the mcu should clear the incoming buffer. Better wait 
 * for 1ms to make sure all sync bytes have arrived. 
 * (At 115200 bps, 16 bytes only takes for about 1us to transmit)
 */
#define END_BYTE 0xFF

enum COMMAND_PREFIX {
    MOTOR_CURRENT, // MOT_ID(starts from 1) CURR_LSB CURR_MSB
    TOTAL_POWER, // CURR_LSB CURR_MSB VOLT_LSB VOLT_MSB
    MOTOR_PROTECT, // MOT_ID
    MOTOR_ANGLE, // MOT_ID ANGLE_LSB ANGLE_MSB
    RUDDER_PROTECT, // RUD_ID(starts from 1)
    TYPE_RESPOND, // TYPE_LSB TYPE_BIT2 TYPE_BIT3 TYPE_MSB


    MOTOR_SET_MODE, // MOT_ID CTRL_MODE
    MOTOR_SET_TRG, // MOT_ID TRG_LSB TRG_MSB
    MOTOR_SET_KP, // MOT_ID KP_LSB KP_MSB
    MOTOR_SET_KI, // MOT_ID KI_LSB KI_MSB
    MOTOR_SET_KD, // MOT_ID KD_LSB KD_MSB
    MOTOR_SET_DIV, // MOT_ID DIV
    RUDDER_SET_TRG, // RUD_ID TRG_VAL(range 0-199)
    LED_SET_TICK, // LED_ID TICK_SLOT TICK_LSB TICK_MSB
    LED_RESET_TICK, // LED_ID
    LED_ON, // LED_ID
    LED_OFF, // LED_ID
    LED_PAUSE, // LED_ID
    BEEP_SET, // BEEP_EN FRAC_LSB FRAC_MSB
    TYPE_QUERY //
};

#endif