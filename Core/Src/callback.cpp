//
// Created by Administrator on 24-10-26.
//
#include "main.h"
#include "tim.h"
#include "can.h"
#include "PID.h"

// CAN receive and transmit
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t TxMailbox = CAN_TX_MAILBOX0;

// receive motor data
float real_position = 0;
float real_speed = 0;

// Speed limits
const float max_speed = 500.0;
const float min_speed = -500.0;


// PID control parameters for tuning
float kp_position = 0.7;
float ki_position = 0.1;
float kd_position = 0.3;
PID position_pid(kp_position, ki_position, kd_position, 20.0, 180);

float kp_speed = 0.6;
float ki_speed = 0.2;
float kd_speed = 0.2;
PID speed_pid(kp_speed, ki_speed, kd_speed, 100.0, max_speed);

float target_position = 0.0;
float target_speed = 100.0;
int16_t control_output;

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return (out_max - out_min) / (in_max - in_min) * (in - in_min) + out_min;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == hcan1.Instance) {
        // Check if the message was successfully received
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            real_position = linearMapping(RxData[0] << 8 | RxData[1], 0, 8191, -180, 180);
            real_speed = (int16_t)(RxData[2] << 8 | RxData[3]);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        // PID position control
        target_speed = position_pid.calc(target_position, real_position);

        // Constrain the speed
        if (target_speed > max_speed) target_speed = max_speed;
        if (target_speed < min_speed) target_speed = min_speed;

        //  PID speed control
        control_output = static_cast<int16_t>(speed_pid.calc(target_speed, real_speed));
        int16_t current_output = static_cast<int16_t>(linearMapping(control_output, min_speed, max_speed, -10000, 10000));

        // Transmit the data
        TxData[0] = (uint8_t)(current_output >> 8);
        TxData[1] = (uint8_t)(current_output & 0xFF);
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}


