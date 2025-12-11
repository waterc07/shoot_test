/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : DM2325 + 2×M3508 Openloop Test (Correct DJI merge)
 ******************************************************************************
 */

#include "dvc_motor.hpp"
#include "cmsis_os.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ==================== DM2325 ====================== */
MotorDM2325 motor2325(
    0x01,       // ESC_ID example
    0x00,       // feedback ID
    M_PI,
    50.0f,
    5.0f,
    NULL     // open-loop
);

fp32 test_torque = 0.0f;

/* ==================== 两个 3508 ====================== */

MotorM3508 motor3508_L(6, NULL, 0, 1); // 电机 ID = 1 → 使用控制帧0x200
MotorM3508 motor3508_R(7, NULL, 0, 1); 

int16_t m3508_current = 500;

/* ==================== CAN 回调 ====================== */
extern "C" void can1RxCallback(can_rx_message_t *msg);


extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);

    TickType_t last = xTaskGetTickCount();

    while (1) {

        /* ==== DM2325 力矩控制 ==== */
        motor2325.openloopControl(test_torque);
        uint32_t mb2325;
        HAL_CAN_AddTxMessage(
            &hcan1,
            motor2325.getMotorControlHeader(),
            motor2325.getMotorControlData(),
            &mb2325);


        /* ==== 3508 左右电机合并控制帧 ==== */

        motor3508_L.openloopControl(m3508_current);
        motor3508_R.openloopControl(-m3508_current);

        auto combined3508 = motor3508_L + motor3508_R;

        uint32_t mb3508;
        HAL_CAN_AddTxMessage(
            &hcan1,
            combined3508.getMotorControlHeader(),
            combined3508.getMotorControlData(),
            &mb3508);


        vTaskDelayUntil(&last, 1);
    }
}


extern "C" void can1RxCallback(can_rx_message_t *msg)
{
    motor2325.decodeCanRxMessageFromISR(msg);
    motor3508_L.decodeCanRxMessageFromISR(msg);
    motor3508_R.decodeCanRxMessageFromISR(msg);
}
