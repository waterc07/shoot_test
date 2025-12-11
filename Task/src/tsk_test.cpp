/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : DM2325 + 2×M3508 Openloop Test (Correct DJI merge)
 ******************************************************************************
 */

#include "dvc_motor.hpp"
#include "dvc_remotecontrol.hpp"
#include "cmsis_os.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Dr16RemoteControl dr16;

/* ==================== 目标速度 rad/s（你可以随时更改） ====================== */
fp32 tar2325 = 2.0f;
fp32 tarL    = 600.0f;
fp32 tarR    = -600.0f;

/* ==================== PID 参数 ====================== */

/* ---- DM2325 的速度 PID（输出 torque） ---- */
SimplePID::PIDParam pid_2325_param = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .outputLimit = 5.0f,     // MIT 扭矩最大 5 Nm
    .intergralLimit = 1.0f
};
SimplePID pid2325(SimplePID::PID_POSITION, pid_2325_param);

/* ---- M3508 的速度 PID（输出电流） ---- */
SimplePID::PIDParam pid_3508_param = {
    .Kp = 5.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .outputLimit = 8000.0f,     // 最大电流
    .intergralLimit = 2000.0f
};

SimplePID pid3508_L(SimplePID::PID_POSITION, pid_3508_param);
SimplePID pid3508_R(SimplePID::PID_POSITION, pid_3508_param);


/* ==================== DM2325 ====================== */
MotorDM2325 motor2325(
    0x01,       // ESC_ID example
    0x00,       // feedback ID
    M_PI,
    50.0f,
    5.0f,
    &pid2325      // open-loop
);

/* ==================== 两个 3508 ====================== */

MotorM3508 motor3508_L(6, &pid3508_L, 0, 1); // 电机 ID = 1 → 使用控制帧0x200
MotorM3508 motor3508_R(7, &pid3508_R, 0, 1); 


/* ==================== CAN 回调 ====================== */
extern "C" void can1RxCallback(can_rx_message_t *msg);


extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);

    TickType_t last = xTaskGetTickCount();

    while (1) {

        dr16.updateEvent();
        auto rs = dr16.getRightSwitchStatus();
        fp32 cmd2325, cmdL, cmdR;
        if (rs == Dr16RemoteControl::SWITCH_DOWN) {
            // DOWN → 停止全部电机
            cmd2325 = 0;
            cmdL = 0;
            cmdR = 0;
        }
        else {
            // MID / UP → 正常旋转
            cmd2325 = tar2325;
            cmdL    = tarL;
            cmdR    = tarR;
        }

        /* DM2325: PID 输出 torque → MIT 控制 */
        motor2325.angularVelocityClosedloopControl(tar2325);

        /* M3508: PID 输出电流 */
        motor3508_L.angularVelocityClosedloopControl(tarL);
        motor3508_R.angularVelocityClosedloopControl(tarR);

        /* ========== DM2325 独立发送 ========== */
        {
            uint32_t mb2325;
            HAL_CAN_AddTxMessage(
                &hcan1,
                motor2325.getMotorControlHeader(),
                motor2325.getMotorControlData(),
                &mb2325);
        }


        /* ========== 两个 M3508 合并发送（ID=6,7 → 同 0x1FF 帧） ========== */
        {
            auto combined3508 = motor3508_L + motor3508_R;
            uint32_t mb3508;
            HAL_CAN_AddTxMessage(
                &hcan1,
                combined3508.getMotorControlHeader(),   // ← 0x1FF
                combined3508.getMotorControlData(),
                &mb3508);
        }


        vTaskDelayUntil(&last, 1);
    }
}


extern "C" void can1RxCallback(can_rx_message_t *msg)
{
    motor2325.decodeCanRxMessageFromISR(msg);
    motor3508_L.decodeCanRxMessageFromISR(msg);
    motor3508_R.decodeCanRxMessageFromISR(msg);
}
