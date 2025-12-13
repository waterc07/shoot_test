/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : DM2325 + 2×M3508 遥控器安全控制测试
 * @note
 *  - 遥控器未 ready → 所有电机强制停
 *  - 右三档 DOWN    → 所有电机停
 *  - 右三档 MID/UP  → 电机按设定速度转
 ******************************************************************************
 */

#include "dvc_motor.hpp"
#include "dvc_remotecontrol.hpp"
#include "cmsis_os.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ==================== 遥控器 ====================== */
Dr16RemoteControl dr16;
static bool rc_ready = false;   // ★ 安全门：遥控器是否已就绪

/* ==================== 目标速度 rad/s ====================== */
fp32 tar2325 = -20.0f;      // DM2325 拨弹轮
fp32 tarL    = 600.0f;     // 左摩擦轮
fp32 tarR    = -600.0f;    // 右摩擦轮

/* ==================== PID 参数 ====================== */

/* ---- DM2325 速度 PID（输出 torque） ---- */
SimplePID::PIDParam pid_2325_param = {
    .Kp = 4.0f,
    .Ki = 1.0f,
    .Kd = 0.0f,
    .outputLimit    = 5.0f,   // Nm
    .intergralLimit = 1.0f
};
/* ⚠️ 速度环必须用 PID_DELTA */
SimplePID pid2325(SimplePID::PID_DELTA, pid_2325_param);

/* ---- M3508 速度 PID（输出电流） ---- */
SimplePID::PIDParam pid_3508_param_L = {
    .Kp = 15.0f,
    .Ki = 1.0f,
    .Kd = 0.2f,
    .outputLimit    = 8000.0f,
    .intergralLimit = 2000.0f
};
SimplePID pid3508_L(SimplePID::PID_POSITION, pid_3508_param_L);

SimplePID::PIDParam pid_3508_param_R = {
    .Kp = 15.0f,
    .Ki = 1.0f,
    .Kd = 0.2f,
    .outputLimit    = 8000.0f,
    .intergralLimit = 2000.0f
};
SimplePID pid3508_R(SimplePID::PID_POSITION, pid_3508_param_R);

/* ==================== 电机对象 ====================== */

/* ---- DM2325（MIT 参数必须与调试助手一致） ---- */
MotorDM2325 motor2325(
    0x01,
    0x00,
    12.5f,     // PMAX
    200.0f,    // VMAX
    10.0f,     // TMAX
    &pid2325
);

/* ---- 两个 3508 ---- */
MotorM3508 motor3508_L(6, &pid3508_L, 0, 1);
MotorM3508 motor3508_R(7, &pid3508_R, 0, 1);

/* ==================== CAN / UART 回调 ====================== */
extern "C" void can1RxCallback(can_rx_message_t *msg);

extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer && Length > 0) {
        dr16.receiveRxDataFromISR(Buffer);
        rc_ready = true;    // ★ 一旦收到遥控器数据，解除安全门
    }
}

/* ==================== 测试任务 ====================== */
extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);
    UART_Init(&huart3, dr16ITCallback, 36);

    TickType_t last = xTaskGetTickCount();

    while (1) {

        /* =========================================================
         *  安全门：遥控器未 ready → 所有电机强制停
         * ========================================================= */
        if (!rc_ready) {

            motor2325.openloopControl(0.0f);
            motor3508_L.openloopControl(0.0f);
            motor3508_R.openloopControl(0.0f);

            // 仍然要发送 CAN 帧
            {
                uint32_t mb;
                HAL_CAN_AddTxMessage(
                    &hcan1,
                    motor2325.getMotorControlHeader(),
                    motor2325.getMotorControlData(),
                    &mb
                );
            }
            {
                auto combined3508 = motor3508_L + motor3508_R;
                uint32_t mb;
                HAL_CAN_AddTxMessage(
                    &hcan1,
                    combined3508.getMotorControlHeader(),
                    combined3508.getMotorControlData(),
                    &mb
                );
            }

            vTaskDelayUntil(&last, 1);
            continue;
        }

        /* ================= 遥控器更新 ================= */
        dr16.updateEvent();
        auto rs = dr16.getRightSwitchStatus();

        fp32 cmd2325 = 0.0f;
        fp32 cmdL    = 0.0f;
        fp32 cmdR    = 0.0f;

        if (rs == Dr16RemoteControl::SWITCH_DOWN) {
            // DOWN → 全停
            cmd2325 = 0.0f;
            cmdL    = 0.0f;
            cmdR    = 0.0f;
        }
        else {
            // MID / UP → 允许转动
            cmd2325 = tar2325;
            cmdL    = tarL;
            cmdR    = tarR;
        }

        /* ================= 电机控制 ================= */

        motor2325.angularVelocityClosedloopControl(cmd2325);
        motor3508_L.angularVelocityClosedloopControl(cmdL);
        motor3508_R.angularVelocityClosedloopControl(cmdR);

        /* ================= CAN 发送 ================= */

        // DM2325
        {
            uint32_t mb2325;
            HAL_CAN_AddTxMessage(
                &hcan1,
                motor2325.getMotorControlHeader(),
                motor2325.getMotorControlData(),
                &mb2325
            );
        }

        // 两个 3508 合并
        {
            auto combined3508 = motor3508_L + motor3508_R;
            uint32_t mb3508;
            HAL_CAN_AddTxMessage(
                &hcan1,
                combined3508.getMotorControlHeader(),
                combined3508.getMotorControlData(),
                &mb3508
            );
        }

        vTaskDelayUntil(&last, 1);
    }
}

/* ==================== CAN 接收回调 ====================== */
extern "C" void can1RxCallback(can_rx_message_t *msg)
{
    motor2325.decodeCanRxMessageFromISR(msg);
    motor3508_L.decodeCanRxMessageFromISR(msg);
    motor3508_R.decodeCanRxMessageFromISR(msg);
}
