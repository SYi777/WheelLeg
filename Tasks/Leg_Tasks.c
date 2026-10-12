#include "Leg_Tasks.h"
#include "VMC.h"
#include "3508_CtrlApplications.h"
#include "balance.h"
#include "INS_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dwt.h"

extern vmc_t vmc_left_s, vmc_right_s;
balance_t bal_left, bal_right;
balance_shared_t bal_sh;

// 电机掉线检查
static int leg_safety_ok(const DM_Motor *m0, const DM_Motor *m1, const vmc_t *vmc)
{
    uint32_t now = HAL_GetTick();
    return m0->online && m1->online && m0->fb.enabled && m1->fb.enabled
        && !m0->fb.fault && !m1->fb.fault
        && now - m0->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && now - m1->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && vmc->fk_s.fk.status == FK_OK && vmc->fk_s.jac_ok;
}

// 左腿任务
void LLeg_task(void *argument)
{
    osDelay(5000);

    DM_8009Motor_Init(&DM8009_Ctrl[0], LeftFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[0]);
    DM_8009Motor_Init(&DM8009_Ctrl[1], LeftBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[1]);
    for (int i = 0; i < 10; i++)
    {
        DM_Enable(&hfdcan1, &DM8009_Ctrl[0]);
        DM_Enable(&hfdcan1, &DM8009_Ctrl[1]);
        osDelay(2);
    }
    VMC_init(&vmc_left_s);

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t period = osKernelGetTickFreq() / 1000U;
    if (period == 0U) period = 1U;
    uint32_t dwt_last = 0;
    (void)DWT_GetDeltaT(&dwt_last);

    while (1)
    {
        float dt = DWT_GetDeltaT(&dwt_last);
        if (dt <= 0.0f || dt >= 0.01f) dt = 0.001f;

        DM_state fb0, fb1;
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[0].fb;
        fb1 = DM8009_Ctrl[1].fb;
        taskEXIT_CRITICAL();

        VMC_StateUpdateWithVelocity(&vmc_left_s, fb0.processed_pos, fb1.processed_pos, fb0.velocity, fb1.velocity, dt);
        int ready = leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s) && dt > 0.0f && dt < 0.01f;

        float ta = 0.0f, tb = 0.0f;
        if (ready)
        {
            // 共享 x/v
            Balance_SharedUpdate(&bal_sh, motor_measure_L.speed_rpm, motor_measure_R.speed_rpm, dt);
            Balance_Update(&bal_left, &bal_sh, &vmc_left_s, &INS);

            // 横滚 + 防劈叉
            float roll_f = Roll_Loop(0.0f, INS.Roll, INS.Gyro[1]);
			
            float anti = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad, vmc_right_s.theta_rel_body_rad);
            anti += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s - vmc_right_s.theta_rel_body_dot_rad_s);
            if (anti >  ANTI_SPLIT_MAX) anti =  ANTI_SPLIT_MAX;
            if (anti < -ANTI_SPLIT_MAX) anti = -ANTI_SPLIT_MAX;
			
            vmc_left_s.F_roll_N = roll_f;
            vmc_left_s.T_p_Nmm = bal_left.Tp_Nmm - anti;

            // 力段: J^T F -> 关节力矩
            VMC_ForceCalc(&vmc_left_s, fb0.processed_pos, fb1.processed_pos);
            ta = vmc_left_s.fk_s.tau_a / 1000.0f;   /* N·mm -> N·m */
            tb = vmc_left_s.fk_s.tau_b / 1000.0f;
        }
        else
        {
            // 保护
            Balance_Reset(&bal_left);
            vmc_left_s.T_p_Nmm = 0.0f;
            vmc_left_s.fk_s.tau_a = vmc_left_s.fk_s.tau_b = 0.0f;
            vmc_left_s.first_flag = 1U;
        }
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;
        Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[0]);
        Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[1]);

        // 轮通道 + 差速阻尼
        if (ready)
        {
            float vd = bal_sh.v_R_m_s - bal_sh.v_L_m_s;     // 左右轮速差
            float td = LQR_WHEEL_SIGN * vd * WHEEL_DIFF_KD; // 差速阻尼
            if (td >  WHEEL_DIFF_LIMIT_NM) td =  WHEEL_DIFF_LIMIT_NM;
            if (td < -WHEEL_DIFF_LIMIT_NM) td = -WHEEL_DIFF_LIMIT_NM;
            float twl = bal_left.T_Nm  + td;
            float twr = bal_right.T_Nm - td;
            if (twl >  LQR_T_LIMIT_NM) twl =  LQR_T_LIMIT_NM;
            if (twl < -LQR_T_LIMIT_NM) twl = -LQR_T_LIMIT_NM;
            if (twr >  LQR_T_LIMIT_NM) twr =  LQR_T_LIMIT_NM;
            if (twr < -LQR_T_LIMIT_NM) twr = -LQR_T_LIMIT_NM;
            WheelTorque_Send(&hfdcan2, twl, twr);
        }
        else
        {
            WheelTorque_Send(&hfdcan2, 0.0f, 0.0f);
        }

        //任务周期维持
        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);
    }
}

// 右腿任务
void RLeg_task(void *argument)
{
    osDelay(5000);

    DM_8009Motor_Init(&DM8009_Ctrl[2], RightFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[2]);
    DM_8009Motor_Init(&DM8009_Ctrl[3], RightBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[3]);
    for (int i = 0; i < 10; i++)
    {
        DM_Enable(&hfdcan1, &DM8009_Ctrl[2]);
        DM_Enable(&hfdcan1, &DM8009_Ctrl[3]);
        osDelay(2);
    }
    VMC_init(&vmc_right_s);

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t period = osKernelGetTickFreq() / 1000U;
    if (period == 0U) period = 1U;
    uint32_t dwt_last = 0;
    (void)DWT_GetDeltaT(&dwt_last);

    while (1)
    {
        float dt = DWT_GetDeltaT(&dwt_last);
        if (dt <= 0.0f || dt >= 0.01f) dt = 0.001f;

        DM_state fb0, fb1;
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[2].fb;
        fb1 = DM8009_Ctrl[3].fb;
        taskEXIT_CRITICAL();

        // 正解 (镜像: 进运动学前取反)
        VMC_StateUpdateWithVelocity(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos, -fb0.velocity, -fb1.velocity, dt);
        int ready = leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s) && dt > 0.0f && dt < 0.01f;

        float ta = 0.0f, tb = 0.0f;
        if (ready)
        {
            // 右腿 LQR 共享 x/v
            Balance_Update(&bal_right, &bal_sh, &vmc_right_s, &INS);

            // 横滚取负 + 防劈叉取正
            float roll_f = Roll_Loop(0.0f, INS.Roll, INS.Gyro[1]);
            float anti = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad,
                                        vmc_right_s.theta_rel_body_rad);
            anti += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s
                                     - vmc_right_s.theta_rel_body_dot_rad_s);
            if (anti >  ANTI_SPLIT_MAX) anti =  ANTI_SPLIT_MAX;
            if (anti < -ANTI_SPLIT_MAX) anti = -ANTI_SPLIT_MAX;
            vmc_right_s.F_roll_N = -roll_f;
            vmc_right_s.T_p_Nmm = bal_right.Tp_Nmm + anti;

            VMC_ForceCalc(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos);
            ta = -vmc_right_s.fk_s.tau_a / 1000.0f;  // 出运动学后取反
            tb = -vmc_right_s.fk_s.tau_b / 1000.0f;
        }
        else
        {
            Balance_Reset(&bal_right);
            vmc_right_s.T_p_Nmm = 0.0f;
            vmc_right_s.fk_s.tau_a = vmc_right_s.fk_s.tau_b = 0.0f;
            vmc_right_s.first_flag = 1U;
        }
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;
        Set_RightOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[2], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[2]);
        Set_RightTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[3], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[3]);

        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);
    }
}

// 横滚环: roll 误差 -> 两腿力差 [N], 左正右负
float Roll_Loop(float roll_set, float roll, float roll_gyro)
{
    float out = ROLL_KP * (roll_set - roll) - ROLL_KD * roll_gyro;
    if (out >  ROLL_MAX_OUT) out =  ROLL_MAX_OUT;
    if (out < -ROLL_MAX_OUT) out = -ROLL_MAX_OUT;
    return out;
}

// 防劈叉: 两腿夹角差 -> 修正力矩 [N·mm], 左减右加
float AntiSplit_Loop(float theta_left, float theta_right)
{
    float out = ANTI_SPLIT_KP * (theta_left - theta_right);
    if (out >  ANTI_SPLIT_MAX) out =  ANTI_SPLIT_MAX;
    if (out < -ANTI_SPLIT_MAX) out = -ANTI_SPLIT_MAX;
    return out;
}
