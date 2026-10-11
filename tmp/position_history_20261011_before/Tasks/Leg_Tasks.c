#include "Leg_Tasks.h"
#include "VMC.h"
#include "3508_CtrlApplications.h"
#include "balance.h"
#include "INS_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dwt.h"
#include <string.h>

extern vmc_t vmc_left_s, vmc_right_s;
balance_t bal_left, bal_right;
balance_pair_t bal_pair;

typedef struct
{
    uint8_t valid, hip_enabled_mask, jac_ok_mask;
    uint32_t tick_ms, cycle;
    float dt_s;
    balance_pair_t pair;
    balance_t left, right;
    float L_L_mm, L_R_mm, L_set_L_mm, L_set_R_mm;
    float L_dot_L_mm_s, L_dot_R_mm_s;
    float F_axis_L_N, F_axis_R_N, F_spring_L_N, F_spring_R_N;
    float F_gravity_L_N, F_gravity_R_N, F_roll_L_N, F_roll_R_N;
    float vmc_Tp_L_Nmm, vmc_Tp_R_Nmm;
    float F_axis_applied_L_N, F_axis_applied_R_N;
    float vmc_Tp_applied_L_Nmm, vmc_Tp_applied_R_Nmm;
    float Tp_sent_L_Nm, Tp_sent_R_Nm, Tp_fb_L_Nm, Tp_fb_R_Nm;
    float tau_request_Nm[4];
    uint8_t allocation_limited_mask;
    float tau_cmd_Nm[4], tau_fb_Nm[4];
    uint32_t hip_tx_status[4];
    int16_t rpm_L, rpm_R, current_fb_L, current_fb_R;
} balance_diag_t;
volatile balance_diag_t balance_diag, balance_diag_standing, balance_diag_latest;
volatile uint32_t balance_control_cycle, balance_control_overruns;
volatile float balance_control_dt_s;

static float limited(float x, float bound)
{
    if (x > bound) return bound;
    if (x < -bound) return -bound;
    return x;
}

static int leg_safety_ok(const DM_Motor *m0, const DM_Motor *m1, const vmc_t *vmc)
{
    uint32_t now = HAL_GetTick();
    return m0->online && m1->online && m0->fb.enabled && m1->fb.enabled
        && !m0->fb.fault && !m1->fb.fault
        && now - m0->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && now - m1->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && vmc->fk_s.fk.status == FK_OK && vmc->fk_s.jac_ok;
}

static void capture(volatile balance_diag_t *d, const float torque[4])
{
    d->valid = 0U;
    d->tick_ms = HAL_GetTick();
    d->cycle = balance_control_cycle;
    d->dt_s = balance_control_dt_s;
    d->pair = bal_pair;
    d->left = bal_left;
    d->right = bal_right;
    d->L_L_mm = vmc_left_s.L_t_mm;
    d->L_R_mm = vmc_right_s.L_t_mm;
    d->L_set_L_mm = vmc_left_s.L_t_set_mm;
    d->L_set_R_mm = vmc_right_s.L_t_set_mm;
    d->L_dot_L_mm_s = vmc_left_s.L_t_dot_mm_s;
    d->L_dot_R_mm_s = vmc_right_s.L_t_dot_mm_s;
    d->F_axis_L_N = vmc_left_s.F_axis_N;
    d->F_axis_R_N = vmc_right_s.F_axis_N;
    d->F_spring_L_N = vmc_left_s.F_spring_N;
    d->F_spring_R_N = vmc_right_s.F_spring_N;
    d->F_gravity_L_N = vmc_left_s.F_gravity_N;
    d->F_gravity_R_N = vmc_right_s.F_gravity_N;
    d->F_roll_L_N = vmc_left_s.F_roll_N;
    d->F_roll_R_N = vmc_right_s.F_roll_N;
    d->vmc_Tp_L_Nmm = vmc_left_s.T_p_Nmm;
    d->vmc_Tp_R_Nmm = vmc_right_s.T_p_Nmm;
    d->F_axis_applied_L_N = vmc_left_s.F_axis_applied_N;
    d->F_axis_applied_R_N = vmc_right_s.F_axis_applied_N;
    d->vmc_Tp_applied_L_Nmm = vmc_left_s.T_p_applied_Nmm;
    d->vmc_Tp_applied_R_Nmm = vmc_right_s.T_p_applied_Nmm;
    /* Coaxial equivalent FK: common rotation gives Tp = tau_a + tau_b. */
    d->Tp_sent_L_Nm = torque[0] + torque[1];
    d->Tp_sent_R_Nm = -(torque[2] + torque[3]);
    d->Tp_fb_L_Nm = DM8009_Ctrl[0].fb.torque + DM8009_Ctrl[1].fb.torque;
    d->Tp_fb_R_Nm = -(DM8009_Ctrl[2].fb.torque + DM8009_Ctrl[3].fb.torque);
    d->tau_request_Nm[0] = vmc_left_s.tau_request_a_Nm;
    d->tau_request_Nm[1] = vmc_left_s.tau_request_b_Nm;
    d->tau_request_Nm[2] = -vmc_right_s.tau_request_a_Nm;
    d->tau_request_Nm[3] = -vmc_right_s.tau_request_b_Nm;
    d->allocation_limited_mask = (vmc_left_s.axis_force_limited ? 1U : 0U)
         | (vmc_right_s.axis_force_limited ? 2U : 0U)
         | (vmc_left_s.hip_moment_limited ? 4U : 0U)
         | (vmc_right_s.hip_moment_limited ? 8U : 0U);
    d->hip_enabled_mask = 0;
    for (int i = 0; i < 4; ++i)
    {
        d->hip_enabled_mask |= DM8009_Ctrl[i].fb.enabled ? (1U << i) : 0U;
        d->tau_cmd_Nm[i] = torque[i];
        d->tau_fb_Nm[i] = DM8009_Ctrl[i].fb.torque;
        d->hip_tx_status[i] = DM8009_Ctrl[i].last_tx_status;
    }
    d->jac_ok_mask = (vmc_left_s.fk_s.jac_ok ? 1U : 0U)
                   | (vmc_right_s.fk_s.jac_ok ? 2U : 0U);
    d->rpm_L = motor_measure_L.speed_rpm;
    d->rpm_R = motor_measure_R.speed_rpm;
    d->current_fb_L = motor_measure_L.given_current;
    d->current_fb_R = motor_measure_R.given_current;
    d->valid = 1U;
}

/* One owner updates BOTH legs, one shared LQR state, then all six motor commands. */
void LLeg_task(void *argument)
{
    (void)argument;
    osDelay(5000); /* Preserve the currently working power-on rise delay. */
    const uint8_t ids[4] = {LeftFrontCAN_ID, LeftBehindCAN_ID,
                            RightFrontCAN_ID, RightBehindCAN_ID};
    for (int i = 0; i < 4; ++i)
    {
        DM_8009Motor_Init(&DM8009_Ctrl[i], ids[i]);
        DM_CtrlStop(&DM8009_Ctrl[i]);
    }
    for (int retry = 0; retry < 10; ++retry)
    {
        for (int i = 0; i < 4; ++i) DM_Enable(&hfdcan1, &DM8009_Ctrl[i]);
        osDelay(2);
    }
    VMC_init(&vmc_left_s);
    VMC_init(&vmc_right_s);
    uint32_t next_wake = osKernelGetTickCount();
    uint32_t period = osKernelGetTickFreq() / 1000U;
    if (period == 0U) period = 1U;
    uint32_t latest_capture_tick = 0;
    uint32_t dwt_last = 0;
    (void)DWT_GetDeltaT(&dwt_last);

    for (;;)
    {
        float dt = DWT_GetDeltaT(&dwt_last);
        if (balance_control_cycle == 0U) dt = 0.001f;
        balance_control_dt_s = dt;
        balance_control_cycle++;
        INS_t ins;
        DM_state fb[4];
        float rpm_left, rpm_right;
        /* A CAN or INS update cannot split the position/velocity pairs we consume. */
        taskENTER_CRITICAL();
        ins = INS;
        for (int i = 0; i < 4; ++i) fb[i] = DM8009_Ctrl[i].fb;
        rpm_left = motor_measure_L.speed_rpm;
        rpm_right = motor_measure_R.speed_rpm;
        taskEXIT_CRITICAL();

        VMC_StateUpdateWithVelocity(&vmc_left_s, fb[0].processed_pos, fb[1].processed_pos,
                                     fb[0].velocity, fb[1].velocity, dt);
        VMC_StateUpdateWithVelocity(&vmc_right_s, -fb[2].processed_pos, -fb[3].processed_pos,
                                     -fb[2].velocity, -fb[3].velocity, dt);
        int ready = dt > 0.0f && dt < 0.01f
                 && leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s)
                 && leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s);
        float torque[4] = {0};
        if (ready)
        {
            Balance_PairUpdate(&bal_pair, &bal_left, &bal_right, &vmc_left_s,
                               &vmc_right_s, &ins, rpm_left, rpm_right, dt);
            float roll_force = Roll_Loop(0.0f, ins.Roll, ins.Gyro[1]);
            float split_torque = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad,
                                                vmc_right_s.theta_rel_body_rad);
            /* Add damping in the same FK angular coordinate as the spring term. */
            split_torque += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s
                                             - vmc_right_s.theta_rel_body_dot_rad_s);
            split_torque = limited(split_torque, ANTI_SPLIT_MAX);
            vmc_left_s.F_roll_N = roll_force;
            vmc_right_s.F_roll_N = -roll_force;
            vmc_left_s.T_p_Nmm = bal_left.Tp_Nmm - split_torque;
            vmc_right_s.T_p_Nmm = bal_right.Tp_Nmm + split_torque;
            VMC_ForceCalc(&vmc_left_s, fb[0].processed_pos, fb[1].processed_pos);
            VMC_ForceCalc(&vmc_right_s, -fb[2].processed_pos, -fb[3].processed_pos);
            torque[0] = vmc_left_s.fk_s.tau_a / 1000.0f;
            torque[1] = vmc_left_s.fk_s.tau_b / 1000.0f;
            torque[2] = -vmc_right_s.fk_s.tau_a / 1000.0f;
            torque[3] = -vmc_right_s.fk_s.tau_b / 1000.0f;
        }
        else
        {
            Balance_PairReset(&bal_pair, &bal_left, &bal_right);
            vmc_left_s.T_p_Nmm = vmc_right_s.T_p_Nmm = 0.0f;
            vmc_left_s.F_axis_applied_N = vmc_right_s.F_axis_applied_N = 0.0f;
            vmc_left_s.T_p_applied_Nmm = vmc_right_s.T_p_applied_Nmm = 0.0f;
            vmc_left_s.fk_s.tau_a = vmc_left_s.fk_s.tau_b = 0.0f;
            vmc_right_s.fk_s.tau_a = vmc_right_s.fk_s.tau_b = 0.0f;
            vmc_left_s.first_flag = vmc_right_s.first_flag = 1U;
        }
        for (int i = 0; i < 4; ++i)
        {
            torque[i] = limited(torque[i], TORQUE_LIMIT);
            DM_MITCtrl(&DM8009_Ctrl[i], 0.0f, 0.0f, 0.0f, 0.0f, torque[i]);
            DM_SendCmd(&hfdcan1, &DM8009_Ctrl[i]);
        }
        WheelTorque_Send(&hfdcan2, bal_left.T_Nm, bal_right.T_Nm);
        /* Refresh only on valid control frames; retained after feedback loss/stop. */
        if (ready && HAL_GetTick() - latest_capture_tick >= 100U)
        {
            capture(&balance_diag_latest, torque);
            latest_capture_tick = HAL_GetTick();
        }
        if (ready && !balance_diag_standing.valid
            && vmc_left_s.L_t_mm >= 210.0f && vmc_right_s.L_t_mm >= 210.0f)
            capture(&balance_diag_standing, torque);
        if (ready && !balance_diag.valid && bal_pair.common.v_m_s < -0.20f)
            capture(&balance_diag, torque);

        next_wake += period;
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next_wake - now) <= 0)
        {
            balance_control_overruns++;
            next_wake = now + period;
        }
        osDelayUntil(next_wake);
    }
}

/* Retained for source compatibility. freertos.c creates only the paired task. */
void RLeg_task(void *argument)
{
    (void)argument;
    for (;;) osDelay(1000);
}

float Roll_Loop(float roll_set, float roll, float roll_gyro)
{
    return limited(ROLL_KP * (roll_set - roll) - ROLL_KD * roll_gyro, ROLL_MAX_OUT);
}

float AntiSplit_Loop(float theta_left, float theta_right)
{
    return limited(ANTI_SPLIT_KP * (theta_left - theta_right), ANTI_SPLIT_MAX);
}
