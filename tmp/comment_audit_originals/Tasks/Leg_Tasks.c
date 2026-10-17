/**
*@file Leg_Tasks.c
*@brief 左右腿控制任务与电机输出
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/10
*
*左腿任务更新共享轮速和位移，右腿任务读取共享状态。
**/
#include "Leg_Tasks.h"
#include "VMC.h"
#include "3508_CtrlApplications.h"
#include "balance.h"
#include "INS_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dwt.h"
#include "RobotState.h"

extern volatile uint8_t rc_online;

extern vmc_t vmc_left_s, vmc_right_s;
balance_t bal_left, bal_right;
balance_shared_t bal_sh;

FKResult l,r;

static int leg_safety_ok(const DM_Motor *m0, const DM_Motor *m1, const vmc_t *vmc)
{
    uint32_t now = HAL_GetTick();
    return m0->online && m1->online && m0->fb.enabled && m1->fb.enabled
        && !m0->fb.fault && !m1->fb.fault
        && now - m0->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && now - m1->last_fb_tick <= DM_COMM_TIMEOUT_MS
        && vmc->fk_s.fk.status == FK_OK && vmc->fk_s.jac_ok;
}
/**
 * @brief  运行左腿 1 ms 平衡与电机输出任务
 * @param  argument  未使用的 RTOS 任务参数
 * @return 无，任务持续循环
 *
 * 安全判断通过且遥控在线时更新共享轮速位移，计算本腿 LQR、横滚与防劈叉补偿，
 * 并下发髋部与两轮力矩；异常或掉线时全部清零。
 */
void LLeg_task(void *argument)
{
    osDelay(2000);

    DM_8009Motor_Init(&DM8009_Ctrl[0], LeftFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[0]);
    DM_8009Motor_Init(&DM8009_Ctrl[1], LeftBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[1]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[0]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[1]);
    osDelay(2);
    VMC_init(&vmc_left_s);

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t period = osKernelGetTickFreq() / 1000U;
    if (period == 0U) period = 1U;
    uint32_t dwt_last = 0;
    (void)DWT_GetDeltaT(&dwt_last);
    uint32_t last_enable_retry_tick[2] = {HAL_GetTick(), HAL_GetTick()};

    while (1)
    {
        float dt = DWT_GetDeltaT(&dwt_last);
        if (dt <= 0.0f || dt >= 0.01f) dt = 0.001f;

        DM_state fb0, fb1;
        INS_t ins;
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[0].fb;
        fb1 = DM8009_Ctrl[1].fb;
        ins = INS;
        taskEXIT_CRITICAL();

        // 使能看护: 未使能且无故障时每 20 ms 补发使能，故障不自动重启
        uint32_t now = HAL_GetTick();
        if (!fb0.enabled && !fb0.fault && now - last_enable_retry_tick[0] >= 20U)
        {
            last_enable_retry_tick[0] = now;
            DM_Enable(&hfdcan1, &DM8009_Ctrl[0]);
        }
        if (!fb1.enabled && !fb1.fault && now - last_enable_retry_tick[1] >= 20U)
        {
            last_enable_retry_tick[1] = now;
            DM_Enable(&hfdcan1, &DM8009_Ctrl[1]);
        }

        VMC_StateUpdateWithVelocity(&vmc_left_s, fb0.processed_pos, fb1.processed_pos, fb0.velocity, fb1.velocity, dt);
        int ready = leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s)
				 && leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s);

        float ta = 0.0f, tb = 0.0f;
        if (ready && rc_online && *getRobotPresentMode() == RobotState_e_CommonCar)
        {
			//髋部输出--------------------------------------------------------------

			// 共享 x/v (临界区: 右腿任务并发读 bal_sh, 防止读到半更新状态)
			taskENTER_CRITICAL();
			Balance_SharedUpdate(&bal_sh, motor_measure_L.speed_rpm, motor_measure_R.speed_rpm, dt);
			taskEXIT_CRITICAL();
			// 对称模态状态: 世界系腿角 = 机体系腿角 - 车身俯仰 (与 982200d 一致)
			float phi_c  = LQR_PITCH_SIGN * ins.Pitch;
			float dphi_c = LQR_PITCH_SIGN * ins.Gyro[PITCH_GYRO_AXIS];
			bal_sh.theta_avg_rad    = 0.5f * (vmc_left_s.theta_rel_body_rad
			                                + vmc_right_s.theta_rel_body_rad) - phi_c;
			bal_sh.dtheta_avg_rad_s = 0.5f * (vmc_left_s.theta_rel_body_dot_rad_s
			                                + vmc_right_s.theta_rel_body_dot_rad_s) - dphi_c;


			Balance_Update(&bal_left, &bal_sh, &vmc_left_s, &ins);

			// 横滚 + 防劈叉
			float roll_f = Roll_Loop(0.0f, ins.Roll, ins.Gyro[1]);

			float anti = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad, vmc_right_s.theta_rel_body_rad);
			anti += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s - vmc_right_s.theta_rel_body_dot_rad_s);
			if (anti >  ANTI_SPLIT_MAX) anti =  ANTI_SPLIT_MAX;
			if (anti < -ANTI_SPLIT_MAX) anti = -ANTI_SPLIT_MAX;

			vmc_left_s.F_roll_N = roll_f;
			vmc_left_s.T_p_Nmm = bal_left.Tp_Nmm - anti;

			// 力段: J^T F -> 关节力矩
			VMC_ForceCalc(&vmc_left_s, fb0.processed_pos, fb1.processed_pos);
			ta = vmc_left_s.fk_s.tau_a / 1000.0f;   // N·mm -> N·m
			tb = vmc_left_s.fk_s.tau_b / 1000.0f;

			if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
			if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
			if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
			if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;

			// 轮毂输出 + 差速阻尼--------------------------------------------------------------

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
            // 保护 (未就绪/掉线: 髋电机也发零扭矩, 防止保持最后一帧力矩)
            Balance_Reset(&bal_left);
            vmc_left_s.T_p_Nmm = 0.0f;
            vmc_left_s.fk_s.tau_a = vmc_left_s.fk_s.tau_b = 0.0f;
            vmc_left_s.first_flag = 1U;
			
			bal_sh.x_m = 0.0f;   // 与 982200d 的 PairReset 一致: 清零位移积分原点

			WheelTorque_Send(&hfdcan2, 0.0f, 0.0f);
        }

		Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], ta);
		DM_SendCmd(&hfdcan1, &DM8009_Ctrl[0]);
		Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], tb);
		DM_SendCmd(&hfdcan1, &DM8009_Ctrl[1]);
		
        // 维持任务周期
        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);

		l = fivebar_fk( fb0.processed_pos, fb1.processed_pos);
    }
}

/**
 * @brief  运行右腿 1 ms 平衡与电机输出任务
 * @param  argument  未使用的 RTOS 任务参数
 * @return 无，任务持续循环
 *
 * 安全判断通过且遥控在线时执行平衡控制；右腿反馈进入运动学前取反，关节力矩输出后再取反。
 * 共享 x/v 由左腿任务更新，本任务只读；未就绪时仍发送零扭矩防止电机维持最后一帧力矩。
 */
void RLeg_task(void *argument)
{
    osDelay(2000);

    DM_8009Motor_Init(&DM8009_Ctrl[2], RightFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[2]);
    DM_8009Motor_Init(&DM8009_Ctrl[3], RightBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[3]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[2]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[3]);
    osDelay(2);
    VMC_init(&vmc_right_s);

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t period = osKernelGetTickFreq() / 1000U;
    if (period == 0U) period = 1U;
    uint32_t dwt_last = 0;
    (void)DWT_GetDeltaT(&dwt_last);
    uint32_t last_enable_retry_tick[2] = {HAL_GetTick(), HAL_GetTick()};

    while (1)
    {
        float dt = DWT_GetDeltaT(&dwt_last);
        if (dt <= 0.0f || dt >= 0.01f) dt = 0.001f;

        DM_state fb0, fb1;
        INS_t ins;
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[2].fb;
        fb1 = DM8009_Ctrl[3].fb;
        ins = INS;
        taskEXIT_CRITICAL();

        // 使能看护: 未使能且无故障时每 20 ms 补发使能，故障不自动重启
        uint32_t now = HAL_GetTick();
        if (!fb0.enabled && !fb0.fault && now - last_enable_retry_tick[0] >= 20U)
        {
            last_enable_retry_tick[0] = now;
            DM_Enable(&hfdcan1, &DM8009_Ctrl[2]);
        }
        if (!fb1.enabled && !fb1.fault && now - last_enable_retry_tick[1] >= 20U)
        {
            last_enable_retry_tick[1] = now;
            DM_Enable(&hfdcan1, &DM8009_Ctrl[3]);
        }

        // 正解 (镜像: 进运动学前取反)
        VMC_StateUpdateWithVelocity(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos, -fb0.velocity, -fb1.velocity, dt);
		
		int ready = leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s)
				 && leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s);

        float ta = 0.0f, tb = 0.0f;
        if (ready && rc_online && *getRobotPresentMode() == RobotState_e_CommonCar)
        {
			//髋部输出--------------------------------------------------------------

			// 共享 x/v (只读, 由左腿任务更新)
			Balance_Update(&bal_right, &bal_sh, &vmc_right_s, &ins);

			// 横滚 + 防劈叉 (右腿: 横滚取负, 防劈叉取正)
			float roll_f = Roll_Loop(0.0f, ins.Roll, ins.Gyro[1]);
			float anti = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad,
										vmc_right_s.theta_rel_body_rad);
			anti += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s
									- vmc_right_s.theta_rel_body_dot_rad_s);
			if (anti >  ANTI_SPLIT_MAX) anti =  ANTI_SPLIT_MAX;
			if (anti < -ANTI_SPLIT_MAX) anti = -ANTI_SPLIT_MAX;
			vmc_right_s.F_roll_N = -roll_f;
			vmc_right_s.T_p_Nmm = bal_right.Tp_Nmm + anti;

			// 力段: J^T F -> 关节力矩 (出运动学后取反)
			VMC_ForceCalc(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos);
			ta = -vmc_right_s.fk_s.tau_a / 1000.0f;
			tb = -vmc_right_s.fk_s.tau_b / 1000.0f;

			if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
			if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
			if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
			if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;

			// 轮毂输出在左腿任务统一完成
        }
        else
        {
            // 保护 (未就绪/掉线: 髋电机也发零扭矩, 防止保持最后一帧力矩)
            Balance_Reset(&bal_right);
            vmc_right_s.T_p_Nmm = 0.0f;
            vmc_right_s.fk_s.tau_a = vmc_right_s.fk_s.tau_b = 0.0f;
            vmc_right_s.first_flag = 1U;

        }

		Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[2], ta);
		DM_SendCmd(&hfdcan1, &DM8009_Ctrl[2]);
		Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[3], tb);
		DM_SendCmd(&hfdcan1, &DM8009_Ctrl[3]);

        // 维持任务周期
        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);
		
		r = fivebar_fk( -fb0.processed_pos, -fb1.processed_pos);
    }
}

// 根据横滚角和角速度计算左右腿力差
float Roll_Loop(float roll_set, float roll, float roll_gyro)
{
    float out = ROLL_KP * (roll_set - roll) - ROLL_KD * roll_gyro;
    if (out >  ROLL_MAX_OUT) out =  ROLL_MAX_OUT;
    if (out < -ROLL_MAX_OUT) out = -ROLL_MAX_OUT;
    return out;
}

// 根据左右腿角度差计算防劈叉修正力矩
float AntiSplit_Loop(float theta_left, float theta_right)
{
    float out = ANTI_SPLIT_KP * (theta_left - theta_right);
    if (out >  ANTI_SPLIT_MAX) out =  ANTI_SPLIT_MAX;
    if (out < -ANTI_SPLIT_MAX) out = -ANTI_SPLIT_MAX;
    return out;
}
