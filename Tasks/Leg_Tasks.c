/**
*@file Leg_Tasks.c
*@brief 左右腿控制任务与电机输出 (双任务)
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/10
*
*共享量由左腿任务集中计算 (参考达妙 DM-balance: 补偿量集中一侧计算):
*整车 x/v、平均腿角、对称 LQR、横滚力差、防劈叉力矩与控制总闸都在左腿算好
*写入 bal_sh, 右腿任务只算本腿正解、发布本腿角度并取用共享量下发。
*遥控离线或非 CommonCar 模式时总闸断开, 全部零扭矩。
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
void RC_OnlineCheck(void);

extern vmc_t vmc_left_s, vmc_right_s;
balance_t bal_left;
balance_shared_t bal_sh;

// 安全判定: 电机在线/使能/无故障/反馈新鲜 + 本腿正解与雅可比有效
static int leg_safety_ok(const DM_Motor *m0, const DM_Motor *m1, const vmc_t *vmc)
{
    uint32_t now = HAL_GetTick();
    return m0->online && m1->online && m0->fb.enabled && m1->fb.enabled
        && !m0->fb.fault && !m1->fb.fault
        && (int32_t)(now - m0->last_fb_tick) <= DM_COMM_TIMEOUT_MS
        && (int32_t)(now - m1->last_fb_tick) <= DM_COMM_TIMEOUT_MS
        && vmc->fk_s.fk.status == FK_OK && vmc->fk_s.jac_ok;
}

/**
 * @brief  左腿 1 ms 周期任务: 共享状态计算与左腿/车轮电机输出
 * @param  argument  未使用的 RTOS 任务参数
 * @return 无, 任务持续循环
 *
 * 共享量唯一生产者: 整车 x/v 积分、对称 LQR(平均腿角)、横滚力差、
 * 防劈叉力矩与控制总闸 ctrl_ok 全部在此算好写入 bal_sh, 右腿任务只读;
 * 本任务同时完成左腿髋电机与左右轮毂的指令下发。
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

        // 遥控器掉线检测 (1kHz, 全车唯一调用点)
        RC_OnlineCheck();

        DM_state fb0, fb1;
        INS_t ins;
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[0].fb;
        fb1 = DM8009_Ctrl[1].fb;
        ins = INS;
        taskEXIT_CRITICAL();

        // 使能看护: 未使能且无故障时每 20 ms 补发使能, 故障不自动重启
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

        // 左腿正解 + 极坐标状态
        VMC_StateUpdateWithVelocity(&vmc_left_s, fb0.processed_pos, fb1.processed_pos, fb0.velocity, fb1.velocity, dt);

        // ---- 共享状态集中计算 (左腿任务独占, 右腿任务只读) ----

        // 整车共享 x/v (临界区: 右腿任务并发读 bal_sh)
        taskENTER_CRITICAL();
        Balance_SharedUpdate(&bal_sh, motor_measure_L.speed_rpm, motor_measure_R.speed_rpm, dt);
        taskEXIT_CRITICAL();

        // 起立完成: 双腿腿长首次同时接近目标时, 一次性清零位移积分原点
        // (起立时轮子随几何变化滚动, 里程计 x 是假位移; 不清掉 LQR 会猛冲一下)
        // 掉闸后重新武装, 下次起立完成再清一次
        static uint8_t rise_done = 0;
        if (!bal_sh.ctrl_ok) rise_done = 0;
        if (rise_done == 0
            && fabsf(vmc_left_s.L_t_mm  - vmc_left_s.L_t_set_mm)  < 5.0f
            && fabsf(vmc_right_s.L_t_mm - vmc_right_s.L_t_set_mm) < 5.0f)
        {
            rise_done = 1;
            bal_sh.x_m = 0.0f;
        }

        // 右腿机体系腿角: 临界区成对快照 (右腿任务发布, 本周期只取这一次)
        float thR, dthR;
        taskENTER_CRITICAL();
        thR  = bal_sh.theta_rel_R_rad;
        dthR = bal_sh.theta_rel_R_dot_rad_s;
        taskEXIT_CRITICAL();

        // 对称模态状态[rad]: 世界系腿角 = 机体系腿角 - 机身俯仰 (与 982200d 一致)
        float phi_c  = LQR_PITCH_SIGN * ins.Pitch;
        float dphi_c = LQR_PITCH_SIGN * ins.Gyro[PITCH_GYRO_AXIS];
        bal_sh.theta_avg_rad    = 0.5f * (LQR_LEG_SIGN * vmc_left_s.theta_rel_body_rad
                                        + LQR_LEG_SIGN * thR) - phi_c;
        bal_sh.dtheta_avg_rad_s = 0.5f * (LQR_LEG_SIGN * vmc_left_s.theta_rel_body_dot_rad_s
                                        + LQR_LEG_SIGN * dthR) - dphi_c;

        // 对称 LQR: 误差向量对称, 双腿输出同值, 只算一份
        Balance_Update(&bal_left, &bal_sh, &vmc_left_s, &ins);

        // 横滚力差与防劈叉力矩, 集中各算一次 (随后随总闸一起提交)
        float roll_f = Roll_Loop(0.0f, ins.Roll, ins.Gyro[1]);
        float anti = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad, thR);
        anti += ANTI_SPLIT_KD * (vmc_left_s.theta_rel_body_dot_rad_s - dthR);
        if (anti >  ANTI_SPLIT_MAX) anti =  ANTI_SPLIT_MAX;
        if (anti < -ANTI_SPLIT_MAX) anti = -ANTI_SPLIT_MAX;

        // 车轮力矩: 公共 LQR ± 差速阻尼
        float vd = bal_sh.v_R_m_s - bal_sh.v_L_m_s;     // 左右轮速差[m/s]
        float td = LQR_WHEEL_SIGN * vd * WHEEL_DIFF_KD; // 差速阻尼[N·m]
        if (td >  WHEEL_DIFF_LIMIT_NM) td =  WHEEL_DIFF_LIMIT_NM;
        if (td < -WHEEL_DIFF_LIMIT_NM) td = -WHEEL_DIFF_LIMIT_NM;
        float twl = bal_left.T_Nm  + td;
        float twr = bal_left.T_Nm  - td;
        if (twl >  LQR_T_LIMIT_NM) twl =  LQR_T_LIMIT_NM;
        if (twl < -LQR_T_LIMIT_NM) twl = -LQR_T_LIMIT_NM;
        if (twr >  LQR_T_LIMIT_NM) twr =  LQR_T_LIMIT_NM;
        if (twr < -LQR_T_LIMIT_NM) twr = -LQR_T_LIMIT_NM;

        // 控制总闸: 4 电机安全 + 两腿 FK/Jac + 遥控在线 + CommonCar 模式 (全车唯一判定点)
        int ready = leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s)
                 && leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s);
        int ok = ready && rc_online
              && *getRobotPresentMode() == RobotState_e_CommonCar;

        // 提交共享输出组 (临界区一致快照: 右腿要么全旧要么全新)
        taskENTER_CRITICAL();
        bal_sh.Tp_common_Nmm = bal_left.Tp_Nmm;
        bal_sh.roll_f_N      = roll_f;
        bal_sh.anti_Nmm      = anti;
        bal_sh.ctrl_ok       = (uint8_t)ok;
        taskEXIT_CRITICAL();

        // ---- 左腿髋输出 ----
        float ta = 0.0f, tb = 0.0f;
        if (ok)
        {
            // 左腿: 共享横滚取正, 防劈叉取负
            vmc_left_s.F_roll_N =  roll_f;
            vmc_left_s.T_p_Nmm  =  bal_left.Tp_Nmm - anti;

            // 力链: J^T F -> 关节力矩[N·mm]
            VMC_ForceCalc(&vmc_left_s, fb0.processed_pos, fb1.processed_pos);
            ta = vmc_left_s.fk_s.tau_a / 1000.0f;   // N·mm -> N·m
            tb = vmc_left_s.fk_s.tau_b / 1000.0f;
        }
        else
        {
            // 保护: 复位 + 零扭矩, 防止保持最后一帧力矩
            Balance_Reset(&bal_left);
            vmc_left_s.T_p_Nmm = 0.0f;
            vmc_left_s.fk_s.tau_a = vmc_left_s.fk_s.tau_b = 0.0f;
            vmc_left_s.first_flag = 1U;
            bal_sh.x_m = 0.0f;   // 清零位移积分原点
        }

        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;
        Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[0]);
        Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[1]);

        // ---- 轮毂输出 ----
        if (ok) WheelTorque_Send(&hfdcan2, twl, twr);
        else    WheelTorque_Send(&hfdcan2, 0.0f, 0.0f);

        // 维持任务周期
        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);
    }
}

/**
 * @brief  右腿 1 ms 周期任务: 只算本腿并下发髋电机指令
 * @param  argument  未使用的 RTOS 任务参数
 * @return 无, 任务持续循环
 *
 * 共享状态纯消费者: 只做本腿反馈快照、本腿正解、向 bal_sh 发布本腿角度,
 * 然后取用左腿任务算好的 Tp_common/roll_f/anti 与总闸 ctrl_ok,
 * 完成本腿力矩映射与下发; 不跑 LQR、横滚、防劈叉与遥控检测。
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
        taskENTER_CRITICAL();
        fb0 = DM8009_Ctrl[2].fb;
        fb1 = DM8009_Ctrl[3].fb;
        taskEXIT_CRITICAL();

        // 使能看护: 未使能且无故障时每 20 ms 补发使能, 故障不自动重启
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

        // 右腿正解 (镜像: 进运动学前取反)
        VMC_StateUpdateWithVelocity(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos, -fb0.velocity, -fb1.velocity, dt);

        // 向共享区发布本腿角度, 供左腿任务计算对称模态 (临界区防撕裂)
        taskENTER_CRITICAL();
        bal_sh.theta_rel_R_rad       = vmc_right_s.theta_rel_body_rad;
        bal_sh.theta_rel_R_dot_rad_s = vmc_right_s.theta_rel_body_dot_rad_s;
        taskEXIT_CRITICAL();

        // ---- 髋输出: 取用共享量 (临界区一致快照) ----
        uint8_t ok_snap; float tp_snap, rf_snap, an_snap;
        taskENTER_CRITICAL();
        ok_snap = bal_sh.ctrl_ok;
        tp_snap = bal_sh.Tp_common_Nmm;
        rf_snap = bal_sh.roll_f_N;
        an_snap = bal_sh.anti_Nmm;
        taskEXIT_CRITICAL();

        float ta = 0.0f, tb = 0.0f;
        if (ok_snap)   // 总闸由左腿任务判定
        {
            // 右腿: 共享横滚取负, 防劈叉取正
            vmc_right_s.F_roll_N = -rf_snap;
            vmc_right_s.T_p_Nmm  =  tp_snap + an_snap;

            // 力链: J^T F -> 关节力矩[N·mm] (出运动学后取反)
            VMC_ForceCalc(&vmc_right_s, -fb0.processed_pos, -fb1.processed_pos);
            ta = -vmc_right_s.fk_s.tau_a / 1000.0f;
            tb = -vmc_right_s.fk_s.tau_b / 1000.0f;
        }
        else
        {
            // 保护: 复位 + 零扭矩, 防止保持最后一帧力矩
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

        // 维持任务周期
        next_wake += period;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + period;
        osDelayUntil(next_wake);
    }
}

/**
 * @brief  横滚 PD 环: 由横滚角与角速度计算左右腿力差
 * @param  roll_set   目标横滚角[rad]
 * @param  roll       当前横滚角[rad]
 * @param  roll_gyro  横滚角速度[rad/s]
 * @return 横滚力差[N], 已限幅
 */
float Roll_Loop(float roll_set, float roll, float roll_gyro)
{
    float out = ROLL_KP * (roll_set - roll) - ROLL_KD * roll_gyro;
    if (out >  ROLL_MAX_OUT) out =  ROLL_MAX_OUT;
    if (out < -ROLL_MAX_OUT) out = -ROLL_MAX_OUT;
    return out;
}

/**
 * @brief  防劈叉 P 环: 由左右腿相对机身角差计算修正力矩
 * @param  theta_left   左腿相对机身角[rad]
 * @param  theta_right  右腿相对机身角[rad]
 * @return 修正力矩[N·mm], 已限幅
 */
float AntiSplit_Loop(float theta_left, float theta_right)
{
    float out = ANTI_SPLIT_KP * (theta_left - theta_right);
    if (out >  ANTI_SPLIT_MAX) out =  ANTI_SPLIT_MAX;
    if (out < -ANTI_SPLIT_MAX) out = -ANTI_SPLIT_MAX;
    return out;
}
