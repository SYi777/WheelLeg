#include "Leg_Tasks.h"
#include "forward_solution.h"
#include "inverse_solution.h"
#include "Remote_control.h"
#include "VMC.h"

static LeftLegCtrl_s LeftCtrl[2];
fp32 left_leg_1_pid[3]={PID_P,PID_I,PID_D};
fp32 left_leg_2_pid[3]={PID_P,PID_I,PID_D};
FKResult s1,s2;
IKResult r1,r2;

extern vmc_t vmc_left_s;
extern vmc_t vmc_right_s;

void LLeg_task(void *argument)
{
	DM_8009Motor_Init(&DM8009_Ctrl[0],LeftFrontCAN_ID);
	DM_CtrlStop(&DM8009_Ctrl[0]);
	DM_8009Motor_Init(&DM8009_Ctrl[1],LeftBehindCAN_ID);
	DM_CtrlStop(&DM8009_Ctrl[1]);
	
	DM_Enable(&hfdcan1, &DM8009_Ctrl[0]);
	DM_Enable(&hfdcan1, &DM8009_Ctrl[1]);
	
	PID_init(&LeftCtrl[0].angle_pid,PID_POSITION,left_leg_1_pid,PID_MAX_OUT,PID_MAX_IOUT);
	PID_init(&LeftCtrl[1].angle_pid,PID_POSITION,left_leg_2_pid,PID_MAX_OUT,PID_MAX_IOUT);
	
	VMC_init(&vmc_left_s);            // L0_set=250、first_flag=1、状态置非法
	
	while(1)
	{
//		Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], remote_ctrl.rc.ch[0]/100);
//		Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], remote_ctrl.rc.ch[2]/100);
		
//		for(int i = 0;i<2;i++)
//		{
//			DM_SendCmd(&hfdcan1, &DM8009_Ctrl[i]);
//		}
		
		/* 1. 读编码器 */
        float alpha_enc = DM8009_Ctrl[0].fb.processed_pos;   // 左前电机
        float beta_enc  = DM8009_Ctrl[1].fb.processed_pos;   // 左后电机

        /* 2. VMC 状态段 + 力段 (Tp 二期接平衡环, 现在传 0) */
        VMC_StateUpdate(&vmc_left_s, alpha_enc, beta_enc, 0.001f);
        VMC_ForceCalc(&vmc_left_s, alpha_enc, beta_enc, 0.0f);

        /* 3. N·mm → N·m + M0 软限幅 */
        float ta = vmc_left_s.fk_s.tau_a / 1000.0f;
        float tb = vmc_left_s.fk_s.tau_b / 1000.0f;
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;

        /* 4. 下发 (注意: Set_LeftOne 内部写死 DM8009_Ctrl[0], LeftTwo 写死 [1]) */
        Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[0]);
        Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[1]);
		
//		for(int i = 0;i<2;i++)
//		{
//			DM_SendCmd(&hfdcan1, &DM8009_Ctrl[i]);
//		}
		static int count = 0;
		if(count % 1000 == 0)
		{	
			int delta = remote_ctrl.rc.ch[0]/60;
			vmc_left_s.L0_set += delta;
		}
		count++;
		
		if(vmc_left_s.L0_set <= 180)
		{
			vmc_left_s.L0_set = 180;
		}
		if(vmc_left_s.L0_set> 300)
		{
			vmc_left_s.L0_set = 300;
		}
		
        osDelay(1);                   // 1ms 控制周期 (和 dt=0.001f 对齐)
		
		s1 = fivebar_fk(DM8009_Ctrl[0].fb.processed_pos,DM8009_Ctrl[1].fb.processed_pos);
		r1 = fivebar_ik(s1.B.x,s1.B.y);
	}
}

void RLeg_task(void *argument)
{
    DM_8009Motor_Init(&DM8009_Ctrl[2], RightFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[2]);
    DM_8009Motor_Init(&DM8009_Ctrl[3], RightBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[3]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[2]);
    DM_Enable(&hfdcan1, &DM8009_Ctrl[3]);

    VMC_init(&vmc_right_s);                      // ← 缺的初始化

    while (1)
    {
        float alpha_enc = -DM8009_Ctrl[2].fb.processed_pos;   // 右腿编码器, 进运动学前取反
        float beta_enc  = -DM8009_Ctrl[3].fb.processed_pos;

        VMC_StateUpdate(&vmc_right_s, alpha_enc, beta_enc, 0.001f);
        VMC_ForceCalc(&vmc_right_s, alpha_enc, beta_enc, 0.0f);

        float ta = -vmc_right_s.fk_s.tau_a / 1000.0f;   // 出运动学后取反
        float tb = -vmc_right_s.fk_s.tau_b / 1000.0f;
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;

        Set_RightOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[2], ta);   // 右腿专用下发, 内部写死 Ctrl[2]
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[2]);
        Set_RightTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[3], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[3]);

        static int count = 0;
        if (count % 1000 == 0)
            vmc_right_s.L0_set += remote_ctrl.rc.ch[1] / 60;    // 右腿自己的目标
        count++;
        if (vmc_right_s.L0_set <= 180) vmc_right_s.L0_set = 180;
        if (vmc_right_s.L0_set >  300) vmc_right_s.L0_set = 300;

        osDelay(1);
    }
}

uint8_t Gap_check(LeftLegCtrl_s *c1,LeftLegCtrl_s *c2)
{
	static fp32 delta = 0.0f;
	
	if((c2->Now_AbsoluteAngle - c1->Now_AbsoluteAngle) >= M_PI)//连杆夹角包含了0
	{
		delta = c1->Now_AbsoluteAngle + 2.0f* M_PI - c2->Now_AbsoluteAngle;
	}
	else
	{
		fp32 diff = c2->Now_AbsoluteAngle - c1->Now_AbsoluteAngle + M_PI;
		fp32 cycle = 2.0f * M_PI;
		while(diff >= cycle)
		{
			diff -= cycle;
		}
		while(diff < 0.0f)
		{
			diff += cycle;
		}
		delta = diff - M_PI;
	}
	
	if(delta >= MAX_DELTA || delta <=MIN_DELTA)
	{
		for(int i = 0;i<2;i++)
		{
			LeftCtrl[i].noPowerflag = 1;
		}
		return 0;
	}
	return 1;
}
