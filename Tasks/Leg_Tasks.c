#include "Leg_Tasks.h"
#include "forward_solution.h"
#include "inverse_solution.h"

static LeftLegCtrl_s LeftCtrl[2];
fp32 left_leg_1_pid[3]={PID_P,PID_I,PID_D};
fp32 left_leg_2_pid[3]={PID_P,PID_I,PID_D};
FKResult s;
IKResult r;
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
	
	while(1)
	{
		for(int i = 0;i<2;i++)
		{
			Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[i], 0);
			DM_SendCmd(&hfdcan1, &DM8009_Ctrl[i]);
		}
		
		s = fivebar_fk(DM8009_Ctrl[0].fb.processed_pos,DM8009_Ctrl[1].fb.processed_pos);
		r = fivebar_ik(s.B.x,s.B.y);
	}
}

void RLeg_task(void *argument)
{
	while(1)
	{
		osDelay(1000);
	}
}

uint8_t Gap_check(LeftLegCtrl_s *c1,LeftLegCtrl_s *c2)
{
	static fp32 delta = 0.0f;
	
	if((c2->Now_AbsoluteAngle - c1->Now_AbsoluteAngle) >= M_PI)//Á¬¸Ë¼Ð½Ç°üº¬ÁË0
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
