#include "cmsis_os.h"
#include "RobotState.h"
#include "remote_control.h"

static uint8_t RobotAuto=0;     								//自动/手动瞄准标志，手动0自动1
static uint8_t CtrlMode=0;     									//键盘/遥控操作标志，遥控0键盘1
static const RC_ctrl_t *rc_p;									//遥控器数据指针
static enum RobotState_e robotState=RobotState_e_Powerless;  	//机器人初始状态

extern volatile uint8_t rc_online; 			// 遥控器在线标志，0 离线 1 在线
extern volatile uint8_t flag;      			// 中断内部标志位
extern volatile uint32_t error_count;		// 未进中断计数

void RobotTotalState_Task(void *argument)
{
	//初始化数据指针
    rc_p=get_remote_control_point();
	
	while(1)
	{
		//判断键鼠操作还是遥控操作
		//当键盘有任何按键开始按下后，就转换到键盘和鼠标控制了
		if(rc_p->rc.ch[4] != 0 || rc_p->rc.ch[0] != 0 || rc_p->rc.ch[2] != 0)
		{
			CtrlMode = 0;
		}
		if(rc_p->key.v != 0)
		{
			CtrlMode = 1;
		}
		
		
		//键鼠操作也是通过遥控器发送数据
		//检查遥控器是否在线，在线给整车使能标志
		RC_OnlineCheck();
		
		
		//拆分为键鼠和手柄用作状态切换
		
		
		osDelay(1);
	}
}
