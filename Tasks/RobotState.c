/**
*@file RobotState.c
*@brief 轮腿整车状态机
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-10-14
*
*按遥控器拨杆或键盘按键切换整车模式; 遥控器离线时强制无力模式。
**/
#include "cmsis_os.h"
#include "RobotState.h"
#include "remote_control.h"

#define RC_ROBOT_MODE_CHANNEL 0

static uint8_t RobotAuto=0;     								// 自动/手动瞄准标志: 手动 0, 自动 1, 当前未使用
static uint8_t CtrlMode=0;     									// 控制来源: 遥控 0, 键鼠 1
static const RC_ctrl_t *rc_p;									// 遥控器数据指针
static enum RobotState_e robotState=RobotState_e_Powerless;  	// 当前整车模式, 初始为无力

extern volatile uint8_t rc_online; 			// 遥控器在线标志: 0 离线, 1 在线
extern volatile uint8_t flag;      			// 中断内部标志位
extern volatile uint32_t error_count;		// 未收到遥控帧计数: 每 1 ms 加 1, 达 100 判离线
/**
 * @brief  整车状态机任务, 按操作来源切换整车模式
 * @param  argument  未使用的 RTOS 任务参数
 * @return 无
 *
 * 每 1 ms 执行一次: 先判断遥控/键鼠来源, 再检查遥控器在线状态;
 * 离线强制无力模式, 在线后按拨杆或按键切换模式。
 */
void RobotTotalState_Task(void *argument)
{
	// 初始化遥控器数据指针
    rc_p=get_remote_control_point();
	
	while(1)
	{
		// 判断控制来源: 拨杆有输入则切回遥控
		// 键盘/鼠标任意按键按下后切换到键鼠控制
		if(rc_p->rc.ch[4] != 0 || rc_p->rc.ch[0] != 0 || rc_p->rc.ch[2] != 0)
		{
			CtrlMode = 0;
		}
		if(rc_p->key.v != 0)
		{
			CtrlMode = 1;
		}
		
		
		// 键鼠操作同样经遥控器链路发送
		// 遥控器在线状态由左腿任务 RC_OnlineCheck() 更新, 这里只读
		
		if(rc_online == 0)
		{
			robotState = RobotState_e_Powerless;
			osDelay(1);
			continue;
		}
		
		// 按控制来源分别处理模式切换
		if(CtrlMode==0)// 遥控拨杆
        {
            if(switch_is_mid(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_Powerless;
            }
            else if(switch_is_down(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_Spinner;// 拨杆向下: 小陀螺
            }
            else if(switch_is_up(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_CommonCar;// 拨杆向上: 底盘不动
            }
        }
		else// 键鼠
		{
			if(rc_p->key.v & KEY_PRESSED_OFFSET_G)   // G 键: 底盘跟随云台
			{
				robotState=RobotState_e_GimbalCar;
			}
			
			if(rc_p->key.v & KEY_PRESSED_OFFSET_CTRL)   // Ctrl 键: 小陀螺
			{
				robotState=RobotState_e_Spinner;
			}
			if(rc_p->key.v & KEY_PRESSED_OFFSET_C)		// C 键: 底盘不动
			{
				robotState=RobotState_e_CommonCar;
			}
		}
		
		
		
		osDelay(1);
	}
}
/**
 * @brief  获取当前整车模式
 * @param  无
 * @return 指向当前模式变量的常量指针
 */
const enum RobotState_e * getRobotPresentMode(void)
{
    return &robotState;
}
