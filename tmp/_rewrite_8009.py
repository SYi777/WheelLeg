# -*- coding: utf-8 -*-
import io, os, re, sys, difflib

ROOT = r'C:\Users\HHH\Desktop\WheelLeg'
WRITE = '--write' in sys.argv


def crlf(s):
    return s.replace('\n', '\r\n')


def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


h_header_old = """/**
*@file 8009_Ctrl_Applications.h
*@brief 对逐个8009的电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*从机器后面俯视
*左后CAN ID:0x001 MASTER ID:0x011
*左前CAN ID:0x002 MASTER ID:0x012
*右后CAN ID:0x003 MASTER ID:0x013
*右前CAN ID:0x004 MASTER ID:0x014
*定义为DM8009_Ctrl[4],依次
*
*8009电机逆时针为正，左边取车前方为零界线
*描述等效在并联腿情况下，只是电机前后交换了
*
**/"""
h_header_new = """/**
*@file 8009_CtrlApplications.h
*@brief 对逐个 8009 电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*从机器后方俯视, 各电机 CAN ID 与 MASTER ID 的对应关系:
*左后 CAN ID:0x001 MASTER ID:0x011
*左前 CAN ID:0x002 MASTER ID:0x012
*右后 CAN ID:0x003 MASTER ID:0x013
*右前 CAN ID:0x004 MASTER ID:0x014
*DM8009_Ctrl[4] 按上述顺序依次定义。
*
*8009 电机逆时针为正; 左侧以车前方为零界线。
*关节编号按并联腿等效模型描述, 只是电机前后交换了。
**/"""

h_init_old = """//初始化电机"""
h_init_new = """/**
 * @brief  初始化一台 DM8009 的控制与反馈状态
 * @param  g_motor  电机控制对象
 * @param  can_id  该电机的 CAN ID
 * @return 无
 */"""

h_torque_sec_old = """//并联腿等效模型下"""
h_torque_sec_new = """// 并联腿等效模型下的 MIT 前馈力矩接口"""

h_l1_old = """//设置左前关节，关节id为1，电机位于左后方"""
h_l1_new = """/**
 * @brief  设置左前关节 (关节 id=1) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左后方, 固定操作 DM8009_Ctrl[0]; 仅更新指令, 不发送 CAN。
 */"""

h_l2_old = """//设置左后关节，关节id为2，电机位于左前方"""
h_l2_new = """/**
 * @brief  设置左后关节 (关节 id=2) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左前方, 固定操作 DM8009_Ctrl[1]; 仅更新指令, 不发送 CAN。
 */"""

h_r1_old = """//设置右前关节，关节id为3，电机位于右前方"""
h_r1_new = """/**
 * @brief  设置右前关节 (关节 id=3) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右前方, 固定操作 DM8009_Ctrl[2]; 仅更新指令, 不发送 CAN。
 */"""

h_r2_old = """//设置右后关节，关节id为4，电机位于右后方"""
h_r2_new = """/**
 * @brief  设置右后关节 (关节 id=4) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右后方, 固定操作 DM8009_Ctrl[3]; 仅更新指令, 不发送 CAN。
 */"""

h_pos_old = """// 位置环"""
h_pos_new = """/**
 * @brief  位置环接口: 以 MIT 位置模式设置各关节目标角度
 *
 * 按 DM8009_Ctrl[0..3] 顺序对应上述四个关节; Kp=10, Kd=2, 仅更新指令, 不发送 CAN。
 */"""

c_header_old = """/**
*@file 8009_Ctrl_Applications.c
*@brief 对逐个8009的电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
**/"""
c_header_new = """/**
*@file 8009_CtrlApplications.c
*@brief 对逐个 8009 电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*各函数按固定索引操作 DM8009_Ctrl[], 形参 hfdcan/g_motor 当前未使用;
*仅更新 MIT 指令, 不发送 CAN。
**/"""

c_init_old = """/**
 * @brief  初始化一台 DM8009 的控制与反馈状态
 * @param  motor  电机控制对象
 * @param  can_id  该电机的 CAN ID
 * @return 无
 */"""
c_init_new = """/**
 * @brief  初始化一台 DM8009 的控制与反馈状态
 * @param  motor  电机控制对象
 * @param  can_id  该电机的 CAN ID
 * @return 无
 *
 * 仅清零软件状态, 不发送 CAN 报文。
 */"""

c_l1_old = """// 设置左腿第 1 台电机的 MIT 前馈力矩，不发送 CAN"""
c_l1_new = h_l1_new
c_l2_old = """// 设置左腿第 2 台电机的 MIT 前馈力矩，不发送 CAN"""
c_l2_new = h_l2_new
c_r1_old = """// 设置右腿第 1 台电机的 MIT 前馈力矩，不发送 CAN"""
c_r1_new = h_r1_new
c_r2_old = """// 设置右腿第 2 台电机的 MIT 前馈力矩，不发送 CAN"""
c_r2_new = h_r2_new

c_pl1_old = """// 设置左腿第 1 台电机的 MIT 目标角度，不发送 CAN"""
c_pl1_new = """/**
 * @brief  设置左前关节 (关节 id=1) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于左后方, 固定操作 DM8009_Ctrl[0]; Kp=10, Kd=2, 不发送 CAN。
 */"""
c_pl2_old = """// 设置左腿第 2 台电机的 MIT 目标角度，不发送 CAN"""
c_pl2_new = """/**
 * @brief  设置左后关节 (关节 id=2) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于左前方, 固定操作 DM8009_Ctrl[1]; Kp=10, Kd=2, 不发送 CAN。
 */"""
c_pr1_old = """// 设置右腿第 1 台电机的 MIT 目标角度，不发送 CAN"""
c_pr1_new = """/**
 * @brief  设置右前关节 (关节 id=3) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于右前方, 固定操作 DM8009_Ctrl[2]; Kp=10, Kd=2, 不发送 CAN。
 */"""
c_pr2_old = """// 设置右腿第 2 台电机的 MIT 目标角度，不发送 CAN"""
c_pr2_new = """/**
 * @brief  设置右后关节 (关节 id=4) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于右后方, 固定操作 DM8009_Ctrl[3]; Kp=10, Kd=2, 不发送 CAN。
 */"""

files = {
    'Applications/8009_CtrlApplications.h': [
        (h_header_old, h_header_new),
        (h_init_old, h_init_new),
        (h_torque_sec_old, h_torque_sec_new),
        (h_l1_old, h_l1_new),
        (h_l2_old, h_l2_new),
        (h_r1_old, h_r1_new),
        (h_r2_old, h_r2_new),
        (h_pos_old, h_pos_new),
    ],
    'Applications/8009_CtrlApplications.c': [
        (c_header_old, c_header_new),
        (c_init_old, c_init_new),
        (c_l1_old, c_l1_new),
        (c_l2_old, c_l2_new),
        (c_r1_old, c_r1_new),
        (c_r2_old, c_r2_new),
        (c_pl1_old, c_pl1_new),
        (c_pl2_old, c_pl2_new),
        (c_pr1_old, c_pr1_new),
        (c_pr2_old, c_pr2_new),
    ],
}


def first_diff(a, b):
    for i, (ca, cb) in enumerate(zip(a, b)):
        if ca != cb:
            return i, a[max(0, i - 60):i + 60], b[max(0, i - 60):i + 60]
    if len(a) != len(b):
        return min(len(a), len(b)), a[-60:], b[-60:]
    return None


results = {}
for rel, reps in files.items():
    src = os.path.join(ROOT, rel)
    old = io.open(src, encoding='utf-8', newline='').read()
    new = old
    for o, n in reps:
        o, n = crlf(o), crlf(n)
        cnt = new.count(o)
        assert cnt == 1, (rel, cnt, repr(o[:80]))
        new = new.replace(o, n, 1)
    # 与任务给定的校验片段一致: 默认换行模式把 CRLF 归一化为 LF
    so = strip(old.replace('\r\n', '\n'))
    sn = strip(new.replace('\r\n', '\n'))
    if so != sn:
        d = first_diff(so, sn)
        print('STRIP DIFF', rel)
        print(repr(d[1]))
        print(repr(d[2]))
        print('lens', len(so), len(sn))
    else:
        print('VALID', rel, 'old_lines', old.count('\r\n'), 'new_lines', new.count('\r\n'))
    results[rel] = (old, new)

if '--preview' in sys.argv:
    for rel, (old, new) in results.items():
        print('=' * 20, rel, '=' * 20)
        print(new)

if WRITE and all(strip(o.replace('\r\n', '\n')) == strip(n.replace('\r\n', '\n'))
                  for o, n in results.values()):
    import shutil
    for rel, (old, new) in results.items():
        src = os.path.join(ROOT, rel)
        bak = os.path.join(ROOT, 'tmp', 'comment_audit_originals', rel)
        os.makedirs(os.path.dirname(bak), exist_ok=True)
        if not os.path.exists(bak):
            shutil.copy2(src, bak)
        data = new.encode('gbk')
        assert data.decode('gbk') == new
        with open(src, 'wb') as f:
            f.write(data)
        print('WROTE', rel)
