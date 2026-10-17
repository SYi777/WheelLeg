# -*- coding: utf-8 -*-
"""Remote_control.h/.c 注释风格统一脚本（仅改注释，代码零改动）"""
import io
import os
import re
import shutil
import difflib

ROOT = r'C:/Users/HHH/Desktop/WheelLeg'
BACKUP_DIR = os.path.join(ROOT, 'tmp', 'comment_audit_originals')


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


def apply_pairs(text, pairs, name):
    t = text
    for i, (old, new) in enumerate(pairs):
        c = t.count(old)
        if c != 1:
            raise AssertionError('%s: 第 %d 条替换命中 %d 次: %r' % (name, i + 1, c, old[:80]))
        t = t.replace(old, new, 1)
    return t


H_OLD = io.open(os.path.join(ROOT, 'Applications', 'Remote_control.h'),
                encoding='gbk').read()
C_OLD = io.open(os.path.join(ROOT, 'Applications', 'Remote_control.c'),
                encoding='gbk').read()

H_PAIRS = [
    (
        '/**\n'
        '*@file Remote_control.h\n'
        '*@brief 遥控器 SBUS 数据结构与接口声明\n'
        '*@Author SYi777\n'
        '*@Email jyh1621839418@163.com\n'
        '*@Date\n'
        '*\n'
        '*遥控器接收与解析实现在 Remote_control.c。\n'
        '**/',
        '/**\n'
        '*@file Remote_control.h\n'
        '*@brief 遥控器 SBUS 数据结构、协议常量与接口声明\n'
        '*@Author SYi777\n'
        '*@Email jyh1621839418@163.com\n'
        '*@Date\n'
        '*\n'
        '*接收解析与掉线检测实现在 Remote_control.c。\n'
        '**/'
    ),
    ('//缓冲区大小', '// SBUS 双缓冲接收区大小[字节]'),
    ('//单帧大小', '// 单帧长度[字节]'),
    ('//遥控器错误阈值', '// 通道错误判定阈值'),
    ('/* ----------------------- RC Switch Definition----------------------------- */',
     '// 拨杆定义'),
    ('/* ----------------------- PC Key Definition-------------------------------- */',
     '// 键盘按键位定义'),
    ('/* ----------------------- Data Struct ------------------------------------- */',
     '// 数据结构'),
    ('//全局变量声明', '// 全局变量声明'),
    ('//--------------functions---------------//', '// 函数声明'),
    (
        '/**\n'
        ' * @brief  获取遥控器数据指针。\n'
        ' * @retval 指向全局遥控器数据结构的常量指针。\n'
        ' */',
        '/**\n'
        ' * @brief  获取遥控器数据指针\n'
        ' * @return 指向全局遥控器数据结构的常量指针\n'
        ' */'
    ),
    (
        '/**\n'
        ' * @brief  判断遥控器数据是否出现错误（如通道值超限或拨杆异常）。\n'
        ' * @retval 0: 数据正常；1: 发生错误，且已将所有控制量归零/置位。\n'
        ' */',
        '/**\n'
        ' * @brief  判断遥控器数据是否异常\n'
        ' * @return 0 数据正常；1 数据异常，所有控制量已清零\n'
        ' *\n'
        ' * 通道绝对值超过 RC_CHANNAL_ERROR_VALUE 或拨杆值为 0 时判为异常，\n'
        ' * 异常时拨杆置为 RC_SW_DOWN。\n'
        ' */'
    ),
    (
        '/**\n'
        ' * @brief  遥控器数据错误时的处理函数。\n'
        ' *         内部调用 RC_restart 重启 DMA 双缓冲接收。\n'
        ' */',
        '/**\n'
        ' * @brief  遥控器数据异常时重启接收\n'
        ' * @return 无\n'
        ' *\n'
        ' * 调用 RC_restart 重启 UART5 DMA 双缓冲接收。\n'
        ' */'
    ),
    (
        '/**\n'
        ' * @brief  遥控器模块底层初始化（DMA 双缓冲配置等）。\n'
        ' */',
        '/**\n'
        ' * @brief  遥控器模块底层初始化\n'
        ' * @return 无\n'
        ' *\n'
        ' * 配置 UART5 DMA 双缓冲接收，缓冲区为 SBUS_MultiRx_Buf。\n'
        ' */'
    ),
    (
        '/**\n'
        ' * @brief  遥控器掉线检测，需 1kHz 周期调用。\n'
        ' *         连续 100ms 未收到 SBUS 帧将 rc_online 置 0。\n'
        ' */',
        '/**\n'
        ' * @brief  遥控器掉线检测，需 1kHz 周期调用\n'
        ' * @return 无\n'
        ' *\n'
        ' * flag 由接收中断置 1；连续约 100 ms 未收到 SBUS 帧时将 rc_online 置 0，\n'
        ' * 恢复收帧后自动置 1。\n'
        ' */'
    ),
]

C_PAIRS = [
    (
        '/**\n'
        '*@file Remote_control.c\n'
        '*@brief 遥控器 SBUS 数据接收与解析\n'
        '*@Author 王草凡 SYi777\n'
        '*@Email jyh1621839418@163.com\n'
        '*@Date 2026-10-13\n'
        '*\n'
        '*参考王草凡的 C 板实现，移植到 H7 并使用 HAL 初始化 DMA，添加遥控器离线保护\n'
        '**/',
        '/**\n'
        '*@file Remote_control.c\n'
        '*@brief 遥控器 SBUS 数据接收、解析与掉线检测\n'
        '*@Author 王草凡 SYi777\n'
        '*@Email jyh1621839418@163.com\n'
        '*@Date 2026-10-13\n'
        '*\n'
        '*参考王草凡的 C 板实现，移植到 H7 并使用 HAL 初始化 DMA，增加遥控器离线保护。\n'
        '**/'
    ),
    ('// 参考王草凡的遥控器协议说明', '// SBUS 协议说明参考王草凡的遥控器实现'),
    ('// 移植 C 板遥控器解析逻辑，使用 HAL 初始化 H7 的 DMA',
     '// 解析逻辑移植自 C 板，DMA 由 HAL 初始化'),
    ('// 遥控器在线标志，0 离线 1 在线', '// 遥控器在线标志[0 离线, 1 在线]'),
    ('// 中断内部标志位', '// 新帧标志[1 表示收到新帧]'),
    ('// 未进中断计数', '// 连续无帧计数[ms]'),
    ('// 遥控器控制变量', '// 遥控器数据'),
    ('// 双缓冲接收 SBUS 原始数据，防止DMA传输越界',
     '// SBUS 双缓冲接收区，固定于 0x24000000，防止 DMA 越界'),
    (
        '/**\n'
        ' * @brief  解析一帧 SBUS 数据并更新遥控器状态\n'
        ' * @param  sbus_buf  收到的 SBUS 原始数据\n'
        ' * @param  remote_ctrl  输出的遥控器状态\n'
        ' * @return 无\n'
        ' */',
        '/**\n'
        ' * @brief  解析一帧 SBUS 数据并更新遥控器状态\n'
        ' * @param  sbus_buf     收到的 SBUS 原始数据\n'
        ' * @param  remote_ctrl  输出的遥控器状态\n'
        ' * @return 无\n'
        ' *\n'
        ' * 各通道为 11 位原始值，解析后统一扣除 RC_CH_VALUE_OFFSET 零偏。\n'
        ' */'
    ),
    ('// 遥控器通道 0', '// 通道 0'),
    ('// 遥控器通道 1', '// 通道 1'),
    ('// 遥控器通道 2', '// 通道 2'),
    ('// 遥控器通道 3', '// 通道 3'),
    ('// 遥控器通道 4', '// 通道 4'),
    ('// 计算遥控器通道值的绝对值', '// 求通道值的绝对值'),
    (
        '/**\n'
        ' * @brief  处理 UART5 DMA 双缓冲接收事件\n'
        ' * @param  huart  UART 句柄\n'
        ' * @param  Size  本次接收的字节数\n'
        ' * @return 无\n'
        ' *\n'
        ' * 根据 DMA 目标缓冲区位选择已完成接收的缓冲区。\n'
        ' */',
        '/**\n'
        ' * @brief  处理 UART5 DMA 双缓冲接收事件\n'
        ' * @param  huart  UART 句柄\n'
        ' * @param  Size   本次接收的字节数\n'
        ' * @return 无\n'
        ' *\n'
        ' * 根据 DMA 目标缓冲区位判断接收完成的是哪个缓冲区，\n'
        ' * 仅在长度等于 RC_FRAME_LENGTH 时解析。\n'
        ' */'
    ),
    ('// 收到接收事件 → 遥控器在线喂狗', '// 收到接收事件，喂狗'),
    ('// 缓冲区0 接收完成', '// 缓冲区 0 接收完成'),
    ('// 缓冲区1 接收完成', '// 缓冲区 1 接收完成'),
    (
        '/**\n'
        ' * @brief  将 UART5 接收事件转交给遥控器处理函数\n'
        ' * @param  huart  触发事件的 UART 句柄\n'
        ' * @param  Size  本次接收的字节数\n'
        ' * @return 无\n'
        ' */',
        '/**\n'
        ' * @brief  将 UART5 接收事件转交给遥控器处理函数\n'
        ' * @param  huart  触发事件的 UART 句柄\n'
        ' * @param  Size   本次接收的字节数\n'
        ' * @return 无\n'
        ' */'
    ),
    ('// 获取全局遥控器状态的只读指针', '// 获取全局遥控器数据指针'),
    (
        '/**\n'
        ' * @brief  检测遥控器通道与拨杆数据是否越界\n'
        ' * @return 0 表示正常，1 表示错误且遥控器状态已清零\n'
        ' */',
        '/**\n'
        ' * @brief  检测遥控器通道与拨杆数据是否异常\n'
        ' * @return 0 正常；1 异常，且遥控器状态已清零\n'
        ' *\n'
        ' * 任一通道绝对值超过 RC_CHANNAL_ERROR_VALUE 或拨杆值为 0 即判为异常，\n'
        ' * 异常时将遥控器数据全部清零，拨杆置为 RC_SW_DOWN。\n'
        ' */'
    ),
    ('// 重启遥控器 DMA 双缓冲接收', '// 重启 UART5 DMA 双缓冲接收'),
    (
        '/**\n'
        ' * @brief  遥控器掉线检测 (1kHz 调用)\n'
        ' * @return 无\n'
        ' *\n'
        ' * flag 由接收中断置 1，本函数由状态任务按 1 kHz 调用。\n'
        ' * 连续约 100 ms 未收到帧时清除 rc_online，恢复收帧后自动置 1。\n'
        ' */',
        '/**\n'
        ' * @brief  遥控器掉线检测，需 1kHz 周期调用\n'
        ' * @return 无\n'
        ' *\n'
        ' * flag 由接收中断置 1，本函数由状态任务按 1 kHz 调用；\n'
        ' * 连续约 100 ms 未收到帧时清除 rc_online，恢复收帧后自动置 1。\n'
        ' */'
    ),
    ('// 清狗, 等待下一帧再喂', '// 清标志，等待下一帧'),
    ('// 有帧 → 在线', '// 有帧，判为在线'),
    ('// 状态任务每毫秒计一次，封顶防溢出', '// 每毫秒加一，封顶防溢出'),
    ('// 约 100 ms 无帧则判掉线', '// 约 100 ms 无帧判掉线'),
]

H_NEW = apply_pairs(H_OLD, H_PAIRS, 'Remote_control.h')
C_NEW = apply_pairs(C_OLD, C_PAIRS, 'Remote_control.c')

# 写盘前强制校验：去掉注释后代码必须逐字节一致
for name, old, new in [('Remote_control.h', H_OLD, H_NEW),
                       ('Remote_control.c', C_OLD, C_NEW)]:
    so, sn = strip_comments(old), strip_comments(new)
    if so != sn:
        diff = list(difflib.unified_diff(so.splitlines(True), sn.splitlines(True),
                                         'old_stripped', 'new_stripped'))
        raise AssertionError('%s 代码被改动, 禁止写盘!\n%s' % (name, ''.join(diff)))

# 备份原文件（二进制原样复制）
for rel, new in [('Applications/Remote_control.h', H_NEW),
                 ('Applications/Remote_control.c', C_NEW)]:
    src = os.path.join(ROOT, rel)
    dst = os.path.join(BACKUP_DIR, rel)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)
    # GBK 写回，强制 CRLF
    with io.open(src, 'w', encoding='gbk', newline='\r\n') as f:
        f.write(new)

# 写后回读验证
for rel in ['Applications/Remote_control.h', 'Applications/Remote_control.c']:
    src = os.path.join(ROOT, rel)
    raw = open(src, 'rb').read()
    text = raw.decode('gbk')
    assert '\ufffd' not in text
    assert raw.count(b'\n') == raw.count(b'\r\n'), '出现裸 LF'
    bak = open(os.path.join(BACKUP_DIR, rel), 'rb').read().decode('gbk')
    assert strip_comments(bak) == strip_comments(text), rel + ' 回读代码不一致'
    print('OK', rel)

print('全部通过')
