# -*- coding: utf-8 -*-
"""Rewrite comments in Tasks/Leg_Tasks.h and Tasks/Leg_Tasks.c (comments only).

Source encodings: Leg_Tasks.h is UTF-8, Leg_Tasks.c is GBK (detected).
Output encoding: GBK for both (project convention), CRLF line endings preserved.
"""
import io
import os
import re
import shutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ORIGINALS = os.path.join(ROOT, 'tmp', 'comment_audit_originals')


def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)   # 块注释
    t = re.sub(r'//[^\n]*', '', t)                  # 行注释
    return t


# ---------------- Leg_Tasks.h ----------------
H_OLD = [
    '//安全区间定义',
    '//pid 常量定义',
    '//左腿电机控制结构体定义，包含内容nopowerflag，tffout，当前角度，目标角度，pid控制结构体',
    '\t//来自电机的值，用于初始化',
    '\t//角度',
    '\t//PID',
    '\t//control',
]
H_NEW = [
    '// 安全角度区间[rad]',
    '// 角度环 PID 常量',
    '// 左腿控制结构体: 电机角度反馈、目标角度与角度环 PID',
    '\t// 电机角度反馈指针, 初始化时绑定',
    '\t// 当前与目标绝对角度[rad]',
    '\t// 角度环 PID',
    '\t// 力矩输出[N·m] 与掉电标志',
]

# ---------------- Leg_Tasks.c ----------------
C_OLD = [
    '*左腿任务更新共享轮速和位移，右腿任务读取共享状态。',
    ' * @brief  运行左腿 1 ms 平衡与电机输出任务\n'
    ' * @param  argument  未使用的 RTOS 任务参数\n'
    ' * @return 无，任务持续循环\n'
    ' *\n'
    ' * 安全判断通过且遥控在线时更新共享轮速位移，计算本腿 LQR、横滚与防劈叉补偿，\n'
    ' * 并下发髋部与两轮力矩；异常或掉线时全部清零。',
    '// 使能看护: 未使能且无故障时每 20 ms 补发使能，故障不自动重启',
    '//髋部输出--------------------------------------------------------------',
    '// 对称模态状态: 世界系腿角 = 机体系腿角 - 车身俯仰 (与 982200d 一致)',
    '// 力段: J^T F -> 关节力矩 (出运动学后取反)',
    '// 力段: J^T F -> 关节力矩',
    '// 轮毂输出 + 差速阻尼--------------------------------------------------------------',
    '// 左右轮速差',
    '// 差速阻尼',
    '// 与 982200d 的 PairReset 一致: 清零位移积分原点',
    ' * @brief  运行右腿 1 ms 平衡与电机输出任务\n'
    ' * @param  argument  未使用的 RTOS 任务参数\n'
    ' * @return 无，任务持续循环\n'
    ' *\n'
    ' * 安全判断通过且遥控在线时执行平衡控制；右腿反馈进入运动学前取反，关节力矩输出后再取反。\n'
    ' * 共享 x/v 由左腿任务更新，本任务只读；未就绪时仍发送零扭矩防止电机维持最后一帧力矩。',
    '// 根据横滚角和角速度计算左右腿力差',
    '// 根据左右腿角度差计算防劈叉修正力矩',
]
C_NEW = [
    '*左腿任务更新共享轮速和位移, 右腿任务读取共享状态.',
    ' * @brief  左腿 1 ms 周期任务: 平衡控制与电机输出\n'
    ' * @param  argument  未使用的 RTOS 任务参数\n'
    ' * @return 无, 任务持续循环\n'
    ' *\n'
    ' * 安全判断通过、遥控在线且处于 CommonCar 模式时, 更新共享轮速与位移,\n'
    ' * 计算本腿 LQR、横滚与防劈叉补偿, 下发髋关节与轮毂力矩;\n'
    ' * 任一条件不满足时全部清零并保持零扭矩.',
    '// 使能看护: 未使能且无故障时每 20 ms 补发使能, 故障不自动重启',
    '// 髋部输出',
    '// 对称模态状态[rad]: 世界系腿角 = 机体系腿角 - 车身俯仰 (与 982200d 一致)',
    '// 力链: J^T F -> 关节力矩[N·mm] (出运动学后取反)',
    '// 力链: J^T F -> 关节力矩[N·mm]',
    '// 轮毂输出 + 差速阻尼',
    '// 左右轮速差[m/s]',
    '// 差速阻尼[N·m]',
    '// 与 982200d 的 PairReset 一致: 清零位移积分原点[m]',
    ' * @brief  右腿 1 ms 周期任务: 平衡控制与电机输出\n'
    ' * @param  argument  未使用的 RTOS 任务参数\n'
    ' * @return 无, 任务持续循环\n'
    ' *\n'
    ' * 安全判断通过、遥控在线且处于 CommonCar 模式时执行平衡控制;\n'
    ' * 右腿反馈进入运动学前取反, 关节力矩输出后再取反.\n'
    ' * 共享 x/v 由左腿任务更新, 本任务只读; 未就绪时仍发送零扭矩, 防止电机保持最后一帧力矩.',
    '/**\n'
    ' * @brief  横滚 PD 环: 由横滚角与角速度计算左右腿力差\n'
    ' * @param  roll_set   目标横滚角[rad]\n'
    ' * @param  roll       当前横滚角[rad]\n'
    ' * @param  roll_gyro  横滚角速度[rad/s]\n'
    ' * @return 横滚力差[N], 已限幅\n'
    ' */',
    '/**\n'
    ' * @brief  防劈叉 P 环: 由左右腿相对机身角差计算修正力矩\n'
    ' * @param  theta_left   左腿相对机身角[rad]\n'
    ' * @param  theta_right  右腿相对机身角[rad]\n'
    ' * @return 修正力矩[N·mm], 已限幅\n'
    ' */',
]

FILES = [
    ('Tasks/Leg_Tasks.h', 'utf-8', H_OLD, H_NEW),
    ('Tasks/Leg_Tasks.c', 'gbk', C_OLD, C_NEW),
]

for rel, src_enc, olds, news in FILES:
    src = os.path.join(ROOT, rel)
    backup = os.path.join(ORIGINALS, rel)
    os.makedirs(os.path.dirname(backup), exist_ok=True)
    shutil.copyfile(src, backup)

    raw = open(backup, 'rb').read()
    assert b'\r\n' in raw and raw.count(b'\n') == raw.count(b'\r\n'), rel + ': mixed line endings'
    assert not raw.startswith(b'\xef\xbb\xbf'), rel + ': BOM present'

    old_text = io.open(backup, encoding=src_enc).read()  # universal newlines -> \n
    new_text = old_text

    for old, new in zip(olds, news):
        n = new_text.count(old)
        assert n >= 1, '%s: snippet not found: %r' % (rel, old[:60])
        if old in ('// 使能看护: 未使能且无故障时每 20 ms 补发使能，故障不自动重启',
                   '//髋部输出--------------------------------------------------------------'):
            assert n == 2, '%s: expected 2 occurrences, got %d: %r' % (rel, n, old[:40])
        else:
            assert n == 1, '%s: expected 1 occurrence, got %d: %r' % (rel, n, old[:60])
        new_text = new_text.replace(old, new)

    # prescribed validation (same strip function); encoding of the backup follows its real encoding
    assert strip(old_text) == strip(new_text), 'code changed, refusing to write: ' + rel

    out = new_text.replace('\n', '\r\n')
    io.open(src, 'w', encoding='gbk', newline='').write(out)

    # post-write verification
    written = io.open(src, encoding='gbk').read()
    assert strip(written) == strip(old_text), 'post-write code mismatch: ' + rel
    print('OK %s -> GBK, %d bytes' % (rel, os.path.getsize(src)))
