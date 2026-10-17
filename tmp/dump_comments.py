# -*- coding: utf-8 -*-
import io
import os
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')
ROOT = r'C:/Users/HHH/Desktop/WheelLeg'
FILES = [
    'Algorithm/VMC.h', 'Algorithm/VMC.c',
    'Algorithm/balance.h', 'Algorithm/balance.c',
    'Kinematics/forward_solution.h', 'Kinematics/forward_solution.c',
    'Kinematics/inverse_solution.h', 'Kinematics/inverse_solution.c',
    'Applications/DM_motor.h', 'Applications/DM_motor.c',
    'Applications/3508_CtrlApplications.h', 'Applications/3508_CtrlApplications.c',
]
BS = chr(92)


def comments(t):
    out = []
    i = 0
    n = len(t)
    line = 1
    while i < n:
        c = t[i]
        if c == '\n':
            line += 1
            i += 1
            continue
        if c == '"':
            i += 1
            while i < n:
                if t[i] == BS:
                    i += 2
                    continue
                if t[i] == '"':
                    i += 1
                    break
                if t[i] == '\n':
                    line += 1
                i += 1
            continue
        if c == "'":
            i += 1
            while i < n:
                if t[i] == BS:
                    i += 2
                    continue
                if t[i] == "'":
                    i += 1
                    break
                if t[i] == '\n':
                    line += 1
                i += 1
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '/':
            j = t.find('\n', i)
            j = n if j < 0 else j
            out.append((line, 'line', t[i:j]))
            i = j
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '*':
            j = t.find('*/', i + 2)
            j = n if j < 0 else j + 2
            txt = t[i:j]
            out.append((line, 'block', txt))
            line += txt.count('\n')
            i = j
            continue
        i += 1
    return out


for rel in FILES:
    t = io.open(os.path.join(ROOT, rel), encoding='gbk').read()
    cs = comments(t)
    print('#' * 78)
    print('### %s  (%d comment spans)' % (rel, len(cs)))
    for ln, typ, txt in cs:
        for k, raw in enumerate(txt.splitlines()):
            s = raw.strip()
            if not s:
                continue
            print('%4d %-5s %s' % (ln + k, typ, s))
