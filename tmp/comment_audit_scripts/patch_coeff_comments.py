# -*- coding: utf-8 -*-
import io
import re
import sys

sys.path.insert(0, __file__.rsplit('\\', 1)[0] if '\\' in __file__ else __file__.rsplit('/', 1)[0])
from scan_comments import scan


def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


path = 'Kinematics/forward_solution.c'
old = io.open('tmp/comment_audit_originals/' + path, encoding='gbk').read()
cur = io.open(path, encoding='gbk').read()
spans = scan(cur)

repl = {
    20: '// a: 半角方程系数 [mm^2] = 2*L4*dx',
    21: '// b: 半角方程系数 [mm^2] = 2*L4*dy',
    22: '// c: 半角方程系数 [mm^2] = L3*L3 - L4*L4 - lac2',
}

parts = []
last = 0
for k, (a, b, typ, ln, txt) in enumerate(spans):
    if k not in repl:
        continue
    parts.append(cur[last:a])
    parts.append(repl[k])
    last = b
parts.append(cur[last:])
new = ''.join(parts)

assert strip(old) == strip(new), '代码被改动, 禁止写盘!'
io.open(path, 'w', encoding='gbk').write(new)
print('OK patched', path, 'replacements:', len(repl))
