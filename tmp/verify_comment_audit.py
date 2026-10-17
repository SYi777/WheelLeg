# -*- coding: utf-8 -*-
"""Verify comment rewrite: code identity + style. Read-only."""
import io
import os
import re
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


def strip(t):
    """Same strip function as the rewrite scripts."""
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


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
                if t[i] == chr(92):
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
                if t[i] == chr(92):
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


HEADER_RE = {
    '@file': re.compile(r'\*@file\s+\S+'),
    '@brief': re.compile(r'\*@brief\s+\S+'),
    '@Author': re.compile(r'\*@Author\s+SYi777\b'),
    '@Email': re.compile(r'\*@Email\s+\S+@\S+'),
    '@Date': re.compile(r'\*@Date'),
}

# English sentence heuristic: a run of >=3 ASCII word tokens with at least one
# common English function word, excluding pure code/formula/unit contexts.
EN_STOP = set('the a an is are was were be been to of in on at for from with and or not this that these those it its as by we you can should must will when then than which where how all any each'.split())
WORD_RE = re.compile(r"[A-Za-z][A-Za-z0-9_']*")
CJK_RE = re.compile(r'[\u4e00-\u9fff]')

for rel in FILES:
    cur_path = os.path.join(ROOT, rel)
    bak_path = os.path.join(ROOT, 'tmp', 'comment_audit_originals', rel)
    cur_bytes = io.open(cur_path, 'rb').read()
    bak_bytes = io.open(bak_path, 'rb').read()

    enc_cur = 'gbk'
    try:
        cur = cur_bytes.decode('gbk')
    except UnicodeDecodeError:
        cur = cur_bytes.decode('utf-8')
        enc_cur = 'utf-8'
    enc_bak = 'gbk'
    try:
        bak = bak_bytes.decode('gbk')
    except UnicodeDecodeError:
        bak = bak_bytes.decode('utf-8')
        enc_bak = 'utf-8'

    cur_n = cur.replace('\r\n', '\n')
    bak_n = bak.replace('\r\n', '\n')
    identical = strip(cur_n) == strip(bak_n)
    raw_identical = strip(cur).encode('gbk', 'replace') == strip(bak).encode('gbk', 'replace')

    print('=' * 72)
    print(rel, '| enc cur=%s bak=%s | code_identical=%s raw_strip=%s' % (
        enc_cur, enc_bak, identical, raw_identical))

    # header check
    head = cur_n.split('*/', 1)[0] if cur_n.lstrip().startswith('/**') else ''
    problems = []
    for key, rx in HEADER_RE.items():
        if not rx.search(head):
            problems.append('header missing %s' % key)
    if head and not head.lstrip().startswith('/**'):
        problems.append('file does not start with /** header')
    if '@Date' in head:
        m = re.search(r'\*@Date[ \t]*(\S*)', head)
        if m and not m.group(1):
            problems.append('header @Date is empty')

    # english sentence check
    en_hits = []
    for ln, typ, txt in comments(cur):
        body = txt[2:] if typ == 'line' else txt[2:-2]
        for raw_line in body.splitlines():
            s = raw_line.strip().lstrip('/*').strip()
            if not s:
                continue
            if CJK_RE.search(s):
                continue
            words = WORD_RE.findall(s)
            low = [w.lower() for w in words]
            if len(words) >= 3 and (EN_STOP & set(low)):
                en_hits.append((ln, s))
    for ln, s in en_hits:
        problems.append('possibly English sentence line %d: %s' % (ln, s[:110]))

    if problems:
        for p in problems:
            print('  ISSUE:', p)
    else:
        print('  style: header OK, no English-sentence hits')
