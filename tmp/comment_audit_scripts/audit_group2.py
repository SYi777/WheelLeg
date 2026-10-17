# -*- coding: utf-8 -*-
"""Audit comment rewrite safety + style for group 2 files. Read-only."""
import io
import os
import re
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')
ROOT = r'C:/Users/HHH/Desktop/WheelLeg'
FILES = [
    'Applications/8009_CtrlApplications.h', 'Applications/8009_CtrlApplications.c',
    'Applications/Remote_control.h', 'Applications/Remote_control.c',
    'Tasks/Leg_Tasks.h', 'Tasks/Leg_Tasks.c',
    'Tasks/Left_leg.h', 'Tasks/Left_leg.c',
    'Tasks/RobotState.h', 'Tasks/RobotState.c',
    'Applications/struct_typedef.h',
]

BS = chr(92)
SQ = chr(39)
DQ = chr(34)


def decode(b):
    try:
        return b.decode('gbk'), 'gbk'
    except UnicodeDecodeError:
        return b.decode('utf-8'), 'utf-8'


def strip_regex(t):
    """Same strip function as the rewrite scripts (after universal-newline read)."""
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


def strip_robust(t):
    """Comment-aware strip that respects string/char literals."""
    out = []
    i = 0
    n = len(t)
    while i < n:
        c = t[i]
        if c == DQ:
            out.append(c)
            i += 1
            while i < n:
                out.append(t[i])
                if t[i] == BS and i + 1 < n:
                    out.append(t[i + 1])
                    i += 2
                    continue
                if t[i] == DQ:
                    i += 1
                    break
                i += 1
            continue
        if c == SQ:
            out.append(c)
            i += 1
            while i < n:
                out.append(t[i])
                if t[i] == BS and i + 1 < n:
                    out.append(t[i + 1])
                    i += 2
                    continue
                if t[i] == SQ:
                    i += 1
                    break
                i += 1
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '/':
            i += 2
            while i < n and t[i] != '\n':
                i += 1
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '*':
            i += 2
            while i + 1 < n and not (t[i] == '*' and t[i + 1] == '/'):
                i += 1
            i += 2
            continue
        out.append(c)
        i += 1
    return ''.join(out)


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
        if c == DQ:
            i += 1
            while i < n:
                if t[i] == BS:
                    i += 2
                    continue
                if t[i] == DQ:
                    i += 1
                    break
                if t[i] == '\n':
                    line += 1
                i += 1
            continue
        if c == SQ:
            i += 1
            while i < n:
                if t[i] == BS:
                    i += 2
                    continue
                if t[i] == SQ:
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
EN_STOP = set('the a an is are was were be been being to of in on at for from with '
              'and or not this that these those it its as by we you can should must '
              'will would when then than which where how all any each there here if '
              'else into over under about after before'.split())
WORD_RE = re.compile(r"[A-Za-z][A-Za-z0-9_']*")
CJK_RE = re.compile(r'[\u4e00-\u9fff]')


for rel in FILES:
    cur_raw = io.open(os.path.join(ROOT, rel), 'rb').read()
    bak_raw = io.open(os.path.join(ROOT, 'tmp', 'comment_audit_originals', rel), 'rb').read()
    cur, enc_cur = decode(cur_raw)
    bak, enc_bak = decode(bak_raw)
    cur_n = cur.replace('\r\n', '\n')
    bak_n = bak.replace('\r\n', '\n')

    identical = strip_regex(cur_n) == strip_regex(bak_n)
    robust = strip_robust(cur_n) == strip_robust(bak_n)

    problems = []
    if not identical:
        a = strip_regex(cur_n)
        b = strip_regex(bak_n)
        for i, (x, y) in enumerate(zip(a, b)):
            if x != y:
                problems.append('CODE DIFF @%d: cur %r / bak %r' % (
                    i, a[max(0, i - 40):i + 40], b[max(0, i - 40):i + 40]))
                break
        else:
            problems.append('CODE DIFF: prefix equal but length %d vs %d' % (len(a), len(b)))
    if not robust:
        problems.append('ROBUST CODE DIFF (string-aware strip)')

    # ---- header template ----
    head = cur_n.split('*/', 1)[0] if cur_n.lstrip().startswith('/**') else ''
    for key, rx in HEADER_RE.items():
        if not rx.search(head):
            problems.append('header missing %s' % key)
    if not head:
        problems.append('file does not start with /** header')
    if '@Date' in head:
        m = re.search(r'\*@Date[ \t]*(\S*)', head)
        if m and not m.group(1):
            problems.append('header @Date is empty')

    # ---- English sentence heuristic ----
    TAGS = set('file brief param return retval author email date version'.split())
    for ln, typ, txt in comments(cur_n):
        body = txt[2:] if typ == 'line' else txt[2:-2]
        for k, raw_line in enumerate(body.splitlines()):
            s = raw_line.strip().lstrip('/*').strip()
            if not s:
                continue
            words = WORD_RE.findall(s)
            seq = []
            seqs = []
            for w in words:
                # all-caps tokens are identifiers/units (CAN, ID, PID, LQR, ...)
                # tags and CamelCase identifiers are not prose either
                if (re.fullmatch(r"[A-Za-z']+", w)
                        and not (w.isupper() and len(w) >= 2)
                        and w.lower() not in TAGS
                        and not re.search(r'[a-z][A-Z]', w)):
                    seq.append(w)
                else:
                    if len(seq) >= 3:
                        seqs.append(seq)
                    seq = []
            if len(seq) >= 3:
                seqs.append(seq)
            bad = [q for q in seqs if any(w.lower() in EN_STOP for w in q)]
            if bad:
                problems.append('possibly English sentence line %d: %s' % (ln + k, s[:120]))

    print('=' * 72)
    print('%s | enc cur=%s bak=%s | code_identical=%s robust=%s | style_ok=%s' % (
        rel, enc_cur, enc_bak, identical, robust, not problems))
    for p in problems:
        print('  ISSUE:', p)
