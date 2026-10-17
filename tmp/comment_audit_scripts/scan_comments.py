import io, sys

def scan(t):
    spans = []
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
                if t[i] == '\\':
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
                if t[i] == '\\':
                    i += 2
                    continue
                if t[i] == "'":
                    i += 1
                    break
                i += 1
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '/':
            j = t.find('\n', i)
            if j == -1:
                j = n
            spans.append((i, j, 'line', line, t[i:j]))
            i = j
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '*':
            j = t.find('*/', i + 2)
            j = n if j == -1 else j + 2
            text = t[i:j]
            spans.append((i, j, 'block', line, text))
            line += text.count('\n')
            i = j
            continue
        i += 1
    return spans

for p in sys.argv[1:]:
    t = io.open(p, encoding='gbk').read()
    spans = scan(t)
    out = io.open('tmp/comment_audit_view/' + p.split('/')[-1] + '.comments.txt', 'w', encoding='utf-8')
    for k, (a, b, typ, ln, txt) in enumerate(spans):
        out.write('### [%d] %s line %d span (%d,%d)\n%s\n\n' % (k, typ, ln, a, b, txt))
    out.close()
    print(p, len(spans), 'comments')
