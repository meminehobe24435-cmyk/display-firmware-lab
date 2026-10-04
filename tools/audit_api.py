# -*- coding: utf-8 -*-
"""列出 dfw.h 声明但 src/*.c 里还没有定义的函数。"""
import io
import re
import sys
from pathlib import Path

SRC = Path(r'D:\zhuomian\工作资料\display-firmware-lab\src')
hdr = (SRC / 'dfw.h').read_text(encoding='utf-8', errors='replace')

decl = set()
for m in re.finditer(r'^\s*(?:const char \*|double|int|unsigned|uint8_t|uint32_t|void|size_t|dfw_vcp_entry \*|const dfw_osd_item \*)\s*'
                     r'\**\s*(dfw_[a-z0-9_]+)\s*\(', hdr, re.M):
    decl.add(m.group(1))

defined = {}
for f in sorted(SRC.glob('*.c')):
    t = f.read_text(encoding='utf-8', errors='replace')
    for m in re.finditer(r'^\s*(?:const char \*|double|int|unsigned|uint8_t|uint32_t|void|size_t|dfw_vcp_entry \*|const dfw_osd_item \*)\s*'
                         r'\**\s*(dfw_[a-z0-9_]+)\s*\(', t, re.M):
        defined.setdefault(m.group(1), f.name)

missing = sorted(d for d in decl if d not in defined)
out = io.StringIO()
out.write('declared=%d  defined=%d  missing=%d\n' % (len(decl), len(defined), len(missing)))
out.write('\n--- 已定义 ---\n')
for k in sorted(defined):
    out.write('  %-34s %s\n' % (k, defined[k]))
out.write('\n--- 缺失（需要实现）---\n')
for k in missing:
    out.write('  %s\n' % k)
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stdout.write(out.getvalue())
