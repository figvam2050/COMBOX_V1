import os, struct, sys, collections, json, re
import capstone

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'combobox_bkp_40000.bin')
BASE = 0x08000000
data = open(BIN, 'rb').read()
END = BASE + 0x3E104

md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB + capstone.CS_MODE_MCLASS)
md.detail = True

def rd(addr, n):
    o = addr - BASE
    if o < 0 or o + n > len(data):
        return None
    return data[o:o + n]

insns = {}
lits = {}
calls = collections.defaultdict(list)
xrefs = collections.defaultdict(list)
func_starts = set()
visited = set()

def disasm_one(addr):
    if addr in insns:
        return insns[addr]
    b = rd(addr, 4)
    if b is None:
        return None
    got = list(md.disasm(b, addr, count=1))
    if not got:
        insns[addr] = dict(mn='<und>', op='', size=2, raw=b[:2].hex())
        return insns[addr]
    i = got[0]
    insns[addr] = dict(mn=i.mnemonic, op=i.op_str, size=i.size, raw=i.bytes.hex())
    return insns[addr]

def lit_value(pc, disp):
    tgt = ((pc + 4) & ~3) + disp
    b = rd(tgt, 4)
    if b is None:
        return None
    return struct.unpack('<I', b)[0]

RE_IMM = re.compile(r'#(-?0x[0-9a-fA-F]+|-?\d+)')

def parse_imm(s):
    m = RE_IMM.search(s)
    if not m:
        return None
    t = m.group(1)
    return int(t, 16) if 'x' in t else int(t)

def walk(start, budget=2000000):
    stack = [start]
    func_starts.add(start)
    while stack and len(visited) < budget:
        addr = stack.pop()
        pending_lit = None
        while addr is not None and addr < END and addr not in visited:
            visited.add(addr)
            ins = disasm_one(addr)
            if ins is None:
                break
            mn, op, size = ins['mn'], ins['op'], ins['size']
            nxt = addr + size
            # literal capture
            if mn.startswith('ldr') and '[pc' in op:
                d = parse_imm(op)
                if d is not None:
                    v = lit_value(addr, d)
                    if v is not None:
                        lits[addr] = v
                        pending_lit = (op.split(',')[0].strip(), v)
            if re.match(r'^b(eq|ne|hs|lo|cc|cs|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)$', mn):
                t = parse_imm(op)
                if t and BASE <= t < END:
                    stack.append(t)
                addr = nxt
            elif mn in ('b', 'b.n', 'b.w'):
                t = parse_imm(op)
                if t and BASE <= t < END:
                    func_starts.add(t)
                    stack.append(t)
                addr = None
            elif mn in ('bl', 'blx') and not re.match(r'^[br]\w*$', op.strip()):
                t = parse_imm(op)
                if t and BASE <= t < END:
                    calls[addr].append(t)
                    xrefs[t].append(addr)
                    func_starts.add(t)
                    stack.append(t)
                addr = nxt
            elif mn in ('bx', 'blx'):
                reg = op.strip()
                if pending_lit and pending_lit[0] == reg:
                    v = pending_lit[1]
                    if BASE <= (v & ~1) < END:
                        t = v & ~1
                        calls[addr].append(t)
                        xrefs[t].append(addr)
                        func_starts.add(t)
                        stack.append(t)
                addr = nxt if mn == 'blx' else None
            elif mn in ('pop',) and 'pc' in op:
                addr = None
            elif mn in ('cbz', 'cbnz'):
                t = parse_imm(op)
                if t and BASE <= t < END:
                    stack.append(t)
                    func_starts.add(t)
                addr = nxt
            elif mn in ('tbb', 'tbh'):
                addr = None
            else:
                addr = nxt

# ---- find prologues: push {..., lr}
prologues = []
i = 0
while i < len(data) - 4:
    hw = struct.unpack_from('<H', data, i)[0]
    # 16-bit push with lr: 0xB5xx
    if 0xB500 <= hw <= 0xB5FF:
        prologues.append(BASE + i)
        i += 2
        continue
    # 32-bit push.w E92D xxxx  (bits: 1110 1001 0010 1101)
    if hw == 0xE92D:
        prologues.append(BASE + i)
        i += 2
        continue
    i += 2
print("prologues found:", len(prologues))

# walk from every prologue + both reset vectors
entries = set(p for p in prologues)
entries.add(0x08004164)   # app reset (ldr+blx pattern handled)
entries.add(0x08000164)
for e in sorted(entries):
    walk(e)
print("instructions:", len(insns), "func_starts:", len(func_starts))

# ---- classify
PERIPH = {}
named = [('USART0',0x40013800),('USART1',0x40004400),('USART2',0x40004800),('UART3',0x40004C00),
         ('UART4',0x40005000),('CAN0',0x40006400),('CAN1',0x40006800),('GPIOA',0x40010800),
         ('GPIOB',0x40010C00),('GPIOC',0x40011000),('GPIOD',0x40011400),('GPIOE',0x40011800),
         ('AFIO',0x40010400),('RCU',0x40021000),('FMC',0x40022000),('IWDG',0x40003000),
         ('WWDG',0x40003400),('DMA1',0x40020000),('DMA2',0x40020400),('ADC0',0x40012400),('ADC1',0x40012800),
         ('SPI0',0x40013000),('SPI1',0x40003800),('I2C0',0x40005400),('I2C1',0x40005800),
         ('TIM0',0x40012C00),('TIM1',0x40000000),('TIM2',0x40000400),('TIM3',0x40000800),('TIM4',0x40000C00),
         ('SYSTICK',0xE000E010),('SCB',0xE000ED00),('AIRCR',0xE000ED0C),('DBGMCU',0x40015800),
         ('CRC',0x40023000),('DAC',0x40007400),('PWR',0x40007000),('BKP',0x40006C00),('FSMC',0xA0000000)]
for n, b in named:
    PERIPH[b] = n

def classify(v):
    if v in PERIPH:
        return PERIPH[v]
    best = None
    for b, n in PERIPH.items():
        if b <= v < b + 0x400:
            if best is None or (v - b) < best[1]:
                best = (n, v - b)
    if best:
        return "%s+0x%X" % best
    if 0x20000000 <= v < 0x20020000:
        return "RAM+0x%X" % (v - 0x20000000)
    if BASE <= v < END:
        return "FL:0x%08X" % v
    return None

json.dump({"insns": {str(k): v for k, v in insns.items()},
           "lits": {str(k): v for k, v in lits.items()},
           "func": sorted(func_starts),
           "calls": {str(k): v for k, v in calls.items()},
           "xrefs": {str(k): v for k, v in xrefs.items()}},
          open(os.path.join(HERE, 'db.json'), 'w'))

fs = sorted(func_starts)
import bisect
report = []
for f in fs:
    i = bisect.bisect_right(fs, f)
    end = fs[i] if i < len(fs) else END
    names = set()
    for ad, v in lits.items():
        if f <= ad < end:
            c = classify(v)
            if c:
                names.add(c)
    if names:
        report.append((f, end - f, sorted(names)))
with open(os.path.join(HERE, 'periph_report.txt'), 'w') as fh:
    for f, sz, names in report:
        fh.write("0x%08X size=%-6d %s\n" % (f, sz, ', '.join(names)))
print("funcs with refs:", len(report))
