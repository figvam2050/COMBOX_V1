import json, sys, os, bisect, re
HERE = os.path.dirname(os.path.abspath(__file__))
db = json.load(open(os.path.join(HERE, 'db.json')))
insns = {int(k): v for k, v in db['insns'].items()}
lits = {int(k): v for k, v in db['lits'].items()}
func = sorted(db['func'])
calls = {int(k): v for k, v in db['calls'].items()}
xrefs = {int(k): v for k, v in db['xrefs'].items()}

def cls(v):
    named = [('USART0',0x40013800),('USART1',0x40004400),('USART2',0x40004800),('UART3',0x40004C00),
             ('UART4',0x40005000),('CAN0',0x40006400),('GPIOA',0x40010800),('GPIOB',0x40010C00),
             ('GPIOC',0x40011000),('GPIOD',0x40011400),('GPIOE',0x40011800),('AFIO',0x40010400),
             ('RCU',0x40021000),('FMC',0x40022000),('IWDG',0x40003000),('WWDG',0x40003400),
             ('DMA1',0x40020000),('ADC0',0x40012400),('ADC1',0x40012800),('SPI0',0x40013000),
             ('I2C0',0x40005400),('TIM0',0x40012C00),('TIM1',0x40000000),('TIM2',0x40000400),
             ('SYSTICK',0xE000E010),('SCB',0xE000ED00),('AIRCR',0xE000ED0C),('CRC',0x40023000),
             ('PWR',0x40007000),('BKP',0x40006C00),('DAC',0x40007400)]
    for n, b in named:
        if b <= v < b + 0x400:
            off = v - b
            return "%s+0x%X%s" % (n, off, " [BASE]" if off == 0 else "")
    if 0x20000000 <= v < 0x20020000:
        return "RAM[0x%08X]" % v
    if 0x08000000 <= v < 0x08040000:
        return "FL[0x%08X]" % v
    if v < 0x10000:
        return "#0x%X" % v
    return "0x%08X" % v

def dump_raw(a, b):
    import capstone, struct as _s
    bin_path = os.path.join(HERE, '..', 'combobox_bkp_40000.bin')
    if not os.path.exists(bin_path):
        print("db.json не містить 0x%08X, а %s не знайдено" % (a, bin_path))
        return
    raw = open(bin_path, 'rb').read()
    base = 0x08000000
    md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB + capstone.CS_MODE_MCLASS)
    off = a - base
    ad = a
    print(";; raw fallback (немає в db.json)")
    for i in md.disasm(raw[off:off + (b - a)], a):
        cm = ""
        if i.mnemonic.startswith('ldr') and '[pc' in i.op_str:
            m = re.search(r'#(-?0x[0-9a-fA-F]+|-?\d+)', i.op_str)
            if m:
                d = int(m.group(1), 16) if 'x' in m.group(1) else int(m.group(1))
                tgt = ((i.address + 4) & ~3) + d
                o = tgt - base
                if 0 <= o + 4 <= len(raw):
                    cm = "    ; = %s" % cls(_s.unpack_from('<I', raw, o)[0])
        print("0x%08X: %-9s %-7s %-38s%s" % (i.address, i.bytes.hex(), i.mnemonic, i.op_str, cm))

def dump(a, b):
    addrs = sorted(x for x in insns if a <= x < b)
    if not addrs:
        dump_raw(a, b)
        return
    prev = None
    for ad in addrs:
        i = insns[ad]
        if prev is not None and ad != prev:
            print("   ...")
        cm = ""
        if ad in lits:
            cm = "    ; = %s" % cls(lits[ad])
        elif re.match(r'^b(l|lx|\.|$)', i['mn']):
            m = re.search(r'#?-?0x[0-9a-fA-F]+', i['op'])
            if m:
                t = int(m.group(0).lstrip('#'), 16)
                f = "FUNC" if t in db['func'] else ""
                cm = "    ; -> 0x%08X %s" % (t, f)
        print("0x%08X: %-9s %-7s %-38s%s%s" % (ad, i['raw'], i['mn'], i['op'], cm,
              "   <=%d call(s)" % len(xrefs.get(ad, [])) if xrefs.get(ad) else ""))
        prev = ad + i['size']

def who_calls(target):
    print("xrefs to 0x%08X: %s" % (target, ['0x%X' % x for x in xrefs.get(target, [])]))

if __name__ == '__main__':
    if sys.argv[1] == 'x':
        who_calls(int(sys.argv[2], 16))
    else:
        dump(int(sys.argv[1], 16), int(sys.argv[2], 16))
