import sys, os, capstone

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, '..', 'combobox_bkp_40000.bin')
BASE = 0x08000000
data = open(BIN, 'rb').read()
END = BASE + len(data)

md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB + capstone.CS_MODE_MCLASS)

def targets(addr):
    """Усі адреси, на які посилаються прямі BL/B/BLX, що починаються в addr."""
    out = []
    b = data[addr - BASE:addr - BASE + 4]
    if len(b) < 4:
        return out
    for i in md.disasm(b, addr, count=1):
        if i.mnemonic in ('b', 'b.n', 'b.w', 'bl', 'blx'):
            op = i.op_str
            if op.startswith('#'):
                try:
                    out.append((int(op[1:], 16), i.mnemonic, i.size))
                except ValueError:
                    pass
    return out

def scan(target, start=BASE, end=None):
    end = end or END
    hits = []
    for off in range(start - BASE, end - BASE, 2):
        for t, mn, sz in targets(BASE + off):
            if t == target:
                hits.append((BASE + off, mn))
    return hits

if __name__ == '__main__':
    tgt = int(sys.argv[1], 16)
    s = int(sys.argv[2], 16) if len(sys.argv) > 2 else BASE
    e = int(sys.argv[3], 16) if len(sys.argv) > 3 else None
    hits = scan(tgt, s, e)
    print("xrefs to 0x%08X (%d):" % (tgt, len(hits)))
    for a, mn in hits:
        print("  0x%08X  %s" % (a, mn))
