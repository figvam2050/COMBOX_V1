import sys, capstone
path = sys.argv[1]
out = sys.argv[2]
base = int(sys.argv[3], 0)
data = open(path,'rb').read()
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB + capstone.CS_MODE_MCLASS)
md.detail = False
md.skipdata = True
lines=[]
# vector table (first 16 entries)
lines.append("; vector table @ 0x%08X" % base)
import struct
for i in range(16):
    v = struct.unpack_from('<I', data, i*4)[0]
    lines.append("0x%08X: 0x%08X  ; vector %d%s" % (base+i*4, v, i, "  (initial SP)" if i==0 else "  (reset)" if i==1 else ""))
lines.append("")
for insn in md.disasm(data, base):
    b = insn.bytes.hex()
    lines.append("0x%08X:  %-8s %-8s %s" % (insn.address, b, insn.mnemonic, insn.op_str))
open(out,'w').write("\n".join(lines)+"\n")
print("instructions:", sum(1 for l in lines if l.startswith("0x") and "  " in l))
