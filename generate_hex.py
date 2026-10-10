Import("env")
import os
import shutil

# Image offset of the 'G5RC' dword the stock bootloader checks at 0x08004150
# (vector table is 0x150 bytes: 16 core slots + IRQ0..67). See ORIGINAL_FIRMWARE.md §13.
G5RC_OFFSET = 0x150
G5RC_MAGIC = b"G5RC"

def _patch_hex(hex_path, offset, blob):
    out_lines = []
    ela = 0
    with open(hex_path, "r") as f:
        lines = [line.strip() for line in f if line.strip()]
    # HEX records use absolute flash addresses; the image base is the lowest
    # data record (== FLASH origin of the linker script).
    base0 = None
    ela = 0
    for line in lines:
        if not line.startswith(":"):
            continue
        rec = bytes.fromhex(line[1:])
        if rec[3] == 4:
            ela = ((rec[4] << 8) | rec[5]) << 16
        elif rec[3] == 0:
            addr = ela | ((rec[1] << 8) | rec[2])
            base0 = addr if base0 is None else min(base0, addr)
    if base0 is None:
        raise RuntimeError("no data records in %s" % hex_path)
    want = {base0 + offset + i: b for i, b in enumerate(blob)}
    ela = 0
    for line in lines:
        if not line.startswith(":"):
            out_lines.append(line)
            continue
        rec = bytes.fromhex(line[1:])
        count, addr, rtype = rec[0], (rec[1] << 8) | rec[2], rec[3]
        payload, csum = rec[4 : 4 + count], rec[4 + count]
        if (sum(rec) & 0xFF) != 0:
            raise RuntimeError("bad Intel HEX checksum in %s" % hex_path)
        if rtype == 4:
            ela = (payload[0] << 8) | payload[1]
        elif rtype == 0:
            base = (ela << 16) | addr
            data = bytearray(payload)
            for i in range(count):
                if (base + i) in want:
                    data[i] = want.pop(base + i)
            if data != payload:
                hdr = bytes([count, rec[1], rec[2], rtype]) + bytes(data)
                line = ":" + (hdr + bytes([-sum(hdr) & 0xFF])).hex().upper()
        out_lines.append(line)
    if want:
        raise RuntimeError("offsets %s missing from %s" % (sorted(hex(k) for k in want), hex_path))
    with open(hex_path, "w", newline="\n") as f:
        f.write("\n".join(out_lines) + "\n")

def _apply_bootloader_image_fixups(bin_file, hex_file):
    with open(bin_file, "rb") as f:
        data = bytearray(f.read())
    if len(data) < G5RC_OFFSET + 4:
        raise RuntimeError("image too small for G5RC magic")
    sp = int.from_bytes(data[0:4], "little")
    if (sp & 0x2FFE0000) != 0x20000000:
        raise RuntimeError("initial SP 0x%08X fails bootloader validation" % sp)
    data[G5RC_OFFSET : G5RC_OFFSET + 4] = G5RC_MAGIC
    with open(bin_file, "wb") as f:
        f.write(data)
    if os.path.exists(hex_file):
        _patch_hex(hex_file, G5RC_OFFSET, G5RC_MAGIC)
    print("G5RC magic written at image offset 0x%X (initial SP=0x%08X)" % (G5RC_OFFSET, sp))

def after_build(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    progname = env.subst("$PROGNAME")
    project_dir = env.subst("$PROJECT_DIR")
    bc = env.BoardConfig()
    dest_dir = os.path.join(project_dir, bc.get("build.firmware_outdir", "compiled_firmware"))
    os.makedirs(dest_dir, exist_ok=True)

    bin_file = os.path.join(build_dir, f"{progname}.bin")
    hex_file = os.path.join(build_dir, f"{progname}.hex")

    if bc.get("build.g5rc_magic", "0") == "1":
        if os.path.exists(bin_file):
            _apply_bootloader_image_fixups(bin_file, hex_file)

    if os.path.exists(bin_file):
        shutil.copy(bin_file, os.path.join(dest_dir, "firmware.bin"))
    if os.path.exists(hex_file):
        # Keep the tracked Intel HEX portable and free of CRLF whitespace.
        with open(hex_file, "r", newline="") as source_file:
            hex_data = source_file.read().replace("\r\n", "\n").replace("\r", "\n")
        with open(os.path.join(dest_dir, "firmware.hex"), "w", newline="\n") as dest_file:
            dest_file.write(hex_data)

    print(f"Firmware copied to {dest_dir}")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        '"$OBJCOPY" -O ihex -R .eeprom "$BUILD_DIR/${PROGNAME}.elf" "$BUILD_DIR/${PROGNAME}.hex"',
        "Building $BUILD_DIR/${PROGNAME}.hex"
    )
)

# BIN is generated after the custom HEX action, so both artifacts are ready here.
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
