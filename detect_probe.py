Import("env")
import os
import sys
import re
import subprocess

def detect_connected_probes():
    probes = []
    
    # 1. Check serial ports (fast & cross-platform for DAPLink and ST-Link V2.1)
    try:
        import serial.tools.list_ports as lp
        for p in lp.comports():
            vid = p.vid
            pid = p.pid
            desc = ((p.description or "") + " " + (p.product or "")).lower()
            if vid == 0x0D28 or "daplink" in desc or "cmsis-dap" in desc:
                if not any(x[0] == "cmsis-dap" for x in probes):
                    probes.append(("cmsis-dap", p.device, f"DAPLink (CDC {p.device})"))
            elif vid == 0x0483 and pid in (0x374B, 0x374F, 0x3752, 0x3753, 0x3754, 0x3757):
                if not any(x[0] == "stlink" for x in probes):
                    probes.append(("stlink", p.device, f"ST-Link V2.1/V3 (CDC {p.device})"))
    except Exception:
        pass

    # 2. Check OS-specific USB device registry (covers ST-Link V2 dongles without CDC port)
    if sys.platform == "darwin":
        try:
            out = subprocess.check_output(["ioreg", "-r", "-c", "IOUSBHostDevice", "-l"], timeout=2).decode("utf-8", "ignore")
            blocks = out.split("+-o ")
            for b in blocks:
                if "idVendor" in b:
                    vid_m = re.search(r'\"idVendor\"\s*=\s*(\d+)', b)
                    pid_m = re.search(r'\"idProduct\"\s*=\s*(\d+)', b)
                    name_m = re.search(r'\"USB Product Name\"\s*=\s*\"([^\"]+)\"', b)
                    if vid_m and pid_m:
                        v = int(vid_m.group(1))
                        p = int(pid_m.group(1))
                        name = name_m.group(1) if name_m else ""
                        if v == 0x0483 and p in (0x3748, 0x374B, 0x374E, 0x374F, 0x3752, 0x3753, 0x3754, 0x3755, 0x3757):
                            if not any(x[0] == "stlink" for x in probes):
                                probes.append(("stlink", None, f"ST-Link V2 (USB {hex(v)}:{hex(p)} {name})"))
                        elif (v == 0x0D28 or "dap" in name.lower()) and not any(x[0] == "cmsis-dap" for x in probes):
                            probes.append(("cmsis-dap", None, f"DAPLink (USB {hex(v)}:{hex(p)} {name})"))
        except Exception:
            pass
    elif sys.platform.startswith("linux"):
        try:
            sys_usb = "/sys/bus/usb/devices"
            if os.path.exists(sys_usb):
                for d in os.listdir(sys_usb):
                    p_vid = os.path.join(sys_usb, d, "idVendor")
                    p_pid = os.path.join(sys_usb, d, "idProduct")
                    if os.path.exists(p_vid) and os.path.exists(p_pid):
                        with open(p_vid) as fv, open(p_pid) as fp:
                            v = int(fv.read().strip(), 16)
                            p = int(fp.read().strip(), 16)
                            if v == 0x0483 and p in (0x3748, 0x374B, 0x374E, 0x374F, 0x3752, 0x3753, 0x3754, 0x3755, 0x3757):
                                if not any(x[0] == "stlink" for x in probes):
                                    probes.append(("stlink", None, f"ST-Link V2 (Linux USB {hex(v)}:{hex(p)})"))
                            elif v == 0x0D28 and not any(x[0] == "cmsis-dap" for x in probes):
                                probes.append(("cmsis-dap", None, f"DAPLink (Linux USB {hex(v)}:{hex(p)})"))
        except Exception:
            pass
    elif sys.platform == "win32":
        try:
            cmd = 'powershell -NoProfile -Command "Get-PnpDevice -PresentOnly | Select-Object -ExpandProperty InstanceId"'
            out = subprocess.check_output(cmd, shell=True, timeout=2).decode("utf-8", "ignore")
            if "VID_0D28" in out.upper():
                if not any(x[0] == "cmsis-dap" for x in probes):
                    probes.append(("cmsis-dap", None, "DAPLink (Windows PNP)"))
            if "VID_0483&PID_374" in out.upper() or "VID_0483&PID_375" in out.upper():
                if not any(x[0] == "stlink" for x in probes):
                    probes.append(("stlink", None, "ST-Link (Windows PNP)"))
        except Exception:
            pass

    return probes

probes = detect_connected_probes()
board_config = env.BoardConfig()

if probes:
    # If DAPLink is connected, use cmsis-dap
    # If only ST-Link is connected, use stlink
    selected_proto, cdc_port, desc = probes[0]
    print(f"\n[PROBE AUTO-DETECT] \033[92mFound hardware probe:\033[0m {desc} -> Setting upload_protocol = \033[96m{selected_proto}\033[0m\n")
    env.Replace(UPLOAD_PROTOCOL=selected_proto)
    board_config.update("upload.protocol", selected_proto)
    board_config.update("debug.tool", selected_proto)
    if cdc_port:
        board_config.update("upload.port", cdc_port)
else:
    current = env.subst("$UPLOAD_PROTOCOL") or "stlink"
    print(f"\n[PROBE AUTO-DETECT] \033[93mNo active hardware probe detected.\033[0m Defaulting to: \033[96m{current}\033[0m\n")
