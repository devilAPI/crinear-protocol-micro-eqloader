#!/usr/bin/env python3
"""
eqloader.py
==============

Python CLI for pushing/pulling parametric-EQ (PEQ) profiles to Walkplay-chipset
USB DAC/dongle devices (e.g. Crinacle "Protocol Micro" / "Protocol Max"), the
same devices driven by https://eq.hangout.audio/.

Requires:
    pip install hidapi

Usage:
    python walkplay_eq.py list
    python walkplay_eq.py pull  --slot 0 -o current.txt
    python walkplay_eq.py push  profile.txt --slot 0
    python walkplay_eq.py enable  --slot 0
    python walkplay_eq.py disable

Profile .txt format (same as eq.hangout.audio's export -- what `push` reads
and `pull` writes):
    Preamp: -6.2 dB
    Filter 1: ON PK Fc 20 Hz Gain -1.0 dB Q 2.000
    Filter 2: ON PK Fc 35 Hz Gain 3.0 dB Q 0.500
    Filter 9: OFF PK Fc 0 Hz Gain 0.0 dB Q 0.000
"""

import argparse
import re
import sys
import time

try:
    import hid
except ImportError:
    print("This script needs the 'hidapi' package: pip install hidapi", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Protocol constants (from walkplayUsbHID.js)
# ---------------------------------------------------------------------------

WALKPLAY_VENDOR_ID = 0x3302

REPORT_ID = 0x4B
READ = 0x80
WRITE = 0x01
END = 0x00

CMD = {
    "FLASH_EQ": 0x01,
    "MIC_GAIN": 0x02,
    "GLOBAL_GAIN": 0x03,
    "PEQ_VALUES": 0x09,
    "TEMP_WRITE": 0x0A,
    "VERSION": 0x0C,
    "GET_SLOT": 0x0F,
    "DAC_FILTER": 0x11,
    "DAC_BALANCE": 0x16,
    "GAIN_MODE": 0x19,
    "DENOISE": 0x1B,
    "DAC_WORK_MODE": 0x1D,
}

FILTER_TYPE_TO_BYTE = {"LSQ": 1, "PK": 2, "HSQ": 3, "LP": 4, "HP": 5}
BYTE_TO_FILTER_TYPE = {v: k for k, v in FILTER_TYPE_TO_BYTE.items()}

# Report payload length (excluding the leading report-ID byte) that we pad
# every outgoing packet up to. This device family uses at least 36-37 bytes
# (the slot byte lives at offset 35); 64 is the common HID report size for
# devices like this and matches what the browser (and OS) pads short
# WebHID reports to. If pushes silently fail to "take", try changing this.
REPORT_LENGTH = 64

DEFAULT_GLOBAL_GAIN_BUFFER = -5  # dB, confirmed for Protocol Micro (SchemeNo11) via browser console log

# Confirmed for the actual "Protocol Micro" unit from the site's console log:
#   [peqConstraints] connected: "Protocol Micro" (vendorId=0x3302 productId=0xC20F)
#   ref=walkplayPeq8Band10dBLsLowpass bands=8 gain=-10/10dB
# It's matched via deviceGroups.SchemeNo11 (productId 0xC20F), which does NOT
# override deviceHandlesPregain, so it inherits false from defaultModelConfig.
# That means the HOST writes the output-gain register on push (confirmed live:
# the log shows writeGlobalGain firing whenever preamp < -5dB buffer) — unlike
# Protocol Max (SchemeNo16), which sets deviceHandlesPregain: true and skips it.
DEFAULT_DEVICE_HANDLES_PREGAIN = False
PROTOCOL_MICRO_PRODUCT_ID = 0xC20F  # confirmed; pass --pid 0xC20F to target directly
DEFAULT_MAX_FILTERS = 8  # confirmed: 8 bands, LS + PK + Lowpass, not full LS/HS shelves


# ---------------------------------------------------------------------------
# Device discovery / low-level I/O
# ---------------------------------------------------------------------------

def list_devices():
    devices = [d for d in hid.enumerate() if d["vendor_id"] == WALKPLAY_VENDOR_ID]
    if not devices:
        print("No Walkplay-vendor (0x3302) HID devices found.")
        return
    for d in devices:
        print(
            f"vid=0x{d['vendor_id']:04X} pid=0x{d['product_id']:04X} "
            f"iface={d.get('interface_number')} usage_page=0x{d.get('usage_page', 0):04X} "
            f"usage=0x{d.get('usage', 0):02X}\n"
            f"    product : {d.get('product_string')}\n"
            f"    manuf   : {d.get('manufacturer_string')}\n"
            f"    path    : {d['path']}"
        )


def open_device(vid=None, pid=None, path=None):
    dev = hid.device()
    if path:
        dev.open_path(path)
        return dev

    vid = vid or WALKPLAY_VENDOR_ID
    if pid:
        dev.open(vid, pid)
        return dev

    candidates = [d for d in hid.enumerate() if d["vendor_id"] == vid]
    if not candidates:
        raise RuntimeError(
            f"No HID device found with vendor id 0x{vid:04X}. "
            f"Run 'list' to see connected devices, or pass --vid/--pid."
        )
    if len(candidates) > 1:
        names = ", ".join(f"0x{d['product_id']:04X} ({d.get('product_string')})" for d in candidates)
        print(
            f"Warning: multiple Walkplay devices/interfaces found ({names}). "
            f"Using the first one. Pass --pid or --path to pick a specific one.",
            file=sys.stderr,
        )
    dev.open_path(candidates[0]["path"])
    return dev


def send_report(dev, report_id, packet):
    """Mirrors sendReport() in the JS: `packet` excludes the report-id byte."""
    payload = list(packet) + [0] * max(0, REPORT_LENGTH - len(packet))
    dev.write(bytes([report_id]) + bytes(payload[:REPORT_LENGTH]))


def _read_report(dev, timeout_ms=200):
    """Read one input report and strip the leading report-ID byte, mirroring
    how WebHID's `event.data` already has the report ID stripped."""
    data = dev.read(REPORT_LENGTH + 1, timeout_ms)
    if not data:
        return None
    return data[1:]


def wait_for_response(dev, expected_cmd, timeout=2.0):
    """Poll for an input report whose stripped byte[1] == expected_cmd
    (byte[0] is READ/WRITE, matching the JS check on data[1])."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining_ms = max(1, int((deadline - time.time()) * 1000))
        data = _read_report(dev, timeout_ms=min(200, remaining_ms))
        if data is None:
            continue
        if len(data) > 1 and data[1] == expected_cmd:
            return data
    raise TimeoutError(f"Timeout waiting for response to cmd 0x{expected_cmd:02X}")


# ---------------------------------------------------------------------------
# IIR filter math (ported verbatim from computeIIRFilter/quantizer)
# ---------------------------------------------------------------------------

def _to_i32(value):
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def quantizer(d_arr, d_arr2):
    i_arr = [round(d * 1073741824) for d in d_arr]
    i_arr2 = [round(d * 1073741824) for d in d_arr2]
    return [i_arr2[0], i_arr2[1], i_arr2[2], -i_arr[1], -i_arr[2]]


def compute_iir_filter(freq, gain, q):
    import math

    sqrt = math.sqrt(10 ** (gain / 20))
    d3 = (freq * 6.283185307179586) / 96000
    sin = math.sin(d3) / (2 * q)
    d4 = sin * sqrt
    d5 = sin / sqrt
    d6 = d5 + 1

    quantizer_data = quantizer(
        [1, (math.cos(d3) * -2) / d6, (1 - d5) / d6],
        [(d4 + 1) / d6, (math.cos(d3) * -2) / d6, (1 - d4) / d6],
    )

    b_arr = [0] * 20
    index = 0
    for value in quantizer_data:
        value = _to_i32(value) & 0xFFFFFFFF
        b_arr[index] = value & 0xFF
        b_arr[index + 1] = (value >> 8) & 0xFF
        b_arr[index + 2] = (value >> 16) & 0xFF
        b_arr[index + 3] = (value >> 24) & 0xFF
        index += 4
    return b_arr


def convert_to_byte_array(value, length):
    return [(value >> (8 * i)) & 0xFF for i in range(length)]


# ---------------------------------------------------------------------------
# Core PEQ API (ported from walkplayUsbHID: getCurrentSlot / pushToDevice /
# pullFromDevice / enablePEQ)
# ---------------------------------------------------------------------------

def get_current_slot(dev):
    send_report(dev, REPORT_ID, [READ, CMD["VERSION"], END])
    resp = wait_for_response(dev, CMD["VERSION"])
    version_bytes = bytes(resp[3:6])
    try:
        version = version_bytes.decode("ascii", errors="ignore")
    except Exception:
        version = ""
    print(f"Firmware version: {version!r}")

    send_report(dev, REPORT_ID, [READ, CMD["PEQ_VALUES"], END])
    resp = wait_for_response(dev, CMD["PEQ_VALUES"])
    slot = resp[35] if len(resp) > 35 else -1
    print(f"Current EQ slot: {slot}")
    return slot


def push_to_device(dev, slot, global_gain, filters, buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER,
                    write_gain=True):
    slot = int(slot)

    for i, f in enumerate(filters):
        b_arr = compute_iir_filter(f["freq"], f["gain"], f["q"])
        packet = (
            [WRITE, CMD["PEQ_VALUES"], 0x18, 0x00, i, 0x00, 0x00]
            + b_arr
            + convert_to_byte_array(f["freq"], 2)
            + convert_to_byte_array(round(f["q"] * 256), 2)
            + convert_to_byte_array(round(f["gain"] * 256) & 0xFFFF, 2)
            + [FILTER_TYPE_TO_BYTE.get(f.get("type", "PK"), 2), 0x00, slot, END]
        )
        send_report(dev, REPORT_ID, packet)
        time.sleep(0.02)

    time.sleep(0.1)

    if write_gain:
        gain_to_write = round(min(0, global_gain - buffer_db))
        write_global_gain(dev, gain_to_write)
        print(f"Set global gain register to {gain_to_write} dB "
              f"(preamp {global_gain} dB, hardware buffer {buffer_db} dB)")
        time.sleep(0.05)

    # Commit sequence matching the site's app order.
    send_report(dev, REPORT_ID, [WRITE, 0x05, END])
    time.sleep(0.02)
    send_report(dev, REPORT_ID, [WRITE, 0x17, END])
    time.sleep(0.02)
    send_report(dev, REPORT_ID, [WRITE, CMD["TEMP_WRITE"], 0x04, 0x00, 0x00, 0xFF, 0xFF, END])
    time.sleep(0.05)
    send_report(dev, REPORT_ID, [WRITE, CMD["FLASH_EQ"], END])

    print(f"Pushed {len(filters)} filter(s) to slot {slot} and flashed to device.")


def write_global_gain(dev, value_db):
    gain_value = round(value_db) & 0xFF
    send_report(dev, REPORT_ID, [WRITE, CMD["GLOBAL_GAIN"], 0x02, 0x00, gain_value])


def read_global_gain(dev):
    send_report(dev, REPORT_ID, [READ, CMD["GLOBAL_GAIN"], 0x00])
    resp = wait_for_response(dev, CMD["GLOBAL_GAIN"], timeout=1.0)
    raw = resp[4]
    signed = raw - 256 if raw > 127 else raw
    return signed


def parse_filter_packet(packet):
    filter_index = packet[4]
    freq = packet[27] | (packet[28] << 8)

    q_raw = packet[29] | (packet[30] << 8)
    q = round((q_raw / 256) * 100) / 100

    gain_raw = packet[31] | (packet[32] << 8)
    if gain_raw > 32767:
        gain_raw -= 65536
    gain = round((gain_raw / 256) * 100) / 100

    ftype = BYTE_TO_FILTER_TYPE.get(packet[33], "PK")

    return {
        "filterIndex": filter_index,
        "freq": freq,
        "q": q,
        "gain": gain,
        "type": ftype,
        "disabled": not (freq or q or gain),
    }


def pull_from_device(dev, max_filters, slot_hint=-1, timeout=10.0):
    filters = {}
    deadline = time.time() + timeout

    for i in range(max_filters):
        send_report(dev, REPORT_ID, [READ, CMD["PEQ_VALUES"], 0x00, 0x00, i, END])
        time.sleep(0.05)
    time.sleep(0.1)

    while len(filters) < max_filters and time.time() < deadline:
        data = _read_report(dev, timeout_ms=200)
        if data is None or len(data) < 32:
            continue
        if data[1] != CMD["PEQ_VALUES"]:
            continue
        parsed = parse_filter_packet(data)
        filters[parsed["filterIndex"]] = parsed

    if len(filters) < max_filters:
        print(f"Warning: only received {len(filters)}/{max_filters} filters before timeout.",
              file=sys.stderr)

    try:
        global_gain = read_global_gain(dev)
    except TimeoutError:
        print("Warning: could not read global gain.", file=sys.stderr)
        global_gain = 0

    ordered = [filters[i] for i in sorted(filters.keys())]
    return {"currentSlot": slot_hint, "globalGain": global_gain, "filters": ordered}


def enable_peq(dev, enable, slot_id=0):
    if not enable:
        slot_id = 0x00
    send_report(dev, REPORT_ID, [WRITE, CMD["FLASH_EQ"], 1 if enable else 0, slot_id, END])


# ---------------------------------------------------------------------------
# Profile .txt (ParametricEQ / site-export format) <-> device
# ---------------------------------------------------------------------------

# .txt file uses "LS"/"HS" shelf shorthand; the device protocol (and
# FILTER_TYPE_TO_BYTE above) uses "LSQ"/"HSQ". PK/LP/HP are the same in both.
TXT_TYPE_TO_INTERNAL = {"LS": "LSQ", "HS": "HSQ", "PK": "PK", "LP": "LP", "HP": "HP"}
INTERNAL_TYPE_TO_TXT = {v: k for k, v in TXT_TYPE_TO_INTERNAL.items()}

# What an "OFF" line becomes when pushed to the device -- mirrors the site's
# own modelConfig.defaultResetFiltersValues for Walkplay devices.
INERT_FILTER = {"type": "PK", "freq": 100, "gain": 0.0, "q": 1.0}

_PREAMP_RE = re.compile(r'^\s*Preamp:\s*([+-]?[\d.]+)\s*dB', re.IGNORECASE)
_FILTER_RE = re.compile(
    r'^\s*Filter\s+\d+:\s*(ON|OFF)\s+(\S+)\s+Fc\s+([\d.]+)\s*Hz\s+'
    r'Gain\s+([+-]?[\d.]+)\s*dB\s+Q\s+([\d.]+)',
    re.IGNORECASE,
)


def load_profile(path):
    """Parse a .txt ParametricEQ profile (same format eq.hangout.audio exports)."""
    preamp = 0.0
    filters = []
    with open(path, "r") as fh:
        for line in fh:
            m = _PREAMP_RE.match(line)
            if m:
                preamp = float(m.group(1))
                continue
            m = _FILTER_RE.match(line)
            if not m:
                continue
            enabled, txt_type, freq, gain, q = m.groups()
            if enabled.upper() == "OFF":
                filters.append(dict(INERT_FILTER))
                continue
            internal_type = TXT_TYPE_TO_INTERNAL.get(txt_type.upper(), "PK")
            filters.append({
                "type": internal_type,
                "freq": float(freq),
                "gain": float(gain),
                "q": float(q),
            })

    if not filters:
        raise ValueError(f"No 'Filter N: ...' lines found in {path}")
    return {"preamp": preamp, "filters": filters}


def save_profile(path, global_gain, filters):
    """Write a .txt ParametricEQ profile in the same format the site exports."""
    lines = [f"Preamp: {global_gain:.1f} dB"]
    for i, f in enumerate(filters, start=1):
        disabled = f.get("disabled", not (f["freq"] or f["q"] or f["gain"]))
        state = "OFF" if disabled else "ON"
        txt_type = INTERNAL_TYPE_TO_TXT.get(f["type"], f["type"])
        freq = int(round(f["freq"]))
        lines.append(
            f"Filter {i}: {state} {txt_type} Fc {freq} Hz "
            f"Gain {f['gain']:.1f} dB Q {f['q']:.3f}"
        )
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=None, help="USB vendor ID override (hex ok, e.g. 0x3302)")
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=None,
                         help="USB product ID override. Confirmed Protocol Micro pid is 0xC20F "
                              "(pass --pid 0xC20F to skip auto-detection if you have multiple Walkplay devices)")
    parser.add_argument("--path", default=None, help="Exact hidapi device path (from 'list'), overrides --vid/--pid")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="List connected Walkplay-vendor HID devices")

    p_pull = sub.add_parser("pull", help="Read the current PEQ profile off the device")
    p_pull.add_argument("-o", "--output", default="pulled_profile.txt")
    p_pull.add_argument("--max-filters", type=int, default=DEFAULT_MAX_FILTERS,
                         help="Number of PEQ bands the device has (default 8, confirmed for Protocol Micro)")

    p_push = sub.add_parser("push", help="Write a PEQ profile .txt file to the device")
    p_push.add_argument("profile", help="Path to profile .txt file (ParametricEQ format)")
    p_push.add_argument("--slot", type=int, default=0)
    p_push.add_argument("--buffer", type=float, default=DEFAULT_GLOBAL_GAIN_BUFFER,
                         help="Fixed hardware gain buffer in dB (default -5, confirmed for Protocol Micro)")
    p_push.add_argument("--no-write-gain", action="store_true",
                         help="Skip writing the output/global gain register. ON by default is to WRITE it "
                              "(confirmed correct for Protocol Micro, deviceHandlesPregain=false); pass this "
                              "flag only if you've confirmed your unit handles pregain internally.")

    p_enable = sub.add_parser("enable", help="Enable PEQ on a given slot")
    p_enable.add_argument("--slot", type=int, default=0)

    sub.add_parser("disable", help="Disable PEQ")

    p_slot = sub.add_parser("slot", help="Print the device's current active slot + firmware version")

    args = parser.parse_args()

    if args.command == "list":
        list_devices()
        return

    dev = open_device(vid=args.vid, pid=args.pid, path=args.path)
    try:
        if args.command == "slot":
            get_current_slot(dev)

        elif args.command == "pull":
            slot = get_current_slot(dev)
            result = pull_from_device(dev, max_filters=args.max_filters, slot_hint=slot)
            save_profile(args.output, result["globalGain"], result["filters"])
            print(f"Saved {len(result['filters'])} filter(s) to {args.output}")

        elif args.command == "push":
            profile = load_profile(args.profile)
            slot = args.slot
            push_to_device(
                dev, slot, profile["preamp"], profile["filters"],
                buffer_db=args.buffer, write_gain=not args.no_write_gain,
            )
            enable_peq(dev, True, slot_id=slot)

        elif args.command == "enable":
            enable_peq(dev, True, slot_id=args.slot)
            print(f"PEQ enabled on slot {args.slot}")

        elif args.command == "disable":
            enable_peq(dev, False)
            print("PEQ disabled")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
