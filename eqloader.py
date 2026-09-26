#!/usr/bin/env python3

import copy
import json
import math
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
import urllib.parse
import urllib.request
from pathlib import Path
from tkinter import ttk, filedialog, messagebox, scrolledtext

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import ticker
import numpy as np

try:
    import hid
except ImportError:
    hid = None


# ===========================================================================
# Theme: "instrument panel" (graphite chassis, two-LED accent)
# ===========================================================================

THEME = {
    "chassis":   "#14161A",  # deep graphite base
    "panel":     "#1C1F26",  # raised frame / surface
    "input":     "#262A33",  # entries, hover
    "line":      "#2E333D",  # hairline borders / grid
    "ink":       "#E6E9EF",  # primary text
    "muted":     "#8A93A3",  # secondary labels
    "accent":    "#4ED0C4",  # cyan signal / idle trace
    "accent_dk": "#2C8F87",  # pressed / darker cyan
    "active":    "#F0A93B",  # amber, selected band
    "danger":    "#E5687A",  # destructive action
}

MONO_FONTS = ("JetBrains Mono", "DejaVu Sans Mono", "Consolas", "Menlo",
              "Courier New", "monospace")
UI_FONTS = ("Inter", "Segoe UI", "Helvetica Neue", "DejaVu Sans", "sans-serif")


def _pick_font(root, families):
    """Return the first installed font family, else the last fallback."""
    try:
        import tkinter.font as tkfont
        available = {f.lower() for f in tkfont.families(root)}
        for fam in families:
            if fam.lower() in available:
                return fam
    except Exception:
        pass
    return families[-1]


# ===========================================================================
# Core protocol / device I/O
# ===========================================================================

WALKPLAY_VENDOR_ID = 0x3302

REPORT_ID = 0x4B
READ = 0x80
WRITE = 0x01
END = 0x00
REPORT_LENGTH = 64

CMD = {
    "FLASH_EQ": 0x01,
    "GLOBAL_GAIN": 0x03,
    "PEQ_VALUES": 0x09,
    "TEMP_WRITE": 0x0A,
    "VERSION": 0x0C,
}

FILTER_TYPE_TO_BYTE = {"LSQ": 1, "PK": 2, "HSQ": 3, "LP": 4, "HP": 5}
BYTE_TO_FILTER_TYPE = {v: k for k, v in FILTER_TYPE_TO_BYTE.items()}

DEFAULT_GLOBAL_GAIN_BUFFER = -5
DEFAULT_MAX_FILTERS = 8


def list_devices():
    devices = [d for d in hid.enumerate() if d["vendor_id"] == WALKPLAY_VENDOR_ID]
    if not devices:
        print("No Walkplay-vendor (0x3302) HID devices found.")
        return
    for d in devices:
        print(
            f"vid=0x{d['vendor_id']:04X} pid=0x{d['product_id']:04X} "
            f"iface={d.get('interface_number')} "
            f"usage_page=0x{d.get('usage_page', 0):04X} "
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
            f"Refresh the device list, or set VID/PID manually."
        )
    if len(candidates) > 1:
        names = ", ".join(
            f"0x{d['product_id']:04X} ({d.get('product_string')})"
            for d in candidates
        )
        print(
            f"Warning: multiple Walkplay devices/interfaces found ({names}). "
            f"Using the first one. Select a specific device in the list, or set PID."
        )
    dev.open_path(candidates[0]["path"])
    return dev


def send_report(dev, report_id, packet):
    payload = list(packet) + [0] * max(0, REPORT_LENGTH - len(packet))
    dev.write(bytes([report_id]) + bytes(payload[:REPORT_LENGTH]))


def _read_report(dev, timeout_ms=200):
    data = dev.read(REPORT_LENGTH + 1, timeout_ms)
    return data[1:] if data else None


def wait_for_response(dev, expected_cmd, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining_ms = max(1, int((deadline - time.time()) * 1000))
        data = _read_report(dev, timeout_ms=min(200, remaining_ms))
        if data is None:
            continue
        if len(data) > 1 and data[1] == expected_cmd:
            return data
    raise TimeoutError(f"Timeout waiting for response to cmd 0x{expected_cmd:02X}")


def _to_i32(value):
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def quantizer(d_arr, d_arr2):
    i_arr = [round(d * 1073741824) for d in d_arr]
    i_arr2 = [round(d * 1073741824) for d in d_arr2]
    return [i_arr2[0], i_arr2[1], i_arr2[2], -i_arr[1], -i_arr[2]]


def compute_iir_filter(freq, gain, q):
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
    value = int(round(value))
    return [(value >> (8 * i)) & 0xFF for i in range(length)]


def get_current_slot(dev):
    send_report(dev, REPORT_ID, [READ, CMD["VERSION"], END])
    resp = wait_for_response(dev, CMD["VERSION"])
    version = bytes(resp[3:6]).decode("ascii", errors="ignore")
    print(f"Firmware version: {version!r}")

    send_report(dev, REPORT_ID, [READ, CMD["PEQ_VALUES"], END])
    resp = wait_for_response(dev, CMD["PEQ_VALUES"])
    slot = resp[35] if len(resp) > 35 else -1
    print(f"Current EQ slot: {slot}")
    return slot


def push_to_device(dev, slot, global_gain, filters,
                   buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER, write_gain=True):
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
        print(
            f"Set global gain register to {gain_to_write} dB "
            f"(preamp {global_gain} dB, hardware buffer {buffer_db} dB)"
        )
        time.sleep(0.05)

    send_report(dev, REPORT_ID, [WRITE, 0x05, END])
    time.sleep(0.02)
    send_report(dev, REPORT_ID, [WRITE, 0x17, END])
    time.sleep(0.02)
    send_report(dev, REPORT_ID,
                [WRITE, CMD["TEMP_WRITE"], 0x04, 0x00, 0x00, 0xFF, 0xFF, END])
    time.sleep(0.05)
    send_report(dev, REPORT_ID, [WRITE, CMD["FLASH_EQ"], END])
    print(f"Pushed {len(filters)} filter(s) to slot {slot} and flashed to device.")


def write_global_gain(dev, value_db):
    gain_value = round(value_db) & 0xFF
    send_report(dev, REPORT_ID,
                [WRITE, CMD["GLOBAL_GAIN"], 0x02, 0x00, gain_value])


def read_global_gain(dev):
    send_report(dev, REPORT_ID, [READ, CMD["GLOBAL_GAIN"], 0x00])
    resp = wait_for_response(dev, CMD["GLOBAL_GAIN"], timeout=1.0)
    raw = resp[4]
    return raw - 256 if raw > 127 else raw


def is_filter_disabled(ftype, freq, gain, q):
    """Return True when the device treats this band as OFF.

    The device stores an off band as an inert flat filter (PK, Fc 100, Gain 0,
    Q 1), so a peaking/shelf band with zero gain is audibly inert and must be
    treated as off. LP/HP filters shape the signal regardless of gain, so only
    a fully-zero slot counts.
    """
    if ftype in ("PK", "LSQ", "HSQ"):
        return gain == 0
    return not (freq or q or gain)


def dedupe_filters(filters):
    """Drop exact-duplicate bands, keeping first occurrence.

    The device always stores a fixed number of slots (typically 8); pushing
    fewer bands leaves it padding the tail by repeating the last band(s), so a
    pulled profile can contain identical copies. Two bands with the same type,
    frequency, gain and Q are audibly one band, so we keep only the first.
    """
    seen = set()
    result = []
    for f in filters:
        key = (f.get("type", "PK"),
               round(float(f["freq"]), 2),
               round(float(f["gain"]), 2),
               round(float(f["q"]), 3))
        if key in seen:
            continue
        seen.add(key)
        result.append(f)
    return result


def parse_filter_packet(packet):
    freq = packet[27] | (packet[28] << 8)
    q_raw = packet[29] | (packet[30] << 8)
    q = round((q_raw / 256) * 100) / 100

    gain_raw = packet[31] | (packet[32] << 8)
    if gain_raw > 32767:
        gain_raw -= 65536
    gain = round((gain_raw / 256) * 100) / 100

    ftype = BYTE_TO_FILTER_TYPE.get(packet[33], "PK")
    return {
        "filterIndex": packet[4],
        "freq": freq,
        "q": q,
        "gain": gain,
        "type": ftype,
        "disabled": is_filter_disabled(ftype, freq, gain, q),
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
        if data is None or len(data) < 34 or data[1] != CMD["PEQ_VALUES"]:
            continue
        parsed = parse_filter_packet(data)
        filters[parsed["filterIndex"]] = parsed

    if len(filters) < max_filters:
        print(f"Warning: only received {len(filters)}/{max_filters} filters before timeout.")

    try:
        global_gain = read_global_gain(dev)
    except TimeoutError:
        print("Warning: could not read global gain.")
        global_gain = 0

    ordered = [filters[i] for i in sorted(filters.keys())]
    return {"currentSlot": slot_hint, "globalGain": global_gain, "filters": ordered}


def enable_peq(dev, enable, slot_id=0):
    if not enable:
        slot_id = 0x00
    send_report(dev, REPORT_ID,
                [WRITE, CMD["FLASH_EQ"], 1 if enable else 0, slot_id, END])


# ===========================================================================
# Profile .txt format
# ===========================================================================

TXT_TYPE_TO_INTERNAL = {"LS": "LSQ", "HS": "HSQ", "PK": "PK", "LP": "LP", "HP": "HP"}
INTERNAL_TYPE_TO_TXT = {v: k for k, v in TXT_TYPE_TO_INTERNAL.items()}

INERT_FILTER = {"type": "PK", "freq": 100.0, "gain": 0.0, "q": 1.0}

_PREAMP_RE = re.compile(r'^\s*Preamp:\s*([+-]?[\d.,]+)\s*dB', re.IGNORECASE)
_FILTER_RE = re.compile(
    r'^\s*Filter\s+\d+:\s*'
    r'(ON|OFF)\s+'
    r'(\S+)\s+'
    r'Fc\s+([\d.,]+)\s*Hz\s+'
    r'Gain\s+([+-]?[\d.,]+)\s*dB\s+'
    r'Q\s+([\d.,]+)',
    re.IGNORECASE,
)


def _to_float(text):
    return float(text.strip().replace(",", "."))


def fmt_num(value, decimals):
    return f"{value:.{decimals}f}".replace(".", ",")


def load_profile(path):
    preamp = 0.0
    filters = []

    with open(path, "r") as fh:
        for line in fh:
            m = _PREAMP_RE.match(line)
            if m:
                preamp = _to_float(m.group(1))
                continue

            m = _FILTER_RE.match(line)
            if not m:
                continue

            enabled, txt_type, freq, gain, q = m.groups()
            if enabled.upper() == "OFF":
                f = dict(INERT_FILTER)
                f["disabled"] = True
                filters.append(f)
                continue

            filters.append({
                "type": TXT_TYPE_TO_INTERNAL.get(txt_type.upper(), "PK"),
                "freq": _to_float(freq),
                "gain": _to_float(gain),
                "q": _to_float(q),
            })

    if not filters:
        raise ValueError(f"No 'Filter N: ...' lines found in {path}")

    return {"preamp": preamp, "filters": filters}


def save_profile(path, global_gain, filters):
    lines = [f"Preamp: {fmt_num(float(global_gain), 1)} dB"]

    for i, f in enumerate(filters, start=1):
        disabled = f.get("disabled", is_filter_disabled(
            f.get("type", "PK"), f["freq"], f["gain"], f["q"]))
        state = "OFF" if disabled else "ON"
        txt_type = INTERNAL_TYPE_TO_TXT.get(f["type"], f["type"])
        lines.append(
            f"Filter {i}: {state} {txt_type} "
            f"Fc {fmt_num(float(f['freq']), 1)} Hz "
            f"Gain {fmt_num(float(f['gain']), 1)} dB "
            f"Q {fmt_num(float(f['q']), 3)}"
        )

    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# ===========================================================================
# AutoEQ (port of the autoeq.app biquad.py / biquad-coeffs-cookbook logic)
# ===========================================================================

AUTOEQ_CONFIG = {
    "default_sample_rate": 48000,
    "treble_start_from": 7000,
    "autoeq_range": (20, 15000),
    "optimize_q_range": (0.5, 2),
    "optimize_gain_range": (-12, 12),
    "optimize_deltas": (
        (10, 10, 10, 5, 0.1, 0.5),
        (10, 10, 10, 2, 0.1, 0.2),
        (10, 10, 10, 1, 0.1, 0.1),
    ),
}


def autoeq_raw_frequencies():
    """~1/96 octave grid from 20 Hz to 20 kHz, used for the optimizer itself."""
    n = math.ceil(math.log(20000 / 20) / math.log(1.0072))
    return [20 * (1.0072 ** i) for i in range(n)]


def parse_frequency_response_file(path):
    """Load a two-column (freq, gain) measurement/target text file.

    Accepts whitespace- or comma-separated columns and ignores blank lines
    and comment lines (starting with '#', '*' or ';'), which covers common
    exports such as REW's frequency-response .txt files.
    """
    points = []
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith(("#", "*", ";")):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                continue
            try:
                freq = float(parts[0])
                gain = float(parts[1])
            except ValueError:
                continue
            points.append((freq, gain))

    if not points:
        raise ValueError(f"No frequency/gain data found in {path}")
    points.sort(key=lambda p: p[0])
    return points


# ---------------------------------------------------------------------------
# AutoEQ model database (search-by-model, à la autoeq.app)
# ---------------------------------------------------------------------------

AUTOEQ_DB_CONFIG_PATH = os.path.expanduser("~/.config/eqloader/autoeq_db.json")
AUTOEQ_LOCAL_CACHE_DIR = os.path.expanduser("~/.cache/eqloader/autoeq_db")

# Public measurement database backing the autoeq.app website. Raw, per-model
# measurements live under measurements/<source>/data/<category>/<model>.csv
# as plain "frequency,raw" CSVs — much cleaner to index than results/, which
# also holds generated EQ outputs, images and impulse-response .wav files.
AUTOEQ_GITHUB_REPO = "jaakkopasanen/AutoEq"
AUTOEQ_GITHUB_BRANCH = "master"
AUTOEQ_GITHUB_API_BASE = f"https://api.github.com/repos/{AUTOEQ_GITHUB_REPO}"
AUTOEQ_GITHUB_RAW_BASE = (
    f"https://raw.githubusercontent.com/{AUTOEQ_GITHUB_REPO}/{AUTOEQ_GITHUB_BRANCH}/")
AUTOEQ_MEASUREMENTS_DIR = "measurements"
AUTOEQ_TARGETS_DIR = "targets"

# Result files, not raw measurements — skip these when indexing a local folder
# (a local clone may point at results/ instead of measurements/).
_AUTOEQ_SKIP_SUFFIXES = ("parametriceq", "graphiceq", "fixedbandeq", " eq")


def load_autoeq_db_path():
    """Return the last-used measurement source ('online' or a folder path)."""
    try:
        with open(AUTOEQ_DB_CONFIG_PATH, "r") as fh:
            path = json.load(fh).get("path")
        if path == "online" or (path and os.path.isdir(path)):
            return path
    except Exception:
        pass
    return None


def save_autoeq_db_path(path):
    try:
        os.makedirs(os.path.dirname(AUTOEQ_DB_CONFIG_PATH), exist_ok=True)
        with open(AUTOEQ_DB_CONFIG_PATH, "w") as fh:
            json.dump({"path": path}, fh)
    except Exception:
        pass


def build_autoeq_model_index(root):
    """Recursively index .txt measurement files under `root` by model name.

    Works with a local clone of the AutoEQ 'results' database (or any folder
    of raw frequency-response .txt files), so brand/model subfolders don't
    matter — only the filename (as the model label) and its containing
    folder (shown as a disambiguating subtitle) are used.
    """
    root = Path(root)
    index = []
    for path in root.rglob("*.txt"):
        if path.stem.lower().endswith(_AUTOEQ_SKIP_SUFFIXES):
            continue
        index.append({
            "label": path.stem,
            "path": str(path),
            "subtitle": str(path.relative_to(root).parent),
            "remote": False,
        })
    index.sort(key=lambda e: e["label"].lower())
    return index


def _autoeq_github_get(url):
    req = urllib.request.Request(
        url, headers={"Accept": "application/vnd.github+json", "User-Agent": "eqloader"})
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.load(resp)


def _autoeq_github_subtree(dir_name):
    """Fetch just one top-level folder's subtree (by its own tree sha) rather
    than the whole repo's recursive tree, which is large enough to get
    truncated by GitHub's API before reaching every file."""
    root = _autoeq_github_get(f"{AUTOEQ_GITHUB_API_BASE}/git/trees/{AUTOEQ_GITHUB_BRANCH}")
    dir_entry = next(
        (e for e in root.get("tree", [])
         if e.get("path") == dir_name and e.get("type") == "tree"),
        None)
    if dir_entry is None:
        raise RuntimeError(f"'{dir_name}' folder not found in repo")

    sub = _autoeq_github_get(
        f"{AUTOEQ_GITHUB_API_BASE}/git/trees/{dir_entry['sha']}?recursive=1")
    if sub.get("truncated"):
        print(f"Warning: GitHub '{dir_name}' listing was truncated; some entries may be missing.")
    return sub.get("tree", [])


def fetch_autoeq_online_index():
    """Query the AutoEQ GitHub repo for its list of raw measurement files.

    Only lists file names/paths (two small API calls); the actual
    measurement content is downloaded lazily, on selection.
    """
    tree = _autoeq_github_subtree(AUTOEQ_MEASUREMENTS_DIR)
    index = []
    for entry in tree:
        path = entry.get("path", "")
        if entry.get("type") != "blob" or "/data/" not in path or not path.endswith(".csv"):
            continue
        repo_path = f"{AUTOEQ_MEASUREMENTS_DIR}/{path}"
        index.append({
            "label": Path(path).stem,
            "path": repo_path,  # repo-relative path; used as both remote key and cache key
            "subtitle": str(Path(path).parent),
            "remote": True,
        })
    index.sort(key=lambda e: e["label"].lower())
    return index


def fetch_autoeq_targets_index():
    """Query the AutoEQ GitHub repo for its list of named target curves
    (Harman, diffuse-field, etc.) under targets/ — the same target library
    hangout.audio's AutoEQ tool picks from, though not necessarily the exact
    same tilt/adjustment it applies on top.
    """
    tree = _autoeq_github_subtree(AUTOEQ_TARGETS_DIR)
    index = []
    for entry in tree:
        path = entry.get("path", "")
        if entry.get("type") != "blob" or not path.endswith(".csv"):
            continue
        repo_path = f"{AUTOEQ_TARGETS_DIR}/{path}"
        index.append({
            "label": Path(path).stem,
            "path": repo_path,
            "subtitle": "",
            "remote": True,
        })
    index.sort(key=lambda e: e["label"].lower())
    return index


def fetch_autoeq_remote_file(repo_path):
    """Download (and locally cache) one measurement file from the AutoEQ repo."""
    cache_path = Path(AUTOEQ_LOCAL_CACHE_DIR) / repo_path
    if cache_path.is_file():
        return str(cache_path)

    url = AUTOEQ_GITHUB_RAW_BASE + urllib.parse.quote(repo_path)
    req = urllib.request.Request(url, headers={"User-Agent": "eqloader"})
    with urllib.request.urlopen(req, timeout=20) as resp:
        content = resp.read()

    cache_path.parent.mkdir(parents=True, exist_ok=True)
    cache_path.write_bytes(content)
    return str(cache_path)


def autoeq_interp(fv, fr):
    """Interpolate values at fv (ascending) from breakpoints fr (ascending).

    Ported as-is from the JS `interp`: the scan index is shared across the
    whole fv pass rather than reset per point, which relies on both fv and
    fr being sorted ascending.
    """
    i = 0
    n = len(fr)
    out = []
    for f in fv:
        found = False
        while i < n - 1:
            f0, v0 = fr[i]
            f1, v1 = fr[i + 1]
            if i == 0 and f < f0:
                out.append((f, v0))
                found = True
                break
            elif f0 <= f < f1:
                v = v0 + (v1 - v0) * (f - f0) / (f1 - f0)
                out.append((f, v))
                found = True
                break
            else:
                i += 1
        if not found:
            out.append((f, fr[-1][1]))
    return out


def _autoeq_lowshelf(freq, q, gain, sample_rate=None):
    sample_rate = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    freq = max(1e-6, min(freq / sample_rate, 1))
    q = max(1e-4, min(q, 1000))
    gain = max(-40, min(gain, 40))

    w0 = 2 * math.pi * freq
    sin, cos = math.sin(w0), math.cos(w0)
    a = 10 ** (gain / 40)
    alpha = sin / (2 * q)
    alphamod = 2 * math.sqrt(a) * alpha

    a0 = (a + 1) + (a - 1) * cos + alphamod
    a1 = -2 * ((a - 1) + (a + 1) * cos)
    a2 = (a + 1) + (a - 1) * cos - alphamod
    b0 = a * ((a + 1) - (a - 1) * cos + alphamod)
    b1 = 2 * a * ((a - 1) - (a + 1) * cos)
    b2 = a * ((a + 1) - (a - 1) * cos - alphamod)
    return (1.0, a1 / a0, a2 / a0, b0 / a0, b1 / a0, b2 / a0)


def _autoeq_highshelf(freq, q, gain, sample_rate=None):
    sample_rate = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    freq = max(1e-6, min(freq / sample_rate, 1))
    q = max(1e-4, min(q, 1000))
    gain = max(-40, min(gain, 40))

    w0 = 2 * math.pi * freq
    sin, cos = math.sin(w0), math.cos(w0)
    a = 10 ** (gain / 40)
    alpha = sin / (2 * q)
    alphamod = 2 * math.sqrt(a) * alpha

    a0 = (a + 1) - (a - 1) * cos + alphamod
    a1 = 2 * ((a - 1) - (a + 1) * cos)
    a2 = (a + 1) - (a - 1) * cos - alphamod
    b0 = a * ((a + 1) + (a - 1) * cos + alphamod)
    b1 = -2 * a * ((a - 1) + (a + 1) * cos)
    b2 = a * ((a + 1) + (a - 1) * cos - alphamod)
    return (1.0, a1 / a0, a2 / a0, b0 / a0, b1 / a0, b2 / a0)


def _autoeq_peaking(freq, q, gain, sample_rate=None):
    sample_rate = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    freq = max(1e-6, min(freq / sample_rate, 1))
    q = max(1e-4, min(q, 1000))
    gain = max(-40, min(gain, 40))

    w0 = 2 * math.pi * freq
    sin, cos = math.sin(w0), math.cos(w0)
    a = 10 ** (gain / 40)
    alpha = sin / (2 * q)

    a0 = 1 + alpha / a
    a1 = -2 * cos
    a2 = 1 - alpha / a
    b0 = 1 + alpha * a
    b1 = -2 * cos
    b2 = 1 - alpha * a
    return (1.0, a1 / a0, a2 / a0, b0 / a0, b1 / a0, b2 / a0)


def autoeq_filters_to_coeffs(filters, sample_rate=None):
    coeffs = []
    for f in filters:
        if not f.get("freq") or not f.get("gain") or not f.get("q"):
            continue
        ftype = f.get("type")
        if ftype == "LSQ":
            coeffs.append(_autoeq_lowshelf(f["freq"], f["q"], f["gain"], sample_rate))
        elif ftype == "HSQ":
            coeffs.append(_autoeq_highshelf(f["freq"], f["q"], f["gain"], sample_rate))
        elif ftype == "PK":
            coeffs.append(_autoeq_peaking(f["freq"], f["q"], f["gain"], sample_rate))
    return coeffs


def autoeq_calc_gains(freqs, coeffs, sample_rate=None):
    """Vectorized port of `calc_gains`; freqs is a 1-D numpy array."""
    sample_rate = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    gains = np.zeros(len(freqs))
    if not coeffs:
        return gains

    w = 2 * np.pi * freqs / sample_rate
    phi = 4 * np.sin(w / 2) ** 2
    for a0, a1, a2, b0, b1, b2 in coeffs:
        num = (b0 + b1 + b2) ** 2 + (b0 * b2 * phi - (b1 * (b0 + b2) + 4 * b0 * b2)) * phi
        den = (a0 + a1 + a2) ** 2 + (a0 * a2 * phi - (a1 * (a0 + a2) + 4 * a0 * a2)) * phi
        gains += 10 * np.log10(np.maximum(num, 1e-12)) - 10 * np.log10(np.maximum(den, 1e-12))
    return gains


def autoeq_apply(fr, filters, sample_rate=None):
    """fr: list of (freq, dB). Returns a new list of (freq, dB) with filters applied."""
    freqs = np.array([f for f, _ in fr], dtype=float)
    values = np.array([v for _, v in fr], dtype=float)
    coeffs = autoeq_filters_to_coeffs(filters, sample_rate)
    values = values + autoeq_calc_gains(freqs, coeffs, sample_rate)
    return list(zip((f for f, _ in fr), values.tolist()))


def autoeq_calc_preamp(fr1, fr2):
    return -max(v2 - v1 for (_, v1), (_, v2) in zip(fr1, fr2))


def autoeq_calc_distance(fr1, fr2):
    v1 = np.array([v for _, v in fr1])
    v2 = np.array([v for _, v in fr2])
    d = np.abs(v1 - v2)
    return float(np.mean(np.where(d >= 0.1, d, 0.0)))


def autoeq_freq_unit(freq):
    if freq < 100:
        return 1
    elif freq < 1000:
        return 10
    elif freq < 10000:
        return 100
    return 1000


def autoeq_strip(filters):
    min_q, max_q = AUTOEQ_CONFIG["optimize_q_range"]
    min_gain, max_gain = AUTOEQ_CONFIG["optimize_gain_range"]
    result = []
    for f in filters:
        unit = autoeq_freq_unit(f["freq"])
        result.append({
            "type": f["type"],
            "freq": math.floor(f["freq"] - f["freq"] % unit),
            "q": min(max(math.floor(f["q"] * 10) / 10, min_q), max_q),
            "gain": min(max(math.floor(f["gain"] * 10) / 10, min_gain), max_gain),
        })
    return result


def autoeq_search_candidates(fr, fr_target, threshold):
    state = 0  # 1: peak, 0: matched, -1: dip
    start_index = -1
    candidates = []
    min_freq, max_freq = AUTOEQ_CONFIG["autoeq_range"]

    for i, (f, v0) in enumerate(fr):
        v1 = fr_target[i][1]
        delta = v0 - v1
        delta_abs = abs(delta)
        next_state = 0 if delta_abs < threshold else (1 if delta > 0 else -1)
        if next_state == state:
            continue

        if start_index >= 0:
            if state != 0:
                start = fr[start_index][0]
                end = f
                center = math.sqrt(start * end)
                gain = (
                    autoeq_interp([center], fr_target[start_index:i + 1])[0][1] -
                    autoeq_interp([center], fr[start_index:i + 1])[0][1]
                )
                q = center / (end - start)
                if min_freq <= center <= max_freq:
                    candidates.append({"type": "PK", "freq": center, "q": q, "gain": gain})
            start_index = -1
        else:
            start_index = i
        state = next_state

    return candidates


def autoeq_optimize(fr, fr_target, filters, iteration, dir_=False):
    filters = autoeq_strip(filters)
    min_freq, max_freq = AUTOEQ_CONFIG["autoeq_range"]
    min_q, max_q = AUTOEQ_CONFIG["optimize_q_range"]
    min_gain, max_gain = AUTOEQ_CONFIG["optimize_gain_range"]
    max_df, max_dq, max_dg, step_df, step_dq, step_dg = (
        AUTOEQ_CONFIG["optimize_deltas"][iteration])

    indices = range(len(filters) - 1, -1, -1) if dir_ else range(len(filters))

    for i in indices:
        f = filters[i]
        fr1 = autoeq_apply(fr, [ff for fi, ff in enumerate(filters) if fi != i])
        fr2 = autoeq_apply(fr1, [f])
        best_filter = dict(f)
        best_distance = autoeq_calc_distance(fr2, fr_target)

        def test_new_filter(df, dq, dg):
            nonlocal best_filter, best_distance
            freq = f["freq"] + df * autoeq_freq_unit(f["freq"]) * step_df
            q = f["q"] + dq * step_dq
            gain = f["gain"] + dg * step_dg
            if (freq < min_freq or freq > max_freq or q < min_q or q > max_q
                    or gain < min_gain or gain > max_gain):
                return False
            new_filter = {"type": f["type"], "freq": freq, "q": q, "gain": gain}
            new_distance = autoeq_calc_distance(
                autoeq_apply(fr1, [new_filter]), fr_target)
            if new_distance < best_distance:
                best_filter = new_filter
                best_distance = new_distance
                return True
            return False

        for df in range(-max_df, max_df):
            for dq in range(max_dq - 1, -max_dq - 1, -1):
                for dg in range(1, max_dg):
                    if not test_new_filter(df, dq, dg):
                        break
                for dg in range(-1, -max_dg - 1, -1):
                    if not test_new_filter(df, dq, dg):
                        break

        filters[i] = best_filter

    if not dir_:
        return autoeq_optimize(fr, fr_target, filters, iteration, True)

    filters = sorted(filters, key=lambda x: x["freq"])

    # Merge close filters.
    i = 0
    while i < len(filters) - 1:
        f1, f2 = filters[i], filters[i + 1]
        if (abs(f1["freq"] - f2["freq"]) <= autoeq_freq_unit(f1["freq"])
                and abs(f1["q"] - f2["q"]) <= 0.1):
            f1["gain"] += f2["gain"]
            del filters[i + 1]
        else:
            i += 1

    # Remove unnecessary filters.
    best_distance = autoeq_calc_distance(autoeq_apply(fr, filters), fr_target)
    i = 0
    while i < len(filters):
        if abs(filters[i]["gain"]) <= 0.1:
            del filters[i]
            continue
        remaining = [ff for fi, ff in enumerate(filters) if fi != i]
        new_distance = autoeq_calc_distance(autoeq_apply(fr, remaining), fr_target)
        if new_distance < best_distance:
            del filters[i]
            best_distance = new_distance
        else:
            i += 1

    return filters


def autoeq_run(fr, fr_target, max_filters):
    """Compute PK filters that reshape `fr` towards `fr_target`.

    `fr` / `fr_target`: list of (freq, dB) on the same, ascending frequency grid.
    """
    deltas = AUTOEQ_CONFIG["optimize_deltas"]
    first_batch_size = max(math.floor(max_filters / 2) - 1, 1)

    first_candidates = autoeq_search_candidates(fr, fr_target, 1)
    first_filters = sorted(
        sorted(
            (c for c in first_candidates
             if c["freq"] <= AUTOEQ_CONFIG["treble_start_from"]),
            key=lambda c: c["q"],
        )[:first_batch_size],
        key=lambda c: c["freq"],
    )
    for i in range(len(deltas)):
        first_filters = autoeq_optimize(fr, fr_target, first_filters, i)

    second_fr = autoeq_apply(fr, first_filters)
    second_batch_size = max_filters - len(first_filters)
    second_candidates = autoeq_search_candidates(second_fr, fr_target, 0.5)
    second_filters = sorted(
        sorted(second_candidates, key=lambda c: c["q"])[:second_batch_size],
        key=lambda c: c["freq"],
    )
    for i in range(len(deltas)):
        second_filters = autoeq_optimize(second_fr, fr_target, second_filters, i)

    all_filters = first_filters + second_filters
    for i in range(len(deltas)):
        all_filters = autoeq_optimize(fr, fr_target, all_filters, i)

    return autoeq_strip(all_filters)


# ---------------------------------------------------------------------------
# ISO 226:2003 equal-loudness normalization (port of hangout.audio's
# graphtool.js normalizePhone()/find_offset()/init_normalize()).
#
# hangout.audio (Crinacle's AutoEQ tool, a CrinGraph fork) doesn't feed raw
# measurement/target dB values straight into Equalizer.autoeq() the way the
# plain autoeq.app tool does — it first computes, independently for each
# curve, the dB offset that brings it to a fixed equal-loudness reference
# (0 phon by default), and adds that offset before running the optimizer.
# Skipping this step is a real source of divergence from hangout.audio's
# output: without it, two curves that each use a different absolute dB
# reference convention (as different measurement sources/rigs do) get
# compared directly, which can make the optimizer chase a systematic level
# offset instead of the actual shape difference. Applying it here makes the
# AutoEQ pipeline reference-convention-agnostic, matching the site's logic.
# ---------------------------------------------------------------------------

_ISO226_F = [
    20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160,
    200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600,
    2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500,
]

_ISO226_A_F = [
    0.532, 0.506, 0.48, 0.455, 0.432, 0.409, 0.387, 0.367, 0.349, 0.33,
    0.315, 0.301, 0.288, 0.276, 0.267, 0.259, 0.253, 0.25, 0.246, 0.244,
    0.243, 0.243, 0.243, 0.242, 0.242, 0.245, 0.254, 0.271, 0.301,
]

_ISO226_L_U = [
    -31.6, -27.2, -23, -19.1, -15.9, -13, -10.3, -8.1, -6.2, -4.5,
    -3.1, -2, -1.1, -0.4, 0, 0.3, 0.5, 0, -2.7, -4.1,
    -1, 1.7, 2.5, 1.2, -2.1, -7.1, -11.2, -10.7, -3.1,
]

_ISO226_T_F = [
    78.5, 68.7, 59.5, 51.1, 44, 37.5, 31.5, 26.5, 22.1, 17.9,
    14.4, 11.4, 8.6, 6.2, 4.4, 3, 2.2, 2.4, 3.5, 1.7,
    -1.3, -4.2, -6, -5.4, -1.5, 6, 12.6, 13.9, 12.3,
]

# Diffuse-field correction curve, ~1/48 octave from 19.4806 Hz, as used by
# hangout.audio's init_normalize() (raw values, before the "-7" dB shift).
_FREE_FIELD_RAW = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0725, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.0896, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.0967, 0, 0, 0, 0, 0, 0,
    0, 0.0886, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0656, 0, 0,
    0, 0, 0, 0.024, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.045, 0, 0, 0, 0, 0, 0, 0.029, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1524, 0.2, 0.2, 0.2386,
    0.3395, 0.4, 0.437, 0.5, 0.5287, 0.6225, 0.7, 0.7063, 0.7962, 0.8, 0.8941, 0.9,
    0.9863, 1, 1.0729, 1.1, 1.1544, 1.2, 1.2504, 1.3, 1.3, 1.3, 1.3, 1.3163,
    1.4, 1.4, 1.4, 1.4, 1.4017, 1.4846, 1.5, 1.5, 1.5748, 1.6, 1.6, 1.653,
    1.7, 1.7, 1.7487, 1.8, 1.8341, 1.9, 1.9, 1.9229, 2, 2, 2, 2.1,
    2.1, 2.1897, 2.2, 2.2, 2.2674, 2.3, 2.3, 2.3567, 2.4, 2.4, 2.4446, 2.5,
    2.5262, 2.6, 2.6234, 2.7149, 2.8, 2.8038, 2.9011, 2.9969, 3.0913, 3.1845, 3.2762, 3.3757,
    3.4649, 3.5617, 3.657, 3.751, 3.8, 3.8432, 3.9332, 4, 4, 4, 4.0121, 4.1,
    4.1, 4.1, 4.0079, 4, 4, 4, 4, 3.9334, 3.9, 3.9, 3.9, 3.8541,
    3.8, 3.8, 3.768, 3.7, 3.6761, 3.6, 3.6, 3.5927, 3.5, 3.5, 3.5, 3.5,
    3.5, 3.5761, 3.6, 3.6, 3.6604, 3.7, 3.7514, 3.8, 3.8, 3.8349, 3.9, 3.9218,
    4.0199, 4.1123, 4.2076, 4.3016, 4.3985, 4.6816, 5.0515, 5.4222, 5.8036, 6.1097, 6.4656, 6.8461,
    7.3316, 7.9083, 8.4305, 8.9369, 9.5105, 10.0759, 10.6024, 11.0027, 11.4847, 12.0482, 12.5152, 12.8994,
    13.2776, 13.7381, 14.1303, 14.5168, 14.8858, 15.273, 15.6547, 15.9731, 16.2596, 16.542, 16.7857, 17.0111,
    17.2325, 17.3532, 17.522, 17.6, 17.6, 17.6, 17.6, 17.5044, 17.41, 17.3145, 17.2205, 17.1255,
    17.0318, 16.9373, 16.784, 16.6459, 16.4536, 16.2578, 16.1234, 15.967, 15.8736, 15.7552, 15.566, 15.3879,
    15.2881, 15.0958, 14.9064, 14.8099, 14.6287, 14.5201, 14.3477, 14.2307, 14.0709, 13.9399, 13.7916, 13.6514,
    13.5552, 13.4604, 13.367, 13.2718, 13.1766, 13.0812, 12.9743, 12.7916, 12.6975, 12.602, 12.5078, 12.3247,
    12.0547, 11.7686, 11.4154, 11.1009, 10.9385, 10.7344, 10.3998, 10.0163, 9.6382, 9.2957, 8.9799, 8.6248,
    8.3404, 8.0424, 7.674, 7.3851, 7.0061, 6.5307, 6.1484, 5.7696, 5.4662, 5.1084, 4.7302, 4.3498,
    3.971, 3.6455, 3.4075, 3.1343, 2.7917, 2.5376, 2.3484, 2.1585, 1.9849, 1.9107, 2, 2,
    2, 2.0894, 2.1844, 2.2787, 2.374, 2.6057, 2.8265, 3.0161, 3.2057, 3.3954, 3.5851, 3.8122,
    4.0967, 4.354, 4.5651, 4.8509, 5.1459, 5.5259, 5.9041, 6.1881, 6.5643, 6.8561, 7.1418, 7.4251,
    7.7093, 8.0593, 8.3192, 8.4541, 8.5493, 8.6437, 8.7, 8.7336, 8.8, 8.8, 8.8, 8.8,
    8.7926, 8.7, 8.7, 8.6079, 8.5133, 8.5, 8.4237, 8.1863, 7.968, 7.7786, 7.4219, 6.948,
    6.4299, 5.8212, 5.1563, 4.4634, 3.7042, 2.8897, 1.9005, 1.2368, 0.5651, -0.2856, -0.8593, -2.9,
]
_FREE_FIELD = [v - 7 for v in _FREE_FIELD_RAW]


def _iso226_init_normalize(freqs):
    """Interpolated ISO 226:2003 loudness parameters at each frequency."""
    par = []
    ff = []
    p_f, p_a, p_lu, p_tf = _ISO226_F, _ISO226_A_F, _ISO226_L_U, _ISO226_T_F
    i = 0
    n = len(p_f)
    for f in freqs:
        if i < n and f >= p_f[i]:
            i += 1
        i0 = max(0, i - 1)
        i1 = min(i, n - 1)
        if i0 == i1:
            a, lu, tf = p_a[i0], p_lu[i0], p_tf[i0]
        else:
            l0, l1, lf = math.log(p_f[i0]), math.log(p_f[i1]), math.log(f)
            frac = (lf - l0) / (l1 - l0)
            a = p_a[i0] + frac * (p_a[i1] - p_a[i0])
            lu = p_lu[i0] + frac * (p_lu[i1] - p_lu[i0])
            tf = p_tf[i0] + frac * (p_tf[i1] - p_tf[i0])
        m = a * (math.log10(4) - 10 + lu / 10)
        k = (0.005076 / (10 ** m)) - (10 ** (a * tf / 10))
        c = (10 ** (9.4 + 4 * m)) / len(freqs)
        par.append((a, k, c))
        ffi = math.floor(0.5 + 48 * math.log2(f / 19.4806))
        ff.append(_FREE_FIELD[max(0, min(479, ffi))])
    return par, ff


def iso226_find_offset(curve, target_phon=0.0):
    """dB offset that brings `curve` (list of (freq, dB)) to `target_phon` loudness."""
    freqs = [f for f, _ in curve]
    values = [v for _, v in curve]
    par, ff = _iso226_init_normalize(freqs)
    l10 = math.log(10) / 10

    def get_step(offset):
        v_total = 0.0
        d_total = 0.0
        for (a, k, c), fr_val, ff_val in zip(par, values, ff):
            v0 = math.exp(l10 * (fr_val + offset - ff_val))
            ds = l10 * v0
            v1 = k + v0 ** a
            ds *= a * (v0 ** (a - 1))
            v_total += c * (v1 ** 4)
            ds *= c * 4 * (v1 ** 3)
            d_total += ds
        return (math.log(v_total) - target_phon * l10) * (v_total / d_total)

    x = 0.0
    for _ in range(100):  # converges in a handful of steps; capped as a safety net
        dx = get_step(x)
        x -= dx
        if abs(dx) <= 0.01:
            break
    return x


def autoeq_loudness_normalize(curve):
    """Shift `curve` by its own ISO-226 offset to 0 phon, matching hangout.audio."""
    offset = iso226_find_offset(curve, 0.0)
    return [(f, v + offset) for f, v in curve]


# ===========================================================================
# GUI helpers
# ===========================================================================

class StdoutRedirector:
    """Thread-safe write target that feeds a queue."""

    def __init__(self, q):
        self.q = q

    def write(self, text):
        if text:
            self.q.put(text)

    def flush(self):
        pass


NEW_BAND = {"type": "PK", "freq": 1000.0, "gain": 0.0, "q": 1.0}
GRAPH_MIN_HEIGHT = 150  # px; below this the graph is auto-hidden


# ===========================================================================
# GUI
# ===========================================================================

class EqLoaderGUI(tk.Tk):

    def __init__(self, graph=None):
        super().__init__()

        # None = auto (hide when short), True = always show, False = always hide
        self._graph_forced = graph
        self._graph_show_threshold = None  # window height at/above which the graph fits
        self._layout_ready = False
        self._resize_after_id = None

        self.title("Walkplay PEQ Loader")
        self.geometry("950x850")
        self.minsize(850, 700)

        self._apply_theme()

        self.log_queue = queue.Queue()
        self.selected_path = None
        self._devices_cache = []

        self.create_filters = [dict(NEW_BAND)]
        self.selected_filter = 0

        self._undo_stack = []
        self._redo_stack = []
        self._dragging_point_idx = None
        self._drag_snapshot_taken = False

        # Live editor state.
        self._loading_editor = False       # suppress traces while loading fields
        self._editor_snapshot_taken = False  # one undo step per edit session

        # AutoEQ model database (lazily built the first time it's browsed).
        self._autoeq_db_path = None
        self._autoeq_model_index = None
        self._autoeq_model_index_path = None
        self._autoeq_target_index = None  # lazily fetched target-curve list

        self._build_widgets()

        if self._graph_forced is False:
            self._set_graph_visible(False)

        # Evaluate initial graph visibility once the window is actually mapped
        # (its real size is only known then); a timed call is a fallback in
        # case <Map> was already delivered.
        self.bind("<Map>", self._on_mapped, add="+")
        self.after(200, self._mark_layout_ready)
        self._theme_classic_widgets()
        self._poll_log_queue()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if hid is None:
            self._log("ERROR: the 'hidapi' package is not installed.\n"
                      "Run:  pip install hidapi\n"
                      "Then restart this app.\n")

    # ------------------------------------------------------------------
    # Theme
    # ------------------------------------------------------------------

    def _apply_theme(self):
        """Skin every ttk widget as a graphite instrument panel."""
        c = THEME
        self.font_ui = _pick_font(self, UI_FONTS)
        self.font_mono = _pick_font(self, MONO_FONTS)

        self.configure(bg=c["chassis"])

        style = ttk.Style(self)
        # 'clam' is the one built-in theme that honours colour overrides.
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        base_font = (self.font_ui, 10)

        style.configure(".", background=c["chassis"], foreground=c["ink"],
                        fieldbackground=c["input"], bordercolor=c["line"],
                        lightcolor=c["line"], darkcolor=c["line"],
                        troughcolor=c["panel"], font=base_font)

        style.configure("TFrame", background=c["chassis"])
        style.configure("TLabel", background=c["chassis"],
                        foreground=c["ink"], font=base_font)
        style.configure("Muted.TLabel", background=c["chassis"],
                        foreground=c["muted"], font=(self.font_ui, 9))

        style.configure("TLabelframe", background=c["chassis"],
                        bordercolor=c["line"], relief="solid", borderwidth=1)
        style.configure("TLabelframe.Label", background=c["chassis"],
                        foreground=c["muted"], font=(self.font_ui, 9, "bold"))

        for widget in ("TEntry", "TSpinbox", "TCombobox"):
            style.configure(widget, background=c["input"],
                            fieldbackground=c["input"], foreground=c["ink"],
                            insertcolor=c["accent"], bordercolor=c["line"],
                            arrowcolor=c["muted"], padding=4)
            style.map(widget, bordercolor=[("focus", c["accent"])],
                      foreground=[("disabled", c["muted"])])

        # readonly combobox field needs explicit state mappings.
        style.map("TCombobox",
                  fieldbackground=[("readonly", c["input"]), ("disabled", c["panel"])],
                  foreground=[("readonly", c["ink"]), ("disabled", c["muted"])],
                  selectbackground=[("readonly", c["input"])],
                  selectforeground=[("readonly", c["ink"])],
                  background=[("focus", c["input"]), ("active", c["line"]),
                              ("!focus", c["input"])],
                  arrowcolor=[("focus", c["accent"]), ("active", c["accent"]),
                              ("!focus", c["muted"])])

        style.configure("TButton", background=c["input"], foreground=c["ink"],
                        bordercolor=c["line"], focuscolor=c["accent"],
                        relief="flat", padding=(10, 6), font=base_font)
        style.map("TButton",
                  background=[("pressed", c["accent_dk"]), ("active", c["line"])],
                  foreground=[("pressed", c["chassis"])],
                  bordercolor=[("active", c["accent"])])

        style.configure("Accent.TButton", background=c["accent"],
                        foreground=c["chassis"], relief="flat",
                        padding=(10, 6), font=(self.font_ui, 10, "bold"))
        style.map("Accent.TButton",
                  background=[("pressed", c["accent_dk"]), ("active", c["accent_dk"])],
                  foreground=[("active", c["chassis"])])

        style.configure("Danger.TButton", background=c["input"],
                        foreground=c["danger"], relief="flat", padding=(10, 6))
        style.map("Danger.TButton",
                  background=[("active", c["danger"]), ("pressed", c["danger"])],
                  foreground=[("active", c["chassis"]), ("pressed", c["chassis"])])

        style.configure("TCheckbutton", background=c["chassis"],
                        foreground=c["ink"], focuscolor=c["accent"])
        style.map("TCheckbutton", background=[("active", c["chassis"])],
                  indicatorcolor=[("selected", c["accent"]), ("!selected", c["input"])])

        style.configure("TNotebook", background=c["chassis"],
                        bordercolor=c["line"], tabmargins=(2, 4, 2, 0))
        style.configure("TNotebook.Tab", background=c["panel"], foreground=c["muted"],
                        bordercolor=c["line"], padding=(14, 7), font=base_font)
        style.map("TNotebook.Tab", background=[("selected", c["chassis"])],
                  foreground=[("selected", c["accent"]), ("active", c["ink"])])

        for sb in ("Vertical.TScrollbar", "Horizontal.TScrollbar"):
            style.configure(sb, background=c["input"], troughcolor=c["panel"],
                            bordercolor=c["panel"], arrowcolor=c["muted"])
            style.map(sb, background=[("active", c["line"])])

        # Combobox dropdown popup is a plain tk.Listbox — style via option_add.
        self.option_add("*TCombobox*Listbox.background", c["input"])
        self.option_add("*TCombobox*Listbox.foreground", c["ink"])
        self.option_add("*TCombobox*Listbox.selectBackground", c["accent"])
        self.option_add("*TCombobox*Listbox.selectForeground", c["chassis"])
        self.option_add("*TCombobox*Listbox.font", (self.font_ui, 10))

    def _theme_classic_widgets(self):
        """Colour the non-ttk (classic tk) widgets to match the theme."""
        c = THEME
        list_opts = dict(
            bg=c["panel"], fg=c["ink"], selectbackground=c["accent"],
            selectforeground=c["chassis"], highlightthickness=1,
            highlightbackground=c["line"], highlightcolor=c["accent"],
            borderwidth=0, activestyle="none", font=(self.font_ui, 10),
        )
        for lb in (self.device_list, self.filter_list):
            lb.configure(**list_opts)

        self.log_text.configure(
            bg=c["chassis"], fg=c["accent"], insertbackground=c["accent"],
            selectbackground=c["input"], selectforeground=c["ink"],
            highlightthickness=1, highlightbackground=c["line"],
            borderwidth=0, font=(self.font_mono, 9), padx=8, pady=6,
        )
        self.log_text.vbar.configure(
            bg=c["input"], troughcolor=c["panel"], activebackground=c["line"],
            highlightbackground=c["panel"], highlightcolor=c["panel"],
            borderwidth=0, relief="flat",
        )

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_widgets(self):
        self.grid_rowconfigure(2, weight=3)
        self.grid_rowconfigure(5, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        # ---- Device (row 0) ----
        dev_frame = ttk.LabelFrame(self, text="Device")
        dev_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=6)

        self.device_list = tk.Listbox(dev_frame, height=4)
        self.device_list.pack(fill="x", padx=6, pady=6, side="left", expand=True)
        self.device_list.bind("<<ListboxSelect>>", self._on_device_select)

        btn_frame = ttk.Frame(dev_frame)
        btn_frame.pack(side="left", padx=6)
        refresh_btn = ttk.Button(btn_frame, text="Refresh List",
                                 command=self._refresh_devices)
        refresh_btn.pack(fill="x", pady=2)
        self._add_tooltip(refresh_btn, "F5")
        slot_btn = ttk.Button(btn_frame, text="Get Slot / Version",
                              command=self._get_slot)
        slot_btn.pack(fill="x", pady=2)
        self._add_tooltip(slot_btn, "Ctrl+G")

        # ---- VID / PID (row 1) ----
        override_frame = ttk.Frame(self)
        override_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

        ttk.Label(override_frame, text="VID (hex):").grid(row=0, column=0, sticky="w")
        self.vid_entry = ttk.Entry(override_frame, width=10)
        self.vid_entry.insert(0, "0x3302")
        self.vid_entry.grid(row=0, column=1, padx=4)

        ttk.Label(override_frame, text="PID (hex, optional):").grid(
            row=0, column=2, sticky="w")
        self.pid_entry = ttk.Entry(override_frame, width=10)
        self.pid_entry.grid(row=0, column=3, padx=4)

        ttk.Label(override_frame, text="Max filters:").grid(
            row=0, column=4, sticky="w", padx=(12, 0))
        int_only = (self.register(lambda s: s == "" or s.isdigit()), "%P")
        self.max_filter_entry = ttk.Spinbox(
            override_frame, from_=1, to=64, increment=1, width=6,
            validate="key", validatecommand=int_only)
        self.max_filter_entry.set(DEFAULT_MAX_FILTERS)
        self.max_filter_entry.grid(row=0, column=5, padx=4)

        # ---- Graph (row 2, col 0) ----
        self.fig = Figure(figsize=(7, 4), dpi=100,
                          facecolor=THEME["chassis"], layout="constrained")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(THEME["panel"])

        # grid_propagate(False) stops the canvas's own size requests from
        # forcing a main-window geometry recalculation on matplotlib redraws.
        self.graph_frame = tk.Frame(self, bg=THEME["chassis"])
        self.graph_frame.grid(row=2, column=0, sticky="nsew", padx=(8, 4), pady=(6, 2))
        self.graph_frame.grid_propagate(False)

        self.canvas_graph = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas_graph.mpl_connect("button_press_event", self._on_press)
        self.canvas_graph.mpl_connect("motion_notify_event", self._on_motion)
        self.canvas_graph.mpl_connect("button_release_event", self._on_release)
        self.canvas_graph.get_tk_widget().pack(fill="both", expand=True)

        self._graph_too_small_label = tk.Label(
            self.graph_frame, text="Window too small to display graph",
            bg=THEME["chassis"], fg=THEME["muted"], font=("TkDefaultFont", 9),
        )

        # ---- Filter list + editor (row 3, col 0) ----
        ctrl = ttk.Frame(self)
        ctrl.grid(row=3, column=0, sticky="ew", padx=(8, 4), pady=2)
        ctrl.columnconfigure(1, weight=1)

        list_frame = ttk.LabelFrame(ctrl, text="Filters")
        list_frame.grid(row=0, column=0, sticky="ns")
        self.filter_list = tk.Listbox(list_frame, height=7, selectmode="extended")
        self.filter_list.pack(fill="both", expand=True, padx=5, pady=5)
        self.filter_list.bind("<<ListboxSelect>>", self._create_select_band)

        edit = ttk.LabelFrame(ctrl, text="Selected Filter")
        edit.grid(row=0, column=1, sticky="nsew", padx=10)
        edit.columnconfigure(1, weight=1)

        for row_i, (lbl, var_name, default, lo, hi, step, fmt) in enumerate((
            ("Frequency (Hz)", "freq_var", "1000", 10, 30000, 1, "%.0f"),
            ("Gain (dB)", "gain_var", "0", -30, 30, 0.1, "%.1f"),
            ("Q", "q_var", "1.0", 0.1, 100, 0.1, "%.1f"),
        )):
            lbl_widget = ttk.Label(edit, text=lbl)
            lbl_widget.grid(row=row_i, column=0, sticky="w", padx=5, pady=3)
            if var_name == "q_var":
                self.q_label = lbl_widget
            setattr(self, var_name, tk.StringVar(value=default))
            ttk.Spinbox(edit, textvariable=getattr(self, var_name),
                        from_=lo, to=hi, increment=step, format=fmt).grid(
                row=row_i, column=1, sticky="ew", padx=5, pady=3)

        ttk.Label(edit, text="Type").grid(row=3, column=0, sticky="w", padx=5, pady=3)
        self.type_var = tk.StringVar(value="PK")
        ttk.Combobox(edit, textvariable=self.type_var,
                     values=["PK", "LSQ", "HSQ", "LP", "HP"], state="readonly").grid(
            row=3, column=1, sticky="ew", padx=5, pady=3)

        self.bw_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(edit, text="Show Q as Bandwidth (oct)", variable=self.bw_mode,
                        command=self._toggle_bw_mode).grid(
            row=4, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        # ---- Operations bar (row 4, col 0) ----
        ops = ttk.Frame(self)
        ops.grid(row=4, column=0, sticky="ew", padx=(8, 4), pady=2)
        ops.columnconfigure(0, weight=1)
        ops.columnconfigure(1, weight=1)

        push_created_frame = ttk.LabelFrame(ops, text="EQ")
        push_created_frame.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        for lbl, attr, default in (
            ("Slot", "create_slot_spin", "0"),
            ("Preamp (dB)", "create_preamp_entry", "0"),
            ("Buffer (dB)", "create_buffer_entry",
             str(float(DEFAULT_GLOBAL_GAIN_BUFFER))),
        ):
            ttk.Label(push_created_frame, text=f"{lbl}:").pack(side="left", padx=(8, 2))
            if lbl == "Slot":
                w = ttk.Spinbox(push_created_frame, from_=0, to=15, width=5)
            else:
                w = ttk.Spinbox(push_created_frame, from_=-30, to=30,
                                increment=0.1, format="%.1f", width=7)
            w.set(default)
            w.pack(side="left", padx=(0, 6))
            setattr(self, attr, w)

        ed_frame = ttk.LabelFrame(ops, text="PEQ Enable / Disable")
        ed_frame.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=2)
        ttk.Label(ed_frame, text="Slot:").pack(side="left", padx=(8, 2))
        self.ed_slot_spin = ttk.Spinbox(ed_frame, from_=0, to=15, width=5)
        self.ed_slot_spin.set(0)
        self.ed_slot_spin.pack(side="left", padx=(0, 8))
        enable_btn = ttk.Button(ed_frame, text="Enable PEQ", command=self._enable,
                                style="Accent.TButton")
        enable_btn.pack(side="left", padx=4, pady=4)
        self._add_tooltip(enable_btn, "Ctrl+Shift+E")
        disable_btn = ttk.Button(ed_frame, text="Disable PEQ", command=self._disable,
                                 style="Danger.TButton")
        disable_btn.pack(side="left", padx=4, pady=4)
        self._add_tooltip(disable_btn, "Ctrl+Shift+X")

        # ---- Log (row 5, col 0) ----
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=5, column=0, sticky="nsew", padx=(8, 4), pady=6)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=5, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        # ---- Actions panel (col 1, rows 2–5) ----
        actions = ttk.LabelFrame(self, text="Actions")
        actions.grid(row=2, column=1, rowspan=4, sticky="nsew", padx=(0, 8), pady=6)
        actions.columnconfigure(0, weight=1)

        action_btns = (
            ("Add Band", self._create_add_band, "TButton", "Ctrl+B", "<Control-b>"),
            ("Delete Band", self._create_delete_band, "Danger.TButton", "Ctrl+D", "<Control-d>"),
            ("Delete All", self._create_delete_all_bands, "Danger.TButton",
             "Ctrl+Shift+D", "<Control-Shift-D>"),
            ("Load EQ from Device", self._create_load_from_device, "TButton",
             "Ctrl+E", "<Control-e>"),
            ("Save Profile to File", self._create_save_profile, "TButton", "Ctrl+S", "<Control-s>"),
            ("Load Profile from File", self._create_load_profile, "TButton",
             "Ctrl+O", "<Control-o>"),
            ("AutoEQ", self._create_autoeq, "TButton",
             "Ctrl+Shift+A", "<Control-Shift-A>"),
            ("Push EQ to Device", self._create_push, "Accent.TButton", "Ctrl+P", "<Control-p>"),
        )
        for i, (text, cmd, style, accel, seq) in enumerate(action_btns):
            actions.rowconfigure(i, weight=1)
            btn = ttk.Button(actions, text=text, command=cmd, style=style)
            btn.grid(row=i, column=0, sticky="nsew", padx=6, pady=2)
            self.bind(seq, lambda _e, c=cmd: c())
            self._add_tooltip(btn, accel)

        # ---- Bindings + initial state ----
        self.bind("<Configure>", self._on_window_resize)
        self.bind("<Control-z>", self._undo)
        self.bind("<Control-y>", self._redo)
        self.bind("<Control-Shift-z>", self._redo)
        self.bind("<F5>", lambda _e: self._refresh_devices())
        self.bind("<Control-g>", lambda _e: self._get_slot())
        self.bind("<Control-Shift-E>", lambda _e: self._enable())
        self.bind("<Control-Shift-X>", lambda _e: self._disable())

        # Live-apply editor fields to the current selection as they change.
        self.freq_var.trace_add("write", lambda *_: self._apply_field_to_selection("freq"))
        self.gain_var.trace_add("write", lambda *_: self._apply_field_to_selection("gain"))
        self.q_var.trace_add("write", lambda *_: self._apply_field_to_selection("q"))
        self.type_var.trace_add("write", lambda *_: self._apply_field_to_selection("type"))

        self._refresh_create_tab()
        self._refresh_devices()

    # ------------------------------------------------------------------
    # Graph auto-hide on resize
    # ------------------------------------------------------------------

    def _set_graph_visible(self, visible):
        canvas_widget = self.canvas_graph.get_tk_widget()
        if visible:
            self._graph_too_small_label.pack_forget()
            if not canvas_widget.winfo_ismapped():
                canvas_widget.pack(fill="both", expand=True)
        else:
            canvas_widget.pack_forget()
            if self._graph_forced is None:
                self._graph_too_small_label.pack(expand=True)

    def _on_mapped(self, event):
        if event.widget is self:
            self._mark_layout_ready()

    def _mark_layout_ready(self):
        self._layout_ready = True
        self.after_idle(self._apply_graph_visibility)

    def _on_window_resize(self, event):
        if not self._layout_ready or self._graph_forced is not None \
                or event.widget is not self:
            return
        if self._resize_after_id is not None:
            self.after_cancel(self._resize_after_id)
        self._resize_after_id = self.after(80, self._apply_graph_visibility)

    def _apply_graph_visibility(self):
        self._resize_after_id = None
        if self._graph_forced is not None:
            return
        # Flush pending geometry so winfo_height() reflects the mapped size and
        # not a stale value from before the window manager sized the window.
        self.update_idletasks()
        win_h = self.winfo_height()
        if self.canvas_graph.get_tk_widget().winfo_ismapped():
            # While visible, the chrome above/below the graph is stable, so the
            # window height at which the frame would hit the minimum is fixed.
            frame_h = self.graph_frame.winfo_height()
            self._graph_show_threshold = win_h - frame_h + GRAPH_MIN_HEIGHT
            if frame_h < GRAPH_MIN_HEIGHT:
                self._set_graph_visible(False)
        elif self._graph_show_threshold is None or win_h >= self._graph_show_threshold:
            self._set_graph_visible(True)

    # ------------------------------------------------------------------
    # Create / editor
    # ------------------------------------------------------------------

    def _refresh_create_tab(self):
        self._render_filter_rows()

        if self.create_filters:
            if self.selected_filter >= len(self.create_filters):
                self.selected_filter = len(self.create_filters) - 1
            self.filter_list.selection_clear(0, "end")
            self.filter_list.selection_set(self.selected_filter)
            self.filter_list.see(self.selected_filter)
        else:
            self.selected_filter = -1

        self._draw_response_graph()

    def _render_filter_rows(self):
        """Rebuild the listbox rows from create_filters (selection untouched)."""
        self.filter_list.delete(0, "end")
        for i, f in enumerate(self.create_filters):
            self.filter_list.insert(
                "end",
                f"{i + 1}: {f['freq']:.1f} Hz  {f['gain']:.1f} dB  "
                f"Q {f['q']:.2f}  {f['type']}")

    @staticmethod
    def _biquad_response_db(freqs, freq0, gain_db, q, ftype, fs=96000):
        q = max(q, 0.001)
        w0 = 2 * np.pi * freq0 / fs
        A = 10 ** (gain_db / 40)
        alpha = np.sin(w0) / (2 * q)
        cw = np.cos(w0)

        if ftype == "PK":
            b0 = 1 + alpha * A
            b1 = -2 * cw
            b2 = 1 - alpha * A
            a0 = 1 + alpha / A
            a1 = -2 * cw
            a2 = 1 - alpha / A
        elif ftype == "LSQ":
            sqA = np.sqrt(A)
            b0 = A * ((A + 1) - (A - 1) * cw + 2 * sqA * alpha)
            b1 = 2 * A * ((A - 1) - (A + 1) * cw)
            b2 = A * ((A + 1) - (A - 1) * cw - 2 * sqA * alpha)
            a0 = (A + 1) + (A - 1) * cw + 2 * sqA * alpha
            a1 = -2 * ((A - 1) + (A + 1) * cw)
            a2 = (A + 1) + (A - 1) * cw - 2 * sqA * alpha
        elif ftype == "HSQ":
            sqA = np.sqrt(A)
            b0 = A * ((A + 1) + (A - 1) * cw + 2 * sqA * alpha)
            b1 = -2 * A * ((A - 1) + (A + 1) * cw)
            b2 = A * ((A + 1) + (A - 1) * cw - 2 * sqA * alpha)
            a0 = (A + 1) - (A - 1) * cw + 2 * sqA * alpha
            a1 = 2 * ((A - 1) - (A + 1) * cw)
            a2 = (A + 1) - (A - 1) * cw - 2 * sqA * alpha
        elif ftype == "HP":
            b0 = (1 + cw) / 2
            b1 = -(1 + cw)
            b2 = (1 + cw) / 2
            a0 = 1 + alpha
            a1 = -2 * cw
            a2 = 1 - alpha
        elif ftype == "LP":
            b0 = (1 - cw) / 2
            b1 = 1 - cw
            b2 = (1 - cw) / 2
            a0 = 1 + alpha
            a1 = -2 * cw
            a2 = 1 - alpha
        else:
            return np.zeros_like(freqs)

        b0 /= a0; b1 /= a0; b2 /= a0
        a1 /= a0; a2 /= a0

        w = 2 * np.pi * freqs / fs
        ejw_n1 = np.exp(-1j * w)
        ejw_n2 = np.exp(-2j * w)
        H = (b0 + b1 * ejw_n1 + b2 * ejw_n2) / (1 + a1 * ejw_n1 + a2 * ejw_n2)
        return 20 * np.log10(np.abs(H) + 1e-12)

    def _draw_response_graph(self):
        c = THEME
        self.ax.clear()

        freqs = np.logspace(np.log10(20), np.log10(20000), 1000)
        response = np.zeros(len(freqs))

        selected = set(self.filter_list.curselection())
        selected.add(self.selected_filter)

        for index, f in enumerate(self.create_filters):
            try:
                center_freq = float(f["freq"])
                gain = float(f["gain"])
                q = max(float(f["q"]), 0.01)
            except (ValueError, TypeError):
                continue
            if center_freq <= 0:
                continue

            response += self._biquad_response_db(
                freqs, center_freq, gain, q, f.get("type", "PK"))

            is_selected = index in selected
            color = c["active"] if is_selected else c["accent"]

            # Amber glow halo around the active band's handle.
            if is_selected:
                self.ax.plot([center_freq], [gain], marker="o", markersize=16,
                             color=c["active"], alpha=0.25, zorder=4)

            self.ax.plot([center_freq], [gain], marker="o", markersize=9,
                         markerfacecolor=color, markeredgecolor=c["chassis"],
                         markeredgewidth=1.5, zorder=5)

        self.fig.set_facecolor(c["chassis"])
        self.ax.set_facecolor(c["panel"])

        # Filled scope trace with a soft glow underneath.
        self.ax.fill_between(freqs, response, 0, color=c["accent"], alpha=0.10, zorder=1)
        for lw, a in ((5, 0.10), (3, 0.18)):  # glow layers
            self.ax.plot(freqs, response, linewidth=lw, color=c["accent"], alpha=a, zorder=2)
        self.ax.plot(freqs, response, linewidth=2.0, color=c["accent"], zorder=3)

        # 0 dB reference line.
        self.ax.axhline(0, color=c["muted"], linewidth=0.8, alpha=0.6, zorder=1)

        self.ax.set_xscale("log")
        self.ax.set_xlim(20, 20000)
        self.ax.set_ylim(-15, 15)

        def freq_formatter(x, pos):
            if x >= 1000:
                return f"{x/1000:.0f} kHz"
            else:
                return f"{x:.0f} Hz"

        self.ax.xaxis.set_major_locator(ticker.LogLocator(base=10, numticks=10))
        self.ax.xaxis.set_major_formatter(ticker.FuncFormatter(freq_formatter))

        # Set custom ticks to show only 20 Hz and 20 kHz (exclude 0)
        major_ticks = [20, 100, 1000, 10000, 20000]
        self.ax.set_xticks(major_ticks)
        self.ax.set_xticklabels([freq_formatter(t, None) for t in major_ticks])

        self.ax.grid(True, which="major", color=c["line"], linewidth=0.8, alpha=0.9)
        self.ax.grid(True, which="minor", color=c["line"], linewidth=0.5, alpha=0.4)

        for side, spine in self.ax.spines.items():
            spine.set_color(c["line"])
            spine.set_visible(side in ("left", "bottom"))
        self.ax.tick_params(colors=c["muted"], labelsize=8, which="both")

        self.ax.set_title("EQ Response", color=c["muted"], fontsize=10,
                          fontweight="bold", loc="left", fontfamily=self.font_ui, pad=10)
        self.ax.set_xlabel("Frequency (Hz)", color=c["muted"], fontsize=9)
        self.ax.set_ylabel("Gain (dB)", color=c["muted"], fontsize=9)

        self.canvas_graph.draw_idle()

    def _create_add_band(self):
        self._snapshot()
        self.create_filters.append(dict(NEW_BAND))
        self.selected_filter = len(self.create_filters) - 1
        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    def _create_delete_band(self):
        sel = self.filter_list.curselection()
        if not sel:
            return

        self._snapshot()
        for index in sorted(sel, reverse=True):
            del self.create_filters[index]

        if not self.create_filters:
            self.selected_filter = -1
        else:
            self.selected_filter = min(sel[0], len(self.create_filters) - 1)

        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    def _create_delete_all_bands(self):
        if not messagebox.askyesno("Delete All Bands", "Remove all EQ bands?"):
            return

        self._snapshot()
        self.create_filters.clear()
        self.selected_filter = -1
        self._refresh_create_tab()

        self.freq_var.set("")
        self.gain_var.set("")
        self.q_var.set("")
        self.type_var.set("PK")

    def _create_load_from_device(self):
        if not messagebox.askyesno(
                "Load EQ from Device",
                "Load EQ from device? This will replace all current filters."):
            return

        max_filters = self._parse_int(self.max_filter_entry.get(), DEFAULT_MAX_FILTERS)

        def task():
            dev = self._open_selected_device()
            try:
                slot = get_current_slot(dev)
                result = pull_from_device(
                    dev, max_filters=max_filters, slot_hint=slot)
            finally:
                dev.close()

            def apply():
                self._snapshot()
                loaded = [
                    {
                        "type": f.get("type", "PK"),
                        "freq": float(f["freq"]) or 1000.0,
                        "gain": float(f["gain"]),
                        "q": float(f["q"]) or 1.0,
                    }
                    for f in result["filters"]
                    if not f.get("disabled", False)
                ]
                self.create_filters = dedupe_filters(loaded)
                self.selected_filter = 0 if self.create_filters else -1
                self.create_preamp_entry.delete(0, "end")
                self.create_preamp_entry.insert(0, str(result["globalGain"]))
                self._refresh_create_tab()

            self.after(0, apply)

        self._run_bg(task)

    def _create_select_band(self, _event):
        sel = self.filter_list.curselection()
        if not sel:
            return
        self.selected_filter = sel[0]
        self._load_selected_filter_into_editor()
        self._draw_response_graph()

    def _load_selected_filter_into_editor(self):
        """Show the primary selected band's values (without re-applying them)."""
        self._editor_snapshot_taken = False
        if not 0 <= self.selected_filter < len(self.create_filters):
            return
        f = self.create_filters[self.selected_filter]

        self._loading_editor = True
        try:
            self.freq_var.set(str(f["freq"]))
            self.gain_var.set(str(f["gain"]))
            if self.bw_mode.get():
                q_val = max(float(f["q"]), 0.001)
                bw = 2 * math.asinh(1 / (2 * q_val)) / math.log(2)
                self.q_var.set(f"{bw:.3f}")
            else:
                self.q_var.set(str(f["q"]))
            self.type_var.set(f["type"])
        finally:
            self._loading_editor = False

    def _apply_field_to_selection(self, field):
        """Live-apply a single edited field to every selected band."""
        if self._loading_editor:
            return
        sel = self.filter_list.curselection() or (
            (self.selected_filter,) if 0 <= self.selected_filter < len(self.create_filters)
            else ())
        if not sel:
            return

        if field == "type":
            value = self.type_var.get()
        elif field == "gain":
            value = self._parse_float(self.gain_var.get())
            if value is None:
                return
        elif field == "freq":
            value = self._parse_float(self.freq_var.get())
            if value is None or value <= 0:
                return
        else:  # q (or bandwidth)
            value = self._parse_float(self.q_var.get())
            if value is None or value <= 0:
                return
            if self.bw_mode.get():
                try:
                    value = 1 / (2 * math.sinh(value * math.log(2) / 2))
                except OverflowError:
                    return
                if value <= 0:
                    return

        if not self._editor_snapshot_taken:
            self._snapshot()
            self._editor_snapshot_taken = True

        for i in sel:
            self.create_filters[i][field] = value

        self._render_filter_rows()
        for i in sel:
            self.filter_list.selection_set(i)
        self._draw_response_graph()

    def _toggle_bw_mode(self):
        val = self._parse_float(self.q_var.get())
        if val is None or val <= 0:
            self.q_label.config(
                text="Bandwidth (oct)" if self.bw_mode.get() else "Q")
            return

        # Only the display unit changes here, not the underlying band, so
        # suppress the live-apply trace while rewriting the field.
        self._loading_editor = True
        try:
            if self.bw_mode.get():
                bw = 2 * math.asinh(1 / (2 * max(val, 0.001))) / math.log(2)
                self.q_var.set(f"{bw:.3f}")
                self.q_label.config(text="Bandwidth (oct)")
            else:
                try:
                    q = 1 / (2 * math.sinh(val * math.log(2) / 2))
                except OverflowError:
                    self.q_label.config(text="Q")
                    return
                self.q_var.set(f"{q:.3f}")
                self.q_label.config(text="Q")
        finally:
            self._loading_editor = False

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------

    def _capture_state(self):
        return (copy.deepcopy(self.create_filters),
                self.create_preamp_entry.get(),
                self.selected_filter)

    def _restore_state(self, state):
        filters, preamp, sel = state
        self.create_filters = filters
        self.selected_filter = max(-1, min(sel, len(self.create_filters) - 1))
        self.create_preamp_entry.delete(0, "end")
        self.create_preamp_entry.insert(0, preamp)
        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    def _snapshot(self):
        self._undo_stack.append(self._capture_state())
        self._redo_stack.clear()

    def _undo(self, _event=None):
        if not self._undo_stack:
            return
        self._redo_stack.append(self._capture_state())
        self._restore_state(self._undo_stack.pop())

    def _redo(self, _event=None):
        if not self._redo_stack:
            return
        self._undo_stack.append(self._capture_state())
        self._restore_state(self._redo_stack.pop())

    # ------------------------------------------------------------------
    # Mouse drag-and-drop graph controls
    # ------------------------------------------------------------------

    def _find_closest_filter(self, event, max_pixels=14):
        """Index of the band nearest the cursor in screen pixels, or -1."""
        if event.x is None or event.y is None:
            return -1

        best_idx = -1
        best_dist = float("inf")
        for i, f in enumerate(self.create_filters):
            try:
                px = float(f.get("freq", 0))
                py = float(f.get("gain", 0))
            except (ValueError, TypeError):
                continue
            if px <= 0:
                continue
            try:
                disp_x, disp_y = self.ax.transData.transform((px, py))
            except (ValueError, TypeError):
                continue
            dist = math.hypot(disp_x - event.x, disp_y - event.y)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        return best_idx if best_dist <= max_pixels else -1

    def _on_press(self, event):
        if event.xdata is None or event.ydata is None:
            return

        freq = float(event.xdata)
        gain = float(event.ydata)
        if freq < 20 or freq > 20000:
            return

        if event.button == 3:
            self._on_right_click_graph(event)
            return

        closest_idx = self._find_closest_filter(event)
        if closest_idx >= 0:
            # Grab an existing band. Don't snapshot yet: a plain selecting click
            # should not create an undo step. The snapshot is taken lazily on
            # the first actual drag move.
            self.selected_filter = closest_idx
            self._dragging_point_idx = closest_idx
            self._drag_snapshot_taken = False
            self._load_selected_filter_into_editor()
            self._draw_response_graph()
        else:
            self._snapshot()
            self.create_filters.append({
                "type": "PK", "freq": round(freq, 1), "gain": round(gain, 1), "q": 1.0,
            })
            self.selected_filter = len(self.create_filters) - 1
            self._dragging_point_idx = self.selected_filter
            # The pre-append snapshot already covers creating and positioning
            # this new band as a single undo step.
            self._drag_snapshot_taken = True
            self._refresh_create_tab()
            self._load_selected_filter_into_editor()

    def _on_motion(self, event):
        if self._dragging_point_idx is None:
            return
        if event.xdata is None or event.ydata is None:
            return

        freq = max(20.0, min(20000.0, float(event.xdata)))
        gain = max(-15.0, min(15.0, float(event.ydata)))

        # Record the pre-drag state once, so the whole drag is one undo step.
        if not self._drag_snapshot_taken:
            self._snapshot()
            self._drag_snapshot_taken = True

        self.create_filters[self._dragging_point_idx]["freq"] = round(freq, 1)
        self.create_filters[self._dragging_point_idx]["gain"] = round(gain, 1)

        self._load_selected_filter_into_editor()
        self._draw_response_graph()

    def _on_release(self, _event):
        if self._dragging_point_idx is not None:
            self._dragging_point_idx = None
            self._drag_snapshot_taken = False
            self._refresh_create_tab()

    def _on_right_click_graph(self, event):
        if not self.create_filters:
            return
        closest_idx = self._find_closest_filter(event)
        if closest_idx < 0:
            return

        f = self.create_filters[closest_idx]
        if not messagebox.askyesno(
                "Delete Band",
                f"Delete band {closest_idx + 1} ({float(f['freq']):.1f} Hz)?"):
            return

        self._snapshot()
        del self.create_filters[closest_idx]
        if not self.create_filters:
            self.selected_filter = -1
        else:
            self.selected_filter = min(closest_idx, len(self.create_filters) - 1)

        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    # ------------------------------------------------------------------
    # Push / save / load profile
    # ------------------------------------------------------------------

    def _create_push(self, then=None):
        if not self.create_filters:
            messagebox.showwarning("No Filters", "Add at least one EQ band first.")
            return

        slot = self._parse_int(self.create_slot_spin.get(), 0)
        preamp = self._parse_float(self.create_preamp_entry.get(), 0)
        buffer_db = self._parse_float(
            self.create_buffer_entry.get(), DEFAULT_GLOBAL_GAIN_BUFFER)
        max_filters = self._parse_int(self.max_filter_entry.get(), DEFAULT_MAX_FILTERS)

        if len(self.create_filters) > max_filters:
            self._confirm_filter_overflow(slot, preamp, buffer_db, max_filters, then=then)
            return

        self._do_push(slot, preamp, buffer_db, max_filters, then=then)

    def _confirm_filter_overflow(self, slot, preamp, buffer_db, max_filters, then=None):
        dlg = tk.Toplevel(self)
        dlg.title("Too Many Bands")
        dlg.configure(bg=THEME["chassis"])
        dlg.transient(self)
        dlg.resizable(False, False)

        msg = (
            f"You have {len(self.create_filters)} EQ bands, but the device is "
            f"set to support only {max_filters} filter slot(s) (see 'Max filters').\n\n"
            f"Writing more bands than your hardware supports will only write the "
            f"first {max_filters} band(s) to the device — the rest will be silently "
            f"dropped.\n\n"
            f"Reduce your EQ to {max_filters} band(s), correct 'Max filters' to "
            f"match your device, or push anyway (will break your EQ). Filters with 0 gain are ignored and will not be written to the device."
        )
        ttk.Label(dlg, text=msg, wraplength=420, justify="left").pack(
            padx=24, pady=(20, 16))

        row = ttk.Frame(dlg)
        row.pack(padx=16, pady=(0, 18))

        def choose(action):
            dlg.destroy()
            if action == "push":
                self._do_push(slot, preamp, buffer_db, max_filters, then=then)
            # "cancel" just closes the dialog

        cancel_btn = ttk.Button(row, text="Cancel", command=lambda: choose("cancel"))
        cancel_btn.pack(side="left", padx=4)
        ttk.Button(row, text="Push Anyway", style="Danger.TButton",
                   command=lambda: choose("push")).pack(side="left", padx=4)

        dlg.bind("<Escape>", lambda _e: choose("cancel"))
        dlg.protocol("WM_DELETE_WINDOW", dlg.destroy)
        dlg.grab_set()
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        cancel_btn.focus_set()

    def _do_push(self, slot, preamp, buffer_db, max_filters, then=None):
        # Pad with inert (gain-0) dummy bands up to the device's slot count, so
        # the device doesn't backfill the unused tail slots with copies of the
        # last real band.
        filters = [dict(f) for f in self.create_filters]
        while len(filters) < max_filters:
            filters.append(dict(INERT_FILTER))

        def task():
            dev = self._open_selected_device()
            try:
                push_to_device(dev, slot=slot, global_gain=preamp, filters=filters,
                               buffer_db=buffer_db, write_gain=True)
                enable_peq(dev, True, slot_id=slot)
                print(f"Created EQ pushed to device on slot {slot} "
                      f"({len(filters)} slots)")
            finally:
                dev.close()
            if then is not None:
                self.after(0, then)  # only reached when the push succeeded

        self._run_bg(task)

    def _create_save_profile(self, then=None):
        if not self.create_filters:
            messagebox.showwarning("No Filters", "Add at least one EQ band first.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not path:
            return

        preamp = self._parse_float(self.create_preamp_entry.get(), 0.0)
        try:
            save_profile(path, preamp, self.create_filters)
            self._log(f"Profile saved to {path}\n")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))
            return
        if then is not None:
            then()

    def _create_load_profile(self):
        path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not path:
            return

        try:
            data = load_profile(path)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            return

        self._snapshot()
        loaded = [
            {
                "type": f.get("type", "PK"),
                "freq": float(f["freq"]) or 1000.0,
                "gain": float(f["gain"]),
                "q": float(f["q"]) or 1.0,
            }
            for f in data["filters"]
            if not f.get("disabled", is_filter_disabled(
                f.get("type", "PK"), f["freq"], f["gain"], f["q"]))
        ]
        self.create_filters = dedupe_filters(loaded)
        self.selected_filter = 0 if self.create_filters else -1

        self.create_preamp_entry.delete(0, "end")
        self.create_preamp_entry.insert(0, str(data["preamp"]))

        self._refresh_create_tab()
        self._load_selected_filter_into_editor()
        self._log(f"Loaded {len(self.create_filters)} filter(s) from {path}\n")

    # ------------------------------------------------------------------
    # AutoEQ
    # ------------------------------------------------------------------

    def _show_busy_dialog(self, title, message):
        """Modal indeterminate-progress dialog for a background task.

        Caller is responsible for calling .destroy() on the returned Toplevel
        (from the main thread, e.g. via self.after(0, ...)) once done.
        """
        dlg = tk.Toplevel(self)
        dlg.title(title)
        dlg.configure(bg=THEME["chassis"])
        dlg.transient(self)
        dlg.resizable(False, False)

        ttk.Label(dlg, text=message, wraplength=320, justify="left").pack(
            padx=24, pady=(20, 10))
        bar = ttk.Progressbar(dlg, mode="indeterminate", length=280)
        bar.pack(padx=24, pady=(0, 20))
        bar.start(12)

        dlg.protocol("WM_DELETE_WINDOW", lambda: None)  # not user-closable
        dlg.grab_set()
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        return dlg

    def _create_autoeq(self):
        self._pick_autoeq_model(self._on_autoeq_model_chosen)

    def _on_autoeq_model_chosen(self, measurement_path):
        if not measurement_path:
            return

        try:
            measurement = parse_frequency_response_file(measurement_path)
        except Exception as e:
            messagebox.showerror("AutoEQ", f"Could not read measurement file:\n{e}")
            return

        self._pick_autoeq_target(
            lambda target_points: self._run_autoeq(measurement, target_points))

    def _pick_autoeq_target(self, callback):
        """Ask flat vs. a target searched from AutoEQ's online library vs. a
        local file. Eventually calls callback(target_points_or_None); None
        means flat. Silently does nothing further if the user cancels.
        """
        choice = self._prompt_for_target_source()
        if choice is None:
            return
        if choice == "flat":
            callback(None)
            return
        if choice == "online":
            def after_fetch():
                path = self._show_target_search_dialog()
                if not path:
                    return
                try:
                    callback(parse_frequency_response_file(path))
                except Exception as e:
                    messagebox.showerror("AutoEQ", f"Could not read target file:\n{e}")
            self._fetch_autoeq_targets(after_fetch)
            return

        # choice == "file"
        target_path = filedialog.askopenfilename(
            title="Select Target Curve File (freq, dB per line)",
            filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")])
        if not target_path:
            return
        try:
            callback(parse_frequency_response_file(target_path))
        except Exception as e:
            messagebox.showerror("AutoEQ", f"Could not read target file:\n{e}")

    def _prompt_for_target_source(self):
        """Ask 'flat' vs. 'online target library' vs. 'local file'. Returns
        'flat'/'online'/'file'/None."""
        dlg = tk.Toplevel(self)
        dlg.title("AutoEQ Target")
        dlg.configure(bg=THEME["chassis"])
        dlg.transient(self)
        dlg.resizable(False, False)

        ttk.Label(dlg, wraplength=380, justify="left", text=(
            "Choose the target curve AutoEQ should reshape your measurement "
            "towards."
        )).pack(padx=24, pady=(20, 16))

        result = {"choice": None}
        row = ttk.Frame(dlg)
        row.pack(padx=16, pady=(0, 18))

        def choose(c):
            result["choice"] = c
            dlg.destroy()

        ttk.Button(row, text="Flat (0 dB)", command=lambda: choose("flat")).pack(
            side="left", padx=4)
        ttk.Button(row, text="Search AutoEQ Targets...", style="Accent.TButton",
                   command=lambda: choose("online")).pack(side="left", padx=4)
        ttk.Button(row, text="Load Target File...",
                   command=lambda: choose("file")).pack(side="left", padx=4)
        ttk.Button(row, text="Cancel", command=lambda: choose(None)).pack(
            side="left", padx=4)

        dlg.bind("<Escape>", lambda _e: choose(None))
        dlg.protocol("WM_DELETE_WINDOW", lambda: choose(None))
        dlg.grab_set()
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        self.wait_window(dlg)
        return result["choice"]

    def _fetch_autoeq_targets(self, on_ready):
        """Fetch (and memoize for this session) AutoEQ's target-curve list."""
        if self._autoeq_target_index is not None:
            on_ready()
            return

        busy = self._show_busy_dialog(
            "AutoEQ Targets", "Fetching target curve list from GitHub...")

        def task():
            print("Fetching AutoEQ target curve list from GitHub...")
            try:
                index = fetch_autoeq_targets_index()
            except Exception as e:
                def fail():
                    busy.destroy()
                    messagebox.showerror(
                        "AutoEQ Targets", f"Could not fetch the target list:\n{e}")
                self.after(0, fail)
                return

            print(f"Fetched {len(index)} target curve(s).")

            def apply():
                busy.destroy()
                self._autoeq_target_index = index
                on_ready()
            self.after(0, apply)

        self._run_bg(task)

    def _show_target_search_dialog(self):
        c = THEME
        dlg = tk.Toplevel(self)
        dlg.title("Search AutoEQ Targets")
        dlg.configure(bg=c["chassis"])
        dlg.transient(self)
        dlg.geometry("620x440")
        dlg.minsize(560, 340)

        result = {"path": None}
        filtered = []

        top = ttk.Frame(dlg)
        top.pack(fill="x", padx=12, pady=(12, 6))
        ttk.Label(top, text="Search:").pack(side="left")
        search_var = tk.StringVar()
        entry = ttk.Entry(top, textvariable=search_var)
        entry.pack(side="left", fill="x", expand=True, padx=(6, 0))

        list_frame = ttk.Frame(dlg)
        list_frame.pack(fill="both", expand=True, padx=12, pady=6)
        listbox = tk.Listbox(
            list_frame, bg=c["panel"], fg=c["ink"],
            selectbackground=c["accent"], selectforeground=c["chassis"],
            highlightthickness=1, highlightbackground=c["line"],
            borderwidth=0, activestyle="none", font=(self.font_ui, 10))
        listbox.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=listbox.yview)
        scrollbar.pack(side="right", fill="y")
        listbox.configure(yscrollcommand=scrollbar.set)

        status_var = tk.StringVar()
        ttk.Label(dlg, textvariable=status_var, style="Muted.TLabel").pack(
            anchor="w", padx=12)

        def refresh_list(*_args):
            query = search_var.get().strip().lower()
            listbox.delete(0, "end")
            nonlocal filtered
            if query:
                terms = query.split()
                filtered = [
                    e for e in self._autoeq_target_index
                    if all(t in e["label"].lower() for t in terms)
                ]
            else:
                filtered = list(self._autoeq_target_index)
            for e in filtered[:300]:
                listbox.insert("end", e["label"])
            status_var.set(
                f"{len(filtered)} match(es)"
                + (" (showing first 300)" if len(filtered) > 300 else ""))

        def choose(_event=None):
            sel = listbox.curselection()
            if not sel or sel[0] >= len(filtered):
                return
            entry_data = filtered[sel[0]]

            select_btn.configure(state="disabled")
            status_var.set(f"Downloading {entry_data['label']}...")

            def task():
                try:
                    local_path = fetch_autoeq_remote_file(entry_data["path"])
                except Exception as e:
                    def fail():
                        messagebox.showerror(
                            "AutoEQ", f"Could not download target:\n{e}")
                        select_btn.configure(state="normal")
                        refresh_list()
                    self.after(0, fail)
                    return

                def done():
                    result["path"] = local_path
                    dlg.destroy()
                self.after(0, done)

            self._run_bg(task)

        def cancel():
            dlg.destroy()

        search_var.trace_add("write", refresh_list)
        listbox.bind("<Double-Button-1>", choose)
        entry.bind("<Return>", lambda _e: choose() if filtered else None)
        entry.bind("<Down>", lambda _e: (listbox.focus_set(), listbox.selection_set(0)))
        dlg.bind("<Escape>", lambda _e: cancel())

        row = ttk.Frame(dlg)
        row.pack(fill="x", padx=12, pady=(6, 12))
        ttk.Button(row, text="Cancel", command=cancel).pack(side="right", padx=4)
        select_btn = ttk.Button(row, text="Select", style="Accent.TButton", command=choose)
        select_btn.pack(side="right", padx=4)

        refresh_list()
        if filtered:
            listbox.selection_set(0)

        dlg.protocol("WM_DELETE_WINDOW", cancel)
        dlg.grab_set()
        entry.focus_set()

        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")

        self.wait_window(dlg)
        return result["path"]

    def _run_autoeq(self, measurement, target_points):
        max_filters = self._parse_int(self.max_filter_entry.get(), DEFAULT_MAX_FILTERS)

        busy = self._show_busy_dialog(
            "AutoEQ",
            "Running AutoEQ optimization...\nThis can take up to a minute "
            "depending on the number of filters.")

        def task():
            print("Running AutoEQ optimization, this may take a while...")
            try:
                freqs = autoeq_raw_frequencies()
                fr = autoeq_interp(freqs, measurement)
                fr_target = (
                    [(f, 0.0) for f in freqs] if target_points is None
                    else autoeq_interp(freqs, target_points))

                # Loudness-normalize both curves before comparing them (matches
                # hangout.audio's pipeline): otherwise two curves that each use
                # a different absolute dB reference convention — as different
                # measurement sources/rigs do — get compared directly, and the
                # optimizer chases a systematic level offset instead of shape.
                fr = autoeq_loudness_normalize(fr)
                fr_target = autoeq_loudness_normalize(fr_target)

                filters = autoeq_run(fr, fr_target, max_filters)
                fr_eq = autoeq_apply(fr, filters)
                preamp = autoeq_calc_preamp(fr, fr_eq)
            except Exception:
                self.after(0, busy.destroy)
                raise
            print(f"AutoEQ generated {len(filters)} band(s), preamp {preamp:.1f} dB")

            def apply_result():
                busy.destroy()
                self._snapshot()
                self.create_filters = [
                    {
                        "type": f["type"],
                        "freq": round(f["freq"], 1),
                        "gain": round(f["gain"], 2),
                        "q": round(f["q"], 3),
                    }
                    for f in filters
                ]
                self.selected_filter = 0 if self.create_filters else -1
                self.create_preamp_entry.delete(0, "end")
                self.create_preamp_entry.insert(0, f"{preamp:.1f}")
                self._refresh_create_tab()
                self._load_selected_filter_into_editor()

            self.after(0, apply_result)

        self._run_bg(task)

    def _pick_autoeq_model(self, callback):
        """Search a measurement database by model name, à la autoeq.app.

        Eventually calls `callback(measurement_path_or_None)` — asynchronously
        when fetching the online database, since that hits the network.
        """
        if self._autoeq_model_index is not None:
            callback(self._show_model_search_dialog())
            return

        saved = load_autoeq_db_path()
        if saved == "online":
            self._fetch_online_index(lambda: callback(self._show_model_search_dialog()))
            return
        if saved and os.path.isdir(saved):
            self._ensure_local_autoeq_index(saved)
            callback(self._show_model_search_dialog())
            return

        choice = self._prompt_for_autoeq_source()
        if choice is None:
            callback(None)
            return
        if choice == "online":
            self._fetch_online_index(lambda: callback(self._show_model_search_dialog()))
            return

        chosen = filedialog.askdirectory(title="Select Measurement Database Folder")
        if not chosen:
            callback(None)
            return
        save_autoeq_db_path(chosen)
        self._ensure_local_autoeq_index(chosen)
        callback(self._show_model_search_dialog())

    def _prompt_for_autoeq_source(self):
        """Ask 'online database' vs 'local folder'. Returns 'online'/'local'/None."""
        dlg = tk.Toplevel(self)
        dlg.title("AutoEQ Model Database")
        dlg.configure(bg=THEME["chassis"])
        dlg.transient(self)
        dlg.resizable(False, False)

        ttk.Label(dlg, wraplength=380, justify="left", text=(
            "No measurement database is configured yet.\n\n"
            "Fetch the online AutoEQ database from GitHub "
            "(jaakkopasanen/AutoEq), or point to a local folder of "
            "measurement .txt files instead."
        )).pack(padx=24, pady=(20, 16))

        result = {"choice": None}
        row = ttk.Frame(dlg)
        row.pack(padx=16, pady=(0, 18))

        def choose(c):
            result["choice"] = c
            dlg.destroy()

        ttk.Button(row, text="Download Online Database", style="Accent.TButton",
                   command=lambda: choose("online")).pack(side="left", padx=4)
        ttk.Button(row, text="Choose Local Folder...",
                   command=lambda: choose("local")).pack(side="left", padx=4)
        ttk.Button(row, text="Cancel", command=lambda: choose(None)).pack(
            side="left", padx=4)

        dlg.bind("<Escape>", lambda _e: choose(None))
        dlg.protocol("WM_DELETE_WINDOW", lambda: choose(None))
        dlg.grab_set()
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        self.wait_window(dlg)
        return result["choice"]

    def _ensure_local_autoeq_index(self, db_path):
        self._autoeq_db_path = db_path
        if self._autoeq_model_index is None or self._autoeq_model_index_path != db_path:
            self._autoeq_model_index = build_autoeq_model_index(db_path)
            self._autoeq_model_index_path = db_path

    def _fetch_online_index(self, on_ready):
        """Fetch the online index in the background; calls on_ready() when set."""
        busy = self._show_busy_dialog(
            "AutoEQ Model Database", "Fetching model list from GitHub...")

        def task():
            print("Fetching AutoEQ database listing from GitHub...")
            try:
                index = fetch_autoeq_online_index()
            except Exception as e:
                def fail():
                    busy.destroy()
                    messagebox.showerror(
                        "AutoEQ Model Database",
                        f"Could not fetch the online database:\n{e}")
                self.after(0, fail)
                return

            print(f"Fetched {len(index)} model(s) from the online database.")

            def apply():
                busy.destroy()
                self._autoeq_model_index = index
                self._autoeq_model_index_path = "online"
                self._autoeq_db_path = "online"
                save_autoeq_db_path("online")
                on_ready()
            self.after(0, apply)

        self._run_bg(task)

    def _show_model_search_dialog(self):
        c = THEME
        dlg = tk.Toplevel(self)
        dlg.title("Search Headphone Model")
        dlg.configure(bg=c["chassis"])
        dlg.transient(self)
        dlg.geometry("700x480")
        dlg.minsize(650, 360)

        result = {"path": None}
        filtered = []

        top = ttk.Frame(dlg)
        top.pack(fill="x", padx=12, pady=(12, 6))
        ttk.Label(top, text="Search:").pack(side="left")
        search_var = tk.StringVar()
        entry = ttk.Entry(top, textvariable=search_var)
        entry.pack(side="left", fill="x", expand=True, padx=(6, 0))

        list_frame = ttk.Frame(dlg)
        list_frame.pack(fill="both", expand=True, padx=12, pady=6)
        listbox = tk.Listbox(
            list_frame, bg=c["panel"], fg=c["ink"],
            selectbackground=c["accent"], selectforeground=c["chassis"],
            highlightthickness=1, highlightbackground=c["line"],
            borderwidth=0, activestyle="none", font=(self.font_ui, 10))
        listbox.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=listbox.yview)
        scrollbar.pack(side="right", fill="y")
        listbox.configure(yscrollcommand=scrollbar.set)

        status_var = tk.StringVar()
        ttk.Label(dlg, textvariable=status_var, style="Muted.TLabel").pack(
            anchor="w", padx=12)

        def refresh_list(*_args):
            query = search_var.get().strip().lower()
            listbox.delete(0, "end")
            nonlocal filtered
            if query:
                terms = query.split()
                filtered = [
                    e for e in self._autoeq_model_index
                    if all(t in e["label"].lower() or t in e["subtitle"].lower()
                           for t in terms)
                ]
            else:
                filtered = list(self._autoeq_model_index)
            for e in filtered[:300]:
                listbox.insert("end", f"{e['label']}   [{e['subtitle']}]")
            status_var.set(
                f"{len(filtered)} match(es) in {os.path.basename(self._autoeq_db_path)}"
                + (" (showing first 300)" if len(filtered) > 300 else ""))

        def choose(_event=None):
            sel = listbox.curselection()
            if not sel or sel[0] >= len(filtered):
                return
            entry_data = filtered[sel[0]]

            if not entry_data.get("remote"):
                result["path"] = entry_data["path"]
                dlg.destroy()
                return

            select_btn.configure(state="disabled")
            status_var.set(f"Downloading {entry_data['label']}...")

            def task():
                try:
                    local_path = fetch_autoeq_remote_file(entry_data["path"])
                except Exception as e:
                    def fail():
                        messagebox.showerror(
                            "AutoEQ", f"Could not download measurement:\n{e}")
                        select_btn.configure(state="normal")
                        refresh_list()
                    self.after(0, fail)
                    return

                def done():
                    result["path"] = local_path
                    dlg.destroy()
                self.after(0, done)

            self._run_bg(task)

        def cancel():
            dlg.destroy()

        def browse_file():
            path = filedialog.askopenfilename(
                title="Select Measurement File (freq, dB per line)",
                filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")])
            if path:
                result["path"] = path
                dlg.destroy()

        def change_db():
            choice = self._prompt_for_autoeq_source()
            if choice is None:
                return
            if choice == "online":
                status_var.set("Fetching online database...")
                self._fetch_online_index(refresh_list)
                return
            chosen = filedialog.askdirectory(title="Select Measurement Database Folder")
            if not chosen:
                return
            save_autoeq_db_path(chosen)
            self._ensure_local_autoeq_index(chosen)
            refresh_list()

        search_var.trace_add("write", refresh_list)
        listbox.bind("<Double-Button-1>", choose)
        entry.bind("<Return>", lambda _e: choose() if filtered else None)
        entry.bind("<Down>", lambda _e: (listbox.focus_set(), listbox.selection_set(0)))
        dlg.bind("<Escape>", lambda _e: cancel())

        row = ttk.Frame(dlg)
        row.pack(fill="x", padx=12, pady=(6, 12))
        ttk.Button(row, text="Browse File Instead...", command=browse_file).pack(
            side="left")
        ttk.Button(row, text="Change Database...", command=change_db).pack(
            side="left", padx=(6, 0))
        ttk.Button(row, text="Cancel", command=cancel).pack(side="right", padx=4)
        select_btn = ttk.Button(row, text="Select", style="Accent.TButton", command=choose)
        select_btn.pack(side="right", padx=4)

        refresh_list()
        # First result pre-selected so Enter works immediately.
        if filtered:
            listbox.selection_set(0)

        dlg.protocol("WM_DELETE_WINDOW", cancel)
        dlg.grab_set()
        entry.focus_set()

        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")

        self.wait_window(dlg)
        return result["path"]

    # ------------------------------------------------------------------
    # Device discovery
    # ------------------------------------------------------------------

    def _refresh_devices(self):
        self.device_list.delete(0, "end")
        self._devices_cache = []

        if hid is None:
            self._log("hidapi not available; cannot list devices.\n")
            return

        found = [d for d in hid.enumerate() if d["vendor_id"] == WALKPLAY_VENDOR_ID]
        if not found:
            self.device_list.insert("end", "(no Walkplay-vendor devices found)")

        for d in found:
            self.device_list.insert(
                "end",
                f"pid=0x{d['product_id']:04X} iface={d.get('interface_number')}  "
                f"{d.get('product_string')}")
            self._devices_cache.append(d)

    def _on_device_select(self, _event):
        sel = self.device_list.curselection()
        if not sel or not self._devices_cache:
            return
        idx = sel[0]
        if idx >= len(self._devices_cache):
            return

        d = self._devices_cache[idx]
        self.selected_path = d["path"]

        self.vid_entry.delete(0, "end")
        self.vid_entry.insert(0, f"0x{d['vendor_id']:04X}")
        self.pid_entry.delete(0, "end")
        self.pid_entry.insert(0, f"0x{d['product_id']:04X}")

    # ------------------------------------------------------------------
    # Tooltips
    # ------------------------------------------------------------------

    def _add_tooltip(self, widget, text):
        """Show a small hover tooltip (used for keyboard-shortcut hints)."""
        state = {"win": None}

        def show(_e=None):
            if state["win"] is not None or not text:
                return
            x = widget.winfo_rootx() + 10
            y = widget.winfo_rooty() + widget.winfo_height() + 4
            win = tk.Toplevel(self)
            win.wm_overrideredirect(True)
            win.wm_geometry(f"+{x}+{y}")
            tk.Label(win, text=text, bg=THEME["input"], fg=THEME["ink"],
                     font=(self.font_ui, 9), padx=6, pady=2,
                     highlightthickness=1, highlightbackground=THEME["line"]).pack()
            state["win"] = win

        def hide(_e=None):
            if state["win"] is not None:
                state["win"].destroy()
                state["win"] = None

        widget.bind("<Enter>", show, add="+")
        widget.bind("<Leave>", hide, add="+")
        widget.bind("<Destroy>", hide, add="+")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _on_close(self):
        dlg = tk.Toplevel(self)
        dlg.title("Quit")
        dlg.configure(bg=THEME["chassis"])
        dlg.transient(self)
        dlg.resizable(False, False)

        ttk.Label(dlg, text="Do you really want to leave this application?").pack(
            padx=24, pady=(20, 16))

        row = ttk.Frame(dlg)
        row.pack(padx=16, pady=(0, 18))

        def choose(action):
            dlg.destroy()
            if action == "quit":
                self.destroy()
            elif action == "push":
                self._create_push(then=self.destroy)
            elif action == "save":
                self._create_save_profile(then=self.destroy)
            # "cancel" just closes the dialog

        quit_btn = ttk.Button(row, text="Quit", style="Danger.TButton",
                              command=lambda: choose("quit"))
        quit_btn.pack(side="left", padx=4)
        ttk.Button(row, text="Cancel",
                   command=lambda: choose("cancel")).pack(side="left", padx=4)
        ttk.Button(row, text="Push to device and quit",
                   command=lambda: choose("push")).pack(side="left", padx=4)
        ttk.Button(row, text="Save to file and quit",
                   command=lambda: choose("save")).pack(side="left", padx=4)

        # Enter = Quit, Escape = Cancel.
        dlg.bind("<Return>", lambda _e: choose("quit"))
        dlg.bind("<KP_Enter>", lambda _e: choose("quit"))
        dlg.bind("<Escape>", lambda _e: choose("cancel"))

        dlg.protocol("WM_DELETE_WINDOW", dlg.destroy)  # dialog's own X = cancel
        dlg.grab_set()
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        quit_btn.focus_set()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, text):
        self.log_queue.put(text)

    def _poll_log_queue(self):
        while True:
            try:
                text = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.configure(state="normal")
            self.log_text.insert("end", text)
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        self.after(100, self._poll_log_queue)

    # ------------------------------------------------------------------
    # Parsing helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_int(s, default=None):
        s = (s or "").strip()
        if not s:
            return default
        try:
            return int(s, 0)
        except ValueError:
            return default

    @staticmethod
    def _parse_float(s, default=None):
        s = (s or "").strip().replace(",", ".")
        if not s:
            return default
        try:
            return float(s)
        except ValueError:
            return default

    # ------------------------------------------------------------------
    # Device helpers / background tasks
    # ------------------------------------------------------------------

    def _open_selected_device(self):
        if hid is None:
            raise RuntimeError("hidapi not installed (pip install hidapi)")
        vid = self._parse_int(self.vid_entry.get(), WALKPLAY_VENDOR_ID)
        pid = self._parse_int(self.pid_entry.get(), None)
        return open_device(vid=vid, pid=pid, path=self.selected_path)

    def _run_bg(self, fn):
        def target():
            old_stdout, old_stderr = sys.stdout, sys.stderr
            sys.stdout = sys.stderr = StdoutRedirector(self.log_queue)
            try:
                fn()
            except Exception as e:
                self._log(f"\nERROR: {e}\n")
            finally:
                sys.stdout, sys.stderr = old_stdout, old_stderr

        threading.Thread(target=target, daemon=True).start()

    def _get_slot(self):
        def task():
            dev = self._open_selected_device()
            try:
                get_current_slot(dev)
            finally:
                dev.close()

        self._run_bg(task)

    def _enable(self):
        slot = self._parse_int(self.ed_slot_spin.get(), 0)

        def task():
            dev = self._open_selected_device()
            try:
                enable_peq(dev, True, slot_id=slot)
                print(f"PEQ enabled on slot {slot}")
            finally:
                dev.close()

        self._run_bg(task)

    def _disable(self):
        def task():
            dev = self._open_selected_device()
            try:
                enable_peq(dev, False)
                print("PEQ disabled")
            finally:
                dev.close()

        self._run_bg(task)


# ===========================================================================
# CLI
# ===========================================================================

def _cli_push(args):
    vid = int(args.vid, 0) if args.vid else WALKPLAY_VENDOR_ID
    pid = int(args.pid, 0) if args.pid else None

    profile = load_profile(args.file)
    filters = [
        f for f in profile["filters"]
        if not f.get("disabled", is_filter_disabled(
            f.get("type", "PK"), f["freq"], f["gain"], f["q"]))
    ]

    dev = open_device(vid=vid, pid=pid)
    try:
        push_to_device(dev, slot=args.slot, global_gain=profile["preamp"],
                       filters=filters, buffer_db=args.buffer,
                       write_gain=not args.no_gain)
        if not args.no_enable:
            enable_peq(dev, True, slot_id=args.slot)
            print(f"PEQ enabled on slot {args.slot}")
    finally:
        dev.close()


def _cli_pull(args):
    vid = int(args.vid, 0) if args.vid else WALKPLAY_VENDOR_ID
    pid = int(args.pid, 0) if args.pid else None

    dev = open_device(vid=vid, pid=pid)
    try:
        slot = get_current_slot(dev)
        result = pull_from_device(dev, max_filters=args.max_filters, slot_hint=slot)
        save_profile(args.file, result["globalGain"], result["filters"])
        print(f"Saved {len(result['filters'])} filter(s) to {args.file}")
    finally:
        dev.close()


def _cli_list(_args):
    list_devices()


def _build_parser():
    import argparse

    p = argparse.ArgumentParser(
        prog="eqloader",
        description="Walkplay PEQ loader — run without arguments to open the GUI.")

    graph_grp = p.add_mutually_exclusive_group()
    graph_grp.add_argument(
        "--graph-no-hide", dest="graph", action="store_true", default=None,
        help="Always show the frequency-response graph (GUI only)")
    graph_grp.add_argument(
        "--no-graph", dest="graph", action="store_false",
        help="Always hide the frequency-response graph (GUI only)")

    sub = p.add_subparsers(dest="cmd")

    pp = sub.add_parser("push", help="Push a .txt profile to the device")
    pp.add_argument("file", help="Profile .txt file to push")
    pp.add_argument("--slot", type=int, default=0, help="Target PEQ slot (default: 0)")
    pp.add_argument("--buffer", type=float, default=DEFAULT_GLOBAL_GAIN_BUFFER,
                    help=f"Hardware gain buffer in dB (default: {DEFAULT_GLOBAL_GAIN_BUFFER})")
    pp.add_argument("--no-gain", action="store_true",
                    help="Skip writing the global gain register")
    pp.add_argument("--no-enable", action="store_true",
                    help="Don't enable PEQ after pushing")
    pp.add_argument("--vid", default=None, help="Device VID in hex (default: 0x3302)")
    pp.add_argument("--pid", default=None, help="Device PID in hex (optional)")

    pu = sub.add_parser("pull", help="Pull the current EQ from the device to a .txt file")
    pu.add_argument("file", help="Output .txt file")
    pu.add_argument("--max-filters", type=int, default=DEFAULT_MAX_FILTERS,
                    help=f"Number of filter slots to read (default: {DEFAULT_MAX_FILTERS})")
    pu.add_argument("--vid", default=None, help="Device VID in hex (default: 0x3302)")
    pu.add_argument("--pid", default=None, help="Device PID in hex (optional)")

    sub.add_parser("list", help="List connected Walkplay HID devices")

    return p


if __name__ == "__main__":
    args = _build_parser().parse_args()

    if args.cmd == "push":
        _cli_push(args)
    elif args.cmd == "pull":
        _cli_pull(args)
    elif args.cmd == "list":
        _cli_list(args)
    else:
        app = EqLoaderGUI(graph=args.graph)
        app.mainloop()
