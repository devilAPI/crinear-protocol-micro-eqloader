#!/usr/bin/env python3
"""Walkplay PEQ loader.

Edit, push and pull parametric EQ on Walkplay-based USB DAC dongles (e.g. the
Crinear Protocol Micro), with an AutoEQ optimizer backed by the public AutoEq
measurement database. Run without arguments for the GUI; see --help for the CLI.

Sections, bottom-up:
    device protocol -> profile files -> filter math -> AutoEQ engine
    -> AutoEQ database -> GUI -> CLI

Throughout, a band ("filter") is a dict {"type", "freq", "gain", "q"} with
type one of FILTER_TYPES, and a curve is a list of (freq, dB) tuples.
"""

import argparse
import contextlib
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
import numpy as np

try:
    import hid
except ImportError:
    hid = None


# ===========================================================================
# Device protocol (Walkplay HID)
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

FILTER_TYPES = ("PK", "LSQ", "HSQ", "LP", "HP")
FILTER_TYPE_TO_BYTE = {"LSQ": 1, "PK": 2, "HSQ": 3, "LP": 4, "HP": 5}
BYTE_TO_FILTER_TYPE = {v: k for k, v in FILTER_TYPE_TO_BYTE.items()}

DEFAULT_GLOBAL_GAIN_BUFFER = -5
DEFAULT_MAX_FILTERS = 8
# Rate the device's biquad coefficients are designed at (see compute_iir_filter).
DEVICE_SAMPLE_RATE = 96000

# How the device stores an unused band; also used to pad pushes.
INERT_FILTER = {"type": "PK", "freq": 100.0, "gain": 0.0, "q": 1.0}


def _require_hid():
    if hid is None:
        raise RuntimeError("hidapi not installed (pip install hidapi)")


def find_devices(vid=WALKPLAY_VENDOR_ID):
    _require_hid()
    return [d for d in hid.enumerate() if d["vendor_id"] == vid]


def list_devices():
    devices = find_devices()
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
    """Open by HID path, else by VID/PID, else the first device with `vid`."""
    _require_hid()
    dev = hid.device()
    if path:
        dev.open_path(path)
        return dev

    vid = vid or WALKPLAY_VENDOR_ID
    if pid:
        dev.open(vid, pid)
        return dev

    candidates = find_devices(vid)
    if not candidates:
        raise RuntimeError(
            f"No HID device found with vendor id 0x{vid:04X}. "
            f"Refresh the device list, or set VID/PID manually."
        )
    if len(candidates) > 1:
        names = ", ".join(
            f"0x{d['product_id']:04X} ({d.get('product_string')})" for d in candidates)
        print(
            f"Warning: multiple Walkplay devices/interfaces found ({names}). "
            f"Using the first one. Select a specific device in the list, or set PID."
        )
    dev.open_path(candidates[0]["path"])
    return dev


@contextlib.contextmanager
def device_session(vid=None, pid=None, path=None):
    """`with device_session(...) as dev:` — opens the device, always closes it."""
    dev = open_device(vid=vid, pid=pid, path=path)
    try:
        yield dev
    finally:
        dev.close()


def send_report(dev, packet):
    payload = bytes(packet[:REPORT_LENGTH]).ljust(REPORT_LENGTH, b"\0")
    dev.write(bytes([REPORT_ID]) + payload)


def _read_report(dev, timeout_ms=200):
    data = dev.read(REPORT_LENGTH + 1, timeout_ms)
    return data[1:] if data else None


def wait_for_response(dev, expected_cmd, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining_ms = max(1, int((deadline - time.time()) * 1000))
        data = _read_report(dev, timeout_ms=min(200, remaining_ms))
        if data is not None and len(data) > 1 and data[1] == expected_cmd:
            return data
    raise TimeoutError(f"Timeout waiting for response to cmd 0x{expected_cmd:02X}")


def _le_bytes(value, length):
    value = int(round(value))
    return [(value >> (8 * i)) & 0xFF for i in range(length)]


def compute_iir_filter(freq, gain, q):
    """The 20 coefficient bytes of one band, as the device expects them.

    RBJ peaking biquad at DEVICE_SAMPLE_RATE (always a peaking design,
    whatever the band's type, matching the vendor tool), as Q2.30 fixed point
    words b0, b1, b2, -a1, -a2, little-endian.
    """
    amp = math.sqrt(10 ** (gain / 20))
    w0 = (freq * (2 * math.pi)) / DEVICE_SAMPLE_RATE
    alpha = math.sin(w0) / (2 * q)
    a0 = alpha / amp + 1
    mid = (math.cos(w0) * -2) / a0
    b0 = (alpha * amp + 1) / a0
    b2 = (1 - alpha * amp) / a0
    a2 = (1 - alpha / amp) / a0

    def q30(x):
        return round(x * (1 << 30))

    words = [q30(b0), q30(mid), q30(b2), -q30(mid), -q30(a2)]
    return [byte for w in words for byte in (w & 0xFFFFFFFF).to_bytes(4, "little")]


def build_filter_packet(index, f, slot):
    """PEQ_VALUES write packet for band `index` of `slot` (parse_filter_packet inverts it)."""
    return (
        [WRITE, CMD["PEQ_VALUES"], 0x18, 0x00, index, 0x00, 0x00]
        + compute_iir_filter(f["freq"], f["gain"], f["q"])
        + _le_bytes(f["freq"], 2)
        + _le_bytes(round(f["q"] * 256), 2)
        + _le_bytes(round(f["gain"] * 256) & 0xFFFF, 2)
        + [FILTER_TYPE_TO_BYTE.get(f.get("type", "PK"), 2), 0x00, slot, END]
    )


def parse_filter_packet(packet):
    freq = packet[27] | (packet[28] << 8)
    q = round(((packet[29] | (packet[30] << 8)) / 256) * 100) / 100

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


def get_current_slot(dev):
    send_report(dev, [READ, CMD["VERSION"], END])
    resp = wait_for_response(dev, CMD["VERSION"])
    version = bytes(resp[3:6]).decode("ascii", errors="ignore")
    print(f"Firmware version: {version!r}")

    send_report(dev, [READ, CMD["PEQ_VALUES"], END])
    resp = wait_for_response(dev, CMD["PEQ_VALUES"])
    slot = resp[35] if len(resp) > 35 else -1
    print(f"Current EQ slot: {slot}")
    return slot


def write_global_gain(dev, value_db):
    send_report(dev, [WRITE, CMD["GLOBAL_GAIN"], 0x02, 0x00, round(value_db) & 0xFF])


def read_global_gain(dev):
    send_report(dev, [READ, CMD["GLOBAL_GAIN"], 0x00])
    raw = wait_for_response(dev, CMD["GLOBAL_GAIN"], timeout=1.0)[4]
    return raw - 256 if raw > 127 else raw


def preamp_to_register(preamp, buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER):
    """Gain register value (whole dB, <= 0) for `preamp`.

    The device already attenuates by the fixed `buffer_db` (-5 dB on the
    Protocol Micro); the register only holds attenuation beyond that.
    """
    return round(min(0, preamp - buffer_db))


def register_to_preamp(register, buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER):
    """Effective preamp the device applies for a gain register value (inverse
    of preamp_to_register, up to its whole-dB rounding and buffer clamp)."""
    return register + buffer_db


def push_to_device(dev, slot, global_gain, filters,
                   buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER, write_gain=True):
    """Write `filters` to `slot`, set the gain register, and flash."""
    slot = int(slot)
    for i, f in enumerate(filters):
        send_report(dev, build_filter_packet(i, f, slot))
        time.sleep(0.02)
    time.sleep(0.1)

    if write_gain:
        gain_to_write = preamp_to_register(global_gain, buffer_db)
        write_global_gain(dev, gain_to_write)
        print(
            f"Set global gain register to {gain_to_write} dB "
            f"(preamp {global_gain} dB, hardware buffer {buffer_db} dB)"
        )
        time.sleep(0.05)

    # Commit sequence as sent by the vendor tool (0x05/0x17 are undocumented).
    for packet, pause in (
        ([WRITE, 0x05, END], 0.02),
        ([WRITE, 0x17, END], 0.02),
        ([WRITE, CMD["TEMP_WRITE"], 0x04, 0x00, 0x00, 0xFF, 0xFF, END], 0.05),
    ):
        send_report(dev, packet)
        time.sleep(pause)
    send_report(dev, [WRITE, CMD["FLASH_EQ"], END])
    print(f"Pushed {len(filters)} filter(s) to slot {slot} and flashed to device.")


def pull_from_device(dev, max_filters, slot_hint=-1, timeout=10.0,
                     buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER):
    """Read `max_filters` bands and the gain register.

    Returns {"currentSlot", "globalGain" (raw register), "preamp" (effective
    preamp in dB, see register_to_preamp), "filters"}; filters are
    parse_filter_packet() dicts ordered by band index.
    """
    for i in range(max_filters):
        send_report(dev, [READ, CMD["PEQ_VALUES"], 0x00, 0x00, i, END])
        time.sleep(0.05)
    time.sleep(0.1)

    filters = {}
    deadline = time.time() + timeout
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
        preamp = register_to_preamp(global_gain, buffer_db)
        print(f"Global gain register {global_gain} dB -> preamp {preamp} dB "
              f"(hardware buffer {buffer_db} dB)")
    except TimeoutError:
        print("Warning: could not read global gain; assuming preamp 0 dB.")
        global_gain, preamp = 0, 0.0

    return {"currentSlot": slot_hint, "globalGain": global_gain, "preamp": preamp,
            "filters": [filters[i] for i in sorted(filters)]}


def enable_peq(dev, enable, slot_id=0):
    send_report(dev, [WRITE, CMD["FLASH_EQ"], 1 if enable else 0,
                      slot_id if enable else 0x00, END])


# ---------------------------------------------------------------------------
# Band-list helpers
# ---------------------------------------------------------------------------

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


def filter_is_off(f):
    """Explicit "disabled" flag if present (profiles, pulls), else inferred."""
    return f.get("disabled", is_filter_disabled(
        f.get("type", "PK"), f["freq"], f["gain"], f["q"]))


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
        if key not in seen:
            seen.add(key)
            result.append(f)
    return result


def active_filters(filters):
    """Editable bands from a loaded/pulled profile: OFF bands dropped, values
    coerced to float (a zero freq/Q replaced by a usable default) and exact
    duplicates collapsed."""
    return dedupe_filters([
        {
            "type": f.get("type", "PK"),
            "freq": float(f["freq"]) or 1000.0,
            "gain": float(f["gain"]),
            "q": float(f["q"]) or 1.0,
        }
        for f in filters if not filter_is_off(f)
    ])


def pad_for_push(filters, max_filters):
    """Copy of `filters` padded with inert bands up to the device's slot count,
    so the device doesn't backfill the unused tail slots with copies of the
    last real band."""
    padded = [dict(f) for f in filters]
    padded += [dict(INERT_FILTER) for _ in range(max_filters - len(padded))]
    return padded


# ===========================================================================
# Profile .txt format (EqualizerAPO / eq.hangout.audio)
# ===========================================================================

TXT_TYPE_TO_INTERNAL = {
    "LS": "LSQ", "LSC": "LSQ", "LSQ": "LSQ", "HS": "HSQ", "HSC": "HSQ", "HSQ": "HSQ",
    "PK": "PK", "LP": "LP", "HP": "HP",
}
INTERNAL_TYPE_TO_TXT = {"LSQ": "LS", "HSQ": "HS", "PK": "PK", "LP": "LP", "HP": "HP"}

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
    """Returns {"preamp": dB, "filters": [...]}; OFF bands come back as
    INERT_FILTER copies flagged "disabled"."""
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
                filters.append(dict(INERT_FILTER, disabled=True))
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
        state = "OFF" if filter_is_off(f) else "ON"
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
# Filter math (RBJ cookbook biquads)
# ===========================================================================

def biquad_coeffs(ftype, freq, gain, q, fs=DEVICE_SAMPLE_RATE):
    """(1, a1, a2, b0, b1, b2) normalized by a0, or None for an unknown type.

    Inputs are clamped as in AutoEq's biquad.py (the source of hangout.audio's
    equalizer.js, which the AutoEQ engine below ports).
    """
    w0 = 2 * math.pi * max(1e-6, min(freq / fs, 1))
    q = max(1e-4, min(q, 1000))
    gain = max(-40, min(gain, 40))
    sin, cos = math.sin(w0), math.cos(w0)
    a = 10 ** (gain / 40)
    alpha = sin / (2 * q)

    if ftype == "PK":
        a0, a1, a2 = 1 + alpha / a, -2 * cos, 1 - alpha / a
        b0, b1, b2 = 1 + alpha * a, -2 * cos, 1 - alpha * a
    elif ftype == "LSQ":
        am = 2 * math.sqrt(a) * alpha
        a0 = (a + 1) + (a - 1) * cos + am
        a1 = -2 * ((a - 1) + (a + 1) * cos)
        a2 = (a + 1) + (a - 1) * cos - am
        b0 = a * ((a + 1) - (a - 1) * cos + am)
        b1 = 2 * a * ((a - 1) - (a + 1) * cos)
        b2 = a * ((a + 1) - (a - 1) * cos - am)
    elif ftype == "HSQ":
        am = 2 * math.sqrt(a) * alpha
        a0 = (a + 1) - (a - 1) * cos + am
        a1 = 2 * ((a - 1) - (a + 1) * cos)
        a2 = (a + 1) - (a - 1) * cos - am
        b0 = a * ((a + 1) + (a - 1) * cos + am)
        b1 = -2 * a * ((a - 1) + (a + 1) * cos)
        b2 = a * ((a + 1) + (a - 1) * cos - am)
    elif ftype == "LP":
        a0, a1, a2 = 1 + alpha, -2 * cos, 1 - alpha
        b0, b1, b2 = (1 - cos) / 2, 1 - cos, (1 - cos) / 2
    elif ftype == "HP":
        a0, a1, a2 = 1 + alpha, -2 * cos, 1 - alpha
        b0, b1, b2 = (1 + cos) / 2, -(1 + cos), (1 + cos) / 2
    else:
        return None
    return (1.0, a1 / a0, a2 / a0, b0 / a0, b1 / a0, b2 / a0)


def biquad_phi(freqs, fs):
    """Per-frequency term of gains_db(); depends only on the grid, so hot
    loops compute it once."""
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    return 4 * np.sin(w / 2) ** 2


def gains_db(phi, coeffs):
    """Summed magnitude response (dB) of biquads `coeffs` at biquad_phi() points."""
    gains = np.zeros(len(phi))
    for a0, a1, a2, b0, b1, b2 in coeffs:
        num = (b0 + b1 + b2) ** 2 + (b0 * b2 * phi - (b1 * (b0 + b2) + 4 * b0 * b2)) * phi
        den = (a0 + a1 + a2) ** 2 + (a0 * a2 * phi - (a1 * (a0 + a2) + 4 * a0 * a2)) * phi
        gains += 10 * np.log10(np.maximum(num, 1e-12)) - 10 * np.log10(np.maximum(den, 1e-12))
    return gains


def filters_response_db(freqs, filters, fs=DEVICE_SAMPLE_RATE):
    """Summed dB response of `filters` (any band type) at `freqs`."""
    coeffs = [biquad_coeffs(f.get("type", "PK"), f["freq"], f["gain"], f["q"], fs)
              for f in filters]
    return gains_db(biquad_phi(freqs, fs), [c for c in coeffs if c is not None])


def q_to_bw(q):
    """Q -> bandwidth in octaves."""
    return 2 * math.asinh(1 / (2 * max(q, 0.001))) / math.log(2)


def bw_to_q(bw):
    """Bandwidth in octaves -> Q (raises OverflowError for absurd widths)."""
    return 1 / (2 * math.sinh(bw * math.log(2) / 2))


# ===========================================================================
# AutoEQ engine (port of hangout.audio's equalizer.js)
# ===========================================================================

AUTOEQ_CONFIG = {
    # hangout.audio uses a generic 48 kHz; model the filters exactly as the
    # device will run them instead, since response near the top of the band
    # differs noticeably between the two rates.
    "default_sample_rate": DEVICE_SAMPLE_RATE,
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

    Accepts whitespace- or comma-separated columns and ignores blank lines,
    header lines and comment lines (starting with '#', '*' or ';'), which
    covers AutoEq's CSVs and common exports such as REW's .txt files.
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
            # A single NaN would poison every distance the optimizer computes.
            if not (math.isfinite(freq) and math.isfinite(gain)) or freq <= 0:
                continue
            points.append((freq, gain))

    if not points:
        raise ValueError(f"No frequency/gain data found in {path}")
    points.sort(key=lambda p: p[0])
    return points


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
                out.append((f, v0 + (v1 - v0) * (f - f0) / (f1 - f0)))
                found = True
                break
            i += 1
        if not found:
            out.append((f, fr[-1][1]))
    return out


def autoeq_filters_to_coeffs(filters, sample_rate=None):
    """Like upstream: bands with a zero freq/gain/Q, and LP/HP, are ignored."""
    fs = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    return [biquad_coeffs(f["type"], f["freq"], f["gain"], f["q"], fs)
            for f in filters
            if f.get("freq") and f.get("gain") and f.get("q")
            and f.get("type") in ("PK", "LSQ", "HSQ")]


def autoeq_calc_gains(freqs, coeffs, sample_rate=None):
    fs = sample_rate or AUTOEQ_CONFIG["default_sample_rate"]
    return gains_db(biquad_phi(freqs, fs), coeffs)


def autoeq_apply(fr, filters, sample_rate=None):
    """Curve `fr` with `filters` applied."""
    freqs = [f for f, _ in fr]
    values = np.array([v for _, v in fr], dtype=float)
    values = values + autoeq_calc_gains(
        freqs, autoeq_filters_to_coeffs(filters, sample_rate), sample_rate)
    return list(zip(freqs, values.tolist()))


def autoeq_calc_preamp(fr1, fr2):
    """Attenuation (<= 0 dB, floored to 0.1 dB) that keeps the EQ's peak boost
    from clipping. Never positive: an all-cut EQ must not add gain."""
    max_boost = max(v2 - v1 for (_, v1), (_, v2) in zip(fr1, fr2))
    return min(0.0, math.floor(-max_boost * 10) / 10)


def _distance(values, target):
    """Mean absolute deviation, ignoring deviations under 0.1 dB."""
    d = np.abs(values - target)
    return float(np.mean(np.where(d >= 0.1, d, 0.0)))


def autoeq_calc_distance(fr1, fr2):
    return _distance(np.array([v for _, v in fr1]), np.array([v for _, v in fr2]))


def autoeq_freq_unit(freq):
    if freq < 100:
        return 1
    elif freq < 1000:
        return 10
    elif freq < 10000:
        return 100
    return 1000


def autoeq_strip(filters):
    """Round bands to device-friendly values and clamp to the optimizer's ranges."""
    min_q, max_q = AUTOEQ_CONFIG["optimize_q_range"]
    min_gain, max_gain = AUTOEQ_CONFIG["optimize_gain_range"]
    return [
        {
            "type": f["type"],
            "freq": math.floor(f["freq"] - f["freq"] % autoeq_freq_unit(f["freq"])),
            "q": min(max(math.floor(f["q"] * 10) / 10, min_q), max_q),
            "gain": min(max(math.floor(f["gain"] * 10) / 10, min_gain), max_gain),
        }
        for f in filters
    ]


def autoeq_search_candidates(fr, fr_target, threshold):
    """One PK candidate per contiguous region where `fr` deviates from the
    target by >= threshold, centred on it and sized to its width."""
    state = 0  # 1: peak, 0: matched, -1: dip
    start_index = -1
    candidates = []
    min_freq, max_freq = AUTOEQ_CONFIG["autoeq_range"]

    for i, (f, v0) in enumerate(fr):
        delta = v0 - fr_target[i][1]
        next_state = 0 if abs(delta) < threshold else (1 if delta > 0 else -1)
        if next_state == state:
            continue

        # Close the peak/dip region that just ended. Upstream toggled
        # start_index on every state change instead, which falls out of step
        # after a direct peak<->dip jump and then drops every later region.
        # (A region still open at the top of the grid is dropped, as upstream.)
        if state != 0 and start_index >= 0:
            start = fr[start_index][0]
            center = math.sqrt(start * f)
            gain = (
                autoeq_interp([center], fr_target[start_index:i + 1])[0][1] -
                autoeq_interp([center], fr[start_index:i + 1])[0][1]
            )
            if min_freq <= center <= max_freq:
                candidates.append(
                    {"type": "PK", "freq": center, "q": center / (f - start), "gain": gain})
        start_index = i if next_state != 0 else -1
        state = next_state

    return candidates


class _AutoEqFit:
    """A measurement and target on one grid, as arrays, so the optimizer's
    inner loop evaluates candidate filters without per-call list conversions."""

    def __init__(self, fr, fr_target):
        fs = AUTOEQ_CONFIG["default_sample_rate"]
        self.values = np.array([v for _, v in fr], dtype=float)
        self.target = np.array([v for _, v in fr_target], dtype=float)
        self.phi = biquad_phi([f for f, _ in fr], fs)

    def apply(self, values, filters):
        return values + gains_db(self.phi, autoeq_filters_to_coeffs(filters))

    def distance(self, filters, values=None):
        """Distance to target after applying `filters` to `values` (default: the measurement)."""
        return _distance(self.apply(self.values if values is None else values, filters),
                         self.target)


def _refine_pass(fit, filters, iteration, reverse):
    """Greedy local search over each band's freq/Q/gain, one band at a time."""
    min_freq, max_freq = AUTOEQ_CONFIG["autoeq_range"]
    min_q, max_q = AUTOEQ_CONFIG["optimize_q_range"]
    min_gain, max_gain = AUTOEQ_CONFIG["optimize_gain_range"]
    max_df, max_dq, max_dg, step_df, step_dq, step_dg = (
        AUTOEQ_CONFIG["optimize_deltas"][iteration])

    for i in (reversed(range(len(filters))) if reverse else range(len(filters))):
        f = filters[i]
        others = fit.apply(fit.values, filters[:i] + filters[i + 1:])
        best_filter = dict(f)
        best_distance = fit.distance([f], others)

        def try_step(df, dq, dg):
            nonlocal best_filter, best_distance
            freq = f["freq"] + df * autoeq_freq_unit(f["freq"]) * step_df
            q = f["q"] + dq * step_dq
            gain = f["gain"] + dg * step_dg
            if not (min_freq <= freq <= max_freq and min_q <= q <= max_q
                    and min_gain <= gain <= max_gain):
                return False
            candidate = {"type": f["type"], "freq": freq, "q": q, "gain": gain}
            distance = fit.distance([candidate], others)
            if distance < best_distance:
                best_filter, best_distance = candidate, distance
                return True
            return False

        # Loop bounds (including their asymmetry) are upstream's.
        for df in range(-max_df, max_df):
            for dq in range(max_dq - 1, -max_dq - 1, -1):  # smaller Q (wider) first
                for dg in range(1, max_dg):
                    if not try_step(df, dq, dg):
                        break
                for dg in range(-1, -max_dg - 1, -1):
                    if not try_step(df, dq, dg):
                        break

        filters[i] = best_filter


def _optimize(fit, filters, iteration):
    """Refine forward then backward, then merge near-duplicates and drop
    bands that don't help."""
    for reverse in (False, True):
        filters = autoeq_strip(filters)
        _refine_pass(fit, filters, iteration, reverse)
    filters.sort(key=lambda x: x["freq"])

    i = 0
    while i < len(filters) - 1:
        f1, f2 = filters[i], filters[i + 1]
        if (abs(f1["freq"] - f2["freq"]) <= autoeq_freq_unit(f1["freq"])
                and abs(f1["q"] - f2["q"]) <= 0.1):
            f1["gain"] += f2["gain"]
            del filters[i + 1]
        else:
            i += 1

    best_distance = fit.distance(filters)
    i = 0
    while i < len(filters):
        if abs(filters[i]["gain"]) <= 0.1:
            del filters[i]
            continue
        distance = fit.distance(filters[:i] + filters[i + 1:])
        if distance < best_distance:
            del filters[i]
            best_distance = distance
        else:
            i += 1
    return filters


def autoeq_optimize(fr, fr_target, filters, iteration):
    return _optimize(_AutoEqFit(fr, fr_target), filters, iteration)


def autoeq_run(fr, fr_target, max_filters):
    """Compute up to `max_filters` PK filters that reshape `fr` towards
    `fr_target` (curves on the same ascending grid, level-aligned).

    Two batches, as upstream: first the widest deviations below the treble,
    then whatever remains, then a joint refinement of all bands.
    """
    if max_filters <= 0:
        return []
    iterations = range(len(AUTOEQ_CONFIG["optimize_deltas"]))

    def widest(candidates, count):
        return sorted(sorted(candidates, key=lambda c: c["q"])[:count],
                      key=lambda c: c["freq"])

    fit = _AutoEqFit(fr, fr_target)
    first = widest(
        [c for c in autoeq_search_candidates(fr, fr_target, 1)
         if c["freq"] <= AUTOEQ_CONFIG["treble_start_from"]],
        max(math.floor(max_filters / 2) - 1, 1))
    for i in iterations:
        first = _optimize(fit, first, i)

    second_fr = autoeq_apply(fr, first)
    second_fit = _AutoEqFit(second_fr, fr_target)
    second = widest(autoeq_search_candidates(second_fr, fr_target, 0.5),
                    max_filters - len(first))
    for i in iterations:
        second = _optimize(second_fit, second, i)

    combined = first + second
    for i in iterations:
        combined = _optimize(fit, combined, i)
    return autoeq_strip(combined)


# ---------------------------------------------------------------------------
# ISO 226:2003 equal-loudness normalization (port of CrinGraph's graphtool.js
# find_offset()/init_normalize(), which hangout.audio's graphs are built on).
#
# Equalizer.autoeq() expects its two curves to be level-aligned already: the
# optimizer only has peaking filters, so it can't correct a constant offset
# between measurement and target and would waste bands chasing one. Different
# measurement sources/rigs use arbitrary absolute dB references, so each
# curve is independently shifted to the same loudness first. 60 phon is
# CrinGraph's (and hangout.audio's) default graph normalization; the exact
# level barely changes the result, it just has to be the same for both.
# ---------------------------------------------------------------------------

AUTOEQ_NORMALIZE_PHON = 60.0

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
# CrinGraph's init_normalize() (raw values, before the "-7" dB shift).
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
    i = 0
    n = len(_ISO226_F)
    for f in freqs:
        if i < n and f >= _ISO226_F[i]:
            i += 1
        i0 = max(0, i - 1)
        i1 = min(i, n - 1)
        if i0 == i1:
            a, lu, tf = _ISO226_A_F[i0], _ISO226_L_U[i0], _ISO226_T_F[i0]
        else:
            l0, l1 = math.log(_ISO226_F[i0]), math.log(_ISO226_F[i1])
            frac = (math.log(f) - l0) / (l1 - l0)
            a = _ISO226_A_F[i0] + frac * (_ISO226_A_F[i1] - _ISO226_A_F[i0])
            lu = _ISO226_L_U[i0] + frac * (_ISO226_L_U[i1] - _ISO226_L_U[i0])
            tf = _ISO226_T_F[i0] + frac * (_ISO226_T_F[i1] - _ISO226_T_F[i0])
        m = a * (math.log10(4) - 10 + lu / 10)
        k = (0.005076 / (10 ** m)) - (10 ** (a * tf / 10))
        c = (10 ** (9.4 + 4 * m)) / len(freqs)
        par.append((a, k, c))
        ffi = math.floor(0.5 + 48 * math.log2(f / 19.4806))
        ff.append(_FREE_FIELD[max(0, min(479, ffi))])
    return par, ff


def iso226_find_offset(curve, target_phon=0.0):
    """dB offset that brings `curve` to `target_phon` loudness (Newton's method)."""
    values = [v for _, v in curve]
    par, ff = _iso226_init_normalize([f for f, _ in curve])
    l10 = math.log(10) / 10

    def step(offset):
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
        dx = step(x)
        x -= dx
        if abs(dx) <= 0.01:
            break
    return x


def autoeq_loudness_normalize(curve):
    """Shift `curve` by its own ISO-226 offset to AUTOEQ_NORMALIZE_PHON."""
    offset = iso226_find_offset(curve, AUTOEQ_NORMALIZE_PHON)
    return [(f, v + offset) for f, v in curve]


def autoeq_compute(measurement, target_points, max_filters):
    """Full AutoEQ pipeline: returns (filters, preamp_db) that bring
    `measurement` towards `target_points` (None = flat). Both inputs are
    (freq, dB) point lists at any resolution."""
    freqs = autoeq_raw_frequencies()
    fr = autoeq_loudness_normalize(autoeq_interp(freqs, measurement))
    fr_target = autoeq_loudness_normalize(
        [(f, 0.0) for f in freqs] if target_points is None
        else autoeq_interp(freqs, target_points))

    filters = autoeq_run(fr, fr_target, max_filters)
    preamp = autoeq_calc_preamp(fr, autoeq_apply(fr, filters))
    filters = [{"type": f["type"], "freq": round(f["freq"], 1),
                "gain": round(f["gain"], 2), "q": round(f["q"], 3)} for f in filters]
    return filters, preamp


# ===========================================================================
# AutoEQ measurement database (online AutoEq repo or a local folder)
# ===========================================================================
#
# Index entries are {"label", "path", "subtitle", "remote"}: `path` is a local
# file, or for remote entries a repo-relative path fetched (and cached) by
# fetch_autoeq_remote_file().

AUTOEQ_DB_CONFIG_PATH = os.path.expanduser("~/.config/eqloader/autoeq_db.json")
AUTOEQ_LOCAL_CACHE_DIR = os.path.expanduser("~/.cache/eqloader/autoeq_db")

# Raw per-model measurements live under measurements/<source>/data/<category>/
# <model>.csv as plain "frequency,raw" CSVs; results/<source>/ holds the
# project's own computed EQs.
AUTOEQ_GITHUB_REPO = "jaakkopasanen/AutoEq"
AUTOEQ_GITHUB_BRANCH = "master"
AUTOEQ_GITHUB_API_BASE = f"https://api.github.com/repos/{AUTOEQ_GITHUB_REPO}"
AUTOEQ_GITHUB_RAW_BASE = (
    f"https://raw.githubusercontent.com/{AUTOEQ_GITHUB_REPO}/{AUTOEQ_GITHUB_BRANCH}/")
AUTOEQ_MEASUREMENTS_DIR = "measurements"
AUTOEQ_TARGETS_DIR = "targets"
AUTOEQ_RESULTS_DIR = "results"

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


def _sorted_index(entries):
    return sorted(entries, key=lambda e: e["label"].lower())


def build_autoeq_model_index(root):
    """Recursively index measurement .txt/.csv files under `root` by model
    name (the file name); the containing folder becomes the subtitle."""
    root = Path(root)
    return _sorted_index(
        {
            "label": path.stem,
            "path": str(path),
            "subtitle": str(path.relative_to(root).parent),
            "remote": False,
        }
        for pattern in ("*.txt", "*.csv")
        for path in root.rglob(pattern)
        if not path.stem.lower().endswith(_AUTOEQ_SKIP_SUFFIXES)
    )


def _autoeq_github_get(url):
    req = urllib.request.Request(
        url, headers={"Accept": "application/vnd.github+json", "User-Agent": "eqloader"})
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.load(resp)


def _autoeq_github_subtree(dir_path):
    """Fetch one folder's subtree (by its own tree sha) rather than the
    whole repo's recursive tree, which is large enough to get truncated by
    GitHub's API before reaching every file. `dir_path` may be nested
    ('results/oratory1990'), walked one level at a time since each level's
    sha is only known from its parent's (non-recursive) listing.
    """
    tree_sha = AUTOEQ_GITHUB_BRANCH
    for part in dir_path.split("/"):
        listing = _autoeq_github_get(f"{AUTOEQ_GITHUB_API_BASE}/git/trees/{tree_sha}")
        entry = next(
            (e for e in listing.get("tree", [])
             if e.get("path") == part and e.get("type") == "tree"),
            None)
        if entry is None:
            raise RuntimeError(f"'{dir_path}' folder not found in repo")
        tree_sha = entry["sha"]

    sub = _autoeq_github_get(f"{AUTOEQ_GITHUB_API_BASE}/git/trees/{tree_sha}?recursive=1")
    if sub.get("truncated"):
        print(f"Warning: GitHub '{dir_path}' listing was truncated; some entries may be missing.")
    return [e for e in sub.get("tree", []) if e.get("type") == "blob"]


def fetch_autoeq_online_index():
    """List the AutoEq repo's raw measurement files (names only; content is
    downloaded lazily, on selection)."""
    return _sorted_index(
        {
            "label": Path(e["path"]).stem,
            "path": f"{AUTOEQ_MEASUREMENTS_DIR}/{e['path']}",
            "subtitle": str(Path(e["path"]).parent),
            "remote": True,
        }
        for e in _autoeq_github_subtree(AUTOEQ_MEASUREMENTS_DIR)
        if "/data/" in e["path"] and e["path"].endswith(".csv")
    )


def fetch_autoeq_targets_index():
    """List the AutoEq repo's named target curves (Harman, diffuse-field, ...)."""
    return _sorted_index(
        {
            "label": Path(e["path"]).stem,
            "path": f"{AUTOEQ_TARGETS_DIR}/{e['path']}",
            "subtitle": "",
            "remote": True,
        }
        for e in _autoeq_github_subtree(AUTOEQ_TARGETS_DIR)
        if e["path"].endswith(".csv")
    )


def fetch_autoeq_precomputed_profiles(source, model_stem):
    """ParametricEQ.txt file(s) the AutoEq project already computed for
    `model_stem` under results/<source>/ — one per target variant it used."""
    matches = []
    for e in _autoeq_github_subtree(f"{AUTOEQ_RESULTS_DIR}/{source}"):
        parent = Path(e["path"]).parent
        if not e["path"].endswith("ParametricEQ.txt") or parent.name != model_stem:
            continue
        variant = str(parent.parent)
        matches.append({
            "label": source if variant in (".", "") else f"{source} / {variant}",
            "path": f"{AUTOEQ_RESULTS_DIR}/{source}/{e['path']}",
            "subtitle": "",
            "remote": True,
        })
    return _sorted_index(matches)


def fetch_autoeq_remote_file(repo_path):
    """Download (and locally cache) one file from the AutoEq repo; returns its local path."""
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


# ===========================================================================
# GUI toolkit: theme, dialogs, small widgets
# ===========================================================================

# "Instrument panel": graphite chassis, two-LED accent.
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

NEW_BAND = {"type": "PK", "freq": 1000.0, "gain": 0.0, "q": 1.0}
GRAPH_MIN_HEIGHT = 150  # px; below this the graph is auto-hidden
GRAPH_GAIN_LIMIT = 15   # dB; graph y-range and drag clamp
FILE_TYPES_PROFILE = [("Text files", "*.txt"), ("All files", "*.*")]
FILE_TYPES_CURVE = [("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")]


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


def _install_theme(root):
    """Skin every ttk widget as a graphite instrument panel; returns (ui_font, mono_font)."""
    c = THEME
    font_ui = _pick_font(root, UI_FONTS)
    font_mono = _pick_font(root, MONO_FONTS)
    base_font = (font_ui, 10)
    root.configure(bg=c["chassis"])

    style = ttk.Style(root)
    try:
        style.theme_use("clam")  # the one built-in theme that honours colour overrides
    except tk.TclError:
        pass

    style.configure(".", background=c["chassis"], foreground=c["ink"],
                    fieldbackground=c["input"], bordercolor=c["line"],
                    lightcolor=c["line"], darkcolor=c["line"],
                    troughcolor=c["panel"], font=base_font)
    style.configure("TFrame", background=c["chassis"])
    style.configure("TLabel", background=c["chassis"], foreground=c["ink"], font=base_font)
    style.configure("Muted.TLabel", background=c["chassis"],
                    foreground=c["muted"], font=(font_ui, 9))
    style.configure("TLabelframe", background=c["chassis"],
                    bordercolor=c["line"], relief="solid", borderwidth=1)
    style.configure("TLabelframe.Label", background=c["chassis"],
                    foreground=c["muted"], font=(font_ui, 9, "bold"))

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
                    padding=(10, 6), font=(font_ui, 10, "bold"))
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

    for sb in ("Vertical.TScrollbar", "Horizontal.TScrollbar"):
        style.configure(sb, background=c["input"], troughcolor=c["panel"],
                        bordercolor=c["panel"], arrowcolor=c["muted"])
        style.map(sb, background=[("active", c["line"])])

    # Combobox dropdown popup is a plain tk.Listbox — style via option_add.
    root.option_add("*TCombobox*Listbox.background", c["input"])
    root.option_add("*TCombobox*Listbox.foreground", c["ink"])
    root.option_add("*TCombobox*Listbox.selectBackground", c["accent"])
    root.option_add("*TCombobox*Listbox.selectForeground", c["chassis"])
    root.option_add("*TCombobox*Listbox.font", (font_ui, 10))
    return font_ui, font_mono


def _listbox(parent, font_ui, **kwargs):
    """A classic tk.Listbox (no ttk equivalent) themed to match."""
    c = THEME
    return tk.Listbox(
        parent, bg=c["panel"], fg=c["ink"], selectbackground=c["accent"],
        selectforeground=c["chassis"], highlightthickness=1,
        highlightbackground=c["line"], highlightcolor=c["accent"],
        borderwidth=0, activestyle="none", font=(font_ui, 10), **kwargs)


def _add_tooltip(widget, text, font_ui):
    """Show a small hover tooltip (used for keyboard-shortcut hints)."""
    state = {"win": None}

    def show(_e=None):
        if state["win"] is not None or not text:
            return
        win = tk.Toplevel(widget)
        win.wm_overrideredirect(True)
        win.wm_geometry(f"+{widget.winfo_rootx() + 10}"
                        f"+{widget.winfo_rooty() + widget.winfo_height() + 4}")
        tk.Label(win, text=text, bg=THEME["input"], fg=THEME["ink"],
                 font=(font_ui, 9), padx=6, pady=2,
                 highlightthickness=1, highlightbackground=THEME["line"]).pack()
        state["win"] = win

    def hide(_e=None):
        if state["win"] is not None:
            state["win"].destroy()
            state["win"] = None

    widget.bind("<Enter>", show, add="+")
    widget.bind("<Leave>", hide, add="+")
    widget.bind("<Destroy>", hide, add="+")


def _dialog(parent, title, resizable=False):
    dlg = tk.Toplevel(parent)
    dlg.title(title)
    dlg.configure(bg=THEME["chassis"])
    dlg.transient(parent)
    if not resizable:
        dlg.resizable(False, False)
    return dlg


def _show_modal(parent, dlg, focus=None):
    """Grab input and place `dlg` centred over the upper third of `parent`."""
    dlg.grab_set()
    dlg.update_idletasks()
    x = parent.winfo_rootx() + (parent.winfo_width() - dlg.winfo_width()) // 2
    y = parent.winfo_rooty() + (parent.winfo_height() - dlg.winfo_height()) // 3
    dlg.geometry(f"+{max(x, 0)}+{max(y, 0)}")
    if focus is not None:
        focus.focus_set()


def ask_choice(parent, title, message, buttons, *, wraplength=380, enter=None, focus=None):
    """Modal message with a row of buttons; blocks until one is clicked.

    `buttons` are (label, value, ttk_style). Returns the clicked button's
    value, or None on Escape / closing the window. `enter` is the value Return
    picks; `focus` the label of the button that gets keyboard focus.
    """
    dlg = _dialog(parent, title)
    ttk.Label(dlg, text=message, wraplength=wraplength, justify="left").pack(
        padx=24, pady=(20, 16))
    row = ttk.Frame(dlg)
    row.pack(padx=16, pady=(0, 18))

    result = {"value": None}

    def choose(value):
        result["value"] = value
        dlg.destroy()

    focus_btn = None
    for label, value, style in buttons:
        btn = ttk.Button(row, text=label, style=style, command=lambda v=value: choose(v))
        btn.pack(side="left", padx=4)
        if label == focus:
            focus_btn = btn

    dlg.bind("<Escape>", lambda _e: choose(None))
    if enter is not None:
        dlg.bind("<Return>", lambda _e: choose(enter))
        dlg.bind("<KP_Enter>", lambda _e: choose(enter))
    dlg.protocol("WM_DELETE_WINDOW", lambda: choose(None))
    _show_modal(parent, dlg, focus_btn)
    parent.wait_window(dlg)
    return result["value"]


def busy_dialog(parent, title, message):
    """Modal, not user-closable spinner; the caller destroys it when done."""
    dlg = _dialog(parent, title)
    ttk.Label(dlg, text=message, wraplength=320, justify="left").pack(padx=24, pady=(20, 10))
    bar = ttk.Progressbar(dlg, mode="indeterminate", length=280)
    bar.pack(padx=24, pady=(0, 20))
    bar.start(12)
    dlg.protocol("WM_DELETE_WINDOW", lambda: None)
    _show_modal(parent, dlg)
    return dlg


class SearchDialog:
    """Modal type-to-filter list over index entries (see the AutoEQ database
    section). Picking a remote entry downloads it first. run() returns
    (local_path, entry), or (None, None) if cancelled."""

    MAX_ROWS = 300

    def __init__(self, app, title, size, min_size, get_items,
                 show_subtitle=False, status_suffix=lambda: ""):
        self.app = app
        self.get_items = get_items
        self.show_subtitle = show_subtitle
        self.status_suffix = status_suffix
        self.result = (None, None)
        self.filtered = []

        self.dlg = dlg = _dialog(app, title, resizable=True)
        dlg.geometry(size)
        dlg.minsize(*min_size)

        top = ttk.Frame(dlg)
        top.pack(fill="x", padx=12, pady=(12, 6))
        ttk.Label(top, text="Search:").pack(side="left")
        self.query = tk.StringVar()
        self.entry = ttk.Entry(top, textvariable=self.query)
        self.entry.pack(side="left", fill="x", expand=True, padx=(6, 0))

        list_frame = ttk.Frame(dlg)
        list_frame.pack(fill="both", expand=True, padx=12, pady=6)
        self.listbox = _listbox(list_frame, app.font_ui)
        self.listbox.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.listbox.configure(yscrollcommand=scrollbar.set)

        self.status = tk.StringVar()
        ttk.Label(dlg, textvariable=self.status, style="Muted.TLabel").pack(anchor="w", padx=12)

        self.button_row = ttk.Frame(dlg)
        self.button_row.pack(fill="x", padx=12, pady=(6, 12))
        ttk.Button(self.button_row, text="Cancel", command=dlg.destroy).pack(side="right", padx=4)
        self.select_btn = ttk.Button(self.button_row, text="Select",
                                     style="Accent.TButton", command=self.choose)
        self.select_btn.pack(side="right", padx=4)

        self.query.trace_add("write", lambda *_: self.refresh())
        self.listbox.bind("<Double-Button-1>", lambda _e: self.choose())
        self.entry.bind("<Return>", lambda _e: self.choose())
        self.entry.bind("<Down>", lambda _e: self.listbox.focus_set())
        dlg.bind("<Escape>", lambda _e: dlg.destroy())
        dlg.protocol("WM_DELETE_WINDOW", dlg.destroy)

    def add_button(self, text, command):
        ttk.Button(self.button_row, text=text, command=command).pack(side="left", padx=(0, 6))

    def refresh(self):
        terms = self.query.get().strip().lower().split()
        self.filtered = [
            e for e in self.get_items()
            if all(t in e["label"].lower() or t in e["subtitle"].lower() for t in terms)
        ]
        self.listbox.delete(0, "end")
        for e in self.filtered[:self.MAX_ROWS]:
            self.listbox.insert(
                "end", f"{e['label']}   [{e['subtitle']}]" if self.show_subtitle else e["label"])
        if self.filtered:
            self.listbox.selection_set(0)  # so Enter in the search field picks the top match
        self.status.set(
            f"{len(self.filtered)} match(es){self.status_suffix()}"
            + (f" (showing first {self.MAX_ROWS})" if len(self.filtered) > self.MAX_ROWS else ""))

    def choose(self):
        sel = self.listbox.curselection()
        if not sel or sel[0] >= len(self.filtered):
            return
        entry = self.filtered[sel[0]]
        if not entry.get("remote"):
            self.finish(entry["path"], entry)
            return

        self.select_btn.configure(state="disabled")
        self.status.set(f"Downloading {entry['label']}...")

        def failed():
            if self.dlg.winfo_exists():
                self.select_btn.configure(state="normal")
                self.refresh()

        self.app.run_task(lambda: fetch_autoeq_remote_file(entry["path"]),
                          lambda path: self.finish(path, entry),
                          error=("AutoEQ", f"Could not download {entry['label']}"),
                          on_error=failed)

    def finish(self, path, entry=None):
        if self.dlg.winfo_exists():  # a download may finish after Cancel
            self.result = (path, entry)
            self.dlg.destroy()

    def run(self):
        self.refresh()
        _show_modal(self.app, self.dlg, self.entry)
        self.app.wait_window(self.dlg)
        return self.result


class UndoHistory:
    """Undo/redo stacks of opaque state snapshots."""

    def __init__(self):
        self._undo = []
        self._redo = []

    def record(self, state):
        """Remember `state` (taken before a change) as one undo step."""
        self._undo.append(state)
        self._redo.clear()

    def undo(self, current):
        """State to restore, or None; `current` becomes redoable."""
        if not self._undo:
            return None
        self._redo.append(current)
        return self._undo.pop()

    def redo(self, current):
        if not self._redo:
            return None
        self._undo.append(current)
        return self._redo.pop()


class _LogStream:
    """stdout/stderr while the GUI runs: text goes to the log panel through a
    queue (so any thread may print) and on to the original stream."""

    def __init__(self, q, original):
        self.q = q
        self.original = original

    def write(self, text):
        if text:
            self.q.put(text)
            if self.original is not None:
                try:
                    self.original.write(text)
                except Exception:
                    pass
        return len(text)

    def flush(self):
        if self.original is not None:
            try:
                self.original.flush()
            except Exception:
                pass


def parse_int(s, default=None):
    """int from user text (decimal or 0x-hex), else `default`."""
    try:
        return int((s or "").strip(), 0)
    except ValueError:
        return default


def parse_float(s, default=None):
    """float from user text (decimal comma accepted), else `default`."""
    try:
        return float((s or "").strip().replace(",", "."))
    except ValueError:
        return default


# ===========================================================================
# GUI: AutoEQ workflow
# ===========================================================================

class AutoEqWorkflow:
    """The GUI side of AutoEQ: pick a measurement (online database or local
    folder), pick a target, run the optimizer — or instead fetch a profile the
    AutoEq project already computed for that model."""

    def __init__(self, app):
        self.app = app
        self.db_path = None        # "online" or a folder, once an index is loaded
        self.model_index = None
        self.target_index = None

    # ---- entry points ----------------------------------------------------

    def compute(self):
        self._pick_model(self._on_measurement_chosen)

    def load_precomputed(self):
        self._pick_model(self._on_precomputed_model_chosen)

    # ---- compute ---------------------------------------------------------

    def _on_measurement_chosen(self, path, _entry):
        if not path:
            return
        try:
            measurement = parse_frequency_response_file(path)
        except Exception as e:
            messagebox.showerror("AutoEQ", f"Could not read measurement file:\n{e}")
            return
        self._pick_target(lambda target: self._run(measurement, target))

    def _run(self, measurement, target):
        max_filters = self.app.max_filters()

        def work():
            print("Running AutoEQ optimization, this may take a while...")
            filters, preamp = autoeq_compute(measurement, target, max_filters)
            print(f"AutoEQ generated {len(filters)} band(s), preamp {preamp:.1f} dB")
            return filters, preamp

        self.app.run_task(
            work, lambda result: self.app.set_filters(*result),
            busy=("AutoEQ", "Running AutoEQ optimization...\n"
                            "This can take a while depending on the number of filters."),
            error=("AutoEQ", "AutoEQ failed"))

    def _pick_target(self, callback):
        """Eventually calls callback(target_points); None means flat."""
        choice = ask_choice(
            self.app, "AutoEQ Target",
            "Choose the target curve AutoEQ should reshape your measurement towards.",
            [("Flat (0 dB)", "flat", "TButton"),
             ("Search AutoEQ Targets...", "online", "Accent.TButton"),
             ("Load Target File...", "file", "TButton"),
             ("Cancel", None, "TButton")])
        if choice == "flat":
            callback(None)
        elif choice == "online":
            self._with_target_index(lambda: self._use_target_file(
                SearchDialog(self.app, "Search AutoEQ Targets", "620x440", (560, 340),
                             lambda: self.target_index).run()[0],
                callback))
        elif choice == "file":
            self._use_target_file(filedialog.askopenfilename(
                title="Select Target Curve File (freq, dB per line)",
                filetypes=FILE_TYPES_CURVE), callback)

    @staticmethod
    def _use_target_file(path, callback):
        if not path:
            return
        try:
            points = parse_frequency_response_file(path)
        except Exception as e:
            messagebox.showerror("AutoEQ", f"Could not read target file:\n{e}")
            return
        callback(points)

    def _with_target_index(self, on_ready):
        """Fetch (once per session) the online target list, then on_ready()."""
        if self.target_index is not None:
            on_ready()
            return

        def work():
            print("Fetching AutoEQ target curve list from GitHub...")
            index = fetch_autoeq_targets_index()
            print(f"Fetched {len(index)} target curve(s).")
            return index

        def done(index):
            self.target_index = index
            on_ready()

        self.app.run_task(
            work, done,
            busy=("AutoEQ Targets", "Fetching target curve list from GitHub..."),
            error=("AutoEQ Targets", "Could not fetch the target list"))

    # ---- pre-computed profiles --------------------------------------------

    def _on_precomputed_model_chosen(self, path, entry):
        if not path:
            return
        if not entry or not entry.get("remote"):
            messagebox.showinfo(
                "Pre-computed Profile",
                "Pre-computed profiles are only available for models picked "
                "from the online AutoEQ database, not local files/folders.")
            return

        # entry["path"] is "measurements/<source>/data/<category>/<model>.csv"
        source = entry["path"].split("/")[1]
        model = entry["label"]

        def found(candidates):
            if not candidates:
                messagebox.showinfo(
                    "Pre-computed Profile",
                    f"No pre-computed ParametricEQ.txt found for {model} under '{source}'.")
            elif len(candidates) == 1:
                self._load_variant(candidates[0])
            else:
                _, variant = SearchDialog(
                    self.app, "Select Target Variant", "560x360", (480, 280),
                    lambda: candidates).run()
                if variant:
                    self._load_variant(variant)

        self.app.run_task(
            lambda: fetch_autoeq_precomputed_profiles(source, model), found,
            busy=("Pre-computed Profile",
                  f"Looking up pre-computed EQ profile(s) for {model}..."),
            error=("Pre-computed Profile", "Lookup failed"))

    def _load_variant(self, candidate):
        def apply(data):
            self.app.set_filters(data["filters"], data["preamp"])
            self.app.log(f"Loaded pre-computed profile ({candidate['label']})\n")

        self.app.run_task(
            lambda: load_profile(fetch_autoeq_remote_file(candidate["path"])), apply,
            busy=("Pre-computed Profile", f"Downloading {candidate['label']}..."),
            error=("Pre-computed Profile", "Could not load profile"))

    # ---- measurement database ----------------------------------------------

    def _pick_model(self, callback):
        """Eventually calls callback(local_path, index_entry); nothing if
        cancelled. index_entry is None for a manually browsed file."""
        def search():
            path, entry = self._model_search()
            if path:
                callback(path, entry)

        if self.model_index is not None:
            search()
        else:
            self._open_db(load_autoeq_db_path() or self._ask_db_source(), search)

    def _ask_db_source(self):
        """Returns "online", a local folder, or None."""
        choice = ask_choice(
            self.app, "AutoEQ Model Database",
            "Search headphone/IEM measurements in the online AutoEQ database "
            "on GitHub (jaakkopasanen/AutoEq), or in a local folder of "
            "measurement .txt/.csv files?",
            [("Download Online Database", "online", "Accent.TButton"),
             ("Choose Local Folder...", "local", "TButton"),
             ("Cancel", None, "TButton")])
        if choice == "local":
            return filedialog.askdirectory(title="Select Measurement Database Folder") or None
        return choice

    def _open_db(self, source, on_ready):
        """Load and remember the model index of `source`, then on_ready()."""
        if source is None:
            return
        if source != "online":
            self.model_index, self.db_path = build_autoeq_model_index(source), source
            save_autoeq_db_path(source)
            on_ready()
            return

        def work():
            print("Fetching AutoEQ database listing from GitHub...")
            index = fetch_autoeq_online_index()
            print(f"Fetched {len(index)} model(s) from the online database.")
            return index

        def done(index):
            self.model_index, self.db_path = index, "online"
            save_autoeq_db_path("online")
            on_ready()

        self.app.run_task(
            work, done,
            busy=("AutoEQ Model Database", "Fetching model list from GitHub..."),
            error=("AutoEQ Model Database", "Could not fetch the online database"))

    def _model_search(self):
        dialog = SearchDialog(
            self.app, "Search Headphone Model", "700x480", (650, 360),
            lambda: self.model_index, show_subtitle=True,
            status_suffix=lambda: f" in {os.path.basename(self.db_path)}")

        def browse_file():
            path = filedialog.askopenfilename(
                title="Select Measurement File (freq, dB per line)",
                filetypes=FILE_TYPES_CURVE)
            if path:
                dialog.finish(path, None)

        def change_db():
            source = self._ask_db_source()
            if source == "online":
                dialog.status.set("Fetching online database...")
            self._open_db(source, dialog.refresh)

        dialog.add_button("Browse File Instead...", browse_file)
        dialog.add_button("Change Database...", change_db)
        return dialog.run()


# ===========================================================================
# GUI: main window
# ===========================================================================

class EqLoaderGUI(tk.Tk):
    """Main window: device picker, EQ editor (graph + band list + fields),
    push/pull/file actions and the log."""

    def __init__(self, graph=None):
        super().__init__()
        self.title("Walkplay PEQ Loader")
        self.geometry("950x850")
        self.minsize(850, 700)
        self.font_ui, self.font_mono = _install_theme(self)

        # Worker threads never touch Tk: they queue log text and completion
        # callbacks, which _poll_queues() runs on the Tk thread.
        self.log_queue = queue.Queue()
        self._ui_calls = queue.Queue()
        self._poll_after_id = None
        # Route print() from any thread into the log panel for the app's lifetime.
        self._saved_streams = (sys.stdout, sys.stderr)
        sys.stdout = _LogStream(self.log_queue, sys.stdout)
        sys.stderr = _LogStream(self.log_queue, sys.stderr)

        # Graph visibility: None = auto (hide when short), True/False = forced.
        self._graph_forced = graph
        self._graph_show_threshold = None  # window height at/above which the graph fits
        self._layout_ready = False
        self._resize_after_id = None

        self.devices = []          # hid.enumerate() entries, in device-list order
        self.selected_path = None

        # Editor state: the EQ being built and the primary selected band.
        self.filters = [dict(NEW_BAND)]
        self.selected = 0
        self.history = UndoHistory()
        self._drag_idx = None           # band being dragged on the graph
        self._drag_recorded = False     # undo step taken for this drag
        self._loading_editor = False    # suppress field traces while filling fields
        self._editor_recorded = False   # one undo step per field-edit session

        self.autoeq = AutoEqWorkflow(self)

        self._build_widgets()
        if self._graph_forced is False:
            self._set_graph_visible(False)

        # Evaluate graph visibility once the window is mapped (its real size
        # is only known then); the timed call covers an already-delivered <Map>.
        self.bind("<Map>", self._on_mapped, add="+")
        self.after(200, self._mark_layout_ready)
        self._poll_queues()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if hid is None:
            self.log("ERROR: the 'hidapi' package is not installed.\n"
                     "Run:  pip install hidapi\n"
                     "Then restart this app.\n")

    def destroy(self):
        if self._poll_after_id is not None:
            self.after_cancel(self._poll_after_id)
        sys.stdout, sys.stderr = self._saved_streams
        super().destroy()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_widgets(self):
        self.grid_rowconfigure(2, weight=3)
        self.grid_rowconfigure(5, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        self._build_device_rows()
        self._build_graph()
        self._build_editor()
        self._build_eq_settings()
        self._build_log()
        self._build_actions()

        self.bind("<Configure>", self._on_window_resize)
        for seq, handler in (
            ("<Control-z>", self._undo),
            ("<Control-y>", self._redo),
            ("<Control-Shift-z>", self._redo),
            ("<Control-Shift-Z>", self._redo),
            ("<F5>", self._refresh_devices),
            ("<Control-g>", self._get_slot),
            ("<Control-Shift-E>", self._enable),
            ("<Control-Shift-X>", self._disable),
        ):
            self.bind(seq, lambda _e, h=handler: h())

        self._refresh_editor()
        self._refresh_devices()

    def _tooltip(self, widget, text):
        _add_tooltip(widget, text, self.font_ui)

    def _build_device_rows(self):
        dev_frame = ttk.LabelFrame(self, text="Device")
        dev_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=6)

        self.device_list = _listbox(dev_frame, self.font_ui, height=4)
        self.device_list.pack(fill="x", padx=6, pady=6, side="left", expand=True)
        self.device_list.bind("<<ListboxSelect>>", self._on_device_select)

        btn_frame = ttk.Frame(dev_frame)
        btn_frame.pack(side="left", padx=6)
        for text, command, accel in (("Refresh List", self._refresh_devices, "F5"),
                                     ("Get Slot / Version", self._get_slot, "Ctrl+G")):
            btn = ttk.Button(btn_frame, text=text, command=command)
            btn.pack(fill="x", pady=2)
            self._tooltip(btn, accel)

        override = ttk.Frame(self)
        override.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

        ttk.Label(override, text="VID (hex):").grid(row=0, column=0, sticky="w")
        self.vid_entry = ttk.Entry(override, width=10)
        self.vid_entry.insert(0, f"0x{WALKPLAY_VENDOR_ID:04X}")
        self.vid_entry.grid(row=0, column=1, padx=4)

        ttk.Label(override, text="PID (hex, optional):").grid(row=0, column=2, sticky="w")
        self.pid_entry = ttk.Entry(override, width=10)
        self.pid_entry.grid(row=0, column=3, padx=4)

        ttk.Label(override, text="Max filters:").grid(row=0, column=4, sticky="w", padx=(12, 0))
        int_only = (self.register(lambda s: s == "" or s.isdigit()), "%P")
        self.max_filter_spin = ttk.Spinbox(
            override, from_=1, to=64, increment=1, width=6,
            validate="key", validatecommand=int_only)
        self.max_filter_spin.set(DEFAULT_MAX_FILTERS)
        self.max_filter_spin.grid(row=0, column=5, padx=4)

    def _build_graph(self):
        self.fig = Figure(figsize=(7, 4), dpi=100,
                          facecolor=THEME["chassis"], layout="constrained")
        self.ax = self.fig.add_subplot(111)

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
            bg=THEME["chassis"], fg=THEME["muted"], font=("TkDefaultFont", 9))

    def _build_editor(self):
        ctrl = ttk.Frame(self)
        ctrl.grid(row=3, column=0, sticky="ew", padx=(8, 4), pady=2)
        ctrl.columnconfigure(1, weight=1)

        list_frame = ttk.LabelFrame(ctrl, text="Filters")
        list_frame.grid(row=0, column=0, sticky="ns")
        self.filter_list = _listbox(list_frame, self.font_ui, height=7, selectmode="extended")
        self.filter_list.pack(fill="both", expand=True, padx=5, pady=5)
        self.filter_list.bind("<<ListboxSelect>>", self._on_list_select)

        edit = ttk.LabelFrame(ctrl, text="Selected Filter")
        edit.grid(row=0, column=1, sticky="nsew", padx=10)
        edit.columnconfigure(1, weight=1)

        self.field_vars = {}
        for row, (field, label, default, lo, hi, step, fmt) in enumerate((
            ("freq", "Frequency (Hz)", "1000", 10, 30000, 1, "%.0f"),
            ("gain", "Gain (dB)", "0", -30, 30, 0.1, "%.1f"),
            ("q", "Q", "1.0", 0.1, 100, 0.1, "%.1f"),
        )):
            label_widget = ttk.Label(edit, text=label)
            label_widget.grid(row=row, column=0, sticky="w", padx=5, pady=3)
            var = self.field_vars[field] = tk.StringVar(value=default)
            ttk.Spinbox(edit, textvariable=var, from_=lo, to=hi, increment=step,
                        format=fmt).grid(row=row, column=1, sticky="ew", padx=5, pady=3)
        self.q_label = label_widget

        ttk.Label(edit, text="Type").grid(row=3, column=0, sticky="w", padx=5, pady=3)
        self.field_vars["type"] = tk.StringVar(value="PK")
        ttk.Combobox(edit, textvariable=self.field_vars["type"],
                     values=list(FILTER_TYPES), state="readonly").grid(
            row=3, column=1, sticky="ew", padx=5, pady=3)

        self.bw_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(edit, text="Show Q as Bandwidth (oct)", variable=self.bw_mode,
                        command=self._toggle_bw_mode).grid(
            row=4, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        # Live-apply editor fields to the current selection as they change.
        for field, var in self.field_vars.items():
            var.trace_add("write", lambda *_, f=field: self._apply_field(f))

    def _build_eq_settings(self):
        ops = ttk.Frame(self)
        ops.grid(row=4, column=0, sticky="ew", padx=(8, 4), pady=2)
        ops.columnconfigure(0, weight=1)
        ops.columnconfigure(1, weight=1)

        eq_frame = ttk.LabelFrame(ops, text="EQ")
        eq_frame.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Label(eq_frame, text="Slot:").pack(side="left", padx=(8, 2))
        self.slot_spin = ttk.Spinbox(eq_frame, from_=0, to=15, width=5)
        self.slot_spin.set("0")
        self.slot_spin.pack(side="left", padx=(0, 6))
        for label, attr, default in (("Preamp (dB)", "preamp_spin", "0"),
                                     ("Buffer (dB)", "buffer_spin",
                                      str(float(DEFAULT_GLOBAL_GAIN_BUFFER)))):
            ttk.Label(eq_frame, text=f"{label}:").pack(side="left", padx=(8, 2))
            spin = ttk.Spinbox(eq_frame, from_=-30, to=30, increment=0.1,
                               format="%.1f", width=7)
            spin.set(default)
            spin.pack(side="left", padx=(0, 6))
            setattr(self, attr, spin)

        ed_frame = ttk.LabelFrame(ops, text="PEQ Enable / Disable")
        ed_frame.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=2)
        ttk.Label(ed_frame, text="Slot:").pack(side="left", padx=(8, 2))
        self.ed_slot_spin = ttk.Spinbox(ed_frame, from_=0, to=15, width=5)
        self.ed_slot_spin.set(0)
        self.ed_slot_spin.pack(side="left", padx=(0, 8))
        for text, command, style, accel in (
            ("Enable PEQ", self._enable, "Accent.TButton", "Ctrl+Shift+E"),
            ("Disable PEQ", self._disable, "Danger.TButton", "Ctrl+Shift+X"),
        ):
            btn = ttk.Button(ed_frame, text=text, command=command, style=style)
            btn.pack(side="left", padx=4, pady=4)
            self._tooltip(btn, accel)

    def _build_log(self):
        c = THEME
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=5, column=0, sticky="nsew", padx=(8, 4), pady=6)
        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=5, state="disabled",
            bg=c["chassis"], fg=c["accent"], insertbackground=c["accent"],
            selectbackground=c["input"], selectforeground=c["ink"],
            highlightthickness=1, highlightbackground=c["line"],
            borderwidth=0, font=(self.font_mono, 9), padx=8, pady=6)
        self.log_text.vbar.configure(
            bg=c["input"], troughcolor=c["panel"], activebackground=c["line"],
            highlightbackground=c["panel"], highlightcolor=c["panel"],
            borderwidth=0, relief="flat")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_actions(self):
        actions = ttk.LabelFrame(self, text="Actions")
        actions.grid(row=2, column=1, rowspan=4, sticky="nsew", padx=(0, 8), pady=6)
        actions.columnconfigure(0, weight=1)

        for i, (text, command, style, accel, seq) in enumerate((
            ("Add Band", self._add_band, "TButton", "Ctrl+B", "<Control-b>"),
            ("Delete Band", self._delete_selected_bands, "Danger.TButton",
             "Ctrl+D", "<Control-d>"),
            ("Delete All", self._delete_all_bands, "Danger.TButton",
             "Ctrl+Shift+D", "<Control-Shift-D>"),
            ("Load EQ from Device", self._load_from_device, "TButton",
             "Ctrl+E", "<Control-e>"),
            ("Save Profile to File", self._save_profile, "TButton", "Ctrl+S", "<Control-s>"),
            ("Load Profile from File", self._load_profile, "TButton",
             "Ctrl+O", "<Control-o>"),
            ("Compute AutoEQ", self.autoeq.compute, "TButton",
             "Ctrl+Shift+A", "<Control-Shift-A>"),
            ("Load Pre-computed AutoEQ", self.autoeq.load_precomputed, "TButton",
             "Ctrl+Shift+L", "<Control-Shift-L>"),
            ("Push EQ to Device", self._push, "Accent.TButton", "Ctrl+P", "<Control-p>"),
        )):
            actions.rowconfigure(i, weight=1)
            btn = ttk.Button(actions, text=text, command=command, style=style)
            btn.grid(row=i, column=0, sticky="nsew", padx=6, pady=2)
            self.bind(seq, lambda _e, c=command: c())
            self._tooltip(btn, accel)

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
    # Editor: band list, fields, graph
    # ------------------------------------------------------------------

    def max_filters(self):
        return parse_int(self.max_filter_spin.get(), DEFAULT_MAX_FILTERS)

    def _set_preamp(self, value):
        self.preamp_spin.delete(0, "end")
        self.preamp_spin.insert(0, value if isinstance(value, str) else f"{value:g}")

    def _selection(self):
        """Indices the editor fields apply to: the list selection, else the primary band."""
        return self.filter_list.curselection() or (
            (self.selected,) if 0 <= self.selected < len(self.filters) else ())

    def _refresh_editor(self):
        """Redraw the band list (selecting the primary band) and the graph."""
        self._render_filter_rows()
        if self.filters:
            self.selected = min(self.selected, len(self.filters) - 1)
            self.filter_list.selection_clear(0, "end")
            self.filter_list.selection_set(self.selected)
            self.filter_list.see(self.selected)
        self._draw_graph()

    def _render_filter_rows(self):
        """Rebuild the listbox rows from self.filters (selection untouched)."""
        self.filter_list.delete(0, "end")
        for i, f in enumerate(self.filters):
            self.filter_list.insert(
                "end", f"{i + 1}: {f['freq']:.1f} Hz  {f['gain']:.1f} dB  "
                       f"Q {f['q']:.2f}  {f['type']}")

    def _draw_graph(self):
        c = THEME
        ax = self.ax
        ax.clear()
        ax.set_facecolor(c["panel"])

        freqs = np.logspace(np.log10(20), np.log10(20000), 1000)
        response = filters_response_db(freqs, self.filters)

        highlighted = set(self.filter_list.curselection()) | {self.selected}
        for index, f in enumerate(self.filters):
            color = c["active"] if index in highlighted else c["accent"]
            if index in highlighted:  # amber glow halo
                ax.plot([f["freq"]], [f["gain"]], marker="o", markersize=16,
                        color=c["active"], alpha=0.25, zorder=4)
            ax.plot([f["freq"]], [f["gain"]], marker="o", markersize=9,
                    markerfacecolor=color, markeredgecolor=c["chassis"],
                    markeredgewidth=1.5, zorder=5)

        # Filled scope trace with a soft glow underneath.
        ax.fill_between(freqs, response, 0, color=c["accent"], alpha=0.10, zorder=1)
        for lw, alpha in ((5, 0.10), (3, 0.18)):
            ax.plot(freqs, response, linewidth=lw, color=c["accent"], alpha=alpha, zorder=2)
        ax.plot(freqs, response, linewidth=2.0, color=c["accent"], zorder=3)
        ax.axhline(0, color=c["muted"], linewidth=0.8, alpha=0.6, zorder=1)

        ax.set_xscale("log")
        ax.set_xlim(20, 20000)
        ax.set_ylim(-GRAPH_GAIN_LIMIT, GRAPH_GAIN_LIMIT)
        ticks = [20, 100, 1000, 10000, 20000]
        ax.set_xticks(ticks)
        ax.set_xticklabels([f"{t // 1000} kHz" if t >= 1000 else f"{t} Hz" for t in ticks])

        ax.grid(True, which="major", color=c["line"], linewidth=0.8, alpha=0.9)
        ax.grid(True, which="minor", color=c["line"], linewidth=0.5, alpha=0.4)
        for side, spine in ax.spines.items():
            spine.set_color(c["line"])
            spine.set_visible(side in ("left", "bottom"))
        ax.tick_params(colors=c["muted"], labelsize=8, which="both")

        ax.set_title("EQ Response", color=c["muted"], fontsize=10,
                     fontweight="bold", loc="left", fontfamily=self.font_ui, pad=10)
        ax.set_xlabel("Frequency (Hz)", color=c["muted"], fontsize=9)
        ax.set_ylabel("Gain (dB)", color=c["muted"], fontsize=9)
        self.canvas_graph.draw_idle()

    def _on_list_select(self, _event):
        sel = self.filter_list.curselection()
        if sel:
            self.selected = sel[0]
            self._load_editor_fields()
            self._draw_graph()

    def _load_editor_fields(self):
        """Show the primary band's values in the fields (without re-applying them)."""
        self._editor_recorded = False
        if not 0 <= self.selected < len(self.filters):
            return
        f = self.filters[self.selected]
        self._loading_editor = True
        try:
            self.field_vars["freq"].set(str(f["freq"]))
            self.field_vars["gain"].set(str(f["gain"]))
            self.field_vars["q"].set(
                f"{q_to_bw(float(f['q'])):.3f}" if self.bw_mode.get() else str(f["q"]))
            self.field_vars["type"].set(f["type"])
        finally:
            self._loading_editor = False

    def _clear_editor_fields(self):
        self._loading_editor = True
        try:
            for field in ("freq", "gain", "q"):
                self.field_vars[field].set("")
            self.field_vars["type"].set("PK")
        finally:
            self._loading_editor = False

    def _apply_field(self, field):
        """Live-apply one edited field to every selected band."""
        if self._loading_editor:
            return
        sel = self._selection()
        if not sel:
            return

        text = self.field_vars[field].get()
        if field == "type":
            value = text
        else:
            value = parse_float(text)
            if value is None or (field != "gain" and value <= 0):
                return
            if field == "q" and self.bw_mode.get():
                try:
                    value = bw_to_q(value)
                except OverflowError:
                    return
                if value <= 0:
                    return

        if not self._editor_recorded:
            self._record()
            self._editor_recorded = True
        for i in sel:
            self.filters[i][field] = value

        self._render_filter_rows()
        for i in sel:
            self.filter_list.selection_set(i)
        self._draw_graph()

    def _toggle_bw_mode(self):
        in_bw = self.bw_mode.get()
        self.q_label.config(text="Bandwidth (oct)" if in_bw else "Q")
        val = parse_float(self.field_vars["q"].get())
        if val is None or val <= 0:
            return
        try:
            converted = q_to_bw(val) if in_bw else bw_to_q(val)
        except OverflowError:
            return
        # Only the display unit changes, not the band: don't live-apply.
        self._loading_editor = True
        try:
            self.field_vars["q"].set(f"{converted:.3f}")
        finally:
            self._loading_editor = False

    # ------------------------------------------------------------------
    # Editor: band operations and undo
    # ------------------------------------------------------------------

    def set_filters(self, filters, preamp):
        """Replace the whole EQ (one undo step): from a file, the device or AutoEQ."""
        self._record()
        self.filters = active_filters(filters)
        self.selected = 0 if self.filters else -1
        self._set_preamp(preamp)
        self._refresh_editor()
        self._load_editor_fields()

    def _add_band(self):
        self._record()
        self.filters.append(dict(NEW_BAND))
        self.selected = len(self.filters) - 1
        self._refresh_editor()
        self._load_editor_fields()

    def _remove_bands(self, indices):
        self._record()
        for index in sorted(indices, reverse=True):
            del self.filters[index]
        self.selected = min(min(indices), len(self.filters) - 1) if self.filters else -1
        self._refresh_editor()
        self._load_editor_fields()

    def _delete_selected_bands(self):
        sel = self.filter_list.curselection()
        if sel:
            self._remove_bands(sel)

    def _delete_all_bands(self):
        if not messagebox.askyesno("Delete All Bands", "Remove all EQ bands?"):
            return
        self._record()
        self.filters.clear()
        self.selected = -1
        self._refresh_editor()
        self._clear_editor_fields()

    def _capture(self):
        return copy.deepcopy(self.filters), self.preamp_spin.get(), self.selected

    def _restore(self, state):
        filters, preamp, selected = state
        self.filters = filters
        self.selected = max(-1, min(selected, len(filters) - 1))
        self._set_preamp(preamp)
        self._refresh_editor()
        self._load_editor_fields()

    def _record(self):
        self.history.record(self._capture())

    def _undo(self):
        state = self.history.undo(self._capture())
        if state is not None:
            self._restore(state)

    def _redo(self):
        state = self.history.redo(self._capture())
        if state is not None:
            self._restore(state)

    # ------------------------------------------------------------------
    # Editor: mouse on the graph
    # ------------------------------------------------------------------

    def _band_at(self, event, max_pixels=14):
        """Index of the band handle nearest the cursor within `max_pixels`, or -1."""
        if event.x is None or event.y is None:
            return -1
        best_idx, best_dist = -1, float("inf")
        for i, f in enumerate(self.filters):
            x, y = self.ax.transData.transform((f["freq"], f["gain"]))
            dist = math.hypot(x - event.x, y - event.y)
            if dist < best_dist:
                best_idx, best_dist = i, dist
        return best_idx if best_dist <= max_pixels else -1

    def _on_press(self, event):
        if event.xdata is None or event.ydata is None or not 20 <= event.xdata <= 20000:
            return
        if event.button == 3:
            self._on_right_click(event)
            return

        index = self._band_at(event)
        if index >= 0:
            # Grab an existing band. A plain selecting click must not create an
            # undo step, so the snapshot waits for the first actual drag move.
            self._drag_recorded = False
            self.selected = index
            self._load_editor_fields()
            self._draw_graph()
        else:
            # Create a band; the pre-append snapshot also covers dragging it.
            self._record()
            self._drag_recorded = True
            self.filters.append({"type": "PK", "freq": round(float(event.xdata), 1),
                                 "gain": round(float(event.ydata), 1), "q": 1.0})
            self.selected = index = len(self.filters) - 1
            self._refresh_editor()
            self._load_editor_fields()
        self._drag_idx = index

    def _on_motion(self, event):
        if self._drag_idx is None or event.xdata is None or event.ydata is None:
            return
        if not self._drag_recorded:  # whole drag = one undo step
            self._record()
            self._drag_recorded = True
        band = self.filters[self._drag_idx]
        band["freq"] = round(max(20.0, min(20000.0, float(event.xdata))), 1)
        band["gain"] = round(max(-GRAPH_GAIN_LIMIT, min(GRAPH_GAIN_LIMIT, float(event.ydata))), 1)
        self._load_editor_fields()
        self._draw_graph()

    def _on_release(self, _event):
        if self._drag_idx is not None:
            self._drag_idx = None
            self._drag_recorded = False
            self._refresh_editor()

    def _on_right_click(self, event):
        index = self._band_at(event)
        if index >= 0 and messagebox.askyesno(
                "Delete Band",
                f"Delete band {index + 1} ({self.filters[index]['freq']:.1f} Hz)?"):
            self._remove_bands([index])

    # ------------------------------------------------------------------
    # Files
    # ------------------------------------------------------------------

    def _save_profile(self, then=None):
        if not self.filters:
            messagebox.showwarning("No Filters", "Add at least one EQ band first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                            filetypes=FILE_TYPES_PROFILE)
        if not path:
            return
        try:
            save_profile(path, parse_float(self.preamp_spin.get(), 0.0), self.filters)
        except Exception as e:
            messagebox.showerror("Save Error", str(e))
            return
        self.log(f"Profile saved to {path}\n")
        if then is not None:
            then()

    def _load_profile(self):
        path = filedialog.askopenfilename(filetypes=FILE_TYPES_PROFILE)
        if not path:
            return
        try:
            data = load_profile(path)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            return
        self.set_filters(data["filters"], data["preamp"])
        self.log(f"Loaded {len(self.filters)} filter(s) from {path}\n")

    # ------------------------------------------------------------------
    # Device
    # ------------------------------------------------------------------

    def _refresh_devices(self):
        self.device_list.delete(0, "end")
        self.devices = []
        self.selected_path = None  # the old pick may be unplugged by now
        if hid is None:
            self.log("hidapi not available; cannot list devices.\n")
            return

        self.devices = find_devices()
        if not self.devices:
            self.device_list.insert("end", "(no Walkplay-vendor devices found)")
        for d in self.devices:
            self.device_list.insert(
                "end", f"pid=0x{d['product_id']:04X} iface={d.get('interface_number')}  "
                       f"{d.get('product_string')}")

    def _on_device_select(self, _event):
        sel = self.device_list.curselection()
        if not sel or sel[0] >= len(self.devices):
            return
        d = self.devices[sel[0]]
        self.selected_path = d["path"]
        self.vid_entry.delete(0, "end")
        self.vid_entry.insert(0, f"0x{d['vendor_id']:04X}")
        self.pid_entry.delete(0, "end")
        self.pid_entry.insert(0, f"0x{d['product_id']:04X}")

    def _with_device(self, action, on_done=None):
        """Run action(dev) in the background on the selected device; errors go to the log."""
        vid = parse_int(self.vid_entry.get(), WALKPLAY_VENDOR_ID)
        pid = parse_int(self.pid_entry.get())
        path = self.selected_path

        def work():
            with device_session(vid, pid, path) as dev:
                return action(dev)

        self.run_task(work, on_done)

    def _get_slot(self):
        self._with_device(get_current_slot)

    def _enable(self):
        slot = parse_int(self.ed_slot_spin.get(), 0)

        def enable(dev):
            enable_peq(dev, True, slot_id=slot)
            print(f"PEQ enabled on slot {slot}")

        self._with_device(enable)

    def _disable(self):
        def disable(dev):
            enable_peq(dev, False)
            print("PEQ disabled")

        self._with_device(disable)

    def _load_from_device(self):
        if not messagebox.askyesno(
                "Load EQ from Device",
                "Load EQ from device? This will replace all current filters."):
            return
        max_filters = self.max_filters()
        buffer_db = parse_float(self.buffer_spin.get(), DEFAULT_GLOBAL_GAIN_BUFFER)
        self._with_device(
            lambda dev: pull_from_device(dev, max_filters, slot_hint=get_current_slot(dev),
                                         buffer_db=buffer_db),
            lambda result: self.set_filters(result["filters"], result["preamp"]))

    def _push(self, then=None):
        """Push the EQ; `then()` runs only after a successful push."""
        if not self.filters:
            messagebox.showwarning("No Filters", "Add at least one EQ band first.")
            return

        slot = parse_int(self.slot_spin.get(), 0)
        preamp = parse_float(self.preamp_spin.get(), 0)
        buffer_db = parse_float(self.buffer_spin.get(), DEFAULT_GLOBAL_GAIN_BUFFER)
        max_filters = self.max_filters()

        if len(self.filters) > max_filters and ask_choice(
                self, "Too Many Bands",
                f"You have {len(self.filters)} EQ bands, but the device is set to "
                f"support only {max_filters} filter slot(s) (see 'Max filters').\n\n"
                f"Only the first {max_filters} band(s) would be written to the "
                f"device — the rest would be silently dropped.\n\n"
                f"Reduce your EQ to {max_filters} band(s), correct 'Max filters' to "
                f"match your device, or push anyway (will break your EQ).",
                [("Cancel", None, "TButton"),
                 ("Push Anyway", "push", "Danger.TButton")],
                wraplength=420, focus="Cancel") != "push":
            return

        filters = pad_for_push(self.filters, max_filters)

        def push(dev):
            push_to_device(dev, slot, preamp, filters, buffer_db=buffer_db)
            enable_peq(dev, True, slot_id=slot)
            print(f"EQ pushed to device on slot {slot} ({len(filters)} slots)")

        self._with_device(push, then and (lambda _result: then()))

    # ------------------------------------------------------------------
    # Background tasks, log, lifecycle
    # ------------------------------------------------------------------

    def run_task(self, work, on_done=None, *, busy=None, error=None, on_error=None):
        """Run work() on a worker thread, then on_done(result) on the Tk thread.

        busy: (title, message) of a modal spinner shown meanwhile.
        error: (title, message) of an error box on failure; failures are
        always logged. on_error(): called (on the Tk thread) after a failure.
        """
        dlg = busy_dialog(self, *busy) if busy else None

        def finish(callback, *args):
            if dlg is not None:
                dlg.destroy()
            if callback is not None:
                callback(*args)

        def failed(exc):
            self.log(f"\nERROR: {exc}\n")
            if error:
                messagebox.showerror(error[0], f"{error[1]}:\n{exc}")
            if on_error is not None:
                on_error()

        def target():
            try:
                result = work()
            except Exception as exc:
                self._ui_calls.put((finish, failed, exc))
            else:
                self._ui_calls.put((finish, on_done, result))

        threading.Thread(target=target, daemon=True).start()

    def log(self, text):
        self.log_queue.put(text)

    def _poll_queues(self):
        texts = []
        while not self.log_queue.empty():
            texts.append(self.log_queue.get_nowait())
        if texts:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", "".join(texts))
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        while not self._ui_calls.empty():
            func, *args = self._ui_calls.get_nowait()
            func(*args)
        self._poll_after_id = self.after(50, self._poll_queues)

    def _on_close(self):
        choice = ask_choice(
            self, "Quit", "Do you really want to leave this application?",
            [("Quit", "quit", "Danger.TButton"),
             ("Cancel", None, "TButton"),
             ("Push to device and quit", "push", "TButton"),
             ("Save to file and quit", "save", "TButton")],
            enter="quit", focus="Quit")
        if choice == "quit":
            self.destroy()
        elif choice == "push":
            self._push(then=self.destroy)
        elif choice == "save":
            self._save_profile(then=self.destroy)


# ===========================================================================
# CLI
# ===========================================================================

def _cli_push(args):
    profile = load_profile(args.file)
    filters = [f for f in profile["filters"] if not filter_is_off(f)]
    if len(filters) > args.max_filters:
        print(f"Warning: {len(filters)} bands but only {args.max_filters} filter "
              f"slots; the device will drop the extra bands.")
    with device_session(args.vid, args.pid) as dev:
        push_to_device(dev, args.slot, profile["preamp"],
                       pad_for_push(filters, args.max_filters),
                       buffer_db=args.buffer, write_gain=not args.no_gain)
        if not args.no_enable:
            enable_peq(dev, True, slot_id=args.slot)
            print(f"PEQ enabled on slot {args.slot}")


def _cli_pull(args):
    with device_session(args.vid, args.pid) as dev:
        result = pull_from_device(dev, args.max_filters, slot_hint=get_current_slot(dev),
                                  buffer_db=args.buffer)
    save_profile(args.file, result["preamp"], result["filters"])
    print(f"Saved {len(result['filters'])} filter(s) to {args.file}")


def _build_parser():
    def hex_int(text):
        return int(text, 0)

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

    def add_device_args(parser):
        parser.add_argument("--buffer", type=float, default=DEFAULT_GLOBAL_GAIN_BUFFER,
                            help=f"Hardware gain buffer in dB "
                                 f"(default: {DEFAULT_GLOBAL_GAIN_BUFFER})")
        parser.add_argument("--max-filters", type=int, default=DEFAULT_MAX_FILTERS,
                            help=f"Number of filter slots on the device "
                                 f"(default: {DEFAULT_MAX_FILTERS})")
        parser.add_argument("--vid", type=hex_int, default=None,
                            help="Device VID in hex (default: 0x3302)")
        parser.add_argument("--pid", type=hex_int, default=None,
                            help="Device PID in hex (optional)")

    pp = sub.add_parser("push", help="Push a .txt profile to the device")
    pp.add_argument("file", help="Profile .txt file to push")
    pp.add_argument("--slot", type=int, default=0, help="Target PEQ slot (default: 0)")
    pp.add_argument("--no-gain", action="store_true",
                    help="Skip writing the global gain register")
    pp.add_argument("--no-enable", action="store_true",
                    help="Don't enable PEQ after pushing")
    add_device_args(pp)

    pu = sub.add_parser("pull", help="Pull the current EQ from the device to a .txt file")
    pu.add_argument("file", help="Output .txt file")
    add_device_args(pu)

    sub.add_parser("list", help="List connected Walkplay HID devices")
    return p


def main(argv=None):
    args = _build_parser().parse_args(argv)
    if args.cmd == "push":
        _cli_push(args)
    elif args.cmd == "pull":
        _cli_pull(args)
    elif args.cmd == "list":
        list_devices()
    else:
        EqLoaderGUI(graph=args.graph).mainloop()


if __name__ == "__main__":
    main()
