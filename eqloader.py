#!/usr/bin/env python3

import copy
import math
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

try:
    import hid
except ImportError:
    hid = None


# ===========================================================================
# ---- Theme: "instrument panel" (graphite chassis, two-LED accent) ----
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

# Numeric readouts and th  ● Claude has context of ⧉ open files and ⧉ selected linese log read like a hardware display: monospace.
MONO_FONTS = ("JetBrains Mono", "DejaVu Sans Mono", "Consolas", "Menlo",
              "Courier New", "monospace")
UI_FONTS = ("Inter", "Segoe UI", "Helvetica Neue", "DejaVu Sans", "sans-serif")


def _pick_font(root, families):
    """Return the first font family actually installed, else the last fallback."""
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
# ---- Core protocol / CLI logic (ported unchanged from eqloader.py) ----
# ===========================================================================

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

FILTER_TYPE_TO_BYTE = {
    "LSQ": 1,
    "PK": 2,
    "HSQ": 3,
    "LP": 4,
    "HP": 5,
}

BYTE_TO_FILTER_TYPE = {
    v: k for k, v in FILTER_TYPE_TO_BYTE.items()
}

REPORT_LENGTH = 64

DEFAULT_GLOBAL_GAIN_BUFFER = -5
DEFAULT_DEVICE_HANDLES_PREGAIN = False
PROTOCOL_MICRO_PRODUCT_ID = 0xC20F
DEFAULT_MAX_FILTERS = 8


def list_devices():
    devices = [
        d for d in hid.enumerate()
        if d["vendor_id"] == WALKPLAY_VENDOR_ID
    ]

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

    candidates = [
        d for d in hid.enumerate()
        if d["vendor_id"] == vid
    ]

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
            f"Warning: multiple Walkplay devices/interfaces found "
            f"({names}). Using the first one. Select a specific device "
            f"in the list, or set PID."
        )

    dev.open_path(candidates[0]["path"])
    return dev


def send_report(dev, report_id, packet):
    payload = list(packet) + [0] * max(
        0,
        REPORT_LENGTH - len(packet)
    )

    dev.write(
        bytes([report_id]) +
        bytes(payload[:REPORT_LENGTH])
    )


def _read_report(dev, timeout_ms=200):
    data = dev.read(
        REPORT_LENGTH + 1,
        timeout_ms
    )

    if not data:
        return None

    return data[1:]


def wait_for_response(dev, expected_cmd, timeout=2.0):
    deadline = time.time() + timeout

    while time.time() < deadline:
        remaining_ms = max(
            1,
            int((deadline - time.time()) * 1000)
        )

        data = _read_report(
            dev,
            timeout_ms=min(200, remaining_ms)
        )

        if data is None:
            continue

        if len(data) > 1 and data[1] == expected_cmd:
            return data

    raise TimeoutError(
        f"Timeout waiting for response to cmd "
        f"0x{expected_cmd:02X}"
    )


def _to_i32(value):
    value &= 0xFFFFFFFF

    return (
        value - 0x100000000
        if value & 0x80000000
        else value
    )


def quantizer(d_arr, d_arr2):
    i_arr = [
        round(d * 1073741824)
        for d in d_arr
    ]

    i_arr2 = [
        round(d * 1073741824)
        for d in d_arr2
    ]

    return [
        i_arr2[0],
        i_arr2[1],
        i_arr2[2],
        -i_arr[1],
        -i_arr[2],
    ]


def compute_iir_filter(freq, gain, q):
    sqrt = math.sqrt(
        10 ** (gain / 20)
    )

    d3 = (
        freq * 6.283185307179586
    ) / 96000

    sin = math.sin(d3) / (2 * q)

    d4 = sin * sqrt
    d5 = sin / sqrt
    d6 = d5 + 1

    quantizer_data = quantizer(
        [
            1,
            (math.cos(d3) * -2) / d6,
            (1 - d5) / d6,
        ],
        [
            (d4 + 1) / d6,
            (math.cos(d3) * -2) / d6,
            (1 - d4) / d6,
        ],
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

    return [
        (value >> (8 * i)) & 0xFF
        for i in range(length)
    ]


def get_current_slot(dev):
    send_report(
        dev,
        REPORT_ID,
        [READ, CMD["VERSION"], END]
    )

    resp = wait_for_response(
        dev,
        CMD["VERSION"]
    )

    version_bytes = bytes(resp[3:6])

    try:
        version = version_bytes.decode(
            "ascii",
            errors="ignore"
        )
    except Exception:
        version = ""

    print(f"Firmware version: {version!r}")

    send_report(
        dev,
        REPORT_ID,
        [READ, CMD["PEQ_VALUES"], END]
    )

    resp = wait_for_response(
        dev,
        CMD["PEQ_VALUES"]
    )

    slot = (
        resp[35]
        if len(resp) > 35
        else -1
    )

    print(f"Current EQ slot: {slot}")

    return slot


def push_to_device(
    dev,
    slot,
    global_gain,
    filters,
    buffer_db=DEFAULT_GLOBAL_GAIN_BUFFER,
    write_gain=True,
):
    slot = int(slot)

    for i, f in enumerate(filters):

        b_arr = compute_iir_filter(
            f["freq"],
            f["gain"],
            f["q"]
        )

        packet = (
            [
                WRITE,
                CMD["PEQ_VALUES"],
                0x18,
                0x00,
                i,
                0x00,
                0x00,
            ]
            + b_arr
            + convert_to_byte_array(
                f["freq"],
                2
            )
            + convert_to_byte_array(
                round(f["q"] * 256),
                2
            )
            + convert_to_byte_array(
                round(f["gain"] * 256) & 0xFFFF,
                2
            )
            + [
                FILTER_TYPE_TO_BYTE.get(
                    f.get("type", "PK"),
                    2
                ),
                0x00,
                slot,
                END,
            ]
        )

        send_report(
            dev,
            REPORT_ID,
            packet
        )

        time.sleep(0.02)

    time.sleep(0.1)

    if write_gain:
        gain_to_write = round(
            min(
                0,
                global_gain - buffer_db
            )
        )

        write_global_gain(
            dev,
            gain_to_write
        )

        print(
            f"Set global gain register to "
            f"{gain_to_write} dB "
            f"(preamp {global_gain} dB, "
            f"hardware buffer {buffer_db} dB)"
        )

        time.sleep(0.05)

    send_report(
        dev,
        REPORT_ID,
        [WRITE, 0x05, END]
    )

    time.sleep(0.02)

    send_report(
        dev,
        REPORT_ID,
        [WRITE, 0x17, END]
    )

    time.sleep(0.02)

    send_report(
        dev,
        REPORT_ID,
        [
            WRITE,
            CMD["TEMP_WRITE"],
            0x04,
            0x00,
            0x00,
            0xFF,
            0xFF,
            END,
        ]
    )

    time.sleep(0.05)

    send_report(
        dev,
        REPORT_ID,
        [WRITE, CMD["FLASH_EQ"], END]
    )

    print(
        f"Pushed {len(filters)} filter(s) "
        f"to slot {slot} and flashed to device."
    )


def write_global_gain(dev, value_db):
    gain_value = round(value_db) & 0xFF

    send_report(
        dev,
        REPORT_ID,
        [
            WRITE,
            CMD["GLOBAL_GAIN"],
            0x02,
            0x00,
            gain_value,
        ]
    )


def read_global_gain(dev):
    send_report(
        dev,
        REPORT_ID,
        [
            READ,
            CMD["GLOBAL_GAIN"],
            0x00,
        ]
    )

    resp = wait_for_response(
        dev,
        CMD["GLOBAL_GAIN"],
        timeout=1.0
    )

    raw = resp[4]

    signed = (
        raw - 256
        if raw > 127
        else raw
    )

    return signed


def is_filter_disabled(ftype, freq, gain, q):
    """Return True when the device treats this band as OFF.

    The device stores an off band as an inert flat filter
    (INERT_FILTER: PK, Fc 100, Gain 0, Q 1), so a peaking/shelf band with
    zero gain is audibly inert and must be treated as off. LP/HP filters
    shape the signal regardless of gain, so only a fully-zero slot counts.
    """
    if ftype in ("PK", "LSQ", "HSQ"):
        return gain == 0

    return not (freq or q or gain)


def parse_filter_packet(packet):
    filter_index = packet[4]

    freq = (
        packet[27]
        | (packet[28] << 8)
    )

    q_raw = (
        packet[29]
        | (packet[30] << 8)
    )

    q = round(
        (q_raw / 256) * 100
    ) / 100

    gain_raw = (
        packet[31]
        | (packet[32] << 8)
    )

    if gain_raw > 32767:
        gain_raw -= 65536

    gain = round(
        (gain_raw / 256) * 100
    ) / 100

    ftype = BYTE_TO_FILTER_TYPE.get(
        packet[33],
        "PK"
    )

    return {
        "filterIndex": filter_index,
        "freq": freq,
        "q": q,
        "gain": gain,
        "type": ftype,
        "disabled": is_filter_disabled(
            ftype, freq, gain, q
        ),
    }


def pull_from_device(
    dev,
    max_filters,
    slot_hint=-1,
    timeout=10.0,
):
    filters = {}

    deadline = time.time() + timeout

    for i in range(max_filters):
        send_report(
            dev,
            REPORT_ID,
            [
                READ,
                CMD["PEQ_VALUES"],
                0x00,
                0x00,
                i,
                END,
            ]
        )

        time.sleep(0.05)

    time.sleep(0.1)

    while (
        len(filters) < max_filters
        and time.time() < deadline
    ):
        data = _read_report(
            dev,
            timeout_ms=200
        )

        if data is None or len(data) < 32:
            continue

        if data[1] != CMD["PEQ_VALUES"]:
            continue

        parsed = parse_filter_packet(data)

        filters[
            parsed["filterIndex"]
        ] = parsed

    if len(filters) < max_filters:
        print(
            f"Warning: only received "
            f"{len(filters)}/{max_filters} "
            f"filters before timeout."
        )

    try:
        global_gain = read_global_gain(dev)
    except TimeoutError:
        print(
            "Warning: could not read global gain."
        )

        global_gain = 0

    ordered = [
        filters[i]
        for i in sorted(filters.keys())
    ]

    return {
        "currentSlot": slot_hint,
        "globalGain": global_gain,
        "filters": ordered,
    }


def enable_peq(dev, enable, slot_id=0):
    if not enable:
        slot_id = 0x00

    send_report(
        dev,
        REPORT_ID,
        [
            WRITE,
            CMD["FLASH_EQ"],
            1 if enable else 0,
            slot_id,
            END,
        ]
    )


# ===========================================================================
# ---- Profile .txt format ----
# ===========================================================================

TXT_TYPE_TO_INTERNAL = {
    "LS": "LSQ",
    "HS": "HSQ",
    "PK": "PK",
    "LP": "LP",
    "HP": "HP",
}

INTERNAL_TYPE_TO_TXT = {
    v: k
    for k, v in TXT_TYPE_TO_INTERNAL.items()
}

INERT_FILTER = {
    "type": "PK",
    "freq": 100.0,
    "gain": 0.0,
    "q": 1.0,
}


_PREAMP_RE = re.compile(
    r'^\s*Preamp:\s*([+-]?[\d.,]+)\s*dB',
    re.IGNORECASE
)

_FILTER_RE = re.compile(
    r'^\s*Filter\s+\d+:\s*'
    r'(ON|OFF)\s+'
    r'(\S+)\s+'
    r'Fc\s+([\d.,]+)\s*Hz\s+'
    r'Gain\s+([+-]?[\d.,]+)\s*dB\s+'
    r'Q\s+([\d.,]+)',
    re.IGNORECASE
)


def _to_float(text):
    return float(
        text.strip().replace(",", ".")
    )


def fmt_num(value, decimals):
    return (
        f"{value:.{decimals}f}"
        .replace(".", ",")
    )


def load_profile(path):
    preamp = 0.0
    filters = []

    with open(path, "r") as fh:
        for line in fh:

            m = _PREAMP_RE.match(line)

            if m:
                preamp = _to_float(
                    m.group(1)
                )

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

            internal_type = (
                TXT_TYPE_TO_INTERNAL.get(
                    txt_type.upper(),
                    "PK"
                )
            )

            filters.append({
                "type": internal_type,
                "freq": _to_float(freq),
                "gain": _to_float(gain),
                "q": _to_float(q),
            })

    if not filters:
        raise ValueError(
            f"No 'Filter N: ...' lines found in {path}"
        )

    return {
        "preamp": preamp,
        "filters": filters,
    }


def save_profile(path, global_gain, filters):
    lines = [
        f"Preamp: {fmt_num(float(global_gain), 1)} dB"
    ]

    for i, f in enumerate(filters, start=1):

        disabled = f.get(
            "disabled",
            is_filter_disabled(
                f.get("type", "PK"),
                f["freq"],
                f["gain"],
                f["q"],
            )
        )

        state = (
            "OFF"
            if disabled
            else "ON"
        )

        txt_type = INTERNAL_TYPE_TO_TXT.get(
            f["type"],
            f["type"]
        )

        freq = float(f["freq"])

        lines.append(
            f"Filter {i}: {state} {txt_type} "
            f"Fc {fmt_num(freq, 1)} Hz "
            f"Gain {fmt_num(float(f['gain']), 1)} dB "
            f"Q {fmt_num(float(f['q']), 3)}"
        )

    with open(path, "w") as fh:
        fh.write(
            "\n".join(lines) + "\n"
        )


# ===========================================================================
# ---- GUI helpers ----
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


# ===========================================================================
# ---- GUI ----
# ===========================================================================

class EqLoaderGUI(tk.Tk):

    def __init__(self):
        super().__init__()

        self.title("Walkplay PEQ Loader")

        self.geometry("950x850")

        self.minsize(850, 700)

        self._apply_theme()

        self.log_queue = queue.Queue()

        self.selected_path = None

        self._devices_cache = []

        self.create_filters = [
            {
                "type": "PK",
                "freq": 1000.0,
                "gain": 0.0,
                "q": 1.0,
            }
        ]

        self.selected_filter = 0

        self._undo_stack = []
        self._redo_stack = []

        self._dragging_point_idx = None
        self._drag_snapshot_taken = False

        self._build_widgets()

        self._theme_classic_widgets()

        self._poll_log_queue()

        if hid is None:
            self._log(
                "ERROR: the 'hidapi' package is not installed.\n"
                "Run:  pip install hidapi\n"
                "Then restart this app.\n"
            )

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
        label_font = (self.font_ui, 10)
        mono_font = (self.font_mono, 10)

        style.configure(".",
                        background=c["chassis"],
                        foreground=c["ink"],
                        fieldbackground=c["input"],
                        bordercolor=c["line"],
                        lightcolor=c["line"],
                        darkcolor=c["line"],
                        troughcolor=c["panel"],
                        font=base_font)

        style.configure("TFrame", background=c["chassis"])
        style.configure("TLabel", background=c["chassis"],
                        foreground=c["ink"], font=label_font)
        # Muted caption label variant.
        style.configure("Muted.TLabel", background=c["chassis"],
                        foreground=c["muted"], font=(self.font_ui, 9))

        # Framed panels with a soft caption.
        style.configure("TLabelframe", background=c["chassis"],
                        bordercolor=c["line"], relief="solid", borderwidth=1)
        style.configure("TLabelframe.Label", background=c["chassis"],
                        foreground=c["muted"],
                        font=(self.font_ui, 9, "bold"))

        # Inputs.
        for widget in ("TEntry", "TSpinbox", "TCombobox"):
            style.configure(widget,
                            background=c["input"],
                            fieldbackground=c["input"],
                            foreground=c["ink"],
                            insertcolor=c["accent"],
                            bordercolor=c["line"],
                            arrowcolor=c["muted"],
                            padding=4)
            style.map(widget,
                      bordercolor=[("focus", c["accent"])],
                      foreground=[("disabled", c["muted"])])

        # readonly combobox field needs explicit state mappings —
        # style.configure values are overridden by the readonly state.
        style.map("TCombobox",
                  fieldbackground=[("readonly", c["input"]),
                                   ("disabled", c["panel"])],
                  foreground=[("readonly", c["ink"]),
                               ("disabled", c["muted"])],
                  selectbackground=[("readonly", c["input"])],
                  selectforeground=[("readonly", c["ink"])],
                  background=[("focus",  c["input"]),
                               ("active", c["line"]),
                               ("!focus", c["input"])],
                  arrowcolor=[("focus",  c["accent"]),
                               ("active", c["accent"]),
                               ("!focus", c["muted"])])

        # Buttons: quiet by default, cyan chassis-LED on hover.
        style.configure("TButton",
                        background=c["input"],
                        foreground=c["ink"],
                        bordercolor=c["line"],
                        focuscolor=c["accent"],
                        relief="flat",
                        padding=(10, 6),
                        font=(self.font_ui, 10))
        style.map("TButton",
                  background=[("pressed", c["accent_dk"]),
                             ("active", c["line"])],
                  foreground=[("pressed", c["chassis"])],
                  bordercolor=[("active", c["accent"])])

        # Primary call-to-action: solid cyan.
        style.configure("Accent.TButton",
                        background=c["accent"],
                        foreground=c["chassis"],
                        relief="flat",
                        padding=(10, 6),
                        font=(self.font_ui, 10, "bold"))
        style.map("Accent.TButton",
                  background=[("pressed", c["accent_dk"]),
                             ("active", c["accent_dk"])],
                  foreground=[("active", c["chassis"])])

        # Destructive action.
        style.configure("Danger.TButton",
                        background=c["input"],
                        foreground=c["danger"],
                        relief="flat",
                        padding=(10, 6))
        style.map("Danger.TButton",
                  background=[("active", c["danger"]),
                             ("pressed", c["danger"])],
                  foreground=[("active", c["chassis"]),
                              ("pressed", c["chassis"])])

        style.configure("TCheckbutton",
                        background=c["chassis"],
                        foreground=c["ink"],
                        focuscolor=c["accent"])
        style.map("TCheckbutton",
                  background=[("active", c["chassis"])],
                  indicatorcolor=[("selected", c["accent"]),
                                  ("!selected", c["input"])])

        # Notebook tabs read like channel selectors.
        style.configure("TNotebook", background=c["chassis"],
                        bordercolor=c["line"], tabmargins=(2, 4, 2, 0))
        style.configure("TNotebook.Tab",
                        background=c["panel"],
                        foreground=c["muted"],
                        bordercolor=c["line"],
                        padding=(14, 7),
                        font=(self.font_ui, 10))
        style.map("TNotebook.Tab",
                  background=[("selected", c["chassis"])],
                  foreground=[("selected", c["accent"]),
                              ("active", c["ink"])])

        # Scrollbars: thin, chassis-toned.
        for sb in ("Vertical.TScrollbar", "Horizontal.TScrollbar"):
            style.configure(sb,
                            background=c["input"],
                            troughcolor=c["panel"],
                            bordercolor=c["panel"],
                            arrowcolor=c["muted"])
            style.map(sb, background=[("active", c["line"])])

        # Combobox dropdown popup is a plain tk.Listbox — style via option_add.
        self.option_add("*TCombobox*Listbox.background",       c["input"])
        self.option_add("*TCombobox*Listbox.foreground",       c["ink"])
        self.option_add("*TCombobox*Listbox.selectBackground", c["accent"])
        self.option_add("*TCombobox*Listbox.selectForeground", c["chassis"])
        self.option_add("*TCombobox*Listbox.font",             (self.font_ui, 10))

    def _theme_classic_widgets(self):
        """Colour the non-ttk (classic tk) widgets to match the theme."""

        c = THEME
        list_opts = dict(
            bg=c["panel"],
            fg=c["ink"],
            selectbackground=c["accent"],
            selectforeground=c["chassis"],
            highlightthickness=1,
            highlightbackground=c["line"],
            highlightcolor=c["accent"],
            borderwidth=0,
            activestyle="none",
            font=(self.font_ui, 10),
        )
        for lb in (getattr(self, "device_list", None),
                   getattr(self, "filter_list", None)):
            if lb is not None:
                lb.configure(**list_opts)

        if getattr(self, "log_text", None) is not None:
            self.log_text.configure(
                bg=c["chassis"],
                fg=c["accent"],
                insertbackground=c["accent"],
                selectbackground=c["input"],
                selectforeground=c["ink"],
                highlightthickness=1,
                highlightbackground=c["line"],
                borderwidth=0,
                font=(self.font_mono, 9),
                padx=8,
                pady=6,
            )
            self.log_text.vbar.configure(
                bg=c["input"],
                troughcolor=c["panel"],
                activebackground=c["line"],
                highlightbackground=c["panel"],
                highlightcolor=c["panel"],
                borderwidth=0,
                relief="flat",
            )

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_widgets(self):

        self.grid_rowconfigure(2, weight=3)
        self.grid_rowconfigure(5, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        # --------------------------------------------------------------
        # Device
        # --------------------------------------------------------------

        dev_frame = ttk.LabelFrame(
            self,
            text="Device"
        )

        dev_frame.grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=8,
            pady=6
        )

        self.device_list = tk.Listbox(
            dev_frame,
            height=4
        )

        self.device_list.pack(
            fill="x",
            padx=6,
            pady=6,
            side="left",
            expand=True
        )

        self.device_list.bind(
            "<<ListboxSelect>>",
            self._on_device_select
        )

        btn_frame = ttk.Frame(dev_frame)

        btn_frame.pack(
            side="left",
            padx=6
        )

        ttk.Button(
            btn_frame,
            text="Refresh List",
            command=self._refresh_devices
        ).pack(
            fill="x",
            pady=2
        )

        ttk.Button(
            btn_frame,
            text="Get Slot / Version",
            command=self._get_slot
        ).pack(
            fill="x",
            pady=2
        )

        # --------------------------------------------------------------
        # VID / PID
        # --------------------------------------------------------------

        override_frame = ttk.Frame(self)

        override_frame.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=8,
            pady=2
        )

        ttk.Label(
            override_frame,
            text="VID (hex):"
        ).grid(
            row=0,
            column=0,
            sticky="w"
        )

        self.vid_entry = ttk.Entry(
            override_frame,
            width=10
        )

        self.vid_entry.insert(
            0,
            "0x3302"
        )

        self.vid_entry.grid(
            row=0,
            column=1,
            padx=4
        )

        ttk.Label(
            override_frame,
            text="PID (hex, optional):"
        ).grid(
            row=0,
            column=2,
            sticky="w"
        )

        self.pid_entry = ttk.Entry(
            override_frame,
            width=10
        )

        self.pid_entry.grid(
            row=0,
            column=3,
            padx=4
        )

        # --------------------------------------------------------------
        # Graph  (row 2, col 0)
        # --------------------------------------------------------------

        self.fig = Figure(
            figsize=(7, 4),
            dpi=100,
            facecolor=THEME["chassis"],
            layout="constrained",
        )

        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(THEME["panel"])

        # Container frame: grid_propagate(False) stops the canvas's own
        # size requests from forcing a main-window geometry recalculation
        # when matplotlib redraws on click.
        self.graph_frame = tk.Frame(self, bg=THEME["chassis"])
        graph_frame = self.graph_frame
        graph_frame.grid(
            row=2,
            column=0,
            sticky="nsew",
            padx=(8, 4),
            pady=(6, 2),
        )
        graph_frame.grid_propagate(False)

        self.canvas_graph = FigureCanvasTkAgg(self.fig, master=graph_frame)

        self.canvas_graph.mpl_connect("button_press_event",   self._on_press)
        self.canvas_graph.mpl_connect("motion_notify_event",  self._on_motion)
        self.canvas_graph.mpl_connect("button_release_event", self._on_release)
        self.canvas_graph.get_tk_widget().pack(fill="both", expand=True)

        # --------------------------------------------------------------
        # Filter list + editor  (row 3, col 0)
        # --------------------------------------------------------------

        ctrl = ttk.Frame(self)

        ctrl.grid(
            row=3,
            column=0,
            sticky="ew",
            padx=(8, 4),
            pady=2,
        )

        ctrl.columnconfigure(1, weight=1)

        list_frame = ttk.LabelFrame(ctrl, text="Filters")

        list_frame.grid(row=0, column=0, sticky="ns")

        self.filter_list = tk.Listbox(list_frame, height=7)

        self.filter_list.pack(fill="both", expand=True, padx=5, pady=5)

        self.filter_list.bind("<<ListboxSelect>>", self._create_select_band)

        edit = ttk.LabelFrame(ctrl, text="Selected Filter")

        edit.grid(row=0, column=1, sticky="nsew", padx=10)

        edit.columnconfigure(1, weight=1)

        for row_i, (lbl, var_name, default) in enumerate((
            ("Frequency (Hz)", "freq_var", "1000"),
            ("Gain (dB)",      "gain_var", "0"),
            ("Q",              "q_var",    "1.0"),
        )):
            lbl_widget = ttk.Label(edit, text=lbl)
            lbl_widget.grid(row=row_i, column=0, sticky="w", padx=5, pady=3)
            if var_name == "q_var":
                self.q_label = lbl_widget
            setattr(self, var_name, tk.StringVar(value=default))
            ttk.Entry(edit, textvariable=getattr(self, var_name)).grid(
                row=row_i, column=1, sticky="ew", padx=5, pady=3
            )

        ttk.Label(edit, text="Type").grid(
            row=3, column=0, sticky="w", padx=5, pady=3
        )

        self.type_var = tk.StringVar(value="PK")

        ttk.Combobox(
            edit,
            textvariable=self.type_var,
            values=["PK", "LSQ", "HSQ", "LP", "HP"],
            state="readonly",
        ).grid(row=3, column=1, sticky="ew", padx=5, pady=3)

        self.bw_mode = tk.BooleanVar(value=False)

        ttk.Checkbutton(
            edit,
            text="Show as Bandwidth (oct)",
            variable=self.bw_mode,
            command=self._toggle_bw_mode,
        ).grid(row=4, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        # --------------------------------------------------------------
        # Operations bar  (row 4, col 0)
        # Four compact sub-sections in a 2×2 grid
        # --------------------------------------------------------------

        ops = ttk.Frame(self)

        ops.grid(
            row=4,
            column=0,
            sticky="ew",
            padx=(8, 4),
            pady=2,
        )

        ops.columnconfigure(0, weight=1)
        ops.columnconfigure(1, weight=1)

        # -- Created EQ params (0,0) --

        push_created_frame = ttk.LabelFrame(ops, text="EQ")

        push_created_frame.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)

        for lbl, attr, default in (
            ("Slot",        "create_slot_spin",   None),
            ("Preamp (dB)", "create_preamp_entry", "0"),
            ("Buffer (dB)", "create_buffer_entry",
             fmt_num(float(DEFAULT_GLOBAL_GAIN_BUFFER), 1)),
        ):
            ttk.Label(push_created_frame, text=f"{lbl}:").pack(
                side="left", padx=(8, 2)
            )
            if lbl == "Slot":
                w = ttk.Spinbox(push_created_frame, from_=0, to=15, width=5)
                w.set(0)
            else:
                w = ttk.Entry(push_created_frame, width=7)
                w.insert(0, default)
            w.pack(side="left", padx=(0, 6))
            setattr(self, attr, w)

        # -- Enable / Disable PEQ (0,1) --

        ed_frame = ttk.LabelFrame(ops, text="PEQ Enable / Disable")

        ed_frame.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=2)

        ttk.Label(ed_frame, text="Slot:").pack(side="left", padx=(8, 2))

        self.ed_slot_spin = ttk.Spinbox(ed_frame, from_=0, to=15, width=5)
        self.ed_slot_spin.set(0)
        self.ed_slot_spin.pack(side="left", padx=(0, 8))

        ttk.Button(
            ed_frame, text="Enable PEQ",
            command=self._enable, style="Accent.TButton",
        ).pack(side="left", padx=4, pady=4)

        ttk.Button(
            ed_frame, text="Disable PEQ",
            command=self._disable, style="Danger.TButton",
        ).pack(side="left", padx=4, pady=4)

        # --------------------------------------------------------------
        # Log  (row 5, col 0)
        # --------------------------------------------------------------

        log_frame = ttk.LabelFrame(self, text="Log")

        log_frame.grid(
            row=5,
            column=0,
            sticky="nsew",
            padx=(8, 4),
            pady=6,
        )

        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=5, state="disabled"
        )

        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        # --------------------------------------------------------------
        # Actions panel  (col 1, rows 2–5)
        # --------------------------------------------------------------

        actions = ttk.LabelFrame(self, text="Actions")

        actions.grid(
            row=2,
            column=1,
            rowspan=4,
            sticky="nsew",
            padx=(0, 8),
            pady=6,
        )

        actions.columnconfigure(0, weight=1)

        _action_btns = (
            ("Reload Graph",            self._create_apply,            "Accent.TButton"),
            ("Add Band",         self._create_add_band,         "TButton"),
            ("Delete Band",      self._create_delete_band,      "Danger.TButton"),
            ("Delete All",       self._create_delete_all_bands, "Danger.TButton"),
            ("Load EQ from Device", self._create_load_from_device, "TButton"),
            ("Save Profile to File",     self._create_save_profile,     "TButton"),
            ("Load Profile from File",     self._create_load_profile,     "TButton"),
            ("Push EQ to Device",  self._create_push,             "Accent.TButton"),
        )

        for i, (text, cmd, style) in enumerate(_action_btns):
            actions.rowconfigure(i, weight=1)
            ttk.Button(
                actions, text=text, command=cmd, style=style,
            ).grid(row=i, column=0, sticky="nsew", padx=6, pady=2)

        # --------------------------------------------------------------
        # Initial state
        # --------------------------------------------------------------

        self.graph_frame.bind("<Configure>", self._on_graph_frame_resize)

        self.bind("<Control-z>", self._undo)
        self.bind("<Control-y>", self._redo)
        self.bind("<Control-Shift-z>", self._redo)

        self._refresh_create_tab()

        self._refresh_devices()

    # ==================================================================
    # Window resize
    # ==================================================================

    def _on_graph_frame_resize(self, event):
        canvas_widget = self.canvas_graph.get_tk_widget()
        if event.height < 150:
            canvas_widget.pack_forget()
        elif not canvas_widget.winfo_ismapped():
            canvas_widget.pack(fill="both", expand=True)

    # ==================================================================
    # Create / Editor
    # ==================================================================

    def _refresh_create_tab(self):

        self.filter_list.delete(
            0,
            "end"
        )

        for i, f in enumerate(
            self.create_filters
        ):
            self.filter_list.insert(
                "end",
                f"{i + 1}: "
                f"{f['freq']:.1f} Hz  "
                f"{f['gain']:.1f} dB  "
                f"Q {f['q']:.2f}  "
                f"{f['type']}"
            )

        if self.create_filters:

            if (
                self.selected_filter
                >= len(self.create_filters)
            ):
                self.selected_filter = (
                    len(self.create_filters) - 1
                )

            self.filter_list.selection_clear(
                0,
                "end"
            )

            self.filter_list.selection_set(
                self.selected_filter
            )

            self.filter_list.see(
                self.selected_filter
            )

        else:
            self.selected_filter = -1

        self._draw_response_graph()

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

        self.ax.clear()

        freqs = np.logspace(
            np.log10(20),
            np.log10(20000),
            1000
        )

        response = np.zeros(len(freqs))

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
                freqs,
                center_freq,
                gain,
                q,
                f.get("type", "PK"),
            )

            is_selected = index == self.selected_filter
            color = THEME["active"] if is_selected else THEME["accent"]

            # Amber glow halo around the active band's handle.
            if is_selected:
                self.ax.plot(
                    [center_freq], [gain],
                    marker="o", markersize=16,
                    color=THEME["active"], alpha=0.25,
                    zorder=4,
                )

            self.ax.plot(
                [center_freq], [gain],
                marker="o", markersize=9,
                markerfacecolor=color,
                markeredgecolor=THEME["chassis"],
                markeredgewidth=1.5,
                zorder=5,
            )

        c = THEME
        self.fig.set_facecolor(c["chassis"])
        self.ax.set_facecolor(c["panel"])

        # Filled scope trace with a soft glow underneath.
        self.ax.fill_between(freqs, response, 0,
                             color=c["accent"], alpha=0.10, zorder=1)
        for lw, a in ((5, 0.10), (3, 0.18)):  # glow layers
            self.ax.plot(freqs, response, linewidth=lw,
                         color=c["accent"], alpha=a, zorder=2)
        self.ax.plot(freqs, response, linewidth=2.0,
                     color=c["accent"], zorder=3)

        # 0 dB reference line.
        self.ax.axhline(0, color=c["muted"], linewidth=0.8,
                        alpha=0.6, zorder=1)

        self.ax.set_xscale("log")
        self.ax.set_xlim(20, 20000)
        self.ax.set_ylim(-15, 15)

        self.ax.grid(True, which="major", color=c["line"],
                     linewidth=0.8, alpha=0.9)
        self.ax.grid(True, which="minor", color=c["line"],
                     linewidth=0.5, alpha=0.4)

        # Frame: hairline spines, muted ticks/labels.
        for side, spine in self.ax.spines.items():
            spine.set_color(c["line"])
            spine.set_visible(side in ("left", "bottom"))
        self.ax.tick_params(colors=c["muted"], labelsize=8, which="both")

        self.ax.set_title("EQ Response", color=c["muted"],
                          fontsize=10, fontweight="bold", loc="left",
                          fontfamily=self.font_ui, pad=10)
        self.ax.set_xlabel("Frequency (Hz)", color=c["muted"], fontsize=9)
        self.ax.set_ylabel("Gain (dB)", color=c["muted"], fontsize=9)

        self.canvas_graph.draw_idle()

    def _create_add_band(self):

        self._snapshot()
        self.create_filters.append({
            "type": "PK",
            "freq": 1000.0,
            "gain": 0.0,
            "q": 1.0,
        })

        self.selected_filter = (
            len(self.create_filters) - 1
        )

        self._refresh_create_tab()

        self._load_selected_filter_into_editor()

    def _create_delete_band(self):

        sel = self.filter_list.curselection()

        if not sel:
            return

        index = sel[0]

        self._snapshot()
        del self.create_filters[index]

        if not self.create_filters:
            self.selected_filter = -1

        else:
            self.selected_filter = min(
                index,
                len(self.create_filters) - 1
            )

        self._refresh_create_tab()

        self._load_selected_filter_into_editor()

    def _create_delete_all_bands(self):

        if not messagebox.askyesno(
            "Delete All Bands",
            "Remove all EQ bands?"
        ):
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
            "Load EQ from device? This will replace all current filters."
        ):
            return

        def task():

            dev = self._open_selected_device()

            try:
                slot = get_current_slot(dev)

                result = pull_from_device(
                    dev,
                    max_filters=DEFAULT_MAX_FILTERS,
                    slot_hint=slot,
                )

            finally:
                dev.close()

            def apply():
                self._snapshot()
                self.create_filters = [
                    {
                        "type": f.get("type", "PK"),
                        "freq": float(f["freq"]) or 1000.0,
                        "gain": float(f["gain"]),
                        "q": float(f["q"]) or 1.0,
                    }
                    for f in result["filters"]
                    if not f.get("disabled", False)
                ]

                self.selected_filter = (
                    0 if self.create_filters else -1
                )

                self.create_preamp_entry.delete(0, "end")

                self.create_preamp_entry.insert(
                    0,
                    str(result["globalGain"])
                )

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

        if (
            self.selected_filter < 0
            or self.selected_filter
            >= len(self.create_filters)
        ):
            return

        f = self.create_filters[
            self.selected_filter
        ]

        self.freq_var.set(
            str(f["freq"])
        )

        self.gain_var.set(
            str(f["gain"])
        )

        q_val = float(f["q"])

        if self.bw_mode.get():
            bw = (
                2 * math.asinh(1 / (2 * max(q_val, 0.001)))
                / math.log(2)
            )
            self.q_var.set(f"{bw:.3f}")
        else:
            self.q_var.set(str(f["q"]))

        self.type_var.set(
            f["type"]
        )

    def _create_apply(self):

        if (
            self.selected_filter < 0
            or self.selected_filter
            >= len(self.create_filters)
        ):
            return

        try:
            freq = self._parse_float(
                self.freq_var.get()
            )

            gain = self._parse_float(
                self.gain_var.get()
            )

            q = self._parse_float(
                self.q_var.get()
            )

            if freq is None:
                raise ValueError(
                    "Frequency is invalid."
                )

            if gain is None:
                raise ValueError(
                    "Gain is invalid."
                )

            if q is None:
                raise ValueError(
                    "Q is invalid."
                )

            if self.bw_mode.get():
                if q <= 0:
                    raise ValueError(
                        "Bandwidth must be greater than 0."
                    )
                try:
                    q = 1 / (
                        2 * math.sinh(q * math.log(2) / 2)
                    )
                except OverflowError:
                    raise ValueError(
                        "Bandwidth value is too large."
                    )

            if freq <= 0:
                raise ValueError(
                    "Frequency must be greater than 0."
                )

            if q <= 0:
                raise ValueError(
                    "Q must be greater than 0."
                )

            if freq < 10 or freq > 30000:
                raise ValueError(
                    "Frequency should be between "
                    "10 Hz and 30000 Hz."
                )

        except ValueError as e:
            messagebox.showerror(
                "Invalid Filter",
                str(e)
            )

            return

        f = self.create_filters[
            self.selected_filter
        ]

        self._snapshot()
        f["freq"] = freq
        f["gain"] = gain
        f["q"] = q
        f["type"] = self.type_var.get()

        self._refresh_create_tab()

    def _toggle_bw_mode(self):

        val = self._parse_float(self.q_var.get())

        if val is None or val <= 0:
            self.q_label.config(
                text="Bandwidth (oct)"
                if self.bw_mode.get()
                else "Q"
            )
            return

        if self.bw_mode.get():
            bw = (
                2 * math.asinh(1 / (2 * max(val, 0.001)))
                / math.log(2)
            )
            self.q_var.set(f"{bw:.3f}")
            self.q_label.config(text="Bandwidth (oct)")
        else:
            try:
                q = 1 / (
                    2 * math.sinh(val * math.log(2) / 2)
                )
            except OverflowError:
                self.q_label.config(text="Q")
                return
            self.q_var.set(f"{q:.3f}")
            self.q_label.config(text="Q")

    def _snapshot(self):
        self._undo_stack.append((
            copy.deepcopy(self.create_filters),
            self.create_preamp_entry.get(),
            self.selected_filter,
        ))
        self._redo_stack.clear()

    def _undo(self, _event=None):

        if not self._undo_stack:
            return

        self._redo_stack.append((
            copy.deepcopy(self.create_filters),
            self.create_preamp_entry.get(),
            self.selected_filter,
        ))

        filters, preamp, sel = self._undo_stack.pop()
        self.create_filters = filters
        self.selected_filter = max(
            -1, min(sel, len(self.create_filters) - 1)
        )
        self.create_preamp_entry.delete(0, "end")
        self.create_preamp_entry.insert(0, preamp)
        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    def _redo(self, _event=None):

        if not self._redo_stack:
            return

        self._undo_stack.append((
            copy.deepcopy(self.create_filters),
            self.create_preamp_entry.get(),
            self.selected_filter,
        ))

        filters, preamp, sel = self._redo_stack.pop()
        self.create_filters = filters
        self.selected_filter = max(
            -1, min(sel, len(self.create_filters) - 1)
        )
        self.create_preamp_entry.delete(0, "end")
        self.create_preamp_entry.insert(0, preamp)
        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    # ==================================================================
    # Mouse drag-and-drop graph controls
    # ==================================================================

    def _find_closest_filter(self, event, max_pixels=14):
        """Return the index of the band nearest the cursor in screen
        pixels, or -1 if none is within ``max_pixels``."""

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
            # Grab an existing band. Don't snapshot yet: a plain click
            # that only selects should not create an undo step. The
            # snapshot is taken lazily on the first actual drag move.
            self.selected_filter = closest_idx
            self._dragging_point_idx = closest_idx
            self._drag_snapshot_taken = False
            self._load_selected_filter_into_editor()
            self._draw_response_graph()
        else:
            self._snapshot()
            self.create_filters.append({
                "type": "PK",
                "freq": round(freq, 1),
                "gain": round(gain, 1),
                "q": 1.0,
            })
            self.selected_filter = len(self.create_filters) - 1
            self._dragging_point_idx = self.selected_filter
            # The pre-append snapshot already covers creating and
            # positioning this new band as a single undo step.
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

        # Record the pre-drag state once, so the whole drag is a single
        # undoable step.
        if not self._drag_snapshot_taken:
            self._snapshot()
            self._drag_snapshot_taken = True

        self.create_filters[self._dragging_point_idx]["freq"] = round(freq, 1)
        self.create_filters[self._dragging_point_idx]["gain"] = round(gain, 1)

        self._load_selected_filter_into_editor()
        self._draw_response_graph()

    def _on_release(self, event):
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
            f"Delete band {closest_idx + 1} "
            f"({float(f['freq']):.1f} Hz)?"
        ):
            return

        self._snapshot()
        del self.create_filters[closest_idx]

        if not self.create_filters:
            self.selected_filter = -1
        else:
            self.selected_filter = min(
                closest_idx,
                len(self.create_filters) - 1,
            )

        self._refresh_create_tab()
        self._load_selected_filter_into_editor()

    def _create_push(self):

        if not self.create_filters:
            messagebox.showwarning(
                "No Filters",
                "Add at least one EQ band first."
            )

            return

        slot = self._parse_int(
            self.create_slot_spin.get(),
            0
        )

        preamp = self._parse_float(
            self.create_preamp_entry.get(),
            0
        )

        buffer_db = self._parse_float(
            self.create_buffer_entry.get(),
            DEFAULT_GLOBAL_GAIN_BUFFER
        )

        if slot is None:
            slot = 0

        if preamp is None:
            preamp = 0

        if buffer_db is None:
            buffer_db = DEFAULT_GLOBAL_GAIN_BUFFER

        filters = [
            dict(f)
            for f in self.create_filters
        ]

        def task():

            dev = self._open_selected_device()

            try:

                push_to_device(
                    dev,
                    slot=slot,
                    global_gain=preamp,
                    filters=filters,
                    buffer_db=buffer_db,
                    write_gain=True,
                )

                enable_peq(
                    dev,
                    True,
                    slot_id=slot
                )

                print(
                    f"Created EQ pushed to "
                    f"device on slot {slot}"
                )

            finally:
                dev.close()

        self._run_bg(task)

    def _create_save_profile(self):

        if not self.create_filters:
            messagebox.showwarning(
                "No Filters",
                "Add at least one EQ band first."
            )
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[
                ("Text files", "*.txt"),
                ("All files", "*.*"),
            ],
        )

        if not path:
            return

        preamp = self._parse_float(
            self.create_preamp_entry.get(),
            0.0,
        )

        try:
            save_profile(path, preamp, self.create_filters)
            self._log(f"Profile saved to {path}\n")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def _create_load_profile(self):

        path = filedialog.askopenfilename(
            filetypes=[
                ("Text files", "*.txt"),
                ("All files", "*.*"),
            ],
        )

        if not path:
            return

        try:
            data = load_profile(path)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            return

        self._snapshot()
        self.create_filters = [
            {
                "type": f.get("type", "PK"),
                "freq": float(f["freq"]) or 1000.0,
                "gain": float(f["gain"]),
                "q": float(f["q"]) or 1.0,
            }
            for f in data["filters"]
            if not f.get("disabled", is_filter_disabled(
                f.get("type", "PK"), f["freq"], f["gain"], f["q"]
            ))
        ]

        self.selected_filter = (
            0 if self.create_filters else -1
        )

        self.create_preamp_entry.delete(0, "end")
        self.create_preamp_entry.insert(0, str(data["preamp"]))

        self._refresh_create_tab()
        self._load_selected_filter_into_editor()
        self._log(
            f"Loaded {len(self.create_filters)} "
            f"filter(s) from {path}\n"
        )

    # ==================================================================
    # Device discovery
    # ==================================================================

    def _refresh_devices(self):

        self.device_list.delete(
            0,
            "end"
        )

        self._devices_cache = []

        if hid is None:

            self._log(
                "hidapi not available; "
                "cannot list devices.\n"
            )

            return

        found = [
            d
            for d in hid.enumerate()
            if d["vendor_id"]
            == WALKPLAY_VENDOR_ID
        ]

        if not found:

            self.device_list.insert(
                "end",
                "(no Walkplay-vendor devices found)"
            )

        for d in found:

            label = (
                f"pid=0x{d['product_id']:04X} "
                f"iface={d.get('interface_number')}  "
                f"{d.get('product_string')}"
            )

            self.device_list.insert(
                "end",
                label
            )

            self._devices_cache.append(
                d
            )

    def _on_device_select(self, _event):

        sel = self.device_list.curselection()

        if (
            not sel
            or not self._devices_cache
        ):
            return

        idx = sel[0]

        if idx >= len(
            self._devices_cache
        ):
            return

        d = self._devices_cache[idx]

        self.selected_path = d["path"]

        self.vid_entry.delete(
            0,
            "end"
        )

        self.vid_entry.insert(
            0,
            f"0x{d['vendor_id']:04X}"
        )

        self.pid_entry.delete(
            0,
            "end"
        )

        self.pid_entry.insert(
            0,
            f"0x{d['product_id']:04X}"
        )

    # ==================================================================
    # Logging
    # ==================================================================

    def _log(self, text):
        self.log_queue.put(text)

    def _poll_log_queue(self):

        while True:

            try:
                text = (
                    self.log_queue.get_nowait()
                )

            except queue.Empty:
                break

            self.log_text.configure(
                state="normal"
            )

            self.log_text.insert(
                "end",
                text
            )

            self.log_text.see(
                "end"
            )

            self.log_text.configure(
                state="disabled"
            )

        self.after(
            100,
            self._poll_log_queue
        )

    # ==================================================================
    # Parsing helpers
    # ==================================================================

    @staticmethod
    def _parse_int(
        s,
        default=None
    ):

        s = (s or "").strip()

        if not s:
            return default

        try:
            return int(
                s,
                0
            )

        except ValueError:
            return default

    @staticmethod
    def _parse_float(
        s,
        default=None
    ):

        s = (
            (s or "")
            .strip()
            .replace(",", ".")
        )

        if not s:
            return default

        try:
            return float(s)

        except ValueError:
            return default

    # ==================================================================
    # Device helpers
    # ==================================================================

    def _open_selected_device(self):

        if hid is None:
            raise RuntimeError(
                "hidapi not installed "
                "(pip install hidapi)"
            )

        vid = self._parse_int(
            self.vid_entry.get(),
            WALKPLAY_VENDOR_ID
        )

        pid = self._parse_int(
            self.pid_entry.get(),
            None
        )

        return open_device(
            vid=vid,
            pid=pid,
            path=self.selected_path
        )

    def _run_bg(self, fn):

        def target():

            old_stdout = sys.stdout
            old_stderr = sys.stderr

            sys.stdout = sys.stderr = (
                StdoutRedirector(
                    self.log_queue
                )
            )

            try:
                fn()

            except Exception as e:

                self._log(
                    f"\nERROR: {e}\n"
                )

            finally:

                sys.stdout = old_stdout
                sys.stderr = old_stderr

        threading.Thread(
            target=target,
            daemon=True
        ).start()

    # ==================================================================
    # Get slot
    # ==================================================================

    def _get_slot(self):

        def task():

            dev = self._open_selected_device()

            try:
                get_current_slot(dev)

            finally:
                dev.close()

        self._run_bg(task)

    # ==================================================================
    # Enable / Disable
    # ==================================================================

    def _enable(self):

        slot = self._parse_int(
            self.ed_slot_spin.get(),
            0
        )

        def task():

            dev = self._open_selected_device()

            try:

                enable_peq(
                    dev,
                    True,
                    slot_id=slot
                )

                print(
                    f"PEQ enabled on slot {slot}"
                )

            finally:
                dev.close()

        self._run_bg(task)

    def _disable(self):

        def task():

            dev = self._open_selected_device()

            try:

                enable_peq(
                    dev,
                    False
                )

                print(
                    "PEQ disabled"
                )

            finally:
                dev.close()

        self._run_bg(task)


# ===========================================================================
# ---- CLI ----
# ===========================================================================

def _cli_push(args):
    vid = int(args.vid, 0) if args.vid else WALKPLAY_VENDOR_ID
    pid = int(args.pid, 0) if args.pid else None

    profile = load_profile(args.file)
    filters = [
        f for f in profile["filters"]
        if not f.get("disabled", is_filter_disabled(
            f.get("type", "PK"), f["freq"], f["gain"], f["q"]
        ))
    ]

    dev = open_device(vid=vid, pid=pid)
    try:
        push_to_device(
            dev,
            slot=args.slot,
            global_gain=profile["preamp"],
            filters=filters,
            buffer_db=args.buffer,
            write_gain=not args.no_gain,
        )
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
        description="Walkplay PEQ loader — run without arguments to open the GUI.",
    )
    sub = p.add_subparsers(dest="cmd")

    # ---- push ----
    pp = sub.add_parser("push", help="Push a .txt profile to the device")
    pp.add_argument("file", help="Profile .txt file to push")
    pp.add_argument("--slot",     type=int,   default=0,
                    help="Target PEQ slot (default: 0)")
    pp.add_argument("--buffer",   type=float, default=DEFAULT_GLOBAL_GAIN_BUFFER,
                    help=f"Hardware gain buffer in dB (default: {DEFAULT_GLOBAL_GAIN_BUFFER})")
    pp.add_argument("--no-gain",  action="store_true",
                    help="Skip writing the global gain register")
    pp.add_argument("--no-enable", action="store_true",
                    help="Don't enable PEQ after pushing")
    pp.add_argument("--vid",      default=None, help="Device VID in hex (default: 0x3302)")
    pp.add_argument("--pid",      default=None, help="Device PID in hex (optional)")

    # ---- pull ----
    pu = sub.add_parser("pull", help="Pull the current EQ from the device to a .txt file")
    pu.add_argument("file", help="Output .txt file")
    pu.add_argument("--max-filters", type=int, default=DEFAULT_MAX_FILTERS,
                    help=f"Number of filter slots to read (default: {DEFAULT_MAX_FILTERS})")
    pu.add_argument("--vid", default=None, help="Device VID in hex (default: 0x3302)")
    pu.add_argument("--pid", default=None, help="Device PID in hex (optional)")

    # ---- list ----
    sub.add_parser("list", help="List connected Walkplay HID devices")

    return p


# ===========================================================================
# ---- Main ----
# ===========================================================================

if __name__ == "__main__":
    import argparse

    parser = _build_parser()
    args = parser.parse_args()

    if args.cmd == "push":
        _cli_push(args)
    elif args.cmd == "pull":
        _cli_pull(args)
    elif args.cmd == "list":
        _cli_list(args)
    else:
        app = EqLoaderGUI()
        app.mainloop()
