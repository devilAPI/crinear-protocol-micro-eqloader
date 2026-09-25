#!/usr/bin/env python3

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
        "disabled": not (
            freq or q or gain
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
                filters.append(
                    dict(INERT_FILTER)
                )

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
            not (
                f["freq"]
                or f["q"]
                or f["gain"]
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

        self._build_widgets()

        self._poll_log_queue()

        if hid is None:
            self._log(
                "ERROR: the 'hidapi' package is not installed.\n"
                "Run:  pip install hidapi\n"
                "Then restart this app.\n"
            )

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_widgets(self):

        # --------------------------------------------------------------
        # Device
        # --------------------------------------------------------------

        dev_frame = ttk.LabelFrame(
            self,
            text="Device"
        )

        dev_frame.pack(
            fill="x",
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

        override_frame.pack(
            fill="x",
            padx=8
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
        # Main notebook
        # --------------------------------------------------------------

        nb = ttk.Notebook(self)

        nb.pack(
            fill="both",
            expand=True,
            padx=8,
            pady=6
        )

        # ==============================================================
        # EQ TAB
        # ==============================================================

        eq_tab = ttk.Frame(nb)

        nb.add(
            eq_tab,
            text="EQ"
        )

        # --------------------------------------------------------------
        # EQ notebook inside EQ tab
        # --------------------------------------------------------------

        eq_notebook = ttk.Notebook(eq_tab)

        eq_notebook.pack(
            fill="both",
            expand=True
        )

        # ==============================================================
        # Profile tab
        # ==============================================================

        profile_tab = ttk.Frame(eq_notebook)

        eq_notebook.add(
            profile_tab,
            text="Profile"
        )

        # --------------------------------------------------------------
        # Pull section
        # --------------------------------------------------------------

        pull_frame = ttk.LabelFrame(
            profile_tab,
            text="Pull from Device"
        )

        pull_frame.pack(
            fill="x",
            padx=8,
            pady=8
        )

        ttk.Label(
            pull_frame,
            text="Output file:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.pull_output_entry = ttk.Entry(
            pull_frame,
            width=55
        )

        self.pull_output_entry.insert(
            0,
            "pulled_profile.txt"
        )

        self.pull_output_entry.grid(
            row=0,
            column=1,
            padx=4,
            sticky="ew"
        )

        ttk.Button(
            pull_frame,
            text="Browse...",
            command=self._browse_pull_output
        ).grid(
            row=0,
            column=2,
            padx=4
        )

        ttk.Label(
            pull_frame,
            text="Max filters:"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.pull_maxfilters_spin = ttk.Spinbox(
            pull_frame,
            from_=1,
            to=32,
            width=6
        )

        self.pull_maxfilters_spin.set(
            DEFAULT_MAX_FILTERS
        )

        self.pull_maxfilters_spin.grid(
            row=1,
            column=1,
            sticky="w",
            padx=4
        )

        ttk.Button(
            pull_frame,
            text="Pull from Device",
            command=self._pull
        ).grid(
            row=2,
            column=0,
            columnspan=3,
            pady=8
        )

        pull_frame.columnconfigure(
            1,
            weight=1
        )

        # --------------------------------------------------------------
        # Push section
        # --------------------------------------------------------------

        push_frame = ttk.LabelFrame(
            profile_tab,
            text="Push Profile to Device"
        )

        push_frame.pack(
            fill="x",
            padx=8,
            pady=8
        )

        ttk.Label(
            push_frame,
            text="Profile .txt:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.push_profile_entry = ttk.Entry(
            push_frame,
            width=55
        )

        self.push_profile_entry.grid(
            row=0,
            column=1,
            padx=4,
            sticky="ew"
        )

        ttk.Button(
            push_frame,
            text="Browse...",
            command=self._browse_push_profile
        ).grid(
            row=0,
            column=2,
            padx=4
        )

        ttk.Label(
            push_frame,
            text="Slot:"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.push_slot_spin = ttk.Spinbox(
            push_frame,
            from_=0,
            to=15,
            width=6
        )

        self.push_slot_spin.set(0)

        self.push_slot_spin.grid(
            row=1,
            column=1,
            sticky="w",
            padx=4
        )

        ttk.Label(
            push_frame,
            text="Gain buffer (dB):"
        ).grid(
            row=2,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.push_buffer_entry = ttk.Entry(
            push_frame,
            width=8
        )

        self.push_buffer_entry.insert(
            0,
            fmt_num(
                float(DEFAULT_GLOBAL_GAIN_BUFFER),
                1
            )
        )

        self.push_buffer_entry.grid(
            row=2,
            column=1,
            sticky="w",
            padx=4
        )

        self.push_no_write_gain = tk.BooleanVar(
            value=False
        )

        ttk.Checkbutton(
            push_frame,
            text="Skip writing global gain register",
            variable=self.push_no_write_gain
        ).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w",
            padx=4
        )

        ttk.Button(
            push_frame,
            text="Push to Device",
            command=self._push
        ).grid(
            row=4,
            column=0,
            columnspan=3,
            pady=8
        )

        push_frame.columnconfigure(
            1,
            weight=1
        )

        # ==============================================================
        # Create / Editor tab
        # ==============================================================

        create_tab = ttk.Frame(eq_notebook)

        eq_notebook.add(
            create_tab,
            text="Editor"
        )

        # --------------------------------------------------------------
        # Graph
        # --------------------------------------------------------------

        self.fig = Figure(
            figsize=(7, 4),
            dpi=100
        )

        self.ax = self.fig.add_subplot(111)

        self.canvas_graph = FigureCanvasTkAgg(
            self.fig,
            master=create_tab
        )

        # Connect the click and drag events
        self.canvas_graph.mpl_connect("button_press_event", self._on_press)
        self.canvas_graph.mpl_connect("motion_notify_event", self._on_motion)
        self.canvas_graph.mpl_connect("button_release_event", self._on_release)
        self.canvas_graph.get_tk_widget().pack(
            fill="both",
            expand=True,
            padx=5,
            pady=5
        )

        # --------------------------------------------------------------
        # Editor controls
        # --------------------------------------------------------------

        ctrl = ttk.Frame(create_tab)

        ctrl.pack(
            fill="x",
            padx=6,
            pady=6
        )

        # Filter list

        list_frame = ttk.LabelFrame(
            ctrl,
            text="Filters"
        )

        list_frame.pack(
            side="left",
            fill="y"
        )

        self.filter_list = tk.Listbox(
            list_frame,
            height=9,
            width=32
        )

        self.filter_list.pack(
            padx=5,
            pady=5
        )

        self.filter_list.bind(
            "<<ListboxSelect>>",
            self._create_select_band
        )

        # Editor

        edit = ttk.LabelFrame(
            ctrl,
            text="Selected Filter"
        )

        edit.pack(
            side="left",
            fill="x",
            expand=True,
            padx=10
        )

        ttk.Label(
            edit,
            text="Frequency (Hz)"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            padx=5,
            pady=4
        )

        self.freq_var = tk.StringVar(
            value="1000"
        )

        ttk.Entry(
            edit,
            textvariable=self.freq_var,
            width=12
        ).grid(
            row=0,
            column=1,
            sticky="w",
            padx=5,
            pady=4
        )

        ttk.Label(
            edit,
            text="Gain (dB)"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            padx=5,
            pady=4
        )

        self.gain_var = tk.StringVar(
            value="0"
        )

        ttk.Entry(
            edit,
            textvariable=self.gain_var,
            width=12
        ).grid(
            row=1,
            column=1,
            sticky="w",
            padx=5,
            pady=4
        )

        ttk.Label(
            edit,
            text="Q"
        ).grid(
            row=2,
            column=0,
            sticky="w",
            padx=5,
            pady=4
        )

        self.q_var = tk.StringVar(
            value="1.0"
        )

        ttk.Entry(
            edit,
            textvariable=self.q_var,
            width=12
        ).grid(
            row=2,
            column=1,
            sticky="w",
            padx=5,
            pady=4
        )

        ttk.Label(
            edit,
            text="Type"
        ).grid(
            row=3,
            column=0,
            sticky="w",
            padx=5,
            pady=4
        )

        self.type_var = tk.StringVar(
            value="PK"
        )

        ttk.Combobox(
            edit,
            textvariable=self.type_var,
            values=[
                "PK",
                "LSQ",
                "HSQ",
                "LP",
                "HP",
            ],
            width=10,
            state="readonly"
        ).grid(
            row=3,
            column=1,
            sticky="w",
            padx=5,
            pady=4
        )

        # --------------------------------------------------------------
        # Editor buttons
        # --------------------------------------------------------------

        button_frame = ttk.Frame(edit)

        button_frame.grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="w",
            padx=5,
            pady=8
        )

        ttk.Button(
            button_frame,
            text="Apply",
            command=self._create_apply
        ).pack(
            side="left",
            padx=2
        )

        ttk.Button(
            button_frame,
            text="Add Band",
            command=self._create_add_band
        ).pack(
            side="left",
            padx=2
        )

        ttk.Button(
            button_frame,
            text="Delete Band",
            command=self._create_delete_band
        ).pack(
            side="left",
            padx=2
        )

        ttk.Button(
            button_frame,
            text="Delete All",
            command=self._create_delete_all_bands
        ).pack(
            side="left",
            padx=2
        )

        ttk.Button(
            button_frame,
            text="Load from Device",
            command=self._create_load_from_device
        ).pack(
            side="left",
            padx=2
        )

        # --------------------------------------------------------------
        # Created EQ push controls
        # --------------------------------------------------------------

        push_created_frame = ttk.LabelFrame(
            create_tab,
            text="Created EQ"
        )

        push_created_frame.pack(
            fill="x",
            padx=6,
            pady=6
        )

        ttk.Label(
            push_created_frame,
            text="Slot:"
        ).pack(
            side="left",
            padx=(8, 4)
        )

        self.create_slot_spin = ttk.Spinbox(
            push_created_frame,
            from_=0,
            to=15,
            width=6
        )

        self.create_slot_spin.set(0)

        self.create_slot_spin.pack(
            side="left",
            padx=4
        )

        ttk.Label(
            push_created_frame,
            text="Preamp (dB):"
        ).pack(
            side="left",
            padx=(15, 4)
        )

        self.create_preamp_entry = ttk.Entry(
            push_created_frame,
            width=8
        )

        self.create_preamp_entry.insert(
            0,
            "0"
        )

        self.create_preamp_entry.pack(
            side="left",
            padx=4
        )

        ttk.Label(
            push_created_frame,
            text="Buffer (dB):"
        ).pack(
            side="left",
            padx=(15, 4)
        )

        self.create_buffer_entry = ttk.Entry(
            push_created_frame,
            width=8
        )

        self.create_buffer_entry.insert(
            0,
            fmt_num(
                float(DEFAULT_GLOBAL_GAIN_BUFFER),
                1
            )
        )

        self.create_buffer_entry.pack(
            side="left",
            padx=4
        )

        ttk.Button(
            push_created_frame,
            text="Push Created EQ",
            command=self._create_push
        ).pack(
            side="left",
            padx=12,
            pady=6
        )

        # --------------------------------------------------------------
        # Enable / Disable
        # --------------------------------------------------------------

        ed_tab = ttk.Frame(nb)

        nb.add(
            ed_tab,
            text="Enable / Disable"
        )

        ttk.Label(
            ed_tab,
            text="Slot:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            padx=4,
            pady=4
        )

        self.ed_slot_spin = ttk.Spinbox(
            ed_tab,
            from_=0,
            to=15,
            width=6
        )

        self.ed_slot_spin.set(0)

        self.ed_slot_spin.grid(
            row=0,
            column=1,
            sticky="w",
            padx=4
        )

        ttk.Button(
            ed_tab,
            text="Enable PEQ",
            command=self._enable
        ).grid(
            row=1,
            column=0,
            pady=8,
            padx=4
        )

        ttk.Button(
            ed_tab,
            text="Disable PEQ",
            command=self._disable
        ).grid(
            row=1,
            column=1,
            pady=8,
            padx=4
        )

        # --------------------------------------------------------------
        # Log
        # --------------------------------------------------------------

        log_frame = ttk.LabelFrame(
            self,
            text="Log"
        )

        log_frame.pack(
            fill="both",
            expand=False,
            padx=8,
            pady=6
        )

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=9,
            state="disabled"
        )

        self.log_text.pack(
            fill="both",
            expand=True,
            padx=4,
            pady=4
        )

        # --------------------------------------------------------------
        # Initial state
        # --------------------------------------------------------------

        self._refresh_create_tab()

        self._refresh_devices()

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

    def _draw_response_graph(self):

        self.ax.clear()

        freqs = np.logspace(
            np.log10(20),
            np.log10(20000),
            1000
        )

        response = np.zeros_like(freqs)

        for index, f in enumerate(
            self.create_filters
        ):

            try:
                center_freq = float(
                    f["freq"]
                )

                gain = float(
                    f["gain"]
                )

                q = max(
                    float(f["q"]),
                    0.01
                )

            except (ValueError, TypeError):
                continue

            if center_freq <= 0:
                continue

            center = np.log10(
                center_freq
            )

            width = 1.0 / q

            response += (
                gain
                * np.exp(
                    -(
                        (
                            np.log10(freqs)
                            - center
                        ) ** 2
                    )
                    /
                    (
                        2
                        * (width / 8) ** 2
                    )
                )
            )

            color = (
                "red"
                if index == self.selected_filter
                else "black"
            )

            self.ax.plot(
                [center_freq],
                [gain],
                marker="o",
                markersize=8,
                color=color
            )

        self.ax.plot(
            freqs,
            response,
            linewidth=2,
            color="tab:blue"
        )

        self.ax.axhline(
            0,
            color="gray",
            linewidth=0.8
        )

        self.ax.set_xscale(
            "log"
        )

        self.ax.set_xlim(
            20,
            20000
        )

        self.ax.set_ylim(
            -15,
            15
        )

        self.ax.grid(
            True,
            which="both",
            alpha=0.3
        )

        self.ax.set_title(
            "EQ Response Preview"
        )

        self.ax.set_xlabel(
            "Frequency (Hz)"
        )

        self.ax.set_ylabel(
            "Gain (dB)"
        )

        self.canvas_graph.draw_idle()

    def _create_add_band(self):

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

        self.create_filters.clear()

        self.selected_filter = -1

        self._refresh_create_tab()

        self.freq_var.set("")
        self.gain_var.set("")
        self.q_var.set("")
        self.type_var.set("PK")

    def _create_load_from_device(self):

        if not messagebox.askyesno(
            "Load from Device",
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

        self.q_var.set(
            str(f["q"])
        )

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

        f["freq"] = freq
        f["gain"] = gain
        f["q"] = q
        f["type"] = self.type_var.get()

        self._refresh_create_tab()

    # ==================================================================
    # Mouse drag-and-drop graph controls
    # ==================================================================

    def _on_press(self, event):
        if event.xdata is None or event.ydata is None:
            return

        freq = float(event.xdata)
        gain = float(event.ydata)

        if freq < 20 or freq > 20000:
            return

        if not hasattr(self, "_dragging_point_idx"):
            self._dragging_point_idx = None

        click_x_log = math.log10(freq)
        closest_idx = -1
        min_dist = float('inf')

        for i, f in enumerate(self.create_filters):
            px = f.get("freq", 0)
            py = f.get("gain", 0)
            if px <= 0:
                continue
            dist = math.hypot((math.log10(px) - click_x_log) * 10, py - gain)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        if min_dist < 2.0:
            self.selected_filter = closest_idx
            self._dragging_point_idx = closest_idx
            self._load_selected_filter_into_editor()
            self._draw_response_graph()
        else:
            self.create_filters.append({
                "type": "PK",
                "freq": round(freq, 1),
                "gain": round(gain, 1),
                "q": 1.0,
            })
            self.selected_filter = len(self.create_filters) - 1
            self._dragging_point_idx = self.selected_filter
            self._refresh_create_tab()
            self._load_selected_filter_into_editor()

    def _on_motion(self, event):
        if getattr(self, "_dragging_point_idx", None) is None:
            return
        if event.xdata is None or event.ydata is None:
            return

        freq = max(20.0, min(20000.0, float(event.xdata)))
        gain = max(-15.0, min(15.0, float(event.ydata)))

        self.create_filters[self._dragging_point_idx]["freq"] = round(freq, 1)
        self.create_filters[self._dragging_point_idx]["gain"] = round(gain, 1)

        self._load_selected_filter_into_editor()
        self._draw_response_graph()

    def _on_release(self, event):
        if getattr(self, "_dragging_point_idx", None) is not None:
            self._dragging_point_idx = None
            self._refresh_create_tab()

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
    # Pull
    # ==================================================================

    def _browse_pull_output(self):

        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[
                ("Text files", "*.txt")
            ]
        )

        if path:

            self.pull_output_entry.delete(
                0,
                "end"
            )

            self.pull_output_entry.insert(
                0,
                path
            )

    def _pull(self):

        output = (
            self.pull_output_entry
            .get()
            .strip()
        )

        max_filters = self._parse_int(
            self.pull_maxfilters_spin.get(),
            DEFAULT_MAX_FILTERS
        )

        if not output:

            messagebox.showwarning(
                "Missing output",
                "Choose an output file first."
            )

            return

        def task():

            dev = self._open_selected_device()

            try:

                slot = get_current_slot(
                    dev
                )

                result = pull_from_device(
                    dev,
                    max_filters=max_filters,
                    slot_hint=slot
                )

                save_profile(
                    output,
                    result["globalGain"],
                    result["filters"]
                )

                print(
                    f"Saved "
                    f"{len(result['filters'])} "
                    f"filter(s) to {output}"
                )

            finally:
                dev.close()

        self._run_bg(task)

    # ==================================================================
    # Push
    # ==================================================================

    def _browse_push_profile(self):

        path = filedialog.askopenfilename(
            filetypes=[
                ("Text files", "*.txt"),
                ("All files", "*.*"),
            ]
        )

        if path:

            self.push_profile_entry.delete(
                0,
                "end"
            )

            self.push_profile_entry.insert(
                0,
                path
            )

    def _push(self):

        profile_path = (
            self.push_profile_entry
            .get()
            .strip()
        )

        if not profile_path:

            messagebox.showwarning(
                "Missing profile",
                "Choose a profile .txt file first."
            )

            return

        slot = self._parse_int(
            self.push_slot_spin.get(),
            0
        )

        buffer_db = self._parse_float(
            self.push_buffer_entry.get(),
            DEFAULT_GLOBAL_GAIN_BUFFER
        )

        write_gain = (
            not self.push_no_write_gain.get()
        )

        def task():

            profile = load_profile(
                profile_path
            )

            dev = self._open_selected_device()

            try:

                push_to_device(
                    dev,
                    slot,
                    profile["preamp"],
                    profile["filters"],
                    buffer_db=buffer_db,
                    write_gain=write_gain
                )

                enable_peq(
                    dev,
                    True,
                    slot_id=slot
                )

                print(
                    f"Profile pushed and PEQ "
                    f"enabled on slot {slot}"
                )

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
# ---- Main ----
# ===========================================================================

if __name__ == "__main__":
    app = EqLoaderGUI()
    app.mainloop()
