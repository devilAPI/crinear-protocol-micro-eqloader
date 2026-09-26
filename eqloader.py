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

        self._build_widgets()

        if self._graph_forced is False:
            self._set_graph_visible(False)

        self.after(150, self._check_initial_graph_visibility)
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
        self.max_filter_entry = ttk.Entry(override_frame, width=6)
        self.max_filter_entry.insert(0, str(DEFAULT_MAX_FILTERS))
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

        for row_i, (lbl, var_name, default) in enumerate((
            ("Frequency (Hz)", "freq_var", "1000"),
            ("Gain (dB)", "gain_var", "0"),
            ("Q", "q_var", "1.0"),
        )):
            lbl_widget = ttk.Label(edit, text=lbl)
            lbl_widget.grid(row=row_i, column=0, sticky="w", padx=5, pady=3)
            if var_name == "q_var":
                self.q_label = lbl_widget
            setattr(self, var_name, tk.StringVar(value=default))
            ttk.Entry(edit, textvariable=getattr(self, var_name)).grid(
                row=row_i, column=1, sticky="ew", padx=5, pady=3)

        ttk.Label(edit, text="Type").grid(row=3, column=0, sticky="w", padx=5, pady=3)
        self.type_var = tk.StringVar(value="PK")
        ttk.Combobox(edit, textvariable=self.type_var,
                     values=["PK", "LSQ", "HSQ", "LP", "HP"], state="readonly").grid(
            row=3, column=1, sticky="ew", padx=5, pady=3)

        self.bw_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(edit, text="Show as Bandwidth (oct)", variable=self.bw_mode,
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
            ("Slot", "create_slot_spin", None),
            ("Preamp (dB)", "create_preamp_entry", "0"),
            ("Buffer (dB)", "create_buffer_entry",
             fmt_num(float(DEFAULT_GLOBAL_GAIN_BUFFER), 1)),
        ):
            ttk.Label(push_created_frame, text=f"{lbl}:").pack(side="left", padx=(8, 2))
            if lbl == "Slot":
                w = ttk.Spinbox(push_created_frame, from_=0, to=15, width=5)
                w.set(0)
            else:
                w = ttk.Entry(push_created_frame, width=7)
                w.insert(0, default)
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

    def _check_initial_graph_visibility(self):
        self._layout_ready = True
        self._apply_graph_visibility()

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

    def _create_push(self):
        if not self.create_filters:
            messagebox.showwarning("No Filters", "Add at least one EQ band first.")
            return

        slot = self._parse_int(self.create_slot_spin.get(), 0)
        preamp = self._parse_float(self.create_preamp_entry.get(), 0)
        buffer_db = self._parse_float(
            self.create_buffer_entry.get(), DEFAULT_GLOBAL_GAIN_BUFFER)
        max_filters = self._parse_int(self.max_filter_entry.get(), DEFAULT_MAX_FILTERS)

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

        self._run_bg(task)

    def _create_save_profile(self):
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
        if messagebox.askyesno("Quit", "Do you really want to leave this application?"):
            self.destroy()

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
