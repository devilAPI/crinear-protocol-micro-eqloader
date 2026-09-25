#!/usr/bin/env python3
import math
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

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

FILTER_TYPE_TO_BYTE = {"LSQ": 1, "PK": 2, "HSQ": 3, "LP": 4, "HP": 5}
BYTE_TO_FILTER_TYPE = {v: k for k, v in FILTER_TYPE_TO_BYTE.items()}

REPORT_LENGTH = 64

DEFAULT_GLOBAL_GAIN_BUFFER = -5  # dB, confirmed for Protocol Micro (SchemeNo11)
DEFAULT_DEVICE_HANDLES_PREGAIN = False
PROTOCOL_MICRO_PRODUCT_ID = 0xC20F  # confirmed
DEFAULT_MAX_FILTERS = 8  # confirmed: 8 bands


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
            f"Refresh the device list, or set VID/PID manually."
        )
    if len(candidates) > 1:
        names = ", ".join(f"0x{d['product_id']:04X} ({d.get('product_string')})" for d in candidates)
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
    if not data:
        return None
    return data[1:]


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
    value = int(round(value))  # freq/gain/Q may arrive as float; bit-shifting needs int
    return [(value >> (8 * i)) & 0xFF for i in range(length)]


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
    send_report(dev, REPORT_ID, [WRITE, CMD["FLASH_EQ"], 1 if enable else 0, slot_id, END])


# ---- Profile .txt (ParametricEQ / site-export format) <-> device ----

TXT_TYPE_TO_INTERNAL = {"LS": "LSQ", "HS": "HSQ", "PK": "PK", "LP": "LP", "HP": "HP"}
INTERNAL_TYPE_TO_TXT = {v: k for k, v in TXT_TYPE_TO_INTERNAL.items()}

INERT_FILTER = {"type": "PK", "freq": 100.0, "gain": 0.0, "q": 1.0}

# Accept either "." or "," as the decimal separator when reading a profile
# (German-locale files commonly use a comma), and normalize to "." for
# float() before use.
_PREAMP_RE = re.compile(r'^\s*Preamp:\s*([+-]?[\d.,]+)\s*dB', re.IGNORECASE)
_FILTER_RE = re.compile(
    r'^\s*Filter\s+\d+:\s*(ON|OFF)\s+(\S+)\s+Fc\s+([\d.,]+)\s*Hz\s+'
    r'Gain\s+([+-]?[\d.,]+)\s*dB\s+Q\s+([\d.,]+)',
    re.IGNORECASE,
)


def _to_float(text):
    """Parse a number that may use '.' or ',' as the decimal separator."""
    return float(text.strip().replace(",", "."))


def fmt_num(value, decimals):
    """Format a number with `decimals` places using a comma as the decimal
    separator (German locale), e.g. fmt_num(20.5, 1) -> '20,5'."""
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
                filters.append(dict(INERT_FILTER))
                continue
            internal_type = TXT_TYPE_TO_INTERNAL.get(txt_type.upper(), "PK")
            filters.append({
                "type": internal_type,
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
        disabled = f.get("disabled", not (f["freq"] or f["q"] or f["gain"]))
        state = "OFF" if disabled else "ON"
        txt_type = INTERNAL_TYPE_TO_TXT.get(f["type"], f["type"])
        freq = float(f["freq"])  # kept as a float (not rounded to whole Hz)
        lines.append(
            f"Filter {i}: {state} {txt_type} Fc {fmt_num(freq, 1)} Hz "
            f"Gain {fmt_num(float(f['gain']), 1)} dB Q {fmt_num(float(f['q']), 3)}"
        )
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# ===========================================================================
# ---- GUI ----
# ===========================================================================

class StdoutRedirector:
    """Thread-safe write target that feeds a queue the GUI polls."""

    def __init__(self, q):
        self.q = q

    def write(self, text):
        if text:
            self.q.put(text)

    def flush(self):
        pass


class EqLoaderGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Walkplay PEQ Loader")
        self.geometry("720x640")

        self.log_queue = queue.Queue()
        self.selected_path = None
        self._devices_cache = []

        self._build_widgets()
        self._poll_log_queue()

        if hid is None:
            self._log("ERROR: the 'hidapi' package is not installed.\n"
                       "Run:  pip install hidapi\nThen restart this app.\n")

    # ---- layout ----
    def _build_widgets(self):
        dev_frame = ttk.LabelFrame(self, text="Device")
        dev_frame.pack(fill="x", padx=8, pady=6)

        self.device_list = tk.Listbox(dev_frame, height=5)
        self.device_list.pack(fill="x", padx=6, pady=6, side="left", expand=True)
        self.device_list.bind("<<ListboxSelect>>", self._on_device_select)

        btn_frame = ttk.Frame(dev_frame)
        btn_frame.pack(side="left", padx=6)
        ttk.Button(btn_frame, text="Refresh List", command=self._refresh_devices).pack(fill="x", pady=2)
        ttk.Button(btn_frame, text="Get Slot / Version", command=self._get_slot).pack(fill="x", pady=2)

        override_frame = ttk.Frame(self)
        override_frame.pack(fill="x", padx=8)
        ttk.Label(override_frame, text="VID (hex):").grid(row=0, column=0, sticky="w")
        self.vid_entry = ttk.Entry(override_frame, width=10)
        self.vid_entry.insert(0, "0x3302")
        self.vid_entry.grid(row=0, column=1, padx=4)
        ttk.Label(override_frame, text="PID (hex, optional):").grid(row=0, column=2, sticky="w")
        self.pid_entry = ttk.Entry(override_frame, width=10)
        self.pid_entry.grid(row=0, column=3, padx=4)
        nb = ttk.Notebook(self)
        nb.pack(fill="x", padx=8, pady=6)

        # Pull tab
        pull_tab = ttk.Frame(nb)
        nb.add(pull_tab, text="Pull")
        ttk.Label(pull_tab, text="Output file:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.pull_output_entry = ttk.Entry(pull_tab, width=40)
        self.pull_output_entry.insert(0, "pulled_profile.txt")
        self.pull_output_entry.grid(row=0, column=1, padx=4)
        ttk.Button(pull_tab, text="Browse...", command=self._browse_pull_output).grid(row=0, column=2, padx=4)

        ttk.Label(pull_tab, text="Max filters:").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        self.pull_maxfilters_spin = ttk.Spinbox(pull_tab, from_=1, to=32, width=6)
        self.pull_maxfilters_spin.set(DEFAULT_MAX_FILTERS)
        self.pull_maxfilters_spin.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Button(pull_tab, text="Pull from Device", command=self._pull).grid(
            row=2, column=0, columnspan=3, pady=8)

        # Push tab
        push_tab = ttk.Frame(nb)
        nb.add(push_tab, text="Push")
        ttk.Label(push_tab, text="Profile .txt:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.push_profile_entry = ttk.Entry(push_tab, width=40)
        self.push_profile_entry.grid(row=0, column=1, padx=4)
        ttk.Button(push_tab, text="Browse...", command=self._browse_push_profile).grid(row=0, column=2, padx=4)

        ttk.Label(push_tab, text="Slot:").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        self.push_slot_spin = ttk.Spinbox(push_tab, from_=0, to=15, width=6)
        self.push_slot_spin.set(0)
        self.push_slot_spin.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(push_tab, text="Gain buffer (dB):").grid(row=2, column=0, sticky="w", padx=4, pady=4)
        self.push_buffer_entry = ttk.Entry(push_tab, width=8)
        self.push_buffer_entry.insert(0, fmt_num(float(DEFAULT_GLOBAL_GAIN_BUFFER), 1))
        self.push_buffer_entry.grid(row=2, column=1, sticky="w", padx=4)

        self.push_no_write_gain = tk.BooleanVar(value=False)
        ttk.Checkbutton(push_tab, text="Skip writing global gain register",
                         variable=self.push_no_write_gain).grid(
            row=3, column=0, columnspan=2, sticky="w", padx=4)

        ttk.Button(push_tab, text="Push to Device", command=self._push).grid(
            row=4, column=0, columnspan=3, pady=8)

        # Enable/Disable tab
        ed_tab = ttk.Frame(nb)
        nb.add(ed_tab, text="Enable / Disable")
        ttk.Label(ed_tab, text="Slot:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.ed_slot_spin = ttk.Spinbox(ed_tab, from_=0, to=15, width=6)
        self.ed_slot_spin.set(0)
        self.ed_slot_spin.grid(row=0, column=1, sticky="w", padx=4)
        ttk.Button(ed_tab, text="Enable PEQ", command=self._enable).grid(row=1, column=0, pady=8, padx=4)
        ttk.Button(ed_tab, text="Disable PEQ", command=self._disable).grid(row=1, column=1, pady=8, padx=4)

        # Log
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.pack(fill="both", expand=True, padx=8, pady=6)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=16, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

        self._refresh_devices()

    # ---- device discovery ----
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
            label = f"pid=0x{d['product_id']:04X} iface={d.get('interface_number')}  {d.get('product_string')}"
            self.device_list.insert("end", label)
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

    # ---- helpers ----
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
        """Parse a float, accepting either '.' or ',' as the decimal separator."""
        s = (s or "").strip().replace(",", ".")
        if not s:
            return default
        try:
            return float(s)
        except ValueError:
            return default

    def _open_selected_device(self):
        if hid is None:
            raise RuntimeError("hidapi not installed (pip install hidapi)")
        vid = self._parse_int(self.vid_entry.get(), WALKPLAY_VENDOR_ID)
        pid = self._parse_int(self.pid_entry.get(), None)
        return open_device(vid=vid, pid=pid, path=self.selected_path)

    def _run_bg(self, fn):
        """Run fn() in a background thread; redirect its prints to the log
        and report any exception instead of crashing the GUI thread."""
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

    # ---- actions ----
    def _get_slot(self):
        def task():
            dev = self._open_selected_device()
            try:
                get_current_slot(dev)
            finally:
                dev.close()
        self._run_bg(task)

    def _browse_pull_output(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                             filetypes=[("Text files", "*.txt")])
        if path:
            self.pull_output_entry.delete(0, "end")
            self.pull_output_entry.insert(0, path)

    def _pull(self):
        output = self.pull_output_entry.get().strip()
        max_filters = self._parse_int(self.pull_maxfilters_spin.get(), DEFAULT_MAX_FILTERS)
        if not output:
            messagebox.showwarning("Missing output", "Choose an output file first.")
            return

        def task():
            dev = self._open_selected_device()
            try:
                slot = get_current_slot(dev)
                result = pull_from_device(dev, max_filters=max_filters, slot_hint=slot)
                save_profile(output, result["globalGain"], result["filters"])
                print(f"Saved {len(result['filters'])} filter(s) to {output}")
            finally:
                dev.close()
        self._run_bg(task)

    def _browse_push_profile(self):
        path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if path:
            self.push_profile_entry.delete(0, "end")
            self.push_profile_entry.insert(0, path)

    def _push(self):
        profile_path = self.push_profile_entry.get().strip()
        if not profile_path:
            messagebox.showwarning("Missing profile", "Choose a profile .txt file first.")
            return
        slot = self._parse_int(self.push_slot_spin.get(), 0)
        buffer_db = self._parse_float(self.push_buffer_entry.get(), DEFAULT_GLOBAL_GAIN_BUFFER)
        write_gain = not self.push_no_write_gain.get()

        def task():
            profile = load_profile(profile_path)
            dev = self._open_selected_device()
            try:
                push_to_device(dev, slot, profile["preamp"], profile["filters"],
                                buffer_db=buffer_db, write_gain=write_gain)
                enable_peq(dev, True, slot_id=slot)
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


if __name__ == "__main__":
    app = EqLoaderGUI()
    app.mainloop()
