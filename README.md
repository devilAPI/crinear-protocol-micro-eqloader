# walkplay-eqloader

A standalone desktop tool for pushing and editing parametric EQ profiles on Walkplay-based USB DAC dongles (e.g. Crinear Protocol Micro) — no internet connection or proprietary app required.

![Screenshot](screenshots/screenshot.png)

## Features

- **Visual EQ editor** — drag band handles directly on the frequency response graph to tune frequency and gain interactively. Click empty space to add a new band; right-click a handle to delete it.
- **5 filter types** — Peaking (PK), Low Shelf (LSQ), High Shelf (HSQ), Low Pass (LP), High Pass (HP).
- **Q / Bandwidth toggle** — switch the Q field to octave bandwidth and back without losing precision.
- **Push to device** — write the current EQ to any PEQ slot with a configurable preamp level and hardware gain buffer.
- **Load from device** — read the current EQ back from the dongle into the editor.
- **Save / load profiles** — read and write the standard `.txt` format used by eq.hangout.audio and EqualizerAPO, so existing community profiles work out of the box.
- **OFF-band handling** — `OFF` bands and peaking/shelf filters with zero gain are automatically skipped when loading, matching the device's own behaviour.
- **Enable / disable PEQ** — toggle the hardware EQ on or off per slot without touching the stored profile.
- **Undo / redo** — full history for all editor changes (Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z).
- **Dark instrument-panel UI** — themed to match the feel of the hardware it talks to.

## Requirements

```
pip install hidapi matplotlib numpy
```

Standard library modules (`tkinter`, `math`, `threading`, etc.) are included with Python 3.

On Linux you may need udev rules or to run as root for raw HID access:

```
sudo python3 eqloader.py
```

## Usage

### Starting up

```
python3 eqloader.py
```

Plug in your dongle, then click **Refresh List** — it will appear in the Device list at the top. Select it to target it for all push/pull operations. If you have multiple Walkplay devices, pick the right one or set the PID field manually.

### Building an EQ from scratch

1. Click anywhere on the graph to place a new band at that frequency and gain.
2. Drag the band handle to adjust it, or type exact values in the **Selected Filter** panel on the left.
3. Choose a filter type from the **Type** dropdown (PK, LSQ, HSQ, LP, HP). Q becomes less relevant for shelves and is hidden from the graph but still editable.
4. Add as many bands as the device supports (default: 8). Use **Delete Band** or right-click a handle on the graph to remove one.
5. Set **Slot**, **Preamp**, and **Buffer** in the Created EQ row, then click **Push Created EQ** in the Actions panel.

### Loading an existing profile

1. Click **Load Profile** in the Actions panel and select a `.txt` file.
2. The bands load into the editor — disabled (`OFF`) and inert bands are filtered out automatically.
3. Review the curve on the graph, tweak if needed, then push.

### Backing up what's on the device

1. Click **Load from Device** — the current EQ is pulled and shown in the editor.
2. Click **Save Profile** to write it to a `.txt` file you can keep or share.

### Enabling / disabling the EQ

**Note:** this is note suported on evey device

Use the **PEQ Enable / Disable** row to turn the hardware EQ on or off for a given slot without overwriting the stored profile. Useful for quick A/B comparisons.

### Keyboard shortcuts

| Shortcut | Action |
|---|---|
| Ctrl+Z | Undo |
| Ctrl+Y / Ctrl+Shift+Z | Redo |

## Profile format

Profiles follow the EqualizerAPO / eq.hangout.audio `.txt` convention:

```
Preamp: -6,0 dB
Filter 1: ON PK Fc 1000,0 Hz Gain 3,5 dB Q 1,000
Filter 2: ON LSQ Fc 80,0 Hz Gain -2,0 dB Q 0,707
Filter 3: OFF PK Fc 100,0 Hz Gain 0,0 dB Q 1,000
```

`OFF` bands and peaking/shelf filters with zero gain are treated as disabled and are not pushed to the device.

## Supported hardware

Any Walkplay-vendor HID device (VID `0x3302`). Tested on the **Crinear Protocol Micro** (PID `0xC20F`). May work on other Walkplay dongles — open an issue if yours behaves differently.
