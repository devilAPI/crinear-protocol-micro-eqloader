# crinear-protocol-micro-eqloader
A work in progress python script as an alternative to eq.hangout.audio.
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
