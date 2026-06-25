# CHIRP Radtel RT-1000 Pro Plugin

A community [CHIRP](https://chirpmyradio.com/) driver for the **Radtel RT-1000 Pro**.
It lets you download from, edit, and (experimentally) upload to the radio using
the open-source CHIRP CPS instead of the vendor's Windows-only software.

The driver was produced from clean-room observations of the Radtel RT-1000 Pro
CPS V1.06 memory map and serial protocol. It contains **no copied CPS source
code**, and is licensed under the GNU GPL v3 (or later).

> ⚠️ **Experimental.** Reading (download) follows the CPS clone sequence and is
> the safe path. Writing (upload) is implemented from the CPS state machine.
> Always download and save a backup image from *your* radio before writing
> anything to it.

## Features

- **Channels** — 1024 memories (48 bytes each, starting at `0x4000`):
  - RX/TX frequency, with duplex `+`/`-`/`split` and RX-only via duplex `off`
  - Modes: FM, NFM, AM, USB, LSB (the radio's single SSB value maps to USB/LSB;
    it decodes back as USB)
  - CTCSS / DCS tones (Tone, TSQL, DTCS, and cross modes), with DCS polarity
  - Power levels (Low ≈ 1 W, High ≈ 5 W), skip/scan flag, and a 16-character name
- **Zones** — exposed through CHIRP's **Banks** tab (250 zones, up to 250
  members each, with editable zone names)
- **Settings** — a broad settings editor covering:
  - Display / UI, startup, power-save and timers, scanning, RF / spectrum,
    audio / tones, work ranges, and programmable key functions
  - **DTMF** code table (20 entries) and DTMF timing/gain parameters
  - **Broadcast FM** presets (80 entries: enable, frequency, alias)
  - **GPS** and **APRS** configuration (callsign, paths, beacon timers, MIC-E,
    DIGI, etc.)

### Not editable in this build

GPS track/record logs, APRS packet/station log tables, and the startup bitmap
data are intentionally left untouched. The SW/MW/LW receiver parameters inside
broadcast-FM records are also preserved as-is.

## Receive / transmit coverage

CHIRP's valid receive bands are exposed as:

| Band | Range |
| --- | --- |
| LW RX | 153–279 kHz |
| MW RX | 520–1710 kHz |
| SW/CB RX | 2–32 MHz (AM/SSB covers 26–32 MHz) |
| VHF/UHF/extended RX | 64–999 MHz (continuous) |

Transmit is governed by the radio's firmware and region. Airband and other TX
ranges may be firmware-disabled or legally restricted — **this driver does not
attempt to unlock disabled transmit.**

## Requirements

- [CHIRP](https://chirpmyradio.com/projects/chirp/wiki/Download) (a recent
  build with module-loading support)
- The standard RT-1000 Pro USB programming cable
- Serial settings: **115200 baud, 8N1** (the CPS default)

## Installation

CHIRP can load this driver as an external module without rebuilding CHIRP:

1. Enable developer mode: **Help → Developer mode**, then restart CHIRP. This
   exposes the **Load Module** option.
2. Download [`radtel_rt1000pro.py`](radtel_rt1000pro.py) from this repository.
3. In CHIRP, open **File → Load Module** and select the `radtel_rt1000pro.py` file.
4. The radio appears as **Radtel RT-1000 Pro** in the radio list.

> The module must be re-loaded each time you restart CHIRP.

## Usage

### Download (recommended first step)

1. Connect the radio with the programming cable and power it on.
2. In CHIRP: **Radio → Download From Radio**.
3. Select your serial port and choose **Radtel / RT-1000 Pro**.
4. Save the resulting image as a backup (**File → Save As**) before making any edits.

### Edit

Use the **Memories** tab for channels, the **Banks** tab for zones, and the
**Settings** tab for everything else.

### Upload (experimental)

1. Keep a known-good backup image made from the *same* radio.
2. **Radio → Upload To Radio**.

Uploads are still experimental, so treat this as use-at-your-own-risk and keep
that backup handy.

## How it works (technical notes)

- The clone image is **1 MiB**, but the CPS only reads/writes selected **1 KiB**
  blocks. The driver mirrors the CPS read/write block sequence.
- Connect/end handshakes and per-block checksums follow the CPS framing
  (`0x52` data blocks, `0x06` ACK, 8-bit additive checksum).
- Key memory-map offsets used by the driver:

  | Region | Offset |
  | --- | --- |
  | Channels | `0x4000` (48 B × 1024) |
  | Config block 1 | `0x2000` |
  | Config block 2 | `0x1C250` |
  | Zones | `0x1E000` (520 B × 250) |
  | Broadcast FM | `0x0F0000` |
  | Bluetooth | `0x0FE000` |
  | GPS | `0x0FE400` |
  | APRS | `0x0FE800` |

- **RX-only channels:** set CHIRP **Duplex = off**. This maps to the CPS
  "Only RX" selector (channel byte +0, bits 4–5). The TX frequency field is
  deliberately left populated so the radio/CPS does not treat the memory as
  invalid or hidden.

## License

GNU General Public License v3.0 or later. See the [`LICENSE`](LICENSE) file for
the full text, and the header of [`radtel_rt1000pro.py`](radtel_rt1000pro.py).

## Disclaimer

This is an unofficial, community-developed driver and is not affiliated with or
endorsed by Radtel or the CHIRP project. Programming a radio outside its
type-approved parameters may be illegal in your jurisdiction — you are
responsible for operating within your license and local regulations.
