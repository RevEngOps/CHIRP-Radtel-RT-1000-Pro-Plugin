# RT-1000 Pro CHIRP custom driver
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This driver was produced from protocol/format observations of the
# Radtel RT-1000 Pro CPS V1.06 executable. It does not contain copied CPS
# source code.

"""CHIRP driver for the Radtel RT-1000 Pro.

TX inhibit / RX-only channels: set CHIRP Duplex to "off".
This maps to the CPS cboxRXTX value "Only RX" in channel byte +0 bits 4..5.
The TX frequency field is intentionally left populated so the radio/CPS does
not treat the memory as invalid/hidden.

Status: experimental.

Known from CPS 260611:
- Clone image is 1 MiB, but the CPS only reads/writes selected 1 KiB blocks.
- Normal channel memories start at 0x4000 and are 48 bytes each.
- There are 1024 normal memories.
- Serial default is 115200 baud, 8N1.

Use this first only to download from the radio and save a backup image.
Writing is implemented from the CPS state machine, but should be treated as
experimental.
"""

import logging
import time

from chirp import chirp_common, directory, errors, memmap
from chirp.settings import (
    RadioSetting,
    RadioSettingGroup,
    RadioSettings,
    RadioSettingValueList,
    RadioSettingValueInteger,
    RadioSettingValueString,
)

LOG = logging.getLogger(__name__)

MEM_SIZE = 0x100000
BLOCK_SIZE = 0x400
CHAN_BASE = 0x4000
CHAN_SIZE = 48
CHAN_COUNT = 1024
VFO_BASE = 0x1C100
CFG_BASE = 0x2000
CFG2_BASE = 0x1C250
ZONE_BASE = 0x1E000
ZONE_SIZE = 520
ZONE_COUNT = 250
ZONE_NAME_LEN = 14
ZONE_MEMBER_OFFSET = 20
ZONE_MEMBER_COUNT = 250
ZONE_EMPTY = 0xFFFF
# Safety cap for the CHIRP Banks tab. The radio supports 250 members,
# but an erased/zero-filled zone table can otherwise explode into many
# duplicate channel-1 entries.
ZONE_VISIBLE_COUNT = 250
FM_BASE = 0x0F0000
BT_BASE = 0x0FE000
GPS_BASE = 0x0FE400
APRS_BASE = 0x0FE800
# Backwards-compatible names used in the expanded settings tables.
ADDR_FM = FM_BASE
ADDR_GPS = GPS_BASE
ADDR_APRS = APRS_BASE

# Structured record tables that are clear enough from the CPS to edit safely.
# DTMF: 20 records, 16 bytes each. Bytes 0..13 are the DTMF code text and
# byte 15 stores the length. The CPS accepts 0-9, A-D, *, and #.
DTMF_CODE_BASE = CFG_BASE + 522
DTMF_CODE_COUNT = 20
DTMF_CODE_SIZE = 16
DTMF_CODE_MAX_LEN = 14
DTMF_ALLOWED_CHARS = "0123456789ABCD*#"

# Broadcast receiver presets: 80 records, 48 bytes each at ADDR_FM. The CPS
# treats byte 0 == 0 as a populated/default FM preset and 0xFF-filled rows as
# empty. Bytes 1..2 store the FM broadcast frequency as MHz*100; for example
# 87.5 MHz is stored as 8750. Offset +30 holds a 16-byte alias. The same record
# also contains SW/MW/LW receiver parameters, but those are deliberately left
# untouched in this build.
BCAST_CH_COUNT = 80
BCAST_CH_SIZE = 48
BCAST_NAME_LEN = 16


CMD_CONNECT = b"\x34\x52\x05\x10\x9B"
CMD_END = b"\x34\x52\x05\xEE\x79"
ACK = b"\x06"

# The CPS read sequence after the initial password/config block.
READ_BLOCKS = (
    [0x0008]
    + list(range(0x0010, 0x0040))
    + [0x0070]
    + list(range(0x0078, 0x00F8))
    + list(range(0x03C0, 0x03C5))
    + list(range(0x03F8, 0x03FC))
)

POWER_LEVELS = [
    chirp_common.PowerLevel("Low", watts=1.0),
    chirp_common.PowerLevel("High", watts=5.0),
]

WORK_RANGE_OPTIONS = ["Range A", "Range B", "Range C"]
WORK_MODE_OPTIONS = ["Freq Mode", "CH Mode", "Zone Mode"]
DISPLAY_MODE_OPTIONS = ["Channel", "Freq", "Alias"]


# Receiver coverage. The radio appears to accept RX memories through the
# VHF/UHF region essentially continuously from 64-999 MHz. Keep the lower
# LW/MW/SW receive-only ranges too, but expose 64-999 MHz as one continuous
# CHIRP receive range instead of the segmented published/CPS ranges.
#
# CHIRP's valid_bands applies to the receive frequency column; transmit is
# still governed by the radio/firmware and by the TX frequency encoded from
# duplex/offset.
VALID_BANDS = [
    (153000, 279000),         # LW RX
    (520000, 1710000),        # MW RX
    (2000000, 32000000),      # SW/CB RX; AM/SSB function covers 26-32 MHz
    (64000000, 999000000),    # continuous VHF/UHF/extended RX
]

# Published transmit-capable ranges. Airband TX may be firmware-disabled and
# legally restricted depending on the individual radio/region. The plugin does
# not attempt to unlock disabled TX.
TX_VALID_BANDS = [
    (26000000, 30000000),
    (108000000, 136000000),
    (136000000, 174000000),
    (350000000, 390000000),
    (400000000, 470000000),
]

# CPS cboxAMFM order is: 0=FM, 1=AM, 2=SSB and lives in channel
# byte +4 bits 4..5. CPS cboxBand order is 0=wide, 1=narrow and lives
# in byte +4 bits 6..7. CHIRP has no literal "SSB" master mode; use
# USB/LSB as standard CHIRP stand-ins and encode both to the radio's
# single SSB value. On download, SSB decodes as USB because the image does
# not distinguish upper/lower sideband.
MODE_TO_CPS = {"FM": 0, "NFM": 0, "AM": 1, "USB": 2, "LSB": 2}
CPS_TO_MODE = {0: "FM", 1: "AM", 2: "USB"}


def _sum8(data):
    return sum(bytearray(data)) & 0xFF


def _block_request(block):
    payload = bytes([0x52, (block >> 8) & 0xFF, block & 0xFF])
    return payload + bytes([_sum8(payload)])


def _write_frame(opcode, block, payload):
    if len(payload) != BLOCK_SIZE:
        raise errors.RadioError("Internal error: bad write block size")
    frame = bytes([opcode, (block >> 8) & 0xFF, block & 0xFF]) + bytes(payload)
    return frame + bytes([_sum8(frame)])


def _set_timeout(pipe, value):
    old = getattr(pipe, "timeout", None)
    try:
        pipe.timeout = value
    except Exception:
        pass
    return old


def _restore_timeout(pipe, old):
    try:
        pipe.timeout = old
    except Exception:
        pass


def _read_exact(pipe, count, timeout=5.0):
    deadline = time.time() + timeout
    data = bytearray()
    while len(data) < count:
        if time.time() > deadline:
            raise errors.RadioError(
                "Radio timed out while reading %i bytes; got %i" %
                (count, len(data)))
        chunk = pipe.read(count - len(data))
        if chunk:
            data.extend(bytearray(chunk))
        else:
            time.sleep(0.02)
    return bytes(data)


def _read_frame(pipe, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        first = pipe.read(1)
        if not first:
            time.sleep(0.02)
            continue
        first = bytes(first)
        if first in (ACK, b"\xA4", b"\x4A"):
            return first
        if first == b"R":
            rest = _read_exact(pipe, BLOCK_SIZE + 3, timeout=timeout)
            frame = first + rest
            if _sum8(frame[:-1]) != frame[-1]:
                raise errors.RadioError("Radio sent a block with a bad checksum")
            return frame
        LOG.debug("Ignoring unexpected byte from radio: %r", first)
    raise errors.RadioError("Radio did not respond")


def _expect_ack(pipe, what="operation"):
    frame = _read_frame(pipe, timeout=8.0)
    if frame != ACK:
        raise errors.RadioError("Expected ACK during %s, got %r" % (what, frame[:8]))


def _expect_r_block(pipe, block, timeout=8.0):
    frame = _read_frame(pipe, timeout=timeout)
    if len(frame) != BLOCK_SIZE + 4 or frame[0] != 0x52:
        raise errors.RadioError("Expected data block %04x, got %r" %
                                (block, frame[:8]))
    got = (frame[1] << 8) | frame[2]
    if got != block:
        raise errors.RadioError("Expected data block %04x, got %04x" %
                                (block, got))
    return frame[3:-1]


def _progress(radio, status, cur, total):
    status.cur = cur
    status.max = total
    radio.status_fn(status)


def do_download(radio):
    pipe = radio.pipe
    old_timeout = _set_timeout(pipe, 0.25)
    status = chirp_common.Status()
    status.msg = "Cloning from radio"
    image = bytearray([0xFF] * MEM_SIZE)
    try:
        pipe.write(CMD_CONNECT)
        _expect_ack(pipe, "connect")

        for idx, block in enumerate(READ_BLOCKS, start=1):
            pipe.write(_block_request(block))
            data = _expect_r_block(pipe, block, timeout=10.0)
            image[block * BLOCK_SIZE:(block + 1) * BLOCK_SIZE] = data
            _progress(radio, status, idx, len(READ_BLOCKS))

        pipe.write(CMD_END)
    finally:
        _restore_timeout(pipe, old_timeout)
    return memmap.MemoryMapBytes(bytes(image))


def _send_and_ack(pipe, opcode, block, data, label):
    pipe.write(_write_frame(opcode, block, data))
    _expect_ack(pipe, label)


def do_upload(radio):
    pipe = radio.pipe
    old_timeout = _set_timeout(pipe, 0.25)
    status = chirp_common.Status()
    status.msg = "Cloning to radio"
    image = bytearray(radio._mmap.get_byte_compatible().get_packed())
    try:
        pipe.write(CMD_CONNECT)
        _expect_ack(pipe, "connect")

        # CPS performs these two reads before it starts sending write blocks.
        pipe.write(_block_request(0x0000))
        _expect_r_block(pipe, 0x0000, timeout=10.0)
        pipe.write(_block_request(0x0008))
        _expect_r_block(pipe, 0x0008, timeout=10.0)

        writes = []
        writes.append((0x90, 0, 0x2000))
        for i in range(48):
            writes.append((0x91, i, 0x4000 + i * BLOCK_SIZE))
        writes.append((0x92, 0, 0x1C000))
        for i in range(128):
            writes.append((0x93, i, 0x1E000 + i * BLOCK_SIZE))
        for i in range(4):
            writes.append((0x98, i, 0xF0000 + i * BLOCK_SIZE))
        for i in range(4):
            writes.append((0x99, i, 0xFE000 + i * BLOCK_SIZE))

        for idx, (opcode, block, offset) in enumerate(writes, start=1):
            data = image[offset:offset + BLOCK_SIZE]
            _send_and_ack(pipe, opcode, block, data,
                          "write opcode %02x block %04x" % (opcode, block))
            _progress(radio, status, idx, len(writes))

        pipe.write(CMD_END)
    finally:
        _restore_timeout(pipe, old_timeout)


def _chan_offset(number):
    if number < 1 or number > CHAN_COUNT:
        raise errors.InvalidMemoryLocation("Memory %s out of range" % number)
    return CHAN_BASE + (number - 1) * CHAN_SIZE


def _decode_freq(data):
    # CPS displays stored integer N as N/100000 MHz, so CHIRP Hz is N*10.
    raw = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
    if raw in (0, 0xFFFFFFFF):
        return 0
    return raw * 10


def _encode_freq(hz):
    raw = int(round(hz / 10.0))
    return bytes([
        raw & 0xFF,
        (raw >> 8) & 0xFF,
        (raw >> 16) & 0xFF,
        (raw >> 24) & 0xFF,
    ])


def _decode_name(data):
    raw = bytes(data).split(b"\x00", 1)[0].split(b"\xFF", 1)[0]
    try:
        return raw.decode("gbk", errors="replace").strip()
    except LookupError:
        return raw.decode("ascii", errors="replace").strip()


def _encode_name(name):
    raw = name.encode("gbk", errors="replace")[:16]
    return raw + (b"\xFF" * (16 - len(raw)))


def _decode_subaudio(lo, hi):
    word = ((hi & 0x0F) << 8) | lo
    mode = hi & 0xF0
    if mode == 0x10:
        return ("Tone", word / 10.0, "N")
    if mode == 0x20:
        return ("DTCS", int("%03o" % word), "N")
    if mode == 0x30:
        return ("DTCS", int("%03o" % word), "R")
    return ("", None, "N")


def _encode_subaudio(mode, value, polarity):
    if mode == "Tone" and value:
        word = int(round(float(value) * 10.0)) & 0x0FFF
        word |= 0x1000
    elif mode == "DTCS" and value:
        # CHIRP stores DCS as the visible octal-looking integer, e.g. 023.
        word = int(str(int(value)), 8) & 0x0FFF
        word |= 0x3000 if polarity == "R" else 0x2000
    else:
        word = 0
    return word & 0xFF, (word >> 8) & 0xFF


def _decode_tones(mem, rec):
    rxmode, rxval, rxpol = _decode_subaudio(rec[13], rec[14])
    txmode, txval, txpol = _decode_subaudio(rec[15], rec[16])
    if rxmode == "DTCS" and rxval not in chirp_common.ALL_DTCS_CODES:
        rxmode, rxval = "", None
    if txmode == "DTCS" and txval not in chirp_common.ALL_DTCS_CODES:
        txmode, txval = "", None
    chirp_common.split_tone_decode(mem, (txmode, txval, txpol),
                                   (rxmode, rxval, rxpol))


def _encode_tones(mem, rec):
    ((txmode, txval, txpol), (rxmode, rxval, rxpol)) = \
        chirp_common.split_tone_encode(mem)
    rec[13], rec[14] = _encode_subaudio(rxmode, rxval, rxpol)
    rec[15], rec[16] = _encode_subaudio(txmode, txval, txpol)


def _nearest_power(power):
    if power is None:
        return 0
    return min(range(len(POWER_LEVELS)), key=lambda i: abs(POWER_LEVELS[i] - power))




def _decode_text(data):
    raw = bytes(data).split(b"\x00", 1)[0].split(b"\xFF", 1)[0]
    try:
        return raw.decode("gbk", errors="replace").strip()
    except LookupError:
        return raw.decode("ascii", errors="replace").strip()


def _encode_text(text, length):
    try:
        raw = text.encode("gbk", errors="replace")[:length]
    except LookupError:
        raw = text.encode("ascii", errors="replace")[:length]
    return raw + (b"\xFF" * (length - len(raw)))


def _zone_offset(index):
    if index < 0 or index >= ZONE_COUNT:
        raise errors.RadioError("Zone index %s out of range" % index)
    return ZONE_BASE + (index * ZONE_SIZE)


def _byte_value(value):
    """Return an integer byte from CHIRP memmap indexing.

    Depending on CHIRP version/backing object, mmap[index] may return an int,
    a one-byte bytes object, a bytearray, or occasionally a one-character str.
    The bank editor hits this path heavily, so normalize before using int().
    """
    if isinstance(value, int):
        return value & 0xFF
    if isinstance(value, (bytes, bytearray)):
        if not value:
            return 0
        return value[0]
    if isinstance(value, str):
        if not value:
            return 0
        return ord(value[0]) & 0xFF
    return int(value) & 0xFF


def _safe_index(value, options, default=0):
    try:
        value = _byte_value(value)
    except Exception:
        return default
    if 0 <= value < len(options):
        return value
    return default


def _safe_byte(mmap_obj, offset, options, default=0):
    return _safe_index(mmap_obj[offset], options, default=default)


def _read_u16le(mmap_obj, offset):
    lo = _byte_value(mmap_obj[offset])
    hi = _byte_value(mmap_obj[offset + 1])
    return lo | (hi << 8)


def _write_u16le(mmap_obj, offset, value):
    value = int(value) & 0xFFFF
    mmap_obj.set(offset, bytes([value & 0xFF, (value >> 8) & 0xFF]))


class RT1000ProZone(chirp_common.NamedBank):
    """A Radtel RT-1000 Pro zone as a CHIRP bank."""

    def get_name(self):
        return self._model._radio._get_zone_name(self._index)

    def set_name(self, name):
        self._model._radio._set_zone_name(self._index, name)
        self._name = name


class RT1000ProZoneBankModel(chirp_common.MTOBankModel):
    """RT-1000 Pro zone model with conservative parsing.

    Note: this version intentionally does not advertise CHIRP bank-index
    support. CHIRP currently has a single Index column for all bank/zone
    memberships, which is confusing for many-to-many zone assignments and can
    leave a stale index value displayed after a zone checkbox is cleared. The
    radio zone order is still preserved internally when reading/writing.

    The CPS stores 250 zones. Each zone has a 14-byte name and 250 little-endian
    channel references. Channel references are zero-based; 0xFFFF is the normal
    unused-slot marker. Some radios/images appear to use zero-filled unused zone
    space, so this parser avoids duplicate channel entries and treats repeated
    zeroes after the first occurrence as filler instead of 250 copies of memory 1.
    """

    def __init__(self, radio, name="Zones"):
        super(RT1000ProZoneBankModel, self).__init__(radio, name=name)
        self._zones = [RT1000ProZone(self, i, self._radio._get_zone_name(i))
                       for i in range(ZONE_VISIBLE_COUNT)]
        self._members_cache = {}
        self._reverse_cache = None

    def get_num_mappings(self):
        return len(self._zones)

    def get_mappings(self):
        return self._zones

    def _raw_members(self, zone_index):
        base = _zone_offset(zone_index) + ZONE_MEMBER_OFFSET
        return [_read_u16le(self._radio._mmap, base + (slot * 2))
                for slot in range(ZONE_MEMBER_COUNT)]

    def _zone_has_programmed_record(self, zone_index):
        # A zone with a non-blank name is worth showing even if it has no
        # members yet. This avoids hiding user-created empty zones.
        return bool(self._radio._get_zone_raw_name(zone_index))

    def _memory_is_programmed(self, number):
        try:
            return not self._radio.get_memory(number).empty
        except Exception:
            return False

    def _decode_members_uncached(self, zone):
        zone_index = zone.get_index()
        raw_values = self._raw_members(zone_index)
        members = []
        seen = set()

        # Entirely erased zone: definitely empty.
        if all(raw == ZONE_EMPTY for raw in raw_values):
            return []

        for slot, raw in enumerate(raw_values):
            if raw == ZONE_EMPTY:
                break

            if raw >= CHAN_COUNT:
                # An invalid value is more likely uninitialized data than a
                # valid zone member. Stop rather than flooding CHIRP's UI.
                break

            number = raw + 1

            # Zero-filled unused space looks like channel 1 repeated. Keep a
            # single real channel-1 entry if it is actually programmed, then
            # treat subsequent repeats as filler.
            if number in seen:
                if raw == 0:
                    break
                continue

            if not self._memory_is_programmed(number):
                # Do not expose zone references to blank memory records.
                # Repeated blank channel-1 values are the common crash/freeze
                # case, so stop immediately when encountered.
                if raw == 0:
                    break
                continue

            members.append(number)
            seen.add(number)

            if len(members) >= ZONE_MEMBER_COUNT:
                break

        return members

    def _members(self, zone):
        index = zone.get_index()
        if index not in self._members_cache:
            self._members_cache[index] = self._decode_members_uncached(zone)
        return list(self._members_cache[index])

    def _invalidate_cache(self):
        self._members_cache = {}
        self._reverse_cache = None

    def _write_members(self, zone, members):
        base = _zone_offset(zone.get_index()) + ZONE_MEMBER_OFFSET
        clean = []
        for number in members:
            try:
                number = int(number)
            except Exception:
                continue
            if 1 <= number <= CHAN_COUNT and number not in clean:
                clean.append(number)
        if len(clean) > ZONE_MEMBER_COUNT:
            raise errors.RadioError("Zone %s is full" % zone.get_name())
        for slot in range(ZONE_MEMBER_COUNT):
            value = (clean[slot] - 1) if slot < len(clean) else ZONE_EMPTY
            _write_u16le(self._radio._mmap, base + (slot * 2), value)
        self._invalidate_cache()

    def _reverse_members(self):
        if self._reverse_cache is None:
            reverse = {}
            for zone in self._zones:
                for number in self._members(zone):
                    reverse.setdefault(number, []).append(zone)
            self._reverse_cache = reverse
        return self._reverse_cache

    def get_mapping_memories(self, zone):
        memories = []
        for number in self._members(zone):
            try:
                mem = self._radio.get_memory(number)
                if not mem.empty:
                    memories.append(mem)
            except errors.InvalidMemoryLocation:
                LOG.warning("Ignoring invalid zone member %s in %s",
                            number, zone.get_name())
        return memories

    def get_memory_mappings(self, memory):
        return list(self._reverse_members().get(memory.number, []))

    def add_memory_to_mapping(self, memory, zone):
        if getattr(memory, "empty", False):
            raise errors.RadioError("Cannot add an empty memory to a zone")
        members = self._members(zone)
        if memory.number not in members:
            if len(members) >= ZONE_MEMBER_COUNT:
                raise errors.RadioError("Zone %s is full" % zone.get_name())
            members.append(memory.number)
            self._write_members(zone, members)

    def remove_memory_from_mapping(self, memory, zone):
        members = self._members(zone)
        if memory.number not in members:
            raise errors.RadioError("Memory %s is not in %s" %
                                    (memory.number, zone.get_name()))
        members = [number for number in members if number != memory.number]
        self._write_members(zone, members)

    def get_index_bounds(self):
        return (1, ZONE_MEMBER_COUNT)

    def get_memory_index(self, memory, zone):
        members = self._members(zone)
        if memory.number not in members:
            raise errors.RadioError("Memory %s is not in %s" %
                                    (memory.number, zone.get_name()))
        return members.index(memory.number) + 1

    def set_memory_index(self, memory, zone, index):
        members = [number for number in self._members(zone)
                   if number != memory.number]
        index = max(1, min(int(index), ZONE_MEMBER_COUNT)) - 1
        members.insert(index, memory.number)
        self._write_members(zone, members)

    def get_next_mapping_index(self, zone):
        members = self._members(zone)
        if len(members) >= ZONE_MEMBER_COUNT:
            raise errors.RadioError("Zone %s is full" % zone.get_name())
        return len(members) + 1


# ---------------------------------------------------------------------------
# Global/settings editor support
#
# These offsets were extracted from the RT-1000 Pro CPS DataToPanel/SaveAllData
# paths. Most scalar settings are simple one-byte combo indexes or little-endian
# integers. This build also exposes the DTMF code table and the broadcast-FM
# preset frequency/name table. More complex structured records (GPS record logs,
# APRS packet/station tables, and startup bitmap data) are still left alone.

ON_OFF_OPTIONS = ["Off", "On"]
GENERIC_BYTE_OPTIONS = ["%d" % i for i in range(256)]
GENERIC_SMALL_OPTIONS = ["%d" % i for i in range(64)]
RANGE_AB_C_OPTIONS = ["Range A", "Range B", "Range C"]
FREQ_DIGIT_OPTIONS = ["6 digits", "8 digits"]
WORK_RANGE_EXT_OPTIONS = ["64-999 MHz", "18-64 MHz"]
CHANNEL_RANGE_OPTIONS = ["CH-A", "CH-B", "CH-C"]

STEP_OPTIONS = [
    "0.01 kHz", "0.02 kHz", "0.03 kHz", "0.05 kHz", "0.10 kHz",
    "0.25 kHz", "0.50 kHz", "1.25 kHz", "2.50 kHz", "5.00 kHz",
    "6.25 kHz", "8.33 kHz", "10.0 kHz", "12.5 kHz", "20.0 kHz",
    "25.0 kHz", "50.0 kHz", "100 kHz", "500 kHz",
]

COLOR_OPTIONS = [
    "Blue", "Green", "Red", "Yellow", "White", "Fuchsia", "Pink",
    "Orange", "Tomato", "Cyan", "Golden",
]

KEY_FUNCTION_OPTIONS = [
    "None", "Monitor", "H/L Power", "Dual Standby", "TX Priority",
    "Scanning", "Backlight On-off", "Roger Beep", "FM Radio",
    "Talkaround", "Alarm", "Freq Detect", "CTC/DCS Scan",
    "Send Single Tone", "Status Query", "Modem", "Spectrum", "Freq Step",
    "NOAA Mode", "Save CH", "Brightness", "VOX", "Zone Select",
    "GPS Manual Rec", "Query GPS Track", "APRS Beacon", "Bandwidth",
    "Work Range", "Repeater Mode", "Bluetooth",
]

RELAY_DELAY_OPTIONS = ["%d ms" % i for i in range(100, 2001, 100)]
PTT_DELAY_OPTIONS = ["%d ms" % i for i in range(200, 601, 50)]
DTMF_DURATION_OPTIONS = ["%d ms" % i for i in range(30, 201, 10)]
DETECT_RANGE_OPTIONS = [
    "64-136 MHz", "136-174 MHz", "174-240 MHz", "240-320 MHz",
    "320-400 MHz", "400-480 MHz", "480-560 MHz", "560-640 MHz",
    "840-920 MHz", "920-1000 MHz",
]
GPS_BAUD_OPTIONS = [
    "4800", "9600", "14400", "19200", "38400", "56000", "57600",
    "115200", "128000", "256000",
]
UTC_OPTIONS = (["UTC 0"] + ["UTC+%s" % x for x in (
    "0.5", "1", "1.5", "2", "2.5", "3", "3.5", "4", "4.5",
    "5", "5.5", "6", "6.5", "7", "7.5", "8", "8.5", "9",
    "9.5", "10", "10.5", "11", "11.5", "12")]
    + ["UTC-%s" % x for x in (
    "0.5", "1", "1.5", "2", "2.5", "3", "3.5", "4", "4.5",
    "5", "5.5", "6", "6.5", "7", "7.5", "8", "8.5", "9",
    "9.5", "10", "10.5", "11", "11.5", "12")])

APRS_STATION_TYPE_OPTIONS = ["GPS", "Fixed"]
APRS_MICE_TYPE_OPTIONS = [
    "M0: OFF DUTY", "M1: En Route", "M2: In Service", "M3: Returning",
    "M4: Committed", "M5: Special", "M6: Priority",
]


def _read_u32le(mmap_obj, offset):
    return (_byte_value(mmap_obj[offset]) |
            (_byte_value(mmap_obj[offset + 1]) << 8) |
            (_byte_value(mmap_obj[offset + 2]) << 16) |
            (_byte_value(mmap_obj[offset + 3]) << 24))


def _write_u32le(mmap_obj, offset, value):
    value = int(value) & 0xFFFFFFFF
    mmap_obj.set(offset, bytes([
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]))


def _clamp_int(value, minimum, maximum):
    try:
        value = int(value)
    except Exception:
        value = minimum
    return max(minimum, min(maximum, value))


def _freq_raw_to_khz(raw):
    # CPS stores these frequency-ish values as 10 Hz units and displays MHz
    # with three decimal places. kHz is a clean integer representation for CHIRP.
    return int(round(int(raw) / 100.0))


def _freq_khz_to_raw(khz):
    return int(khz) * 100


def _sanitize_dtmf(text):
    text = str(text).upper()
    return "".join(ch for ch in text if ch in DTMF_ALLOWED_CHARS)[:DTMF_CODE_MAX_LEN]


def _read_dtmf_code(mmap_obj, index):
    base = DTMF_CODE_BASE + (index * DTMF_CODE_SIZE)
    stored_len = _byte_value(mmap_obj[base + 15])
    if stored_len > DTMF_CODE_MAX_LEN:
        stored_len = DTMF_CODE_MAX_LEN
    raw = bytearray()
    for offset in range(stored_len):
        byte = _byte_value(mmap_obj[base + offset])
        if byte in (0x00, 0xFF):
            break
        raw.append(byte)
    try:
        return _sanitize_dtmf(raw.decode("ascii", errors="ignore"))
    except Exception:
        return ""


def _write_dtmf_code(mmap_obj, index, text):
    base = DTMF_CODE_BASE + (index * DTMF_CODE_SIZE)
    code = _sanitize_dtmf(text).encode("ascii")
    record = bytearray([0xFF] * DTMF_CODE_SIZE)
    record[:len(code)] = code
    record[15] = len(code) & 0xFF
    mmap_obj.set(base, bytes(record))


def _broadcast_offset(index):
    return ADDR_FM + (index * BCAST_CH_SIZE)


def _broadcast_record_is_empty(mmap_obj, index):
    base = _broadcast_offset(index)
    rec = bytearray(mmap_obj[base:base + BCAST_CH_SIZE])
    return (rec == bytearray([0xFF] * BCAST_CH_SIZE) or
            rec == bytearray([0x00] * BCAST_CH_SIZE))


def _read_bcast_freq_khz(mmap_obj, index):
    # CPS stores FM broadcast frequency as MHz*100. Convert to kHz for CHIRP
    # settings so 87.5 MHz appears as 87500 kHz.
    base = _broadcast_offset(index)
    raw = _read_u16le(mmap_obj, base + 1)
    if raw in (0, 0xFFFF):
        return 87500
    khz = int(round(raw * 10))
    return _clamp_int(khz, 64000, 108000)


def _write_bcast_freq_khz(mmap_obj, index, khz):
    base = _broadcast_offset(index)
    khz = _clamp_int(khz, 64000, 108000)
    raw = int(round(khz / 10.0))
    _write_u16le(mmap_obj, base + 1, raw)


def _read_bcast_name(mmap_obj, index):
    base = _broadcast_offset(index)
    return _decode_text(mmap_obj[base + 30:base + 30 + BCAST_NAME_LEN])


def _write_bcast_name(mmap_obj, index, text):
    base = _broadcast_offset(index)
    mmap_obj.set(base + 30, _encode_text(str(text), BCAST_NAME_LEN))


def _set_bcast_enabled(mmap_obj, index, enabled):
    base = _broadcast_offset(index)
    if not enabled:
        mmap_obj.set(base, b"\xFF" * BCAST_CH_SIZE)
        return
    if _broadcast_record_is_empty(mmap_obj, index):
        rec = bytearray([0xFF] * BCAST_CH_SIZE)
        rec[0] = 0x00
        # Reasonable safe default if a user enables an empty row.
        rec[1:3] = bytes([0x2E, 0x22])  # 8750 = 87.5 MHz
        rec[3] = 0x00   # SW demod index
        rec[4] = 0x00   # SW step
        rec[5] = 0x00   # SW bandwidth/AGC defaults
        rec[12] = 0x00  # MW demod
        rec[13] = 0x00  # MW step
        rec[14] = 0x00  # MW bandwidth
        rec[15] = 0x00  # MW AGC
        rec[21] = 0x00  # LW demod
        rec[22] = 0x00  # LW step
        rec[23] = 0x00  # LW bandwidth
        rec[24] = 0x00  # LW AGC
        mmap_obj.set(base, bytes(rec))
    else:
        mmap_obj.set(base, b"\\x00")


# name, label, absolute offset, option-list, default-index, group-key
LIST_SETTING_DEFS = [
    ("rt1000.startup_logo", "Startup logo", CFG_BASE + 16, ON_OFF_OPTIONS, 0, "startup"),
    ("rt1000.startup_tone", "Startup tone", CFG_BASE + 19, ON_OFF_OPTIONS, 0, "startup"),
    ("rt1000.voice_prompt", "Voice prompt", CFG_BASE + 92, GENERIC_SMALL_OPTIONS, 0, "ui"),
    ("rt1000.key_beep", "Key beep", CFG_BASE + 93, ON_OFF_OPTIONS, 0, "ui"),
    ("rt1000.lock_timer", "Lock timer", CFG_BASE + 95, GENERIC_BYTE_OPTIONS, 0, "ui"),
    ("rt1000.brightness", "Brightness", CFG_BASE + 97, GENERIC_SMALL_OPTIONS, 4, "ui"),
    ("rt1000.lcd_timer", "LCD timer", CFG_BASE + 98, GENERIC_BYTE_OPTIONS, 0, "ui"),
    ("rt1000.save_mode", "Save mode", CFG_BASE + 99, GENERIC_BYTE_OPTIONS, 0, "power"),
    ("rt1000.menu_exit_timer", "Menu exit timer", CFG_BASE + 101, GENERIC_BYTE_OPTIONS, 0, "ui"),
    ("rt1000.talkaround", "Talkaround", CFG_BASE + 103, ON_OFF_OPTIONS, 0, "behavior"),
    ("rt1000.alarm_mode", "Alarm mode", CFG_BASE + 104, GENERIC_BYTE_OPTIONS, 0, "behavior"),
    ("rt1000.auto_power_off", "Auto power off", CFG_BASE + 105, ON_OFF_OPTIONS, 0, "power"),
    ("rt1000.work_range", "Work range", CFG_BASE + 115, WORK_RANGE_EXT_OPTIONS, 0, "work"),
    ("rt1000.tx_priority", "TX priority", CFG_BASE + 126, ON_OFF_OPTIONS, 0, "behavior"),
    ("rt1000.ch_alias_color", "CH alias color", CFG_BASE + 130, COLOR_OPTIONS, 1, "ui"),
    ("rt1000.scanning_mode", "Scanning mode", CFG_BASE + 163, GENERIC_BYTE_OPTIONS, 0, "scan"),
    ("rt1000.return_after_scanning", "Return after scanning", CFG_BASE + 164, GENERIC_BYTE_OPTIONS, 0, "scan"),
    ("rt1000.scanning_dwell", "Scanning dwell", CFG_BASE + 165, GENERIC_BYTE_OPTIONS, 0, "scan"),
    ("rt1000.scanning_interval", "Scanning interval", CFG_BASE + 166, GENERIC_BYTE_OPTIONS, 0, "scan"),
    ("rt1000.rssi_refresh", "RSSI refresh", CFG_BASE + 169, ON_OFF_OPTIONS, 0, "behavior"),
    ("rt1000.freq_digits", "Frequency digits", CFG_BASE + 234, FREQ_DIGIT_OPTIONS, 0, "ui"),
    ("rt1000.tx_start_tone", "TX start tone", CFG_BASE + 267, ON_OFF_OPTIONS, 0, "audio"),
    ("rt1000.tx_end_tone", "TX end tone", CFG_BASE + 268, ON_OFF_OPTIONS, 0, "audio"),
    ("rt1000.freq_detect_range", "Freq detect range", CFG_BASE + 272, DETECT_RANGE_OPTIONS, 0, "rf"),
    ("rt1000.reply_delay", "Reply delay", CFG_BASE + 273, RELAY_DELAY_OPTIONS, 0, "rf"),
    ("rt1000.ch_reverse", "CH reverse", CFG_BASE + 842, ON_OFF_OPTIONS, 0, "behavior"),
    ("rt1000.carrier_led", "Carrier LED", CFG_BASE + 852, ON_OFF_OPTIONS, 0, "ui"),

    ("rt1000.key_lock", "Key lock", CFG2_BASE + 0, ON_OFF_OPTIONS, 0, "ui"),
    ("rt1000.current_range", "Current range", CFG2_BASE + 1, RANGE_AB_C_OPTIONS, 0, "work"),
    ("rt1000.dual_standby", "Dual standby", CFG2_BASE + 2, ON_OFF_OPTIONS, 0, "behavior"),
    ("rt1000.scanning_direction", "Scanning direction", CFG2_BASE + 4, GENERIC_BYTE_OPTIONS, 0, "scan"),
    ("rt1000.freq_step", "Frequency step", CFG2_BASE + 5, STEP_OPTIONS, 0, "rf"),
    ("rt1000.broadcast_standby", "Broadcast radio standby", CFG2_BASE + 19, ON_OFF_OPTIONS, 0, "broadcast"),
    ("rt1000.work_mode_a", "Range A work mode", CFG2_BASE + 20, WORK_MODE_OPTIONS, 0, "work"),
    ("rt1000.work_mode_b", "Range B work mode", CFG2_BASE + 21, WORK_MODE_OPTIONS, 0, "work"),
    ("rt1000.work_mode_c", "Range C work mode", CFG2_BASE + 22, WORK_MODE_OPTIONS, 0, "work"),
    ("rt1000.display_mode_a", "Range A show", CFG2_BASE + 23, DISPLAY_MODE_OPTIONS, 0, "work"),
    ("rt1000.display_mode_b", "Range B show", CFG2_BASE + 24, DISPLAY_MODE_OPTIONS, 0, "work"),
    ("rt1000.display_mode_c", "Range C show", CFG2_BASE + 25, DISPLAY_MODE_OPTIONS, 0, "work"),
    ("rt1000.multiple_ptt", "Multiple PTT", CFG2_BASE + 35, ON_OFF_OPTIONS, 0, "behavior"),

    ("rt1000.func_fs1_short", "FS-1 press short", CFG2_BASE + 36, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.func_fs1_long", "FS-1 press long", CFG2_BASE + 37, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.func_fs2_short", "FS-2 press short", CFG2_BASE + 38, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.func_fs2_long", "FS-2 press long", CFG2_BASE + 39, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.func_alarm_short", "Alarm key press short", CFG2_BASE + 40, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.func_alarm_long", "Alarm key press long", CFG2_BASE + 41, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_0", "Number key 0", CFG2_BASE + 42, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_1", "Number key 1", CFG2_BASE + 43, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_2", "Number key 2", CFG2_BASE + 44, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_3", "Number key 3", CFG2_BASE + 45, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_4", "Number key 4", CFG2_BASE + 46, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_5", "Number key 5", CFG2_BASE + 47, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_6", "Number key 6", CFG2_BASE + 48, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_7", "Number key 7", CFG2_BASE + 49, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_8", "Number key 8", CFG2_BASE + 50, KEY_FUNCTION_OPTIONS, 0, "keys"),
    ("rt1000.numkey_9", "Number key 9", CFG2_BASE + 51, KEY_FUNCTION_OPTIONS, 0, "keys"),

    ("rt1000.dtmf_send_delay", "DTMF send delay", CFG_BASE + 512, DTMF_DURATION_OPTIONS, 0, "dtmf"),
    ("rt1000.dtmf_send_duration", "DTMF send duration", CFG_BASE + 513, DTMF_DURATION_OPTIONS, 0, "dtmf"),
    ("rt1000.dtmf_send_interval", "DTMF send interval", CFG_BASE + 514, DTMF_DURATION_OPTIONS, 0, "dtmf"),
    ("rt1000.dtmf_send_mode", "DTMF send mode", CFG_BASE + 515, GENERIC_BYTE_OPTIONS, 0, "dtmf"),
    ("rt1000.dtmf_send_select", "DTMF send select", CFG_BASE + 516, ["DTMF-%02d" % i for i in range(1, 17)], 0, "dtmf"),
    ("rt1000.dtmf_display", "DTMF display", CFG_BASE + 517, ON_OFF_OPTIONS, 0, "dtmf"),
    ("rt1000.dtmf_remote", "DTMF remote", CFG_BASE + 520, ON_OFF_OPTIONS, 0, "dtmf"),

]

# Add GPS/APRS list settings after the bases are known. Defined this way to keep
# GPS/APRS addresses visually grouped and to avoid accidentally using a stale
# calculated constant above.
GPS_LIST_SETTING_DEFS = [
    ("rt1000.gps_on", "GPS", 0, ON_OFF_OPTIONS, 0, "gps"),
    ("rt1000.gps_baud", "GPS baud", 1, GPS_BAUD_OPTIONS, 0, "gps"),
    ("rt1000.gps_utc", "UTC", 2, UTC_OPTIONS, 0, "gps"),
    ("rt1000.gps_degree", "GPS degree display", 3, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_record", "GPS record", 4, ON_OFF_OPTIONS, 0, "gps"),
    ("rt1000.gps_we_format", "WE format", 9, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_speed_unit", "Speed unit", 10, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_distance_unit", "Distance unit", 11, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_height_unit", "Height unit", 12, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_mileage", "Mileage", 17, ON_OFF_OPTIONS, 0, "gps"),
    ("rt1000.gps_current_station", "Current station", 18, GENERIC_BYTE_OPTIONS, 0, "gps"),
    ("rt1000.gps_move_alarm", "Move alarm", 19, ON_OFF_OPTIONS, 0, "gps"),
    ("rt1000.gps_callout_alarm", "Callout alarm", 24, ON_OFF_OPTIONS, 0, "gps"),
]

APRS_LIST_SETTING_DEFS = [
    ("rt1000.aprs_on", "APRS", 0, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_station_type", "Station type", 1, APRS_STATION_TYPE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_rx_channel", "RX channel", 2, CHANNEL_RANGE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_rx_mute", "RX CH mute", 4, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_demod_tone", "Demod tone", 5, GENERIC_BYTE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_beacon_popup", "RX beacon pop-up", 6, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_beacon_store", "Beacon store", 7, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_tx_channel", "TX channel", 8, CHANNEL_RANGE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_ptt_delay", "TX delay", 11, PTT_DELAY_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_report_voltage", "Report voltage", 13, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_stars_count", "Report stars count", 16, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_report_mileage", "Report mileage", 17, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_ptt_link", "PTT after", 18, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_send_after_call", "Send beacon after call", 19, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_timed_beacon", "Timed beacon", 20, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_smart_beacon", "Smart beacon", 25, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_ssid", "SSID", 51, GENERIC_SMALL_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_symbol_table", "Symbol table", 52, GENERIC_BYTE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_mice", "MIC-E on-off", 182, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_mice_mode", "MIC-E mode", 183, APRS_MICE_TYPE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_path1_count", "Path 1 count", 190, GENERIC_SMALL_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_path2_count", "Path 2 count", 197, GENERIC_SMALL_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_digi_channel", "DIGI channel", 198, CHANNEL_RANGE_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_digi1_on", "DIGI 1 on-off", 199, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_digi2_on", "DIGI 2 on-off", 208, ON_OFF_OPTIONS, 0, "aprs"),
    ("rt1000.aprs_repeat_time", "Timed beacon timer", 215, GENERIC_BYTE_OPTIONS, 0, "aprs"),
]

# name, label, offset, min, max, default, group-key
INT8_SETTING_DEFS = [
    ("rt1000.save_start_timer", "Save start timer", CFG_BASE + 100, 0, 255, 10, "power"),
    ("rt1000.am_gain", "AM gain", CFG_BASE + 241, 0, 255, 25, "rf"),
    ("rt1000.squelch", "SQ", CFG_BASE + 258, 0, 255, 4, "rf"),
    ("rt1000.mic_gain", "MIC gain", CFG_BASE + 261, 0, 255, 18, "audio"),
    ("rt1000.spk_gain", "SPK gain", CFG_BASE + 262, 0, 255, 55, "audio"),
    ("rt1000.glitch_threshold", "Glitch threshold", CFG_BASE + 276, 0, 255, 10, "rf"),
    ("rt1000.tone_timer", "Tone timer", CFG_BASE + 277, 0, 255, 5, "audio"),
    ("rt1000.dtmf_gain", "DTMF gain", CFG_BASE + 518, 0, 255, 64, "dtmf"),
    ("rt1000.dtmf_decode_threshold", "DTMF decode threshold", CFG_BASE + 519, 0, 255, 24, "dtmf"),
    ("rt1000.spectrum_rssi", "Spectrum RSSI", CFG2_BASE + 16, 0, 255, 80, "rf"),
]

U16_SETTING_DEFS = [
    ("rt1000.tone_frequency", "Tone frequency", CFG_BASE + 256, 0, 65535, 1750, "audio"),
]

U32_SECONDS_SETTING_DEFS = [
    ("rt1000.gps_record_interval", "GPS record interval (s)", 5, 0, 86400, 60, "gps"),
    ("rt1000.gps_move_alarm_timer", "Move alarm timer (s)", 20, 0, 86400, 10, "gps"),
    ("rt1000.gps_callout_alarm_timer", "Callout alarm timer (s)", 25, 0, 86400, 10, "gps"),
]

APRS_U32_SECONDS_SETTING_DEFS = [
    ("rt1000.aprs_timed_beacon_timer", "Timed beacon timer (s)", 21, 0, 86400, 10, "aprs"),
    ("rt1000.aprs_low_speed_timer", "Low speed timer (s)", 30, 0, 86400, 10, "aprs"),
    ("rt1000.aprs_medium_speed_timer", "Medium speed timer (s)", 34, 0, 86400, 30, "aprs"),
    ("rt1000.aprs_high_speed_timer", "High speed timer (s)", 38, 0, 86400, 10, "aprs"),
]

APRS_U16_SETTING_DEFS = [
    ("rt1000.aprs_low_speed", "Low speed", 26, 0, 65535, 10, "aprs"),
    ("rt1000.aprs_high_speed", "High speed", 28, 0, 65535, 80, "aprs"),
    ("rt1000.aprs_steering_angle", "Steering angle", 42, 0, 65535, 10, "aprs"),
]

FREQ_KHZ_SETTING_DEFS = [
    ("rt1000.scanning_start_khz", "Scanning start (kHz)", CFG_BASE + 844, 0, 999000, 400125, "scan"),
    ("rt1000.scanning_end_khz", "Scanning end (kHz)", CFG_BASE + 848, 0, 999000, 439975, "scan"),
    ("rt1000.spectrum_center_khz", "Spectrum center (kHz)", CFG2_BASE + 8, 0, 999000, 435125, "rf"),
    ("rt1000.spectrum_step_khz", "Spectrum step (kHz)", CFG2_BASE + 12, 0, 999000, 1, "rf"),
]

# name, label, base offset of 32-bit total seconds, component, min, max, group
HMS_SETTING_DEFS = [
    ("rt1000.apo_hours", "Auto power off hours", CFG_BASE + 106, "h", 0, 23, "power"),
    ("rt1000.apo_minutes", "Auto power off minutes", CFG_BASE + 106, "m", 0, 59, "power"),
    ("rt1000.apo_seconds", "Auto power off seconds", CFG_BASE + 106, "s", 0, 59, "power"),
    ("rt1000.awu_hours", "Auto wake up hours", CFG_BASE + 111, "h", 0, 23, "power"),
    ("rt1000.awu_minutes", "Auto wake up minutes", CFG_BASE + 111, "m", 0, 59, "power"),
    ("rt1000.awu_seconds", "Auto wake up seconds", CFG_BASE + 111, "s", 0, 59, "power"),
]

STR_SETTING_DEFS = [
    ("rt1000.aprs_callsign", "APRS callsign", 44, 7, "aprs"),
    ("rt1000.aprs_comment", "APRS comment", 54, 128, "aprs"),
    ("rt1000.aprs_path1", "Path 1 name", 184, 6, "aprs"),
    ("rt1000.aprs_path2", "Path 2 name", 191, 6, "aprs"),
    ("rt1000.aprs_digi1_name", "DIGI 1 name", 202, 6, "aprs"),
    ("rt1000.aprs_digi2_name", "DIGI 2 name", 209, 6, "aprs"),
]

LIST_SETTING_BY_NAME = {d[0]: d for d in LIST_SETTING_DEFS}
GPS_LIST_SETTING_BY_NAME = {d[0]: d for d in GPS_LIST_SETTING_DEFS}
APRS_LIST_SETTING_BY_NAME = {d[0]: d for d in APRS_LIST_SETTING_DEFS}
INT8_SETTING_BY_NAME = {d[0]: d for d in INT8_SETTING_DEFS}
U16_SETTING_BY_NAME = {d[0]: d for d in U16_SETTING_DEFS}
FREQ_KHZ_SETTING_BY_NAME = {d[0]: d for d in FREQ_KHZ_SETTING_DEFS}
HMS_SETTING_BY_NAME = {d[0]: d for d in HMS_SETTING_DEFS}
U32_SECONDS_SETTING_BY_NAME = {d[0]: d for d in U32_SECONDS_SETTING_DEFS}
APRS_U32_SECONDS_SETTING_BY_NAME = {d[0]: d for d in APRS_U32_SECONDS_SETTING_DEFS}
APRS_U16_SETTING_BY_NAME = {d[0]: d for d in APRS_U16_SETTING_DEFS}
STR_SETTING_BY_NAME = {d[0]: d for d in STR_SETTING_DEFS}


@directory.register
class RadtelRT1000ProRadio(chirp_common.CloneModeRadio):
    """Radtel RT-1000 Pro"""

    VENDOR = "Radtel"
    MODEL = "RT-1000 Pro"
    BAUD_RATE = 115200

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = (
            "This is an experimental Radtel RT-1000 Pro driver produced "
            "from the vendor CPS protocol and memory map. Download from "
            "the radio and save a backup image before writing anything. "
            "Writing has not been verified on hardware."
        )
        rp.pre_download = (
            "Use the normal RT-1000 Pro programming cable. The CPS default "
            "baud rate is 115200."
        )
        rp.pre_upload = (
            "Writing is experimental. Keep a known-good backup image made "
            "from this same radio before continuing."
        )
        return rp

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = True
        rf.has_bank_names = True
        rf.has_bank_index = False
        rf.has_settings = True
        rf.memory_bounds = (1, CHAN_COUNT)
        rf.valid_bands = VALID_BANDS
        rf.valid_modes = ["FM", "NFM", "AM", "USB", "LSB"]
        rf.valid_duplexes = ["", "+", "-", "split", "off"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = list(chirp_common.CROSS_MODES)
        rf.valid_tuning_steps = [2.5, 5.0, 6.25, 10.0, 12.5, 25.0]
        rf.valid_skips = ["", "S"]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_name_length = 16
        return rf

    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    def sync_out(self):
        do_upload(self)

    def process_mmap(self):
        if not hasattr(self, "_mmap") or self._mmap is None:
            self._mmap = memmap.MemoryMapBytes(bytes([0xFF] * MEM_SIZE))
        self._bank_model = None

    def get_bank_model(self):
        if not hasattr(self, "_bank_model") or self._bank_model is None:
            self._bank_model = RT1000ProZoneBankModel(self)
        return self._bank_model

    def _get_zone_raw_name(self, index):
        base = _zone_offset(index) + 6
        return _decode_text(self._mmap[base:base + ZONE_NAME_LEN])

    def _get_zone_name(self, index):
        name = self._get_zone_raw_name(index)
        if not name:
            name = "Zone %03i" % (index + 1)
        return name

    def _set_zone_name(self, index, name):
        base = _zone_offset(index) + 6
        self._mmap.set(base, _encode_text(name, ZONE_NAME_LEN))
        if hasattr(self, "_bank_model") and self._bank_model is not None:
            try:
                self._bank_model._zones[index]._name = self._get_zone_name(index)
            except Exception:
                pass

    def _zone_label(self, index):
        return "%03i: %s" % (index + 1, self._get_zone_name(index))

    def _current_channel(self, base_offset):
        raw = _read_u16le(self._mmap, base_offset)
        if raw >= CHAN_COUNT:
            return 1
        return raw + 1

    def _set_current_channel(self, base_offset, number):
        number = max(1, min(CHAN_COUNT, int(number)))
        _write_u16le(self._mmap, base_offset, number - 1)

    def get_raw_memory(self, number):
        offset = _chan_offset(number)
        return self._mmap.printable(offset, offset + CHAN_SIZE)

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        offset = _chan_offset(number)
        rec = bytearray(self._mmap[offset:offset + CHAN_SIZE])

        # The vendor CPS treats byte0 bits 6..7 == 1 (0x40 set) as a
        # programmed channel. The previous experimental plugin had this
        # backwards, which made every valid memory appear empty in CHIRP.
        programmed = ((rec[0] >> 6) & 0x03) == 1
        mem.freq = _decode_freq(rec[5:9])
        if (not programmed or not mem.freq or
                rec == bytearray([0xFF] * CHAN_SIZE) or
                rec == bytearray([0x00] * CHAN_SIZE)):
            mem.empty = True
            return mem
        txfreq = _decode_freq(rec[9:13])
        mem.name = _decode_name(rec[32:48])
        cps_mode = (rec[4] >> 4) & 0x03
        cps_bandwidth = (rec[4] >> 6) & 0x03
        mem.mode = CPS_TO_MODE.get(cps_mode, "FM")
        if cps_mode == 0 and cps_bandwidth == 1:
            mem.mode = "NFM"
        mem.skip = "S" if ((rec[3] >> 6) & 0x01) else ""
        pwr_index = (rec[2] >> 6) & 0x03
        if pwr_index < len(POWER_LEVELS):
            mem.power = POWER_LEVELS[pwr_index]
        else:
            mem.power = POWER_LEVELS[0]

        rxtx = (rec[0] >> 4) & 0x03
        if rxtx == 1 or txfreq == 0:
            mem.duplex = "off"
            mem.offset = 0
        elif txfreq == mem.freq:
            mem.duplex = ""
            mem.offset = 0
        else:
            diff = txfreq - mem.freq
            if abs(diff) <= 100000000:
                mem.duplex = "+" if diff > 0 else "-"
                mem.offset = abs(diff)
            else:
                mem.duplex = "split"
                mem.offset = txfreq

        _decode_tones(mem, rec)
        mem.empty = not bool(mem.freq)
        return mem

    def set_memory(self, mem):
        offset = _chan_offset(mem.number)
        if mem.empty:
            self._mmap.set(offset, b"\xFF" * CHAN_SIZE)
            return

        old = bytearray(self._mmap[offset:offset + CHAN_SIZE])
        rec = old if old != bytearray([0xFF] * CHAN_SIZE) else bytearray(CHAN_SIZE)

        # Mark this as a programmed channel. CPS uses byte0 bits 6..7 == 1
        # (0x40 set) for valid/programmed channel memories; bits 4..5 are
        # the RX/TX selector. Preserve the low nibble of unknown flags.
        rec[0] = (rec[0] & 0x0F) | 0x40
        if mem.duplex == "off":
            # CHIRP standard TX-inhibit/RX-only representation.
            # CPS cboxRXTX order is 0=RX+TX, 1=Only RX, 2=Only TX;
            # the value lives in channel byte +0 bits 4..5.
            #
            # Do NOT zero the stored TX frequency. The radio/CPS appears to
            # use a zero TX frequency as a cue that the memory is invalid or
            # should be hidden. Keep a real TX frequency in the record and let
            # the RX/TX selector bit be the actual transmit-inhibit mechanism.
            old_txfreq = _decode_freq(old[9:13]) if len(old) >= 13 else 0
            txfreq = old_txfreq or mem.freq
            rec[0] |= (1 << 4)
        elif mem.duplex == "+":
            txfreq = mem.freq + mem.offset
        elif mem.duplex == "-":
            txfreq = mem.freq - mem.offset
        elif mem.duplex == "split":
            txfreq = mem.offset
        else:
            txfreq = mem.freq

        rec[5:9] = _encode_freq(mem.freq)
        rec[9:13] = _encode_freq(txfreq)
        _encode_tones(mem, rec)

        rec[2] = (rec[2] & 0x3F) | ((_nearest_power(mem.power) & 0x03) << 6)
        rec[3] = (rec[3] & 0x3F) | ((1 if mem.skip == "S" else 0) << 6)
        cps_mode = MODE_TO_CPS.get(mem.mode, 0)
        cps_bandwidth = 1 if mem.mode == "NFM" else 0
        rec[4] = (rec[4] & 0x0F) | ((cps_mode & 0x03) << 4) | ((cps_bandwidth & 0x03) << 6)
        rec[32:48] = _encode_name(mem.name)

        self._mmap.set(offset, bytes(rec))

    def _groups_for_settings(self):
        return {
            "basic": RadioSettingGroup("basic", "Driver / notes"),
            "startup": RadioSettingGroup("startup", "Startup"),
            "ui": RadioSettingGroup("ui", "Display / UI"),
            "power": RadioSettingGroup("power", "Power save / timers"),
            "behavior": RadioSettingGroup("behavior", "Radio behavior"),
            "scan": RadioSettingGroup("scan", "Scanning"),
            "rf": RadioSettingGroup("rf", "RF / spectrum"),
            "audio": RadioSettingGroup("audio", "Audio / tones"),
            "work": RadioSettingGroup("work", "Work ranges"),
            "keys": RadioSettingGroup("keys", "Key definitions"),
            "zones": RadioSettingGroup("zones", "Zones"),
            "dtmf": RadioSettingGroup("dtmf", "DTMF settings"),
            "broadcast": RadioSettingGroup("broadcast", "Broadcast radio"),
            "broadcast_channels": RadioSettingGroup("broadcast_channels", "Broadcast FM presets"),
            "gps": RadioSettingGroup("gps", "GPS settings"),
            "aprs": RadioSettingGroup("aprs", "APRS settings"),
            "tables": RadioSettingGroup("tables", "Structured editors"),
        }

    def _append_list_setting(self, groups, definition, base_adjust=0):
        name, label, offset, options, default, group = definition
        offset += base_adjust
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueList(
                options,
                current_index=_safe_byte(self._mmap, offset, options,
                                         default=default))))

    def _append_int_setting(self, groups, definition, base_adjust=0):
        name, label, offset, minimum, maximum, default, group = definition
        offset += base_adjust
        try:
            value = _byte_value(self._mmap[offset])
        except Exception:
            value = default
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueInteger(minimum, maximum,
                                     _clamp_int(value, minimum, maximum))))

    def _append_u16_setting(self, groups, definition, base_adjust=0):
        name, label, offset, minimum, maximum, default, group = definition
        offset += base_adjust
        try:
            value = _read_u16le(self._mmap, offset)
        except Exception:
            value = default
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueInteger(minimum, maximum,
                                     _clamp_int(value, minimum, maximum))))

    def _append_u32_seconds_setting(self, groups, definition, base_adjust=0):
        name, label, offset, minimum, maximum, default, group = definition
        offset += base_adjust
        try:
            value = _read_u32le(self._mmap, offset)
        except Exception:
            value = default
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueInteger(minimum, maximum,
                                     _clamp_int(value, minimum, maximum))))

    def _append_freq_khz_setting(self, groups, definition):
        name, label, offset, minimum, maximum, default, group = definition
        try:
            value = _freq_raw_to_khz(_read_u32le(self._mmap, offset))
        except Exception:
            value = default
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueInteger(minimum, maximum,
                                     _clamp_int(value, minimum, maximum))))

    def _append_hms_setting(self, groups, definition):
        name, label, offset, component, minimum, maximum, group = definition
        total = _read_u32le(self._mmap, offset)
        hours = total // 3600
        minutes = (total % 3600) // 60
        seconds = total % 60
        value = {"h": hours, "m": minutes, "s": seconds}[component]
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueInteger(minimum, maximum,
                                     _clamp_int(value, minimum, maximum))))

    def _append_string_setting(self, groups, definition, base_adjust=0):
        name, label, offset, length, group = definition
        offset += base_adjust
        value = _decode_text(self._mmap[offset:offset + length])
        groups[group].append(RadioSetting(
            name, label,
            RadioSettingValueString(0, length, value)))

    def get_settings(self):
        groups = self._groups_for_settings()
        groups["basic"].append(RadioSetting(
            "driver_status", "Driver status",
            RadioSettingValueList(["experimental / settings expanded"],
                                  current_index=0)))
        groups["tables"].append(RadioSetting(
            "rt1000.structured_note", "Structured tables note",
            RadioSettingValueList([
                "DTMF code strings and broadcast FM presets are editable; GPS/APRS record logs and startup bitmap data are not editable in this build"
            ], current_index=0)))

        for index in range(DTMF_CODE_COUNT):
            groups["dtmf"].append(RadioSetting(
                "rt1000.dtmf_code_%02i" % index,
                "DTMF-%02i code" % (index + 1),
                RadioSettingValueString(0, DTMF_CODE_MAX_LEN,
                                        _read_dtmf_code(self._mmap, index))))

        for index in range(BCAST_CH_COUNT):
            enabled = not _broadcast_record_is_empty(self._mmap, index)
            groups["broadcast_channels"].append(RadioSetting(
                "rt1000.bcast_enabled_%02i" % index,
                "FM preset %02i enabled" % (index + 1),
                RadioSettingValueList(ON_OFF_OPTIONS,
                                      current_index=1 if enabled else 0)))
            groups["broadcast_channels"].append(RadioSetting(
                "rt1000.bcast_freq_%02i" % index,
                "FM preset %02i frequency (kHz)" % (index + 1),
                RadioSettingValueInteger(64000, 108000,
                                         _read_bcast_freq_khz(self._mmap, index))))
            groups["broadcast_channels"].append(RadioSetting(
                "rt1000.bcast_name_%02i" % index,
                "FM preset %02i alias" % (index + 1),
                RadioSettingValueString(0, BCAST_NAME_LEN,
                                        _read_bcast_name(self._mmap, index))))

        for definition in LIST_SETTING_DEFS:
            self._append_list_setting(groups, definition)
        for definition in INT8_SETTING_DEFS:
            self._append_int_setting(groups, definition)
        for definition in U16_SETTING_DEFS:
            self._append_u16_setting(groups, definition)
        for definition in FREQ_KHZ_SETTING_DEFS:
            self._append_freq_khz_setting(groups, definition)
        for definition in HMS_SETTING_DEFS:
            self._append_hms_setting(groups, definition)

        for definition in GPS_LIST_SETTING_DEFS:
            self._append_list_setting(groups, definition, base_adjust=ADDR_GPS)
        for definition in U32_SECONDS_SETTING_DEFS:
            self._append_u32_seconds_setting(groups, definition,
                                            base_adjust=ADDR_GPS)

        for definition in APRS_LIST_SETTING_DEFS:
            self._append_list_setting(groups, definition, base_adjust=ADDR_APRS)
        for definition in APRS_U32_SECONDS_SETTING_DEFS:
            self._append_u32_seconds_setting(groups, definition,
                                            base_adjust=ADDR_APRS)
        for definition in APRS_U16_SETTING_DEFS:
            self._append_u16_setting(groups, definition, base_adjust=ADDR_APRS)
        for definition in STR_SETTING_DEFS:
            self._append_string_setting(groups, definition, base_adjust=ADDR_APRS)

        zone_options = [self._zone_label(i) for i in range(ZONE_COUNT)]
        for label, suffix, off in (("Range A", "a", 26),
                                   ("Range B", "b", 27),
                                   ("Range C", "c", 28)):
            groups["zones"].append(RadioSetting(
                "rt1000.zone_%s" % suffix,
                "%s zone" % label,
                RadioSettingValueList(
                    zone_options,
                    current_index=_safe_byte(self._mmap, CFG2_BASE + off,
                                             zone_options))))

        for label, suffix, off in (("Range A", "a", 29),
                                   ("Range B", "b", 31),
                                   ("Range C", "c", 33)):
            groups["zones"].append(RadioSetting(
                "rt1000.channel_%s" % suffix,
                "%s channel" % label,
                RadioSettingValueInteger(
                    1, CHAN_COUNT, self._current_channel(CFG2_BASE + off))))

        for index in range(ZONE_COUNT):
            groups["zones"].append(RadioSetting(
                "rt1000.zone_name_%03i" % index,
                "Zone %03i name" % (index + 1),
                RadioSettingValueString(
                    0, ZONE_NAME_LEN, self._get_zone_raw_name(index))))

        return RadioSettings(groups["basic"], groups["startup"], groups["ui"],
                             groups["power"], groups["behavior"],
                             groups["scan"], groups["rf"], groups["audio"],
                             groups["work"], groups["zones"], groups["keys"],
                             groups["dtmf"], groups["broadcast"],
                             groups["broadcast_channels"], groups["gps"],
                             groups["aprs"], groups["tables"])

    def _set_hms_component(self, offset, component, value):
        total = _read_u32le(self._mmap, offset)
        hours = total // 3600
        minutes = (total % 3600) // 60
        seconds = total % 60
        if component == "h":
            hours = _clamp_int(value, 0, 23)
        elif component == "m":
            minutes = _clamp_int(value, 0, 59)
        else:
            seconds = _clamp_int(value, 0, 59)
        _write_u32le(self._mmap, offset, hours * 3600 + minutes * 60 + seconds)

    def set_settings(self, settings):
        for element in settings:
            if hasattr(element, "get_name") and hasattr(element, "value"):
                name = element.get_name()
                value = element.value
            elif isinstance(element, RadioSettingGroup):
                self.set_settings(element)
                continue
            else:
                LOG.warning("Skipping unexpected RT-1000 Pro setting element %r",
                            element)
                continue

            if name in ("driver_status", "rt1000.structured_note"):
                continue
            elif name in LIST_SETTING_BY_NAME:
                _, _, offset, options, _, _ = LIST_SETTING_BY_NAME[name]
                self._mmap.set(offset, bytes([options.index(str(value)) & 0xFF]))
            elif name in GPS_LIST_SETTING_BY_NAME:
                _, _, offset, options, _, _ = GPS_LIST_SETTING_BY_NAME[name]
                self._mmap.set(ADDR_GPS + offset,
                               bytes([options.index(str(value)) & 0xFF]))
            elif name in APRS_LIST_SETTING_BY_NAME:
                _, _, offset, options, _, _ = APRS_LIST_SETTING_BY_NAME[name]
                self._mmap.set(ADDR_APRS + offset,
                               bytes([options.index(str(value)) & 0xFF]))
            elif name in INT8_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = INT8_SETTING_BY_NAME[name]
                self._mmap.set(offset, bytes([_clamp_int(value, minimum, maximum) & 0xFF]))
            elif name in U16_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = U16_SETTING_BY_NAME[name]
                _write_u16le(self._mmap, offset,
                             _clamp_int(value, minimum, maximum))
            elif name in FREQ_KHZ_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = FREQ_KHZ_SETTING_BY_NAME[name]
                khz = _clamp_int(value, minimum, maximum)
                _write_u32le(self._mmap, offset, _freq_khz_to_raw(khz))
            elif name in HMS_SETTING_BY_NAME:
                _, _, offset, component, _, _, _ = HMS_SETTING_BY_NAME[name]
                self._set_hms_component(offset, component, int(value))
            elif name in U32_SECONDS_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = U32_SECONDS_SETTING_BY_NAME[name]
                _write_u32le(self._mmap, ADDR_GPS + offset,
                             _clamp_int(value, minimum, maximum))
            elif name in APRS_U32_SECONDS_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = APRS_U32_SECONDS_SETTING_BY_NAME[name]
                _write_u32le(self._mmap, ADDR_APRS + offset,
                             _clamp_int(value, minimum, maximum))
            elif name in APRS_U16_SETTING_BY_NAME:
                _, _, offset, minimum, maximum, _, _ = APRS_U16_SETTING_BY_NAME[name]
                _write_u16le(self._mmap, ADDR_APRS + offset,
                             _clamp_int(value, minimum, maximum))
            elif name in STR_SETTING_BY_NAME:
                _, _, offset, length, _ = STR_SETTING_BY_NAME[name]
                self._mmap.set(ADDR_APRS + offset, _encode_text(str(value), length))
            elif name.startswith("rt1000.dtmf_code_"):
                index = int(name.rsplit("_", 1)[1])
                if 0 <= index < DTMF_CODE_COUNT:
                    _write_dtmf_code(self._mmap, index, str(value))
            elif name.startswith("rt1000.bcast_enabled_"):
                index = int(name.rsplit("_", 1)[1])
                if 0 <= index < BCAST_CH_COUNT:
                    _set_bcast_enabled(self._mmap, index, str(value) == "On")
            elif name.startswith("rt1000.bcast_freq_"):
                index = int(name.rsplit("_", 1)[1])
                if 0 <= index < BCAST_CH_COUNT:
                    _set_bcast_enabled(self._mmap, index, True)
                    _write_bcast_freq_khz(self._mmap, index, int(value))
            elif name.startswith("rt1000.bcast_name_"):
                index = int(name.rsplit("_", 1)[1])
                if 0 <= index < BCAST_CH_COUNT:
                    _set_bcast_enabled(self._mmap, index, True)
                    _write_bcast_name(self._mmap, index, str(value))
            elif name.startswith("rt1000.zone_") and \
                    not name.startswith("rt1000.zone_name_"):
                suffix = name.rsplit("_", 1)[1]
                offsets = {"a": 26, "b": 27, "c": 28}
                try:
                    zone_index = int(str(value).split(":", 1)[0]) - 1
                except Exception:
                    zone_index = 0
                zone_index = max(0, min(ZONE_COUNT - 1, zone_index))
                self._mmap.set(CFG2_BASE + offsets[suffix],
                               bytes([zone_index]))
            elif name.startswith("rt1000.channel_"):
                suffix = name.rsplit("_", 1)[1]
                offsets = {"a": 29, "b": 31, "c": 33}
                self._set_current_channel(CFG2_BASE + offsets[suffix],
                                          int(value))
            elif name.startswith("rt1000.zone_name_"):
                index = int(name.rsplit("_", 1)[1])
                self._set_zone_name(index, str(value))
            else:
                LOG.warning("Unhandled RT-1000 Pro setting %s", name)
