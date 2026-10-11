"""Minimal WPILib .wpilog reader/writer for the keys this project logs (AdvantageKit writes outputs
under /RealOutputs/<key>). Pose2d structs are 3 little-endian doubles (x, y, theta).

Reading uses robotpy's wpiutil DataLogReader when available and falls back to a pure-Python parser.
"""

import struct
from collections import defaultdict

POSE2D = struct.Struct("<3d")


# ---------------------------------------------------------------------------- writing
class WpilogWriter:
    def __init__(self, path, extra_header="alignsim"):
        self.f = open(path, "wb")
        eh = extra_header.encode()
        self.f.write(b"WPILOG" + struct.pack("<H", 0x0100) + struct.pack("<I", len(eh)) + eh)
        self.ids = {}

    @staticmethod
    def _len_bytes(v, maxb):
        for n in range(1, maxb + 1):
            if v < (1 << (8 * n)):
                return n
        return maxb

    def _record(self, entry, ts_us, payload):
        ni = self._len_bytes(entry, 4)
        np_ = self._len_bytes(len(payload), 4)
        nt = self._len_bytes(ts_us, 8)
        hdr = (ni - 1) | ((np_ - 1) << 2) | ((nt - 1) << 4)
        self.f.write(bytes([hdr]) + entry.to_bytes(ni, "little") + len(payload).to_bytes(np_, "little")
                     + ts_us.to_bytes(nt, "little") + payload)

    def entry(self, name, type_, ts_us=0):
        if name in self.ids:
            return self.ids[name]
        eid = len(self.ids) + 1
        self.ids[name] = eid
        nb, tb = name.encode(), type_.encode()
        payload = (bytes([0]) + struct.pack("<I", eid) + struct.pack("<I", len(nb)) + nb
                   + struct.pack("<I", len(tb)) + tb + struct.pack("<I", 0))
        self._record(0, ts_us, payload)
        return eid

    def double(self, name, ts, v):
        self._record(self.entry(name, "double"), int(ts * 1e6), struct.pack("<d", v))

    def doubles(self, name, ts, vs):
        self._record(self.entry(name, "double[]"), int(ts * 1e6), struct.pack(f"<{len(vs)}d", *vs))

    def boolean(self, name, ts, v):
        self._record(self.entry(name, "boolean"), int(ts * 1e6), bytes([1 if v else 0]))

    def pose(self, name, ts, p):
        self._record(self.entry(name, "struct:Pose2d"), int(ts * 1e6), POSE2D.pack(*p))

    def poses(self, name, ts, ps):
        self._record(self.entry(name, "struct:Pose2d[]"), int(ts * 1e6), b"".join(POSE2D.pack(*p) for p in ps))

    def close(self):
        self.f.close()


# ---------------------------------------------------------------------------- reading
def _decode(type_, raw):
    if type_ == "double":
        return struct.unpack("<d", raw)[0]
    if type_ == "double[]":
        return list(struct.unpack(f"<{len(raw) // 8}d", raw))
    if type_ == "boolean":
        return raw[0] != 0
    if type_ in ("int64", "int"):
        return struct.unpack("<q", raw)[0]
    if type_ == "struct:Pose2d":
        return POSE2D.unpack(raw)
    if type_ == "struct:Pose2d[]":
        return [POSE2D.unpack_from(raw, i) for i in range(0, len(raw), 24)]
    if type_ == "string":
        return raw.decode(errors="replace")
    return raw


def read(path, keys=None):
    """-> {name: [(t_seconds, value), ...]} for the requested names (exact or suffix match)."""
    with open(path, "rb") as f:
        data = f.read()
    assert data[:6] == b"WPILOG", "not a wpilog file"
    pos = 12 + struct.unpack_from("<I", data, 8)[0]
    entries = {}
    out = defaultdict(list)

    def wanted(name):
        return keys is None or any(name == k or name.endswith("/" + k) or name.endswith(k) for k in keys)

    n = len(data)
    while pos < n:
        hdr = data[pos]
        ni, np_, nt = (hdr & 3) + 1, ((hdr >> 2) & 3) + 1, ((hdr >> 4) & 7) + 1
        pos += 1
        eid = int.from_bytes(data[pos:pos + ni], "little")
        pos += ni
        size = int.from_bytes(data[pos:pos + np_], "little")
        pos += np_
        ts = int.from_bytes(data[pos:pos + nt], "little")
        pos += nt
        payload = data[pos:pos + size]
        pos += size
        if eid == 0:
            if payload and payload[0] == 0:  # start
                e = struct.unpack_from("<I", payload, 1)[0]
                ln = struct.unpack_from("<I", payload, 5)[0]
                name = payload[9:9 + ln].decode()
                lt = struct.unpack_from("<I", payload, 9 + ln)[0]
                type_ = payload[13 + ln:13 + ln + lt].decode()
                entries[e] = (name, type_)
            continue
        if eid in entries:
            name, type_ = entries[eid]
            if wanted(name):
                out[name].append((ts / 1e6, _decode(type_, payload)))
    return dict(out)
