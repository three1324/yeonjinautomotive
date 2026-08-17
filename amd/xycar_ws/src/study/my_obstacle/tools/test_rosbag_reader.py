#!/usr/bin/env python3
"""rosbag_reader 자체 검증. 실제 bag 없이 지금 돌릴 수 있다.

    python3 tools/test_rosbag_reader.py

실제 rosbag 이 내일 오는데, 리더가 틀리면 그 데이터로 내리는 모든 결론이
틀린다. 그래서 **ROS 가 만드는 것과 같은 바이트열**을 직접 만들어
왕복(encode -> decode)이 맞는지 먼저 확인한다.

여기서 만드는 CDR 은 ROS 2(Fast-CDR)가 LaserScan 을 직렬화할 때와 같은
규칙이다 — encapsulation 4바이트, 자기 크기 정렬, string 은 길이(널 포함)+
널종료 바이트열, sequence 는 개수+요소.
"""

import math
import os
import struct
import sys
import tempfile

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rosbag_reader import (  # noqa: E402
    decode_laserscan, find_db3, list_topics, read_scans)


class CDRWriter:
    """검증용 인코더. ROS 2 가 쓰는 규칙을 그대로 흉내낸다."""

    def __init__(self, little=True):
        self.little = little
        self.e = "<" if little else ">"
        self.b = bytearray(b"\x00\x01\x00\x00" if little else b"\x00\x00\x00\x00")
        self.base = 4

    def _align(self, n):
        off = (len(self.b) - self.base) % n
        if off:
            self.b += b"\x00" * (n - off)

    def _put(self, fmt, size, v):
        self._align(size)
        self.b += struct.pack(self.e + fmt, v)

    def u32(self, v):
        self._put("I", 4, v)

    def i32(self, v):
        self._put("i", 4, v)

    def f32(self, v):
        self._put("f", 4, v)

    def string(self, s):
        raw = s.encode("utf-8") + b"\x00"
        self.u32(len(raw))
        self.b += raw

    def f32_seq(self, xs):
        self.u32(len(xs))
        self._align(4)
        self.b += struct.pack(f"{self.e}{len(xs)}f", *xs)


def encode_laserscan(sec, nsec, frame, a_min, a_max, a_inc,
                     r_min, r_max, ranges, intensities, little=True):
    w = CDRWriter(little)
    w.i32(sec)
    w.u32(nsec)
    w.string(frame)
    w.f32(a_min)
    w.f32(a_max)
    w.f32(a_inc)
    w.f32(0.0)          # time_increment
    w.f32(0.1)          # scan_time
    w.f32(r_min)
    w.f32(r_max)
    w.f32_seq(ranges)
    w.f32_seq(intensities)
    return bytes(w.b)


def make_bag(path, n_msgs=5, n_beams=360):
    """ros2 bag 과 같은 스키마의 sqlite3 파일을 만든다."""
    import sqlite3
    con = sqlite3.connect(path)
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, "
                "type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, "
                "timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES (1, '/scan', 'sensor_msgs/msg/LaserScan', 'cdr', '')")
    con.execute("INSERT INTO topics VALUES (2, '/imu', 'sensor_msgs/msg/Imu', 'cdr', '')")

    a_min, a_max = -math.pi, math.pi
    a_inc = (a_max - a_min) / n_beams
    for k in range(n_msgs):
        ranges = [1.0 + 0.001 * i + 0.1 * k for i in range(n_beams)]
        blob = encode_laserscan(1700000000 + k, 500000000 + k, "laser",
                                a_min, a_max, a_inc, 0.05, 12.0,
                                ranges, [])
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)",
                    (1700000000000000000 + k * 100000000, blob))
    con.commit()
    con.close()
    return a_min, a_inc


def check(name, cond, detail=""):
    print(f"  {'OK  ' if cond else '실패'} {name}{('  ' + detail) if detail else ''}")
    return cond


def main():
    ok = []

    print("[1] CDR 왕복 — 리틀엔디안")
    ranges = [0.5, 1.25, 3.0, float("inf"), 7.75]
    blob = encode_laserscan(12345, 678, "laser_frame",
                            -math.pi, math.pi, 0.01, 0.05, 12.0,
                            ranges, [1.0, 2.0])
    m = decode_laserscan(blob)
    ok.append(check("stamp", m.stamp_sec == 12345 and m.stamp_nsec == 678,
                    f"{m.stamp_sec}.{m.stamp_nsec}"))
    ok.append(check("frame_id", m.frame_id == "laser_frame", repr(m.frame_id)))
    ok.append(check("angle_increment", abs(m.angle_increment - 0.01) < 1e-6))
    ok.append(check("range_max", abs(m.range_max - 12.0) < 1e-6))
    ok.append(check("ranges 개수", len(m.ranges) == 5, str(len(m.ranges))))
    ok.append(check("ranges 값", all(
        (math.isinf(a) and math.isinf(b)) or abs(a - b) < 1e-6
        for a, b in zip(m.ranges, ranges))))
    ok.append(check("inf 보존", math.isinf(m.ranges[3])))
    ok.append(check("intensities", len(m.intensities) == 2))

    print("\n[2] CDR 왕복 — 빅엔디안 (엔디안 플래그를 실제로 읽는가)")
    blob_be = encode_laserscan(1, 2, "be", -1.0, 1.0, 0.5, 0.1, 5.0,
                               [1.0, 2.0, 3.0], [], little=False)
    mbe = decode_laserscan(blob_be)
    ok.append(check("frame_id", mbe.frame_id == "be"))
    ok.append(check("ranges", [round(x, 3) for x in mbe.ranges] == [1.0, 2.0, 3.0],
                    str(mbe.ranges)))

    print("\n[3] 빈 스캔 / 긴 frame_id (정렬 경계)")
    for fid in ("", "a", "ab", "abc", "abcd", "abcde", "velodyne_laser_link"):
        b = encode_laserscan(0, 0, fid, -1.0, 1.0, 0.1, 0.0, 1.0, [9.5], [])
        mm = decode_laserscan(b)
        good = mm.frame_id == fid and abs(mm.ranges[0] - 9.5) < 1e-6
        ok.append(check(f"frame_id={fid!r}", good))

    print("\n[4] sqlite3 bag 읽기")
    tmp = tempfile.mkdtemp(prefix="bagtest_")
    db3 = os.path.join(tmp, "cone_section_0.db3")
    make_bag(db3, n_msgs=5, n_beams=360)

    topics = list_topics(db3)
    ok.append(check("토픽 목록", len(topics) == 2, str([t[0] for t in topics])))
    scan_row = [t for t in topics if t[0] == "/scan"][0]
    ok.append(check("메시지 수", scan_row[2] == 5, str(scan_row[2])))

    got = list(read_scans(db3, "/scan"))
    ok.append(check("스캔 5개", len(got) == 5, str(len(got))))
    ok.append(check("빔 360개", len(got[0].ranges) == 360, str(len(got[0].ranges))))
    ok.append(check("시간 순서", all(got[i].stamp < got[i + 1].stamp
                                  for i in range(len(got) - 1))))
    ok.append(check("메시지별로 값이 다름",
                    abs(got[0].ranges[0] - got[1].ranges[0] + 0.1) < 1e-5,
                    f"{got[0].ranges[0]:.3f} vs {got[1].ranges[0]:.3f}"))

    got2 = list(read_scans(db3, "/scan", limit=2))
    ok.append(check("limit=2", len(got2) == 2))
    got3 = list(read_scans(db3, "/scan", stride=2))
    ok.append(check("stride=2", len(got3) == 3, str(len(got3))))

    print("\n[5] 오류 처리")
    try:
        list(read_scans(db3, "/nope"))
        ok.append(check("없는 토픽 -> KeyError", False))
    except KeyError as e:
        ok.append(check("없는 토픽 -> KeyError", True, "있는 토픽 안내함"
                        if "/scan" in str(e) else ""))
    try:
        list(read_scans(db3, "/imu"))
        ok.append(check("LaserScan 아님 -> TypeError", False))
    except TypeError:
        ok.append(check("LaserScan 아님 -> TypeError", True))

    ok.append(check("디렉터리로 db3 찾기", find_db3(tmp) == db3))

    import shutil
    shutil.rmtree(tmp, ignore_errors=True)

    print()
    n = sum(ok)
    print(f"{n}/{len(ok)} 통과")
    return 0 if n == len(ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
