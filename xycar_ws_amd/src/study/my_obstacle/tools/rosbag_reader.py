"""ROS 2 bag(.db3) 을 **ROS 설치 없이** 읽는다.

왜 필요한가: 개발 PC(Windows)에는 ROS 가 없다. 젯슨에서 녹화한 rosbag 을
여기서 열어 복도 추정을 오프라인 튜닝하려면 직접 파싱해야 한다.

ROS 2 bag 은 그냥 **sqlite3 파일**이고, 메시지는 **CDR**(DDS 직렬화)로
들어 있다. 둘 다 표준 라이브러리로 읽을 수 있다.

    topics    (id, name, type, serialization_format, offered_qos_profiles)
    messages  (id, topic_id, timestamp, data BLOB)   <- data 가 CDR

지원 메시지: sensor_msgs/LaserScan 만. 복도 추정에 필요한 건 그것뿐이다.

────────────────────────────────────────────────────────────────────────
CDR 규칙 (여기서 쓰는 부분만)

- 앞 4바이트는 encapsulation header. `00 01` = little-endian, `00 00` = big.
  본문 정렬(alignment)은 **이 4바이트 다음을 0으로 보고** 계산한다.
- 각 원시 타입은 자기 크기에 정렬된다 (float32 -> 4, uint32 -> 4).
- string: uint32 길이(널 포함) + 바이트열(널 종료).
- sequence<T>: uint32 개수 + 요소들.

Humble 기본 저장 포맷이 sqlite3 라 이걸 가정한다. mcap 으로 녹화했다면
(`--storage mcap`) 이 리더는 못 읽는다 — 그 경우 젯슨에서 db3 로 다시
녹화하거나 `ros2 bag convert` 를 쓸 것.
"""

import sqlite3
import struct


class CDRReader:
    """CDR 버퍼에서 정렬을 지켜가며 값을 꺼낸다."""

    def __init__(self, buf):
        if len(buf) < 4:
            raise ValueError("CDR 버퍼가 너무 짧다")
        # buf[0] 는 보통 0, buf[1] 이 엔디안 플래그
        self.little = (buf[1] & 0x01) == 1
        self._b = buf
        self._p = 4          # encapsulation header 4바이트 건너뜀
        self._base = 4       # 정렬 기준점

    def _align(self, n):
        off = (self._p - self._base) % n
        if off:
            self._p += n - off

    def _get(self, fmt, size):
        self._align(size)
        e = "<" if self.little else ">"
        v = struct.unpack_from(e + fmt, self._b, self._p)[0]
        self._p += size
        return v

    def u32(self):
        return self._get("I", 4)

    def i32(self):
        return self._get("i", 4)

    def f32(self):
        return self._get("f", 4)

    def f64(self):
        return self._get("d", 8)

    def string(self):
        n = self.u32()
        s = self._b[self._p:self._p + n]
        self._p += n
        return s.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def f32_seq(self):
        n = self.u32()
        self._align(4)
        e = "<" if self.little else ">"
        v = struct.unpack_from(f"{e}{n}f", self._b, self._p)
        self._p += 4 * n
        return list(v)


class LaserScanMsg:
    __slots__ = ("stamp_sec", "stamp_nsec", "frame_id", "angle_min", "angle_max",
                 "angle_increment", "time_increment", "scan_time",
                 "range_min", "range_max", "ranges", "intensities")

    @property
    def stamp(self):
        return self.stamp_sec + self.stamp_nsec * 1e-9


def decode_laserscan(data):
    r = CDRReader(data)
    m = LaserScanMsg()
    m.stamp_sec = r.i32()
    m.stamp_nsec = r.u32()
    m.frame_id = r.string()
    m.angle_min = r.f32()
    m.angle_max = r.f32()
    m.angle_increment = r.f32()
    m.time_increment = r.f32()
    m.scan_time = r.f32()
    m.range_min = r.f32()
    m.range_max = r.f32()
    m.ranges = r.f32_seq()
    m.intensities = r.f32_seq()
    return m


def list_topics(db3_path):
    """[(name, type, 메시지수), ...]"""
    con = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
    try:
        rows = con.execute(
            "SELECT t.name, t.type, COUNT(m.id) "
            "FROM topics t LEFT JOIN messages m ON m.topic_id = t.id "
            "GROUP BY t.id ORDER BY t.name").fetchall()
    finally:
        con.close()
    return rows


def read_scans(db3_path, topic="/scan", limit=None, stride=1):
    """LaserScan 메시지를 순서대로 내놓는다 (제너레이터)."""
    con = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
    try:
        row = con.execute(
            "SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
        if row is None:
            names = [r[0] for r in con.execute("SELECT name FROM topics")]
            raise KeyError(f"토픽 '{topic}' 이 bag 에 없다. 있는 토픽: {names}")
        topic_id, type_name = row
        if "LaserScan" not in type_name:
            raise TypeError(f"'{topic}' 은 {type_name} 이다. LaserScan 만 지원한다.")

        q = con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? "
            "ORDER BY timestamp", (topic_id,))
        n = 0
        for i, (_ts, blob) in enumerate(q):
            if i % stride:
                continue
            yield decode_laserscan(blob)
            n += 1
            if limit and n >= limit:
                return
    finally:
        con.close()


def find_db3(path):
    """디렉터리를 주면 안에서 .db3 를 찾는다. 파일이면 그대로."""
    import os
    if os.path.isfile(path):
        return path
    if os.path.isdir(path):
        cands = sorted(f for f in os.listdir(path) if f.endswith(".db3"))
        if not cands:
            raise FileNotFoundError(f"{path} 안에 .db3 가 없다 "
                                    f"(mcap 으로 녹화했다면 db3 로 변환할 것)")
        return os.path.join(path, cands[0])
    raise FileNotFoundError(path)
