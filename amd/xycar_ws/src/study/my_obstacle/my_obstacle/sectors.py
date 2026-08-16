"""LaserScan -> 섹터별 최근접 거리.

ROS 의존성 없음 (numpy만). 단위 테스트와 오프라인 검증을 위해 분리했다.

역할 분담:
    YOLO는 "무엇"인지 알지만 거리를 모르고, 라이다는 거리를 알지만 그게 뭔지 모른다.
    둘은 상보적이다. 여기서는 거리만 낸다.
"""

import math

import numpy as np


def sector_min(ranges, angle_min, angle_increment, lo_deg, hi_deg,
               range_min, range_max, min_points=3):
    """[lo_deg, hi_deg] 각도 구간의 최근접 거리.

    각도는 라이다 정면이 0도, 좌측이 +(반시계) 기준.
    유효값이 min_points 개 미만이면 range_max 를 반환한다 (= 비어 있다고 본다).

    단발 노이즈에 끌려가지 않도록 최소값이 아니라 **하위 몇 개의 중앙값**을 쓴다.
    라이다는 반사/난반사로 가짜 근접값이 튀는 일이 잦다.
    """
    n = len(ranges)
    if n == 0:
        return range_max

    r = np.asarray(ranges, dtype=np.float64)
    idx = np.arange(n)
    ang = np.degrees(angle_min + idx * angle_increment)
    # [-180, 180) 로 정규화
    ang = (ang + 180.0) % 360.0 - 180.0

    sel = (ang >= lo_deg) & (ang <= hi_deg)
    vals = r[sel]
    vals = vals[np.isfinite(vals)]
    vals = vals[(vals >= range_min) & (vals <= range_max)]

    if vals.size < min_points:
        return range_max

    k = max(min_points, int(vals.size * 0.05))
    nearest = np.sort(vals)[:k]
    return float(np.median(nearest))


def time_to_collision(front_dist, speed_mps, min_speed=0.05):
    """전방 거리와 현재 속도로 충돌예상시간(TTC)을 낸다.

    f1tenth safety_node 와 같은 발상. 속도가 거의 0이면 무한대로 본다.
    """
    if speed_mps < min_speed:
        return math.inf
    return front_dist / speed_mps
