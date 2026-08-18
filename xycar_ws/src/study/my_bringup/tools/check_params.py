#!/usr/bin/env python3
"""drive_params.yaml 과 노드의 declare_parameters() 를 대조한다.

    python3 tools/check_params.py

왜 필요한가:
    ROS2 노드는 선언(declare)하지 않은 파라미터가 yaml 에 있으면 **시작 시 예외로 죽는다**.
    반대로 노드가 선언했는데 yaml 에 없으면 조용히 기본값을 쓴다 (이건 허용).
    파라미터를 추가/이름변경할 때 한쪽만 고치는 실수가 잦아서, 젯슨에 올리기 전에
    여기서 미리 잡는다. ROS2 없이 PC 에서 돌아간다.

    실제로 이 검사가 light.miss_tolerance 누락(노드 시작 실패)을 잡아냈다.

형식 주의:
    ROS2 파라미터 yaml 은 중첩 구조로 쓴다.
        lane:
          y_lo: 270      ->  파라미터 이름 "lane.y_lo"
"""

import os
import re
import sys

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
STUDY = os.path.normpath(os.path.join(HERE, "..", ".."))

CONFIG = os.path.join(STUDY, "my_bringup", "config", "drive_params.yaml")
NODES = [
    ("perception_node", "my_perception/my_perception/perception_node.py"),
    ("obstacle_node", "my_obstacle/my_obstacle/obstacle_node.py"),
    ("driver_node", "my_driver/my_driver/driver_node.py"),
    ("viz_node", "my_debug/my_debug/viz_node.py"),
]


def flatten(d, prefix=""):
    """중첩 dict -> {"a.b.c": value} 형태. ROS2 파라미터 이름 규칙과 동일."""
    out = {}
    for key, value in d.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            out.update(flatten(value, name + "."))
        else:
            out[name] = value
    return out


def declared_names(path):
    """노드 소스의 declare_parameters(...) 블록에서 파라미터 이름을 뽑는다."""
    src = open(path, encoding="utf-8").read()
    if "declare_parameters(" not in src:
        return set()
    body = src.split("declare_parameters(", 1)[1]
    return set(re.findall(r'\("([^"]+)",', body))


def main():
    cfg = yaml.safe_load(open(CONFIG, encoding="utf-8"))
    problems = 0

    for node, rel in NODES:
        path = os.path.join(STUDY, rel)
        if not os.path.exists(path):
            print(f"[{node}] 소스를 찾을 수 없음: {path}")
            problems += 1
            continue
        if node not in cfg:
            print(f"[{node}] yaml 에 해당 노드 섹션이 없음")
            problems += 1
            continue

        provided = set(flatten(cfg[node]["ros__parameters"]))
        declared = declared_names(path)

        undeclared = sorted(provided - declared)
        defaulted = sorted(declared - provided)

        print(f"[{node}]  yaml {len(provided)}개 / 선언 {len(declared)}개")
        if undeclared:
            print("   [실패] 노드가 선언하지 않은 키 (노드가 시작 시 죽는다):")
            for k in undeclared:
                print(f"          - {k}")
            problems += 1
        if defaulted:
            print(f"   [참고] yaml 에 없어 기본값 사용: {', '.join(defaulted)}")

    print()
    if problems:
        print(f"{problems}개 노드에서 문제 발견 — 젯슨에 올리기 전에 고칠 것")
        return 1
    print("모든 파라미터 이름이 일치한다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
