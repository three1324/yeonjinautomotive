#!/usr/bin/env python3
"""차량 연결 전/후 사전 점검. **실차에 전원 넣기 전에 이것부터 돌린다.**

    python3 tools/preflight.py            # 정적 점검 (ROS 실행 불필요)
    python3 tools/preflight.py --live     # + 실제 토픽이 오는지 실측 (센서 launch 를 먼저 띄울 것)

왜 필요한가:
    실차 테스트에서 시간을 잡아먹는 실패는 알고리즘이 아니라 **연결·경로·권한**이었다.
    udev 규칙이 없어 /dev/ttyLIDAR 가 안 생기거나, 카메라 해상도가 driver_node 의
    image_width 와 달라 조향이 한쪽으로 치우치거나, 워크스페이스를 다시 빌드하지 않아
    옛 코드가 도는 식이다. 전부 차를 움직이기 전에 1분이면 잡을 수 있는 것들이라
    여기 모아둔다.

    이 파일은 **판정만 한다.** 아무것도 고치지 않고 아무 노드도 띄우지 않는다.
    고치는 방법은 각 실패 항목의 [조치] 줄에 그대로 적혀 있다.

종료 코드: 실패(FAIL)가 하나라도 있으면 1, 아니면 0.
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
STUDY = os.path.normpath(os.path.join(HERE, "..", ".."))
WS = os.path.normpath(os.path.join(STUDY, "..", ".."))   # <ws>/src/study -> <ws>

PARAMS = os.path.join(STUDY, "my_bringup", "config", "drive_params.yaml")

# udev 규칙이 식별하는 USB 칩. README '센서 udev 규칙' 절과 같은 값이어야 한다.
USB_IDS = {
    "0483:5740": ("VESC (ChibiOS) — 모터 컨트롤러",
                  "젯슨 차량이면 연결 필요. AMD 차량은 모터가 별도 ROS1 도커라 없어도 정상이다"),
    "10c4:ea60": ("CP2102 — YDLidar",
                  "라이다를 연결할 것. 두 차량 모두 필요하다 (없으면 /scan 이 안 나온다)"),
}

# 실차에서 반드시 빌드돼 있어야 하는 패키지 (install/ 아래)
REQUIRED_PKGS = [
    "my_perception", "my_obstacle", "my_driver", "my_bringup", "my_debug",
    "xycar_cam", "xycar_lidar",
]
# 젯슨 전용 (AMD 차량은 모터가 별도 ROS1 도커라 없어도 된다)
JETSON_PKGS = ["xycar_motor", "vesc_driver"]

_results = []


def _mark(level, name, detail="", fix=""):
    _results.append((level, name, detail, fix))
    icon = {"PASS": "  OK ", "WARN": " WARN", "FAIL": " FAIL"}[level]
    print(f"[{icon}] {name}" + (f"  — {detail}" if detail else ""))
    if fix and level != "PASS":
        for line in fix.strip().splitlines():
            print(f"         [조치] {line.strip()}")


def _section(title):
    print(f"\n=== {title} " + "=" * max(0, 62 - len(title)))


# --------------------------------------------------------------------- 1. 환경

def check_env():
    _section("1. 실행 환경")

    distro = os.environ.get("ROS_DISTRO", "")
    if distro:
        _mark("PASS", "ROS 2 환경", f"ROS_DISTRO={distro}")
    else:
        _mark("FAIL", "ROS 2 환경", "ROS_DISTRO 가 비어 있다",
              "source /opt/ros/humble/setup.bash")

    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if domain:
        _mark("WARN", "ROS_DOMAIN_ID", f"={domain}",
              "차량과 노트북이 서로 안 보이면 양쪽 값이 같은지 확인할 것")
    else:
        _mark("PASS", "ROS_DOMAIN_ID", "미설정(기본 0)")

    install = os.path.join(WS, "install")
    if not os.path.isdir(install):
        _mark("FAIL", "워크스페이스 빌드", f"{install} 없음",
              f"cd {WS} && colcon build --symlink-install")
        return

    built = set(os.listdir(install))
    missing = [p for p in REQUIRED_PKGS if p not in built]
    if missing:
        _mark("FAIL", "필수 패키지 빌드", "빠짐: " + ", ".join(missing),
              f"cd {WS} && colcon build --symlink-install")
    else:
        _mark("PASS", "필수 패키지 빌드", f"{len(REQUIRED_PKGS)}개 모두 있음")

    missing_j = [p for p in JETSON_PKGS if p not in built]
    if missing_j:
        _mark("WARN", "모터 스택(젯슨용)", "빠짐: " + ", ".join(missing_j),
              "젯슨 차량이면 빌드할 것. AMD 차량은 모터가 별도 ROS1 도커이므로 정상이다")
    else:
        _mark("PASS", "모터 스택(젯슨용)", "빌드됨")

    # 소스가 install 보다 새로우면 옛 코드가 도는 것이다 (--symlink-install 이라도
    # entry_point/launch/config 추가는 재빌드가 필요하다).
    stale = []
    for pkg in REQUIRED_PKGS:
        src = os.path.join(STUDY, pkg)
        dst = os.path.join(install, pkg)
        if not os.path.isdir(src) or not os.path.isdir(dst):
            continue
        newest = max((os.path.getmtime(os.path.join(r, f))
                      for r, _, fs in os.walk(src) for f in fs
                      if not f.endswith(".pyc")), default=0)
        if newest > os.path.getmtime(dst):
            stale.append(pkg)
    if stale:
        _mark("WARN", "빌드 최신성", "소스가 더 새로움: " + ", ".join(stale),
              f"cd {WS} && colcon build --symlink-install --packages-select "
              + " ".join(stale))
    else:
        _mark("PASS", "빌드 최신성", "install 이 소스보다 최신")


# ------------------------------------------------------------------- 2. GPU/모델

def check_gpu_and_model():
    _section("2. GPU 가속 / YOLO 모델")

    try:
        import torch
        if torch.cuda.is_available():
            _mark("PASS", "CUDA", f"{torch.cuda.get_device_name(0)} (torch {torch.__version__})")
        else:
            _mark("WARN", "CUDA", f"사용 불가 (torch {torch.__version__}) — CPU 추론",
                  "젯슨이면 Jetson 용 torch 휠인지 확인. AMD 차량이면 정상이지만,\n"
                  "CPU 추론은 훨씬 느리므로 driver_node 의 stale_timeout_sec 을 넉넉히 둘 것")
    except ImportError:
        _mark("WARN", "CUDA", "torch 를 import 할 수 없음",
              "perception_node 는 ultralytics/torch 가 필요하다. pip 설치 확인")

    try:
        import ultralytics  # noqa: F401
        _mark("PASS", "ultralytics", "import 가능")
    except ImportError:
        _mark("FAIL", "ultralytics", "import 실패",
              "perception_node 가 시작하자마자 죽는다. pip install ultralytics")

    models = os.path.join(STUDY, "my_perception", "models")
    pt = os.path.join(models, "best5.pt")
    if os.path.exists(pt):
        _mark("PASS", "YOLO 가중치", f"{pt} ({os.path.getsize(pt) // 1024}KB)")
    else:
        _mark("FAIL", "YOLO 가중치", f"{pt} 없음",
              "best5.pt 를 my_perception/models/ 에 넣고 재빌드할 것")

    eng = os.path.join(models, "best5.engine")
    if os.path.exists(eng):
        _mark("WARN", "TensorRT 엔진", "존재한다. 기본값은 .pt 라 지금은 안 쓰인다",
              "[실측 2026-08-19] 실영상 전체 파이프라인 .pt 63.2ms vs .engine 44.8ms\n"
              "(1.4배). 순수 추론만은 38.5ms -> 12.9ms (3배). 검출 결과도 동일하다.\n"
              "⚠️ 반드시 task='segment' 로 로드할 것 — 안 주면 detect 로 오인식해\n"
              "   가속이 전혀 안 걸린다(38.6ms, .pt 와 동일). 이 함정 때문에 처음에\n"
              "   '이득 없음'으로 잘못 측정했다.\n"
              "쓰려면: model_path:=.../best5.engine (그 보드에서 export 한 것이어야 함)")


# ------------------------------------------------------------------ 3. 하드웨어

def _dev_list(pattern):
    return sorted(glob.glob(pattern))


def _lsusb_ids():
    if not shutil.which("lsusb"):
        return None
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    return set(re.findall(r"ID ([0-9a-f]{4}:[0-9a-f]{4})", out))


def check_hardware():
    _section("3. 하드웨어 연결")

    # --- 카메라 ---
    videos = _dev_list("/dev/video*")
    if videos:
        _mark("PASS", "카메라 장치", ", ".join(videos))
    else:
        _mark("FAIL", "카메라 장치", "/dev/video* 없음",
              "USB 카메라를 연결할 것. 연결했는데도 없으면 lsusb 로 인식 여부부터 확인")

    # --- USB 칩 식별 ---
    ids = _lsusb_ids()
    if ids is None:
        _mark("WARN", "lsusb", "실행할 수 없음", "sudo apt install usbutils")
    else:
        for usb_id, (what, fix) in USB_IDS.items():
            if usb_id in ids:
                _mark("PASS", f"USB {usb_id}", what)
            else:
                _mark("WARN", f"USB {usb_id}", f"{what} — 미연결", fix)

    # --- 시리얼 장치 ---
    serials = _dev_list("/dev/ttyUSB*") + _dev_list("/dev/ttyACM*") + _dev_list("/dev/ttyTHS*")
    if serials:
        _mark("PASS", "시리얼 장치", ", ".join(serials))
    else:
        _mark("FAIL", "시리얼 장치", "ttyUSB/ttyACM/ttyTHS 없음",
              "라이다·VESC 를 연결할 것")

    # --- udev 심볼릭 링크 ---
    for link, rule, what in (
            ("/dev/ttyLIDAR", "99-ydlidar.rules", "라이다"),
            ("/dev/ttyMOTOR", "99-vesc.rules", "VESC")):
        if os.path.exists(link):
            target = os.path.realpath(link)
            ok = os.access(link, os.R_OK | os.W_OK)
            _mark("PASS" if ok else "FAIL", f"udev {link}", f"-> {target}"
                  + ("" if ok else " (읽기/쓰기 권한 없음)"),
                  "" if ok else "sudo usermod -aG dialout $USER 후 재로그인, "
                                "또는 udev 규칙에 MODE:=\"0666\" 추가")
        else:
            has_rule = os.path.exists(f"/etc/udev/rules.d/{rule}")
            _mark("WARN", f"udev {link}", f"없음 ({what})"
                  + ("" if has_rule else f" · 규칙 파일 {rule} 도 없음"),
                  "README '센서 udev 규칙' 절의 명령을 이 보드에서 실행할 것:\n"
                  "  sudo tee /etc/udev/rules.d/" + rule + " ... "
                  "&& sudo udevadm control --reload-rules && sudo udevadm trigger\n"
                  "장치를 직접 경로(/dev/ttyUSB0 등)로 지정해 쓸 거면 무시해도 된다")

    # --- ydlidar.yaml 의 port 가 실제로 존재하는가 ---
    # [실측 2026-08-18] 실차에서 라이다가 안 나온 원인이 이것이었다.
    # ~/.ros/log/xycar_lidar_node_*.log 에 "Unknown error" 만 한 줄 찍히는데,
    # 그건 SDK 의 connect() 가 실패했다는 뜻(=포트를 못 열었다)이다. 그러면
    # xycar_lidar_node 는 스캔 루프에 **아예 들어가지 않고** 프로세스만 살아 있어서
    # "노드는 떠 있는데 /scan 이 없다"는 조용한 실패가 된다.
    ydlidar = os.path.join(WS, "src", "xycar_device", "xycar_lidar",
                           "params", "ydlidar.yaml")
    if os.path.exists(ydlidar):
        m = re.search(r"^\s*port:\s*(\S+)", open(ydlidar, encoding="utf-8").read(), re.M)
        if m:
            port = m.group(1)
            if os.path.exists(port):
                ok = os.access(port, os.R_OK | os.W_OK)
                _mark("PASS" if ok else "FAIL", "ydlidar.yaml port",
                      f"{port} (존재)" + ("" if ok else " — 읽기/쓰기 권한 없음"),
                      "" if ok else "sudo usermod -aG dialout $USER 후 재로그인")
            else:
                _mark("FAIL", "ydlidar.yaml port", f"{port} 가 존재하지 않는다",
                      "라이다가 /scan 을 한 번도 발행하지 않는다(노드는 조용히 살아 있다).\n"
                      "  1) 라이다 USB 연결 확인:  lsusb | grep 10c4:ea60\n"
                      "  2) 실제 경로 확인:        ls -l /dev/ttyUSB*\n"
                      f"  3) udev 규칙을 걸거나 {ydlidar} 의 port 를 실제 경로로 바꿀 것\n"
                      "  확인:  ros2 launch xycar_lidar xycar_lidar.launch.py  로그에\n"
                      "        'Unknown error' 가 뜨면 아직 포트를 못 연 것이다")

    # --- vesc.yaml 의 port 가 실제로 존재하는가 ---
    vesc = os.path.join(WS, "src", "xycar_motor", "config", "vesc.yaml")
    if os.path.exists(vesc):
        m = re.search(r"^\s*port:\s*(\S+)", open(vesc, encoding="utf-8").read(), re.M)
        if m:
            port = m.group(1)
            if os.path.exists(port):
                _mark("PASS", "vesc.yaml port", f"{port} (존재)")
            else:
                _mark("FAIL", "vesc.yaml port", f"{port} 가 존재하지 않는다",
                      f"vesc_driver 가 이 경로를 못 열면 모터가 안 돈다.\n"
                      f"실제 경로를 ls -l /dev/tty* 로 확인해 {vesc} 를 고칠 것")


# ----------------------------------------------------------------- 4. 파라미터

def check_params():
    _section("4. 파라미터 정합성")
    tool = os.path.join(HERE, "check_params.py")
    if not os.path.exists(tool):
        _mark("WARN", "check_params.py", "없음")
        return
    r = subprocess.run([sys.executable, tool], capture_output=True, text=True)
    if r.returncode == 0:
        _mark("PASS", "drive_params.yaml <-> 노드 선언", "모든 이름 일치")
    else:
        _mark("FAIL", "drive_params.yaml <-> 노드 선언", "불일치",
              f"python3 {tool} 를 직접 돌려 상세 내용을 볼 것 "
              "(선언 안 된 키가 있으면 노드가 시작 시 죽는다)")
        print(r.stdout)


# ------------------------------------------------------------------- 5. 실측

def check_live(seconds):
    _section(f"5. 토픽 실측 ({seconds}s)")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import Float32MultiArray
    except ImportError as exc:
        _mark("FAIL", "rclpy", f"import 실패: {exc}",
              "source /opt/ros/humble/setup.bash && source install/setup.bash")
        return

    import yaml
    cfg = yaml.safe_load(open(PARAMS, encoding="utf-8"))
    want_w = cfg.get("driver_node", {}).get("ros__parameters", {}).get("image_width")

    sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    counts = {"image": 0, "scan": 0, "motor": 0}
    size = {}

    rclpy.init()
    node = Node("preflight_probe")

    def on_img(msg):
        counts["image"] += 1
        size["wh"] = (msg.width, msg.height)

    node.create_subscription(Image, "/image_raw", on_img, sensor_qos)
    node.create_subscription(LaserScan, "/scan",
                             lambda m: counts.__setitem__("scan", counts["scan"] + 1),
                             sensor_qos)
    node.create_subscription(Float32MultiArray, "/xycar_motor",
                             lambda m: counts.__setitem__("motor", counts["motor"] + 1), 10)

    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    for key, topic, need in (("image", "/image_raw", True),
                             ("scan", "/scan", True),
                             ("motor", "/xycar_motor", False)):
        hz = counts[key] / seconds
        if counts[key] == 0:
            fix = ("해당 드라이버 launch 를 먼저 띄웠는지 확인 "
                   "(ros2 launch my_bringup drive.launch.py)")
            if topic == "/scan":
                fix += (
                    "\n라이다 노드가 떠 있는데도 안 오면 ~/.ros/log/xycar_lidar_node_*.log "
                    "를 볼 것.\n'Unknown error' = 시리얼 포트를 못 열었다는 뜻이다 "
                    "(위 ydlidar.yaml port 항목 참고).\n"
                    "※ ros2 topic echo /scan 으로 확인할 때는 반드시 "
                    "--qos-reliability best_effort 를 붙일 것 —\n"
                    "   라이다는 BEST_EFFORT 로 발행하므로 기본(RELIABLE)으로 구독하면 "
                    "실제로는 발행 중인데도 아무것도 안 보인다.")
            _mark("FAIL" if need else "WARN", f"{topic}", "메시지 없음", fix)
        else:
            _mark("PASS", f"{topic}", f"{hz:.1f} Hz ({counts[key]}개)")

    if "wh" in size:
        w, h = size["wh"]
        if want_w is None or w == want_w:
            _mark("PASS", "카메라 해상도", f"{w}x{h} (driver_node.image_width={want_w})")
        else:
            _mark("FAIL", "카메라 해상도",
                  f"실제 {w}x{h} != driver_node.image_width={want_w}",
                  "화면 중심이 어긋나 조향이 계통적으로 한쪽으로 치우친다.\n"
                  f"drive_params.yaml 의 driver_node.image_width 를 {w} 로 맞출 것")


# --------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="실차 테스트 전 사전 점검")
    ap.add_argument("--live", action="store_true",
                    help="실제 토픽이 오는지 실측한다 (센서 launch 를 먼저 띄울 것)")
    ap.add_argument("--seconds", type=float, default=5.0, help="--live 측정 시간")
    args = ap.parse_args()

    print("차량 사전 점검 — preflight.py")
    print(f"워크스페이스: {WS}")

    check_env()
    check_gpu_and_model()
    check_hardware()
    check_params()
    if args.live:
        check_live(args.seconds)
    else:
        _section("5. 토픽 실측")
        print("  (건너뜀) 센서를 연결하고 launch 를 띄운 뒤 --live 로 다시 돌릴 것")

    fails = [r for r in _results if r[0] == "FAIL"]
    warns = [r for r in _results if r[0] == "WARN"]
    print("\n" + "=" * 70)
    print(f"결과: 실패 {len(fails)} · 경고 {len(warns)} · "
          f"통과 {len(_results) - len(fails) - len(warns)}")
    if fails:
        print("\n먼저 해결할 것:")
        for _, name, detail, _fix in fails:
            print(f"  - {name}: {detail}")
        print("\n⚠️ 실패 항목이 남은 채로 /drive_enable 을 켜지 말 것.")
    elif warns:
        print("\n경고만 남음 — 위 [조치] 를 읽고 이 차량에 해당하는지 판단할 것.")
    else:
        print("\n전부 통과. 차를 들어올린 상태에서 먼저 확인할 것:")
        print("  ros2 launch my_bringup drive.launch.py rviz:=true")
        print("  ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
