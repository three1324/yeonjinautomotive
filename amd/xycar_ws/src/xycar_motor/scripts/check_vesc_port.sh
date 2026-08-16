#!/usr/bin/env bash
# VESC 시리얼 포트 연결 점검 (젯슨에서 실행).
#
#   ./check_vesc_port.sh              # 자동 탐색
#   ./check_vesc_port.sh /dev/ttyTHS1 # 특정 포트 지정
#
# 아무것도 바꾸지 않는다. 점검만 하고 문제가 있으면 해결 명령을 알려준다.
# USB 에서 UART 로 교체한 뒤 "모터가 반응하지 않는다" 를 진단할 때 쓴다.

set -uo pipefail

PORT="${1:-}"
ok=0
warn=0

say()  { printf '%s\n' "$*"; }
good() { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; ok=1; }
note() { printf '  [주의] %s\n' "$*"; warn=1; }

say "=============================================="
say " VESC 시리얼 포트 점검"
say "=============================================="

# ── 1. 포트 탐색 ──────────────────────────────────────────────
say ""
say "[1] 시리얼 장치 탐색"
found=()
for d in /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB* /dev/ttyMOTOR; do
  [ -e "$d" ] && found+=("$d")
done
if [ ${#found[@]} -eq 0 ]; then
  bad "시리얼 장치를 찾을 수 없다. 케이블/전원 확인."
else
  for d in "${found[@]}"; do
    printf '         %s  (%s)\n' "$d" "$(stat -c '%A %U:%G' "$d" 2>/dev/null)"
  done
fi

if [ -z "$PORT" ]; then
  # UART 우선 (이번에 UART 로 교체했으므로)
  for d in /dev/ttyMOTOR /dev/ttyTHS1 /dev/ttyTHS0 /dev/ttyUSB0 /dev/ttyACM0; do
    [ -e "$d" ] && { PORT="$d"; break; }
  done
fi
say ""
say "  검사 대상: ${PORT:-(없음)}"
[ -z "$PORT" ] && { say ""; say "포트를 찾지 못해 중단한다."; exit 1; }

# ── 2. nvgetty (젯슨 UART 최대 함정) ──────────────────────────
say ""
say "[2] nvgetty (시리얼 콘솔) — UART 포트를 점유하는 주범"
if [[ "$PORT" == *ttyTHS* ]]; then
  if systemctl is-active --quiet nvgetty 2>/dev/null; then
    bad "nvgetty 가 실행 중이다. 이 상태로는 $PORT 를 열 수 없다."
    say "         해결:"
    say "           sudo systemctl stop nvgetty"
    say "           sudo systemctl disable nvgetty"
    say "           sudo reboot"
  else
    good "nvgetty 비활성 (정상)"
  fi
else
  good "UART 포트가 아니므로 해당 없음"
fi

# ── 3. 권한 ───────────────────────────────────────────────────
say ""
say "[3] 권한"
grp=$(stat -c '%G' "$PORT" 2>/dev/null)
if [ -r "$PORT" ] && [ -w "$PORT" ]; then
  good "읽기/쓰기 가능"
else
  bad "읽기 또는 쓰기 권한 없음 (그룹: $grp)"
  say "         해결:  sudo usermod -aG $grp \$USER   # 그 뒤 재로그인"
fi

# ── 4. 포트 점유 프로세스 ─────────────────────────────────────
say ""
say "[4] 다른 프로세스가 쓰고 있는지"
if command -v fuser >/dev/null 2>&1; then
  pids=$(fuser "$PORT" 2>/dev/null)
  if [ -n "${pids// /}" ]; then
    bad "다른 프로세스가 점유 중: $pids"
    say "         확인:  ps -p $pids -o pid,cmd"
  else
    good "점유 프로세스 없음"
  fi
else
  note "fuser 없음 — 점유 여부를 확인하지 못했다 (sudo apt install psmisc)"
fi

# ── 5. 실제로 열어보기 ────────────────────────────────────────
say ""
say "[5] 115200 8N1 로 실제 열기 시도"
if command -v python3 >/dev/null 2>&1; then
  python3 - "$PORT" <<'PY'
import sys
port = sys.argv[1]
try:
    import serial
except ImportError:
    print("  [주의] pyserial 이 없어 열기 시험을 건너뛴다 (pip install pyserial)")
    sys.exit(0)
try:
    s = serial.Serial(port, 115200, timeout=1, rtscts=False)
    s.close()
    print("  [OK]   포트를 열고 닫는 데 성공")
except Exception as e:
    print(f"  [FAIL] 열기 실패: {e}")
    sys.exit(1)
PY
  [ $? -ne 0 ] && ok=1
else
  note "python3 없음 — 열기 시험 생략"
fi

# ── 요약 ──────────────────────────────────────────────────────
say ""
say "=============================================="
if [ $ok -ne 0 ]; then
  say " 문제 발견 — 위 [FAIL] 항목을 먼저 해결할 것"
else
  say " 포트 점검 통과: $PORT"
  say ""
  say " 다음 단계:"
  say "   1) xycar_motor/config/vesc.yaml 의 port 를 '$PORT' 로 맞춘다"
  say "   2) colcon build --packages-select vesc_driver xycar_motor"
  say "   3) ros2 launch xycar_motor xycar_motor.launch.py"
  say "   4) 다른 터미널:  ros2 topic echo /sensors/core"
  say "      전압/온도가 올라오면 VESC 통신 성공."
fi
say "=============================================="
say ""
say " 참고: 흐름제어(RTS/CTS)는 vesc_interface.cpp 에서 NONE 으로 고쳐두었다."
say "       UART 에는 RTS/CTS 핀이 없어 HARDWARE 로 두면 송신이 막힌다."
exit $ok
