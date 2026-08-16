#!/usr/bin/env bash
# amd/ 정리 스크립트 — xycar 대회 프로젝트에 불필요한 항목 제거
# 원본 백업: ~/Downloads/amd.zip (건드리지 않음, 복구용으로 유지)
set -euo pipefail
cd "$(dirname "$0")"   # amd/ 디렉토리 기준으로 실행

echo "=== 정리 전 전체 용량 ==="
du -sh . 2>/dev/null

# ---------------------------------------------------------------
# 1) 완전히 무관한 홈 디렉토리 잡동사니
# ---------------------------------------------------------------
echo "[1/7] snap/, Pictures/, Videos/, Templates/, Music/, Public/, Documents/ 삭제"
rm -rf snap Pictures Videos Templates Music Public Documents

# ---------------------------------------------------------------
# 2) Downloads — VESC 설정 PDF만 살리고 나머지(설치파일 등) 삭제
# ---------------------------------------------------------------
echo "[2/7] Downloads/ 정리 (VESC PDF만 보존)"
mkdir -p ../reference/docs
[ -f "Downloads/모터제어기_VESC_설정방법.pdf" ] && \
    mv "Downloads/모터제어기_VESC_설정방법.pdf" ../reference/docs/
rm -rf Downloads

# ---------------------------------------------------------------
# 3) ROS2 네이티브 포팅 결정으로 더 이상 필요 없는 ros1_bridge
# ---------------------------------------------------------------
echo "[3/7] ros-humble-ros1-bridge/ 삭제 (ROS2 네이티브 모터로 결정됨)"
rm -rf ros-humble-ros1-bridge

# ---------------------------------------------------------------
# 4) colcon/catkin 빌드 산출물 — 다시 빌드하면 생기는 것들
# ---------------------------------------------------------------
echo "[4/7] build/install/log 산출물 삭제 (재빌드로 복구됨)"
rm -rf build install log
rm -rf xycar_ws/build xycar_ws/install xycar_ws/log
rm -rf xycar_ws/src/build xycar_ws/src/install xycar_ws/src/log
rm -rf noetic_ws/build noetic_ws/devel

# ---------------------------------------------------------------
# 5) xycar_ws/etc — 데스크톱 아이콘/설정은 삭제, VESC 펌웨어·설정만 보존
#    (vesc_tool_6.05 GUI 앱 160MB는 공식 사이트에서 재다운로드 가능하므로 제거)
# ---------------------------------------------------------------
echo "[5/7] xycar_ws/etc/ 정리 (VESC 펌웨어/설정만 reference로 이동)"
mkdir -p ../reference/vesc
if [ -d "xycar_ws/etc/motor_vesc" ]; then
    mv xycar_ws/etc/motor_vesc/2026_0617_vesc_Motor_cfg.xml ../reference/vesc/ 2>/dev/null || true
    mv xycar_ws/etc/motor_vesc/VESC_2.18.bin ../reference/vesc/ 2>/dev/null || true
    mv xycar_ws/etc/motor_vesc/VESC_5.03.bin ../reference/vesc/ 2>/dev/null || true
fi
rm -rf xycar_ws/etc

# ---------------------------------------------------------------
# 6) study_backup.zip(중복 백업, 24MB), __pycache__ 전체 삭제
# ---------------------------------------------------------------
echo "[6/7] 중복 백업 zip 및 __pycache__ 삭제"
rm -f xycar_ws/src/study_backup.zip
find . -type d -name "__pycache__" -prune -exec rm -rf {} +

# ---------------------------------------------------------------
# 7) 비밀번호로 추정되는 파일 — 삭제하지 않고 프로젝트 밖으로 격리
# ---------------------------------------------------------------
echo "[7/7] 민감 파일 격리 (와이파이 비밀번호로 추정)"
if [ -f "sword 비밀번호 ifname wlx90de804117f9" ]; then
    mkdir -p ../../_sensitive_quarantine
    mv "sword 비밀번호 ifname wlx90de804117f9" ../../_sensitive_quarantine/
    echo "  -> E:\\자율주행\\_sensitive_quarantine\\ 로 이동함. 필요없으면 직접 삭제하세요."
fi

echo
echo "=== 정리 후 전체 용량 ==="
du -sh . 2>/dev/null
echo
echo "완료. 남은 항목: xycar_ws(소스), noetic_ws(ROS1 모터 참고용), Desktop(시뮬레이터/센서드라이버/SLAM)"
echo "Desktop 안의 개인 폴더(2026_0618, Zoo, basic, ji, kyungmin, study, yeonjin, 새, x*.desktop 등)는"
echo "자동 삭제하지 않았습니다 — 필요 여부를 직접 확인해주세요."
