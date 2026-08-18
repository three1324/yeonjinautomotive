#!/usr/bin/env python3
"""기록된 waypoint 를 평활화하고 **곡률**을 계산한다. (2단계 준비물)

record_waypoints 로 얻은 원본 궤적은 측위 노이즈로 들쭉날쭉하다. 그대로 쓰면
곡률이 요동쳐서 속도계획이 튄다. B-스플라인으로 매끄럽게 만든 뒤 곡률을 낸다.

f1tenth_ws 의 clean_waypoints.py 를 참고했으나 두 가지가 다르다:
    - 경로를 하드코딩하지 않고 인자로 받는다
    - **곡률과 권장속도까지 계산해 저장한다** (2단계에서 선행 감속에 쓰는 값)

ROS 불필요. PC 에서 오프라인으로 돌린다.

    python3 tools/clean_waypoints.py wp.csv -o wp_clean.csv --smooth 0.5

의존성: numpy, scipy  (pip install scipy)
"""

import argparse

import numpy as np


def curvature(x, y):
    """매개변수 곡선의 곡률 k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)."""
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    num = np.abs(dx * ddy - dy * ddx)
    den = np.power(dx * dx + dy * dy, 1.5)
    den[den < 1e-9] = 1e-9
    return num / den


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input', help='record_waypoints 가 만든 CSV')
    ap.add_argument('-o', '--output', default='', help='출력 CSV (기본: 입력_clean.csv)')
    ap.add_argument('--smooth', type=float, default=0.5,
                    help='B-스플라인 평활 계수. 키울수록 원본을 무시하고 매끄러워진다')
    ap.add_argument('--points', type=int, default=2000, help='출력 점 개수')
    ap.add_argument('--v-max', type=float, default=12.0,
                    help='직선 권장속도 (xycar_motor speed 단위)')
    ap.add_argument('--v-min', type=float, default=4.0, help='최대곡률 구간 권장속도')
    ap.add_argument('--k-max', type=float, default=2.0,
                    help='이 곡률(1/m) 이상이면 v_min. 트랙 최소 회전반경의 역수 근처')
    ap.add_argument('--plot', action='store_true', help='결과를 그려서 확인')
    args = ap.parse_args()

    from scipy.interpolate import splev, splprep

    raw = np.loadtxt(args.input, delimiter=',', comments='#')
    if raw.ndim != 2 or raw.shape[0] < 10:
        raise SystemExit(f'점이 너무 적다: {raw.shape}')
    x, y = raw[:, 0], raw[:, 1]
    print(f'입력 {len(x)} points')

    # 닫힌 경로(한 바퀴)면 per=True 로 시작/끝을 이어 붙인다
    closed = float(np.hypot(x[0] - x[-1], y[0] - y[-1])) < 1.0
    tck, _ = splprep([x, y], s=args.smooth, per=closed)
    u = np.linspace(0.0, 1.0, args.points)
    xs, ys = splev(u, tck)
    print(f'평활화 {len(xs)} points (closed={closed})')

    k = curvature(xs, ys)
    # 곡률 -> 권장속도: 급할수록 느리게. 선형 감쇠.
    t = np.clip(k / max(args.k_max, 1e-6), 0.0, 1.0)
    v = args.v_max - (args.v_max - args.v_min) * t

    yaw = np.arctan2(np.gradient(ys), np.gradient(xs))

    out = args.output or args.input.rsplit('.', 1)[0] + '_clean.csv'
    with open(out, 'w', encoding='utf-8') as f:
        f.write('# x_m,y_m,yaw_rad,curvature,speed\n')
        for i in range(len(xs)):
            f.write(f'{xs[i]:.4f},{ys[i]:.4f},{yaw[i]:.4f},{k[i]:.4f},{v[i]:.2f}\n')
    print(f'저장 -> {out}')
    print(f'곡률 중앙값 {np.median(k):.3f}  최대 {k.max():.3f} (1/m)')
    print(f'권장속도 범위 {v.min():.1f} ~ {v.max():.1f}')

    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 6))
        ax[0].plot(x, y, '.', ms=1, label='raw')
        ax[0].plot(xs, ys, '-', lw=1.5, label='smoothed')
        ax[0].set_aspect('equal')
        ax[0].legend()
        ax[0].set_title('path')
        sc = ax[1].scatter(xs, ys, c=v, s=6, cmap='viridis')
        ax[1].set_aspect('equal')
        ax[1].set_title('recommended speed')
        fig.colorbar(sc, ax=ax[1])
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
