from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # 원래는 usb_cam 의 config/params.yaml 을 읽었으나 그 파일은 usb_cam 에
    # 존재하지 않는다(params_1.yaml / params_2.yaml 뿐). 그래서 노드가 파라미터를
    # 하나도 못 받고 usb_cam 내장 기본값으로 떴고, 그 기본값 brightness=50 이
    # 문제였다 — 이 센서의 brightness 범위는 min=-64 max=64 default=0 이라
    # 50 은 거의 최대치다. 화면의 20~24% 가 포화되어 신호등 램프가 하얗게
    # 날아갔고 /light 가 NONE 으로만 나왔다 (2026-08-19 실차).
    # brightness=0 으로 되돌리니 포화 23.6% -> 2.4%, 신호등·차선 모두 정상 검출.
    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='xycar_cam',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'io_method': 'mmap',
            'brightness': 0,      # [실측] 센서 중립값. 50 이면 과노출로 신호등 소실
            'autoexposure': True,
            # image_transport 는 설치된 publisher 플러그인을 전부 붙인다.
            # compressedDepth 는 32FC1/16UC1 깊이 영상만 처리하는데 여기로는
            # yuv422_yuy2 컬러가 들어오니 프레임마다(30Hz) 에러를 뱉는다.
            # theora 는 쓰지도 않으면서 CPU 만 먹는다. 둘 다 끈다.
            # /image_raw 와 /image_raw/compressed 는 그대로 나온다.
            'image_raw.disable_pub_plugins': [
                'image_transport/compressedDepth',
                'image_transport/theora',
            ],
        }],
    ))

    return ld
