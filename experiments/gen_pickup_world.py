"""撿球道驗證專用世界 pickup_test.world：以 indoor_court.world 為底，只改物理精度跟球的參數。

為什麼：新車體 (3D 列印) 的牆只有 2~3 mm 厚。預設物理 (步長 1 ms、球接觸 kp 1e5、min_depth 1 mm)
下，球以 ~3 m/s 撞牆會陷進 ~1.6 mm (> 牆厚一半)，ODE 就把球從牆的另一邊推出去 (穿牆)。
2026-10-26 實測：球最後停在電子艙「箱子裡面」，物理上不可能。
這裡：步長 0.5 ms、ODE 100 次迭代、球 kp 1e6 (3 m/s 只陷 ~0.5 mm)、min_depth 0。
contact_max_correcting_vel 1000 (預設 100)：2026-10-26 實測預設值會把滾輪關節卡在 100 rad/s (約 955 rpm)，
指令一超過 100，左右滾輪都變成 +100 同方向轉而且回不來；真實滾輪約 1218 rpm = 128 rad/s。改 1000 後 127.6 rad/s 正常反向轉。

用法: python3 experiments/gen_pickup_world.py [--mass 0.0577]   (預設 ITF 球 57.7 g；--mass 0.027 = 舊模擬)
"""
import argparse
import os
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'indoor_court.world'
DST = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'pickup_test.world'

PHYSICS = '''    <physics type="ode">
      <max_step_size>0.0005</max_step_size>
      <real_time_update_rate>2000</real_time_update_rate>
      <ode><solver><type>quick</type><iters>100</iters></solver>
        <constraints><contact_max_correcting_vel>1000</contact_max_correcting_vel><contact_surface_layer>0.0002</contact_surface_layer></constraints></ode>
    </physics>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mass', type=float, default=0.0577)
    a = ap.parse_args()
    s = SRC.read_text()
    s = s.replace('<world name="indoor_court_world">', '<world name="pickup_test_world">\n' + PHYSICS, 1)
    gui_pose = os.environ.get('GUI_CAM_POSE')   # 錄影用：Gazebo 視窗的視角 "x y z roll pitch yaw"
    gui_track = os.environ.get('GUI_TRACK_XYZ')  # 錄影用：Gazebo 視窗跟著車子，相機在車體座標的位置 "x y z"
    if gui_track:
        s = s.replace('<world name="pickup_test_world">',
                      '<world name="pickup_test_world">\n    <gui fullscreen="0"><camera name="user_camera">'
                      '<track_visual><name>tennis_bot</name><static>true</static><use_model_frame>true</use_model_frame>'
                      f'<xyz>{gui_track}</xyz><inherit_yaw>true</inherit_yaw></track_visual></camera></gui>', 1)
    elif gui_pose:
        s = s.replace('<world name="pickup_test_world">',
                      '<world name="pickup_test_world">\n    <gui fullscreen="0"><camera name="user_camera">'
                      f'<pose>{gui_pose}</pose></camera></gui>', 1)
    s = s.replace('<kp>100000.0</kp><kd>1.0</kd><min_depth>0.001</min_depth>',
                  '<kp>1000000.0</kp><kd>1.0</kd><min_depth>0.0</min_depth>')
    # 球的質量與慣性 (實心薄殼近似 I = 2/3 m r^2，r = 0.033；跟 gen_ball_layout 同公式)
    inertia = 2.0 / 3.0 * a.mass * 0.033 ** 2
    s = re.sub(r'<mass>0\.0\d+</mass>(\s*)<inertia><ixx>[^<]+</ixx><iyy>[^<]+</iyy><izz>[^<]+</izz></inertia>',
               lambda m: (f'<mass>{a.mass}</mass>{m.group(1)}<inertia><ixx>{inertia:.4e}</ixx>'
                          f'<iyy>{inertia:.4e}</iyy><izz>{inertia:.4e}</izz></inertia>'), s)
    DST.write_text(s)
    n_balls = s.count('<model name="ball_')
    print(f'寫入 {DST}，球 {a.mass * 1000:.1f} g，球數 {n_balls}')


if __name__ == '__main__':
    main()
