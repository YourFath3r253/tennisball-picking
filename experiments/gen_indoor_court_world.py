"""產生室內網球場世界 indoor_court.world (2026-10-12，對應實體車現況)：

以 tennis_court.world 為底 (地面摩擦 0.63、砂礫、網子、球的物理都沿用)，改外觀：
  - 地面顏色照實拍影片 (D 同學 9/19 在室內網球場錄的 inside_only.avi / outside_only.avi，640x480)：
      場內 BGR 中位數 (124, 126, 124)、場外 (129, 133, 130)、白線 (240, 243, 243)
  - ITF 白色場地線 (只有外觀)
  - 半場黃色毛線 (half_court.YARN_SEGMENTS)：實拍毛線 RGB 約 (159, 158, 117) (yellow_sample.jpg，原始畫面)
  - 拿掉半透明綠色牆 (實體球場沒有，相機會看到一片綠)
顏色是「Gazebo 算圖之後相機看到的像素」要對上實拍，不是材質參數本身；材質參數用 ALBEDO_GAIN 換算，
校正方法見 calibrate 段落 (experiments/實驗數據/indoor_court_calibration.md)。

另外產生 classroom_floor.world：同樣的黃毛線，但地面換成 10/02、10/11 實測用的教室木地板顏色、沒有白線。
實體車的測試數據全部是在教室拍的，模擬要先在「同一種地板」上跟實測對照，才能拿室內球場版做預測
(兩種地板對 D 的黃線偵測影響很大：木地板 Lab b 0.024 本身就超過 D 的黃色門檻 0.02，灰色球場只有 0.008)。

用法: python3 experiments/gen_indoor_court_world.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src' / 'tennis_bot' / 'tennis_bot'))
from half_court import YARN_SEGMENTS, YARN_WIDTH_M, YARN_HEIGHT_M  # noqa: E402

SRC = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'tennis_court.world'
DST = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'indoor_court.world'
DST_CLASSROOM = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'classroom_floor.world'

# 材質顏色 (0~1，ambient = diffuse)。校正 (2026-10-12)：這組燈光下「相機像素 ≈ 材質值 x 280.6」，
# 用實拍像素 / 280.6 得到材質值；驗證：場內 (124,126,124)、白線 (240,243,243)、毛線 (117,158,159) BGR 都跟實拍一致。
COURT_RGB = (0.442, 0.449, 0.442)
OUTSIDE_RGB = (0.463, 0.474, 0.460)
LINE_RGB = (0.866, 0.866, 0.855)
# 毛線：實拍 (yellow_sample.jpg) 沿線每一行取最亮的像素，中位數 BGR (220, 238, 237) = 灰階 236 (地面 133)。
# 毛線是蓬鬆的淺黃色，核心很亮；之前取「最黃」的像素 (117,158,159) 只有灰階 153，D 程式的 Canny 抓不到邊。
YARN_RGB = (0.845, 0.848, 0.784)
# 光暈：實拍毛線蓬鬆 + 鏡頭模糊，D 程式在實拍上取到的「線條帶」(320x240 畫面 ±5 px) 中位數是淡黃色
# (yellow_sample.jpg 特徵 r/g/b 0.356/0.341/0.301、V 0.69 -> RGB 約 (176, 168, 149))，14 mm 的單色細線做不到
# (帶子裡大半是地板)。所以毛線 = 中間亮核心 + 兩側淡黃光暈，光暈寬度用「實車多遠開始看到黃線」校正
# (10/11 實測：第一次看到中位數 0.81 m，最遠 1.34 m)。是等效外觀，不是毛線真的這麼寬。
YARN_HALO_RGB = (0.627, 0.600, 0.531)
YARN_HALO_WIDTH_M = float(__import__('os').environ.get('YARN_HALO_WIDTH_M', '0.14'))
# 教室木地板：yellow_sample.jpg (原始畫面) 地板 BGR 中位數 (129.5, 134, 132)
WOOD_RGB = (0.470, 0.478, 0.462)
# 室內均勻照明：場景環境光 + 正上方一盞平行光 (取代 model://sun 的斜射陽光)
SCENE_AMBIENT = (0.6, 0.6, 0.6)
LIGHT_DIFFUSE = (0.5, 0.5, 0.5)

# ITF 場地 (m)：全長 23.77、雙打寬 10.97、單打寬 8.23、發球線離網 6.40；線寬 5 cm (底線可到 10 cm)
HALF_L = 11.885
DOUBLES_Y = 5.485
SINGLES_Y = 4.115
SERVICE_X = 6.40
LINE_W = 0.05
BASELINE_W = 0.10


def _rgb(c):
    return f'{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1'


def _box_visual(name, cx, cy, cz, sx, sy, sz, rgb):
    return (f'        <visual name="{name}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>'
            f'<cast_shadows>false</cast_shadows>'
            f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
            f'<material><ambient>{_rgb(rgb)}</ambient><diffuse>{_rgb(rgb)}</diffuse>'
            f'<specular>0 0 0 1</specular></material></visual>')


def court_block(classroom=False):
    out = ['    <!-- 室內網球場外觀 (gen_indoor_court_world.py)：場地面 + ITF 白線 + 半場黃毛線，都只有外觀沒有碰撞 -->',
           '    <model name="indoor_court">', '      <static>true</static>', '      <link name="court_link">']
    if classroom:
        out += yarn_visuals() + ['      </link>', '    </model>']
        return '\n'.join(out) + '\n'
    # 場地面 (雙打範圍，往外 0.0 m；場外是地面本身的顏色)
    out.append(_box_visual('court_surface', 0, 0, 0.0002, 2 * HALF_L, 2 * DOUBLES_Y, 0.0004, COURT_RGB))
    z = 0.0006
    lines = []
    for sx in (-1, 1):
        lines.append((f'baseline_{sx}', sx * (HALF_L - BASELINE_W / 2), 0, BASELINE_W, 2 * DOUBLES_Y))
        lines.append((f'service_{sx}', sx * SERVICE_X, 0, LINE_W, 2 * SINGLES_Y))
    for sy in (-1, 1):
        lines.append((f'doubles_{sy}', 0, sy * (DOUBLES_Y - LINE_W / 2), 2 * HALF_L, LINE_W))
        lines.append((f'singles_{sy}', 0, sy * (SINGLES_Y - LINE_W / 2), 2 * HALF_L, LINE_W))
    lines.append(('center_service', 0, 0, 2 * SERVICE_X, LINE_W))
    for sx in (-1, 1):
        lines.append((f'center_mark_{sx}', sx * (HALF_L - 0.10 - 0.05), 0, 0.10, LINE_W))
    for name, cx, cy, lx, ly in lines:
        out.append(_box_visual(f'line_{name}'.replace('-', 'm'), cx, cy, z, lx, ly, 0.0004, LINE_RGB))
    out += yarn_visuals()
    out += ['      </link>', '    </model>']
    return '\n'.join(out) + '\n'


def yarn_visuals():
    out = []
    # 黃毛線 (稍微高出地面)
    for name, ((x0, y0), (x1, y1)) in YARN_SEGMENTS.items():
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if abs(x1 - x0) > abs(y1 - y0):
            lx, ly = abs(x1 - x0) + YARN_WIDTH_M, YARN_WIDTH_M
        else:
            lx, ly = YARN_WIDTH_M, abs(y1 - y0) + YARN_WIDTH_M
        out.append(_box_visual(f'yarn_{name}', cx, cy, YARN_HEIGHT_M / 2 + 0.0002, lx, ly, YARN_HEIGHT_M, YARN_RGB))
        # 光暈 (比核心低 0.2 mm，核心蓋在上面)
        if abs(x1 - x0) > abs(y1 - y0):
            hx, hy = abs(x1 - x0) + YARN_HALO_WIDTH_M, YARN_HALO_WIDTH_M
        else:
            hx, hy = YARN_HALO_WIDTH_M, abs(y1 - y0) + YARN_HALO_WIDTH_M
        out.append(_box_visual(f'yarn_halo_{name}', cx, cy, YARN_HEIGHT_M / 2, hx, hy, YARN_HEIGHT_M, YARN_HALO_RGB))
    return out


def light_block():
    return (f'    <scene><ambient>{_rgb(SCENE_AMBIENT)}</ambient><background>0.7 0.7 0.7 1</background>'
            f'<shadows>false</shadows></scene>\n'
            f'    <light name="ceiling" type="directional"><cast_shadows>false</cast_shadows>'
            f'<pose>0 0 10 0 0 0</pose><diffuse>{_rgb(LIGHT_DIFFUSE)}</diffuse><specular>0 0 0 1</specular>'
            f'<direction>0 0 -1</direction></light>\n')


def build(classroom=False):
    content = SRC.read_text()
    content = content.replace('<world name="tennis_court_world">',
                              '<world name="classroom_floor_world">' if classroom else '<world name="indoor_court_world">')
    # 陽光 -> 室內均勻照明
    content = content.replace('    <include><uri>model://sun</uri></include>\n', light_block())
    # 地面 (場外) 顏色
    content = content.replace(
        '<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>',
        f'<material><ambient>{_rgb(WOOD_RGB if classroom else OUTSIDE_RGB)}</ambient>'
        f'<diffuse>{_rgb(WOOD_RGB if classroom else OUTSIDE_RGB)}</diffuse>'
        f'<specular>0 0 0 1</specular></material>', 1)
    # 拿掉半透明綠牆 (只有外觀的那個 model)
    content, n = re.subn(r'    <model name="court_walls">.*?</model>\n', '', content, flags=re.S)
    if n != 1:
        raise RuntimeError('找不到 court_walls')
    m = re.search(r'    <model name="tennis_net">.*?</model>\n', content, re.S)
    if not m:
        raise RuntimeError('找不到 tennis_net 模型')
    content = content[:m.end()] + '\n' + court_block(classroom) + content[m.end():]
    dst = DST_CLASSROOM if classroom else DST
    dst.write_text(content)
    print(f'寫入 {dst}')


def main():
    build(False)
    build(True)


if __name__ == '__main__':
    main()
