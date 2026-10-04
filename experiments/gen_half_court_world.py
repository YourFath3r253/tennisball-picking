"""產生 D 同學提案用的半場世界 half_court_lines.world：
以 tennis_court.world 為底 (地面、網子、牆的半透明參考線都一樣，之後模擬真實化的
地面摩擦/砂礫也會跟著帶過來)，再加上四條彩色邊界色帶 (只有外觀、沒有碰撞，就像貼在地上的膠帶)。
球的位置之後用 gen_ball_layout.py --world ... --half-court 寫進去。

用法: python3 experiments/gen_half_court_world.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src' / 'tennis_bot' / 'tennis_bot'))
from half_court import LINES, LINE_WIDTH  # noqa: E402

SRC = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'tennis_court.world'
DST = ROOT / 'src' / 'tennis_bot' / 'worlds' / 'half_court_lines.world'


def lines_block():
    out = ['    <!-- D 同學提案：半場四條邊界各一種顏色 (experiments/gen_half_court_world.py 產生，'
           '定義在 tennis_bot/half_court.py)。只有外觀沒有碰撞，像貼在地上的色帶。 -->',
           '    <model name="court_lines">', '      <static>true</static>', '      <link name="lines_link">']
    for color, d in LINES.items():
        (x0, y0), (x1, y1) = d['seg']
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        # 線段兩端各多延伸半個線寬，四個角才會接起來
        if abs(x1 - x0) > abs(y1 - y0):
            sx, sy = abs(x1 - x0) + LINE_WIDTH, LINE_WIDTH
        else:
            sx, sy = LINE_WIDTH, abs(y1 - y0) + LINE_WIDTH
        r, g, b = d['rgb']
        out.append(f'        <visual name="{color.lower()}_line"><pose>{cx:.4f} {cy:.4f} 0.0005 0 0 0</pose>'
                   f'<geometry><box><size>{sx:.4f} {sy:.4f} 0.001</size></box></geometry>'
                   f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'
                   f'</visual>')
    out += ['      </link>', '    </model>']
    return '\n'.join(out) + '\n'


def main():
    content = SRC.read_text()
    content = content.replace('<world name="tennis_court_world">', '<world name="half_court_lines_world">')
    # 插在網子模型後面
    m = re.search(r'    <model name="tennis_net">.*?</model>\n', content, re.S)
    if not m:
        raise RuntimeError('找不到 tennis_net 模型')
    content = content[:m.end()] + '\n' + lines_block() + content[m.end():]
    DST.write_text(content)
    print(f'寫入 {DST}')


if __name__ == '__main__':
    main()
