#!/usr/bin/env python3
"""原地轉向系統的方塊圖 + 各方塊轉移函數 + 極點零點對照 (2026-10-05)。

輸出 (本資料夾)：
  圖1_轉向迴路現況.png   (a) 外迴路 (相機 -> STM32 轉向 P -> 輪速迴路 -> 車體)  (b) 輪速迴路展開
  圖2_延遲補償.png       (c) 加上相機延遲補償後的方塊圖 + 代數推導
  圖3_極點零點對照.png   每個方塊的極點/零點由哪些參數決定 + 閉迴路裕度

數字來源：main.c (B)、D 同學 Jetson 程式、motor_model.DEFAULT_PARAMS (擬合)、vision_node 相機節奏。
線性化：死區/飽和/積分分離/量化/不等間隔取樣都當成非線性另外標，不進極點零點。
用法：python3 make_block_diagram.py
"""
import math
import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

OUT = Path(__file__).resolve().parent
DPI = 150
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['mathtext.fontset'] = 'cm'
plt.rcParams['axes.unicode_minus'] = False

# ---------------------------------------------------------------- 參數與數值計算
T = 0.01                                   # STM32 TIM6
KP_W, KI_W, KD_W = 5.0, 1.0, 0.01          # main.c 輪速 PID (離散，每 tick)
KPC, KIC, KDC = KP_W, KI_W / T, KD_W * T   # 連續等效
KM, TAU_M = 550.0 / 870.0, 0.35            # motor_model.DEFAULT_PARAMS
TAU_F = -T / math.log(0.8)                 # 低通 0.8/0.2
R_W, L_W, ETA = 0.052, 0.243, 0.70
KPSI = 6.0 * ETA * R_W / L_W               # deg/s per rpm 差
INTERVALS = (0.111, 0.111, 0.453)          # 視覺輸出節奏 (實體 CSV)
TBAR = sum(INTERVALS) / 3
T_HOLD = sum(t * t for t in INTERVALS) / (2 * sum(INTERVALS))   # 保持的平均年齡
ALPHA = 0.35
T_D = 0.60
T_TOT = T_D + T_HOLD

num_cg = np.polymul([KDC, KPC, KIC], [KM])
den_cg = np.polymul([1.0, 0.0], [TAU_M, 1.0])
NUM_TW = np.polymul(num_cg, [TAU_F, 1.0])
DEN_TW = np.polyadd(np.polymul(den_cg, [TAU_F, 1.0]), num_cg)
TW_POLES = np.roots(DEN_TW)
TW_ZEROS = np.roots(NUM_TW)


def tw(s):
    c = (KDC * s ** 2 + KPC * s + KIC) / s
    g = KM / (TAU_M * s + 1)
    h = np.exp(-s * T / 2) / (TAU_F * s + 1)
    return c * g / (1 + c * g * h)


def loop_nocomp(w, kp):
    s = 1j * w
    ema = ALPHA / (1 - (1 - ALPHA) * np.exp(-s * TBAR))
    return kp * KPSI * ema * np.exp(-s * T_TOT) * tw(s) / s


W = np.logspace(-2, 1.2, 200000)
PH = np.unwrap(np.angle(loop_nocomp(W, 1.0)))
I180 = int(np.argmax(PH <= -np.pi))
W180 = W[I180]
KP_CRIT = 1.0 / abs(loop_nocomp(W180, 1.0))


def margins(kp):
    m = np.abs(loop_nocomp(W, kp))
    ic = int(np.argmax(m < 1))
    return 180 + math.degrees(PH[ic]), KP_CRIT / kp


def comp_dominant_pole(kp):
    p = np.roots(np.polyadd(np.polymul([1.0, 0.0], DEN_TW), kp * KPSI * NUM_TW))
    real = [x.real for x in p if abs(x.imag) < 1e-9]
    return max(real)


def comp_kp_limit():
    for kp in np.arange(0.5, 40, 0.05):
        p = np.roots(np.polyadd(np.polymul([1.0, 0.0], DEN_TW), kp * KPSI * NUM_TW))
        if max(p.real) > 0:
            return kp
    return None


# ---------------------------------------------------------------- 畫圖工具
C_JET = ('#E8F5E9', '#2E7D32')   # Jetson (D 同學程式)
C_STM = ('#E3F2FD', '#1565C0')   # STM32 (B 同學 main.c)
C_PHY = ('#F2F2F2', '#424242')   # 馬達 / 車體 (物理)
C_CMP = ('#FFF3E0', '#E65100')   # 延遲補償 (新增)
INK = '#1A1A1A'
ORANGE = '#E65100'


class Board:
    def __init__(self, fig, rect, width, height):
        self.fig = fig
        self.ax = fig.add_axes(rect)
        self.ax.set_xlim(0, width)
        self.ax.set_ylim(0, height)
        self.ax.axis('off')
        self.r = fig.canvas.get_renderer()

    # 中文 + 數學混排 (matplotlib 3.5 的 mathtext 沒有中文字型，只能分段排)
    def text(self, x, y, s, fs=12, ha='center', va='center', color=INK, weight='normal'):
        segs = [p for p in re.split(r'(\$[^$]*\$)', s) if p]
        if not segs:
            return []
        objs = [self.ax.text(0, 0, p, fontsize=fs, ha='left', va='baseline', color=color,
                             weight=weight) for p in segs]
        bbs = [o.get_window_extent(self.r) for o in objs]
        total = sum(b.width for b in bbs)
        x0, y0 = self.ax.transData.transform((x, y))
        start = {'center': x0 - total / 2, 'left': x0, 'right': x0 - total}[ha]
        base0 = self.ax.transData.transform((0, 0))[1]   # 量測時所有片段的 baseline
        ylo = min(b.y0 for b in bbs) - base0 + y0
        yhi = max(b.y1 for b in bbs) - base0 + y0
        # 所有片段共用同一條 baseline，整體再依 va 對齊
        base = y0
        if va == 'center':
            base = y0 - ((ylo + yhi) / 2 - y0)
        elif va == 'bottom':
            base = y0 + (y0 - ylo)
        elif va == 'top':
            base = y0 - (yhi - y0)
        inv = self.ax.transData.inverted()
        cx = start
        for o, b in zip(objs, bbs):
            o.set_position(inv.transform((cx, base)))
            cx += b.width
        return objs

    def lines(self, x, y, rows, fs=10.5, dy=0.30, color='#444', ha='center'):
        for i, row in enumerate(rows):
            self.text(x, y - i * dy, row, fs=fs, ha=ha, va='top', color=color)

    def block(self, cx, cy, w, h, expr, title=None, note=None, colors=C_STM, dashed=False, fs=19,
              title_fs=11.5, note_fs=10.5):
        fc, ec = colors
        self.ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                         boxstyle='round,pad=0.0,rounding_size=0.10',
                                         fc=fc, ec=ec, lw=1.8, ls=(0, (4, 2.5)) if dashed else '-',
                                         zorder=2))
        if expr:
            self.text(cx, cy, expr, fs=fs)
        if title:
            self.text(cx, cy + h / 2 + 0.07, title, fs=title_fs, va='bottom', color=ec, weight='bold')
        if note:
            self.lines(cx, cy - h / 2 - 0.07, note, fs=note_fs)
        return {'l': (cx - w / 2, cy), 'r': (cx + w / 2, cy), 't': (cx, cy + h / 2), 'b': (cx, cy - h / 2)}

    def nonlin(self, cx, cy, w, h, kind, title, note=None, colors=C_STM):
        port = self.block(cx, cy, w, h, None, title, note, colors, dashed=True)
        a = 0.36 * min(w, h)
        ec = colors[1]
        self.ax.plot([cx - a, cx + a], [cy, cy], color='#999', lw=0.8, zorder=3)
        self.ax.plot([cx, cx], [cy - a, cy + a], color='#999', lw=0.8, zorder=3)
        if kind == 'sat':
            xs = [cx - a, cx - a * 0.45, cx + a * 0.45, cx + a]
            ys = [cy - a * 0.6, cy - a * 0.6, cy + a * 0.6, cy + a * 0.6]
        else:   # 死區
            xs = [cx - a, cx - a * 0.35, cx + a * 0.35, cx + a]
            ys = [cy - a * 0.7, cy, cy, cy + a * 0.7]
        self.ax.plot(xs, ys, color=ec, lw=2.2, zorder=3)
        return port

    def sumj(self, x, y, signs, color=INK, r=0.22):
        self.ax.add_patch(Circle((x, y), r, fc='white', ec=color, lw=1.8, zorder=3))
        d = r * 0.7071
        self.ax.plot([x - d, x + d], [y - d, y + d], color=color, lw=1.2, zorder=4)
        self.ax.plot([x - d, x + d], [y + d, y - d], color=color, lw=1.2, zorder=4)
        off = {'l': (-r - 0.13, 0.20), 'b': (0.20, -r - 0.13), 't': (0.20, r + 0.13), 'r': (r + 0.13, 0.20)}
        for side, sg in signs.items():
            dx, dy = off[side]
            self.ax.text(x + dx, y + dy, sg, fontsize=15, ha='center', va='center', color=color,
                         weight='bold')
        return {'l': (x - r, y), 'r': (x + r, y), 't': (x, y + r), 'b': (x, y - r)}

    def sampler(self, x, y, title=None, note=None, color=INK):
        a = 0.26
        for px in (x - a, x + a):
            self.ax.add_patch(Circle((px, y), 0.045, fc='white', ec=color, lw=1.4, zorder=4))
        self.ax.plot([x - a, x + a * 0.75], [y, y + 0.26], color=color, lw=1.8, zorder=4)
        self.ax.annotate('', xy=(x + 0.05, y + 0.30), xytext=(x - 0.15, y + 0.12),
                         arrowprops=dict(arrowstyle='->', color=color, lw=1.0,
                                         connectionstyle='arc3,rad=-0.4'))
        if title:
            self.text(x, y + 0.42, title, fs=11.5, va='bottom', color=color, weight='bold')
        if note:
            self.lines(x, y - 0.18, note)
        return {'l': (x - a, y), 'r': (x + a, y)}

    def wire(self, pts, color=INK, lw=1.6, arrow=True, label=None, lpos=None, lfs=14, lha='center'):
        xs, ys = zip(*pts)
        if arrow and len(pts) >= 2:
            self.ax.plot(xs[:-1], ys[:-1], color=color, lw=lw, zorder=1, solid_capstyle='butt')
            self.ax.annotate('', xy=pts[-1], xytext=pts[-2],
                             arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, shrinkA=0, shrinkB=0,
                                             mutation_scale=17), zorder=1)
        else:
            self.ax.plot(xs, ys, color=color, lw=lw, zorder=1)
        if label:
            if lpos is None:
                lpos = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2 + 0.24)
            self.text(lpos[0], lpos[1], label, fs=lfs, ha=lha, color=color)

    def dot(self, x, y, color=INK):
        self.ax.add_patch(Circle((x, y), 0.065, fc=color, ec=color, zorder=5))

    def legend(self, x, y, items, fs=11):
        for i, (colors, label, dashed) in enumerate(items):
            yy = y - i * 0.36
            self.ax.add_patch(Rectangle((x, yy - 0.12), 0.42, 0.24, fc=colors[0], ec=colors[1], lw=1.6,
                                        ls=(0, (3, 2)) if dashed else '-'))
            self.text(x + 0.55, yy, label, fs=fs, ha='left')


LEGEND = [(C_JET, 'Jetson（D 同學程式）', False),
          (C_STM, 'STM32（B 同學 main.c）', False),
          (C_PHY, '馬達 / 車體（物理）', False),
          (('#FFFFFF', '#666666'), '虛線框 = 非線性（不進極點零點）', True)]


# ---------------------------------------------------------------- 圖 1
def figure1():
    fig = plt.figure(figsize=(20, 15.0), dpi=DPI)
    fig.patch.set_facecolor('white')
    ha_, hb_ = 7.7, 7.3
    a = Board(fig, [0, hb_ / 15.0, 1, ha_ / 15.0], 20, ha_)
    b = Board(fig, [0, 0, 1, hb_ / 15.0], 20, hb_)

    # ---------------- (a) 外迴路
    a.text(0.3, 7.25, '(a) 現在的原地轉向迴路（實車 main.c，沒有延遲補償）', fs=17, ha='left', weight='bold')
    a.text(0.3, 6.78, '角度 θ 以「往球的方向轉」為正（順時針）；球在右邊方位角為正。每個方塊裡是轉移函數，方塊上方是名稱，下方是參數。',
           fs=11.5, ha='left', color='#555')
    a.legend(14.6, 7.3, LEGEND)
    yt, yb = 5.05, 2.45
    s1 = a.sumj(1.4, yt, {'l': '+', 'b': '−'})
    a.wire([(0.15, yt), s1['l']], label=r'$\theta_{ball}$', lpos=(0.55, yt + 0.3))
    a.text(0.55, yt - 0.32, '球的方向', fs=10.5, color='#444')
    cam = a.block(3.4, yt, 2.0, 1.1, r'$e^{-T_d\,s}$', '相機 + YOLO + Jetson',
                  [r'$T_d\approx0.60$ s（推估，未實測）'], C_JET)
    a.wire([s1['r'], cam['l']], label=r'$b$')
    smp = a.sampler(5.45, yt, r'取樣 $T_k$', ['0.111 / 0.111 / 0.453 s', '循環（不等間隔）'], C_JET[1])
    a.wire([cam['r'], smp['l']], arrow=False)
    ema = a.block(7.55, yt, 2.3, 1.2, r'$\dfrac{\alpha\,z}{z-(1-\alpha)}$', 'EMA 平滑（D 同學程式）',
                  [r'$\alpha$ = 0.35 → 極點 $z$ = 0.65', r'≈ 連續 $s=-1.9$ rad/s（$\tau$≈0.52 s）'], C_JET)
    a.wire([smp['r'], ema['l']])
    kp = a.block(10.25, yt, 1.2, 1.1, r'$K_p$', '轉向 P', ['rpm/°，實車 0.6'], C_STM)
    a.wire([ema['r'], kp['l']], label=r'$\hat b_k$')
    zoh = a.block(12.5, yt, 2.1, 1.2, r'$\dfrac{1-e^{-sT_k}}{s}$', '保持到下一筆 BALL',
                  [r'平均多 $T_h$ = 0.17 s 延遲'], C_STM)
    a.wire([kp['r'], zoh['l']], label=r'$u_k$')
    sat = a.nonlin(14.85, yt, 1.2, 1.1, 'sat', '每輪飽和', ['±45 rpm'], C_STM)
    a.wire([zoh['r'], sat['l']], label=r'$u$')
    a.text(17.2, yt + 0.3, r'左 $+u/2$，右 $-u/2$', fs=12)
    twb = a.block(16.75, yb, 2.8, 1.2, r'$T_w(s)$', '輪速 PID 迴路 ×2（展開見 b）',
                  [r'$\Delta n=n_L-n_R$', r'兩輪相同時 $\Delta N=T_w(s)\,U$'], C_STM)
    a.wire([sat['r'], (19.0, yt), (19.0, yb), twb['r']])
    kps = a.block(12.65, yb, 2.5, 1.25, r'$K_\psi=\dfrac{6\,\eta\,r}{L}$', '差速運動學（原地轉）',
                  ['r = 0.052 m、L = 0.243 m、η = 0.70', '= 0.899 °/s per rpm'], C_PHY)
    a.wire([twb['l'], kps['r']], label=r'$\Delta n$')
    itg = a.block(9.6, yb, 1.0, 1.1, r'$\dfrac{1}{s}$', '積分', ['極點 s = 0'], C_PHY)
    a.wire([kps['l'], itg['r']], label=r'$\dot\theta$')
    a.wire([itg['l'], (6.2, yb)], arrow=False, label=r'$\theta$', lpos=(7.6, yb + 0.26))
    a.dot(6.2, yb)
    a.wire([(6.2, yb), (6.2, yb - 0.75)])
    a.text(6.35, yb - 0.72, '車頭轉角（輸出）', fs=11, ha='left', color='#444')
    a.wire([(6.2, yb), (1.4, yb), s1['b']])
    a.text(1.55, (yt + yb) / 2, '相機直接量的是「誤差」b，\n所以延遲在誤差之後、控制器之前',
           fs=10.5, ha='left', color='#444')
    a.text(0.3, 0.95, r'開迴路：  $L(s)=K_p\;F_{EMA}(s)\;e^{-s\,(T_d+T_h)}\;T_w(s)\;\dfrac{K_\psi}{s}$'
           r'          閉迴路特徵方程：  $1+L(s)=0$', fs=15, ha='left')
    a.text(0.3, 0.3, r'延遲 $e^{-sT}$ 在迴路「裡面」：頻率 ω 時多 ωT rad 相位落後 → Kp 加大、交越頻率變高，相位裕度就被吃光 → 擺盪（實車 Kp = 1.5）',
           fs=12, ha='left', color='#333')

    # ---------------- (b) 輪速迴路
    b.ax.plot([0.2, 19.8], [hb_ - 0.08, hb_ - 0.08], color='#BBBBBB', lw=1.0)
    b.text(0.3, 6.85, r'(b) 輪速迴路 $T_w(s)$：每個輪子各一個，STM32 每 10 ms（TIM6）跑一次', fs=17, ha='left',
           weight='bold')
    yt, yb = 5.0, 2.65
    s = b.sumj(1.6, yt, {'l': '+', 'b': '−'})
    b.wire([(0.2, yt), s['l']], label=r'$n_{ref}$', lpos=(0.65, yt + 0.3))
    pid = b.block(4.55, yt, 3.8, 1.2, r'$K_P+\dfrac{K_I}{T\,s}+K_D\,T\,s$', '輪速 PID（main.c PID_Calc）',
                  [r'$K_P$=5、$K_I$=1、$K_D$=0.01、$T$=10 ms',
                   r'零點 $-K_I/(K_P T)=-20$ rad/s，極點 $s=0$',
                   r'積分分離：|e| > max(30, 0.5 $n_{ref}$) 清積分（非線性）'], C_STM)
    b.wire([s['r'], pid['l']], label=r'$e$')
    sat = b.nonlin(7.95, yt, 1.2, 1.1, 'sat', 'PWM 限幅', ['±1000'], C_STM)
    b.wire([pid['r'], sat['l']])
    b.text(6.98, yt + 0.27, 'PWM', fs=10.5)
    dz = b.nonlin(9.95, yt, 1.2, 1.1, 'dead', '靜摩擦死區', ['±130 PWM'], C_PHY)
    b.wire([sat['r'], dz['l']])
    mot = b.block(12.75, yt, 2.6, 1.2, r'$\dfrac{K_m}{\tau_m\,s+1}$', '直流減速馬達（含車身慣量）',
                  [r'$K_m$ = 550/870 = 0.632 rpm/PWM', r'$\tau_m$ = 0.35 s（擬合）→ 極點 −2.86 rad/s'], C_PHY)
    b.wire([dz['r'], mot['l']])
    b.wire([mot['r'], (15.4, yt)], arrow=False)
    b.dot(15.4, yt)
    b.wire([(15.4, yt), (16.6, yt)])
    b.text(16.7, yt, r'$n$  輪子轉速 [rpm]', fs=13, ha='left')
    enc = b.block(12.75, yb, 2.9, 1.15, r'$\approx e^{-sT/2}$', '編碼器 M 法（827.2 count/圈）',
                  ['平均 5 ms 延遲；量化 7.25 rpm（非線性）'], C_STM)
    b.wire([(15.4, yt), (15.4, yb), enc['r']])
    lpf = b.block(7.6, yb, 4.1, 1.25, r'$\dfrac{0.2\,z}{z-0.8}\approx\dfrac{1}{\tau_f\,s+1}$', '低通濾波 0.8 / 0.2',
                  [r'$\tau_f=-T/\ln 0.8$ = 44.8 ms → 極點 −22.3 rad/s'], C_STM)
    b.wire([enc['l'], lpf['r']], label=r'$n_{meas}$')
    b.wire([lpf['l'], (1.6, yb), s['b']], label=r'$n_{filt}$', lpos=(3.6, yb + 0.26))
    p = sorted(TW_POLES, key=lambda x: x.real)
    pc = [x for x in TW_POLES if x.imag > 0][0]
    wn, zeta = abs(pc), -pc.real / abs(pc)
    num = NUM_TW / DEN_TW[0]
    zs = sorted([z.real for z in TW_ZEROS if abs(z) < 1000])
    real_pole = [x.real for x in TW_POLES if abs(x.imag) < 1e-9][0]
    b.text(0.3, 1.2, r'$T_w(s)=\dfrac{C(s)\,G_m(s)}{1+C(s)\,G_m(s)\,H(s)}\approx'
           r'\dfrac{%.2f\,(s+%.1f)(s+%.1f)}{(s+%.1f)\,(s^2+%.2f\,s+%.0f)}$' %
           (num[1], -zs[1], -zs[0], -real_pole, -2 * pc.real, abs(pc) ** 2) +
           r'      DC 增益 = 1（PID 有積分，穩態追得到 $n_{ref}$）', fs=15, ha='left')
    b.text(0.3, 0.4, r'閉迴路極點 −%.1f、−%.1f ± %.1fj（$\omega_n$ = %.1f rad/s，ζ ≈ %.2f）。'
           r'零點 −20.0 / −22.3 跟極點 −21.5 幾乎抵消 → 主要看那對低阻尼極點。' %
           (-real_pole, -pc.real, pc.imag, wn, zeta) +
           '（線性化；非線性模型步階 overshoot 40~60%，實車目前沒有輪速步階數據可以驗證）', fs=11.5, ha='left',
           color='#333')
    fig.savefig(OUT / '圖1_轉向迴路現況.png', dpi=DPI, facecolor='white')
    plt.close(fig)


# ---------------------------------------------------------------- 圖 2
def figure2():
    H = 13.6
    fig = plt.figure(figsize=(20, H), dpi=DPI)
    fig.patch.set_facecolor('white')
    c = Board(fig, [0, 0, 1, 1], 20, H)
    c.text(0.3, H - 0.4, '(c) 加上相機延遲補償（Gazebo 已實作並驗證；實車 main.c 還沒移植）', fs=17, ha='left', weight='bold')
    c.text(0.3, H - 0.87, '橘色是新增的部分，其他方塊跟圖 (a) 完全一樣。收到 BALL 時算「目標航向」θ*，之後每 10 ms 用陀螺儀/編碼器的最新轉角追它。',
           fs=11.5, ha='left', color='#555')
    c.legend(14.6, H - 0.35, LEGEND[:3] + [(C_CMP, '延遲補償（新增）', False)])
    yt, ym, yb = 10.75, 8.3, 5.95
    s1 = c.sumj(1.4, yt, {'l': '+', 'b': '−'})
    c.wire([(0.15, yt), s1['l']], label=r'$\theta_{ball}$', lpos=(0.55, yt + 0.3))
    cam = c.block(3.3, yt, 1.8, 1.1, r'$e^{-T_d\,s}$', '相機 + YOLO + Jetson', [r'$T_d\approx0.60$ s'], C_JET)
    c.wire([s1['r'], cam['l']], label=r'$b$')
    smp = c.sampler(5.05, yt, r'取樣 $T_k$', None, C_JET[1])
    c.wire([cam['r'], smp['l']], arrow=False)
    ema = c.block(7.0, yt, 2.1, 1.2, r'$\dfrac{\alpha\,z}{z-(1-\alpha)}$', 'EMA（D 程式）', [r'$\alpha$ = 0.35'], C_JET)
    c.wire([smp['r'], ema['l']])
    s2 = c.sumj(9.15, yt, {'l': '+', 'b': '+'}, color=ORANGE)
    c.wire([ema['r'], s2['l']], label=r'$\hat b_k$')
    hold = c.block(10.95, yt, 1.75, 1.2, r'$\dfrac{1-e^{-sT_k}}{s}$', r'保持 $\theta^*$', None, C_CMP)
    c.text(10.95, yt - 0.68, 'BALL 進來才更新', fs=10.5, va='top', color='#444')
    c.wire([s2['r'], hold['l']], color=ORANGE, label=r'$\theta^*_k$', lfs=13)
    s3 = c.sumj(12.85, yt, {'l': '+', 'b': '−'}, color=ORANGE)
    c.wire([hold['r'], s3['l']], color=ORANGE)
    kp = c.block(14.55, yt, 1.2, 1.1, r'$K_p$', '轉向 P', ['每 10 ms 更新'], C_STM)
    c.wire([s3['r'], kp['l']], label=r'$e_c$')
    sat = c.nonlin(16.5, yt, 1.2, 1.1, 'sat', '每輪飽和', ['±45 rpm'], C_STM)
    c.wire([kp['r'], sat['l']], label=r'$u$')
    twb = c.block(16.75, yb, 2.8, 1.2, r'$T_w(s)$', '輪速 PID 迴路 ×2', ['見圖 (b)'], C_STM)
    c.wire([sat['r'], (19.0, yt), (19.0, yb), twb['r']])
    kps = c.block(12.65, yb, 2.5, 1.25, r'$K_\psi=\dfrac{6\,\eta\,r}{L}$', '差速運動學', None, C_PHY)
    c.wire([twb['l'], kps['r']], label=r'$\Delta n$')
    itg = c.block(9.6, yb, 1.0, 1.1, r'$\dfrac{1}{s}$', '積分', None, C_PHY)
    c.wire([kps['l'], itg['r']], label=r'$\dot\theta$')
    xs = 5.55
    c.wire([itg['l'], (xs, yb)], arrow=False, label=r'$\theta$', lpos=(7.6, yb + 0.26))
    c.dot(xs, yb)
    c.wire([(xs, yb), (1.4, yb), s1['b']])
    sen = c.block(xs, ym, 2.3, 1.1, r'$g$', '轉角感測', None, C_CMP)
    c.text(xs - 1.3, ym + 0.17, '陀螺儀積分：g = 1（+ 零偏漂移、雜訊）', fs=10.5, ha='right', color='#444')
    c.text(xs - 1.3, ym - 0.17, '編碼器積分：g = 1/η ≈ 1.43（側滑）', fs=10.5, ha='right', color='#444')
    c.wire([(xs, yb), sen['b']], color=ORANGE)
    xd = 8.05
    dly = c.block(9.15, ym, 1.4, 1.05, r'$e^{-D\,s}$', None, None, C_CMP)
    c.text(9.98, ym + 0.2, '取 D 秒前的轉角', fs=11.5, ha='left', color=ORANGE, weight='bold')
    c.text(9.98, ym - 0.17, 'D = 0.8 s（設定值）', fs=10.5, ha='left', color='#444')
    c.wire([sen['r'], (xd, ym)], color=ORANGE, arrow=False, label=r'$\theta_m$', lpos=(7.45, ym + 0.27))
    c.dot(xd, ym, ORANGE)
    c.wire([(xd, ym), dly['l']], color=ORANGE)
    c.wire([dly['t'], s2['b']], color=ORANGE, label=r'$\theta_m(t_k-D)$', lpos=(9.3, (ym + yt) / 2 - 0.05),
           lfs=13, lha='left')
    yr = ym - 1.1
    c.wire([(xd, ym), (xd, yr), (12.85, yr), s3['b']], color=ORANGE,
           label=r'$\theta_m(t)$（每 10 ms 最新值）', lpos=(10.55, yr + 0.25), lfs=12.5)

    # 代數
    y0 = 4.35
    c.ax.plot([0.2, 19.8], [y0 + 0.4, y0 + 0.4], color='#BBBBBB', lw=1.0)
    c.text(0.3, y0, r'代數推導（$F$ = EMA，$T=T_d+T_h$ = 0.77 s，$g$ = 轉角感測增益；大寫為拉氏轉換）', fs=13.5,
           ha='left', weight='bold')
    c.text(0.5, y0 - 0.7, r'$\theta^*_k=\theta_m(t_k-D)+\hat b_k,\qquad e_c(t)=\theta^*-\theta_m(t)'
           r'\quad\Rightarrow\quad E_c(s)=F(s)\,e^{-sT}\left[\Theta_{ball}(s)-\Theta(s)\right]'
           r'\;-\;g\left(1-e^{-sD}\right)\Theta(s)$', fs=16, ha='left')
    c.text(0.5, y0 - 1.6, r'特徵方程：  $1+K_pK_\psi\,\dfrac{T_w(s)}{s}\left[\,F(s)\,e^{-sT}+g\left(1-e^{-sD}\right)\right]=0$'
           r'        （沒有補償時中括號只有 $F\,e^{-sT}$，就是圖 a）', fs=15, ha='left')
    c.text(0.5, y0 - 2.55, r'理想 $F\approx1,\;g=1,\;D=T$：中括號 = 1  →  $1+K_pK_\psi\,\dfrac{T_w(s)}{s}=0$ 沒有延遲了，'
           r'而  $\dfrac{\Theta(s)}{\Theta_{ball}(s)}=e^{-sT}\,\dfrac{K_pK_\psi T_w(s)/s}{1+K_pK_\psi T_w(s)/s}$',
           fs=15, ha='left')
    p06, p15 = comp_dominant_pole(0.6), comp_dominant_pole(1.5)
    klim = comp_kp_limit()
    c.text(0.5, y0 - 3.35, r'→ 延遲只讓整條響應晚 T 秒出發，不會再造成擺盪。主極點 $s\approx-K_pK_\psi$：'
           r'Kp 0.6 → %.2f（τ≈%.1f s），Kp 1.5 → %.2f（τ≈%.2f s）；線性模型要到 Kp≈%.0f，輪速迴路那對極點才越過虛軸。'
           % (p06, -1 / p06, p15, -1 / p15, klim), fs=12, ha='left', color='#333')
    c.text(0.5, y0 - 3.8, r'實際不理想：F 是 EMA 不是純延遲，中括號不會剛好 = 1（D 取 0.8 > 0.6 是在補 EMA 和保持的落後，離線掃 D 得到的）；'
           r'編碼器 g≈1.43 → 內迴路以為轉多了 → 比陀螺儀慢（Gazebo 2.20 s vs 1.60 s）。', fs=12, ha='left', color='#333')
    c.text(0.5, y0 - 4.2, r'跟經典 Smith predictor 的差別：經典版用模型 $\hat G(s)U(s)$ 算「沒延遲的輸出」；這裡直接用量到的 $\theta_m$ 代替模型輸出 → '
           r'不需要馬達/車體模型，只需要知道 D。', fs=12, ha='left', color='#333')
    fig.savefig(OUT / '圖2_延遲補償.png', dpi=DPI, facecolor='white')
    plt.close(fig)


# ---------------------------------------------------------------- 圖 3
def figure3():
    H = 17.0
    fig = plt.figure(figsize=(20, H), dpi=DPI)
    fig.patch.set_facecolor('white')
    t = Board(fig, [0, 0, 1, 1], 20, H)
    t.text(0.3, H - 0.4, '極點 / 零點由哪些參數決定（連續時間等效，單位 rad/s）', fs=17, ha='left', weight='bold')
    cols = [0.3, 3.2, 6.6, 9.0, 12.0, 16.2]
    heads = ['方塊', '轉移函數', '極點', '零點', '由哪些參數決定', '目前數值']
    ytop = H - 1.15
    for x, h in zip(cols, heads):
        t.text(x, ytop, h, fs=13, ha='left', weight='bold')
    t.ax.plot([0.2, 19.8], [ytop - 0.3, ytop - 0.3], color=INK, lw=1.2)
    pc = [x for x in TW_POLES if x.imag > 0][0]
    rp = [x.real for x in TW_POLES if abs(x.imag) < 1e-9][0]
    rows = [
        (C_JET, ['相機 + YOLO 延遲', r'$e^{-T_d\,s}$', '沒有（非有理式）', r'Padé：右半面 $+2/T$',
                 r'畫面延遲 $T_d$、保持 $T_h$', r'$T$ = 0.77 s → +2.6（非最小相位）']),
        (C_JET, ['不等間隔取樣 + 保持', r'$\dfrac{1-e^{-sT_k}}{s}$', '—', '—', 'D 程式輸出節奏',
                 r'$T_h=\Sigma T_k^2/(2\Sigma T_k)$ = 0.17 s']),
        (C_JET, ['EMA 平滑', r'$\dfrac{\alpha\,z}{z-(1-\alpha)}$', r'$z=1-\alpha$', r'$z=0$',
                 r'$\alpha$、平均間隔 $\bar T$ = 0.225 s', r'$z$=0.65 → $s$=−1.9（τ 0.52 s）']),
        (C_STM, ['轉向 P', r'$K_p$', '—', '—', 'Kp：只把閉迴路極點沿根軌跡推', '實車 0.6']),
        (C_STM, ['輪速 PID', r'$K_P+\dfrac{K_I}{Ts}+K_DTs$', r'$s=0$',
                 r'$-\dfrac{K_I}{K_PT}$、$-\dfrac{K_P}{K_DT}$', 'B 的 Ki/Kp 比值、T = 10 ms', '−20、−5×10⁴（可忽略）']),
        (C_PHY, ['直流馬達', r'$\dfrac{K_m}{\tau_m\,s+1}$', r'$-1/\tau_m$', '—',
                 r'$\tau_m\approx R\,J_{eq}/(K_tK_e)$：馬達常數 + 車身慣量', '−2.86（τ 0.35 s，擬合）']),
        (C_STM, ['編碼器 M 法', r'$e^{-sT/2}$', '—', '—', 'T = 10 ms', '5 ms；量化 7.25 rpm']),
        (C_STM, ['低通濾波', r'$\dfrac{0.2\,z}{z-0.8}$', r'$\ln 0.8\,/\,T$', '—（閉迴路變零點）', '0.8、T', '−22.3']),
        (C_STM, ['輪速閉迴路', r'$T_w(s)$', ('%.1f、%.1f ± %.1fj' % (rp, pc.real, pc.imag)).replace('-', '−'), '−20.0、−22.3',
                 '上面 PID、馬達、編碼器、低通全部', r'$\omega_n$=%.1f，ζ≈%.2f' % (abs(pc), -pc.real / abs(pc))]),
        (C_PHY, ['差速運動學', r'$K_\psi=6\,\eta\,r/L$', '—', '—', 'r、L、η（原地轉側滑）', '0.899 °/s per rpm']),
        (C_PHY, ['積分（轉速 → 角度）', r'$\dfrac{1}{s}$', r'$s=0$', '—', '—', '型 1 系統：步階穩態誤差 0']),
        (C_CMP, ['延遲補償（新增）', r'$g\,(1-e^{-Ds})$ 回授支路', '—', '—', 'D、轉角感測 g',
                 'D = 0.8 s；理想時把延遲移出特徵方程']),
    ]
    y = ytop - 0.85
    rh = 0.74
    for colors, cells in rows:
        t.ax.add_patch(Rectangle((0.2, y - rh / 2), 19.6, rh, fc=colors[0], ec='none', zorder=0, alpha=0.65))
        t.ax.add_patch(Rectangle((0.2, y - rh / 2), 0.08, rh, fc=colors[1], ec='none', zorder=1))
        for x, cell in zip(cols, cells):
            fs = 15 if (x == cols[1] and '$' in cell) else 12
            t.text(x if x != cols[0] else x + 0.12, y, cell, fs=fs, ha='left')
        y -= rh + 0.06

    # 閉迴路
    y -= 0.25
    t.ax.plot([0.2, 19.8], [y + 0.35, y + 0.35], color='#BBBBBB', lw=1.0)
    t.text(0.3, y, r'閉迴路（線性化，$K_\psi$ = 0.899，$T$ = 0.77 s + EMA + 輪速迴路）', fs=14, ha='left', weight='bold')
    y -= 0.6
    kps = (0.3, 0.6, 1.0, 1.5)
    cx = [0.5, 5.2, 7.6, 10.0, 12.4, 14.8]
    for x, h in zip(cx, ['', 'Kp = 0.3', '0.6', '1.0', '1.5', '臨界']):
        t.text(x, y, h, fs=12.5, ha='left', weight='bold')
    y -= 0.45
    pm = [margins(k) for k in kps]
    t.text(cx[0], y, '無補償：相位裕度', fs=12.5, ha='left')
    for x, (p, g) in zip(cx[1:], pm):
        t.text(x, y, '%.0f°' % p, fs=12.5, ha='left')
    t.text(cx[5], y, r'Kp ≈ %.1f（ω = %.2f rad/s，週期 %.1f s）' % (KP_CRIT, W180, 2 * math.pi / W180), fs=12.5, ha='left')
    y -= 0.42
    t.text(cx[0], y, '無補償：增益裕度', fs=12.5, ha='left')
    for x, (p, g) in zip(cx[1:], pm):
        t.text(x, y, '%.2f' % g, fs=12.5, ha='left')
    y -= 0.42
    t.text(cx[0], y, '理想補償：主極點', fs=12.5, ha='left')
    for x, k in zip(cx[1:], kps):
        t.text(x, y, ('%.2f' % comp_dominant_pole(k)).replace('-', '−'), fs=12.5, ha='left')
    t.text(cx[5], y, 'Kp ≈ %.0f（輪速迴路那對極點越過虛軸）' % comp_kp_limit(), fs=12.5, ha='left')
    y -= 0.55
    t.lines(0.5, y, [
        '非線性模型檢查（delay_comp_design.closed_loop，跑 40 s，看最後 15 s 的擺幅）：Kp 1.2 → 1.7°、Kp 1.5 → 8.1°（還在慢慢收斂）、'
        'Kp 2.0 → 64°（持續擺盪），跟線性臨界 Kp ≈ 2.0 一致。',
        '實車 Kp = 1.5 是持續擺盪 18~30°（端點還會停約 3 s），比模型更靠近穩定邊界；可能是實際延遲比 0.77 s 長'
        '（實測等效延遲 0.53~0.92 s），原因未確認。',
    ], fs=11.5, dy=0.36, color='#333', ha='left')
    y -= 1.05
    t.ax.plot([0.2, 19.8], [y + 0.3, y + 0.3], color='#BBBBBB', lw=1.0)
    t.text(0.3, y, '非線性（不在極點零點裡，但會改變實際響應）', fs=14, ha='left', weight='bold')
    t.lines(0.5, y - 0.45, [
        '死區 ±130 PWM（起步要等積分累積）、PWM 限幅 ±1000、每輪 ±45 rpm、積分分離 |e| > max(30, 0.5 n_ref)、編碼器量化 7.25 rpm、',
        '不等間隔取樣（最長 0.453 s 才來一筆）、球距 < 30 cm 停車、連續 2 幀才送 BALL、5 幀沒看到送 BALL_OFF（Chassis_Stop 會清積分）。',
    ], fs=11.5, dy=0.36, color='#333', ha='left')
    fig.savefig(OUT / '圖3_極點零點對照.png', dpi=DPI, facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    figure1()
    figure2()
    figure3()
    print('Kpsi=%.4f  Th=%.3f  Kp_crit=%.2f  w180=%.3f' % (KPSI, T_HOLD, KP_CRIT, W180))
    print('T_w poles', np.round(TW_POLES, 3), 'zeros', np.round(TW_ZEROS, 3))
