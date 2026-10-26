"""A 同學的新車體 CAD (trunkasb.stp, 2026-10-25) -> 模擬用的網格 + 質量性質 + URDF (tennis_bot_v2.urdf)。

座標 (CAD 檢查結果，見記憶 project_tennisbot_cad_v2)：A 的組合件 +Z = 車頭、+Y = 上、+X = 車子左邊，單位 mm；
車子中線 x = -200，鋼板上表面 y = 0。轉成 ROS (REP-103)：base_link 放在「兩驅動輪軸中點正下方的地面」，
  x_ros = (z - AXLE_Z) / 1000、y_ros = (x + 200) / 1000、z_ros = (y + PLATE_TOP_H) / 1000
AXLE_Z、AXLE_Y = 驅動馬達 (motor1/2) 出軸軸心；PLATE_TOP_H = 輪半徑 - AXLE_Y (鋼板上表面離地高度)。

CAD 沒有的東西 (照 Sean 說明自己建)：驅動輪、萬向輪、撿球滾輪 (左右各一，海綿 1 cm，尺寸同舊車)。
質量：鋼板用 A 的 Inventor 值；電池/滾輪/驅動輪用 Sean 估的；Jetson、STM32、馬達用網路規格；
3D 列印件用 體積 x PLA 1.24 g/cm³ x 填充係數 PRINT_FILL (預設 0.6，還沒秤，之後換實測)。

用法: python3 experiments/cad_v2_to_urdf.py   (需要 pip 的 cadquery-ocp)
輸出: src/tennis_bot/urdf/meshes/v2/*.stl (公尺、base_link 座標)、src/tennis_bot/urdf/tennis_bot_v2.urdf、
      3D檔案/A_車體v2_20261025/質量表.csv
"""
import csv
import math
import os
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ROOT = Path(__file__).resolve().parent.parent
CAD_DIR = ROOT / '3D檔案' / 'A_車體v2_20261025'
STEP = CAD_DIR / 'trunkasb.stp'
MESH_DIR = ROOT / 'src' / 'tennis_bot' / 'urdf' / 'meshes' / 'v2'
URDF = ROOT / 'src' / 'tennis_bot' / 'urdf' / 'tennis_bot_v2.urdf'

# ---- 幾何 (CAD 量測 + 假設) ----
WHEEL_R = 0.052          # 驅動輪半徑：main.c WHEEL_RADIUS (同一款輪子，待量)
WHEEL_W = 0.026          # 輪寬 (假設)
AXLE_Y, AXLE_Z = 16.8, 58.0   # motor1/2 出軸軸心 (STEP mm)
PLATE_TOP_H = WHEEL_R * 1000 - AXLE_Y   # 35.2 mm
TRACK = float(os.environ.get('V2_TRACK', '0.34'))  # 輪距：輪子在鋼板/車體 x=-50~0 的缺口裡 -> 約 0.33~0.35 (待量)
CASTER_R = (PLATE_TOP_H - 2.0) / 2000.0  # 萬向輪：鋼板底面到地面 (待量位置/尺寸)
CASTER_XY = [(-0.47, 0.15), (-0.47, -0.15)]   # 後車廂底下兩角 (假設)
# 滾輪 (左右各一)：軸 = motor3/4 主軸 (CAD PCA)，往前上方 48°；掛在出軸尖端往下 5 mm
ROLLER_AXIS_STEP = np.array([0.0, 0.745, 0.667])
ROLLER_TIP_STEP = {'right': np.array([-272.6, 39.5, 164.0]), 'left': np.array([-127.5, 39.5, 164.0])}
ROLLER_R_VIS, ROLLER_L = 0.05, 0.04   # 同舊車：直徑 100 mm (含 1 cm 海綿)、寬 40 mm
# 碰撞半徑：舊車用 0.045 (比外觀小 5 mm)；海綿實際外徑是 0.05，壓縮用軟接觸 kp/kd 表示 -> v2 預設 0.05
ROLLER_R_COL = float(os.environ.get('V2_ROLLER_R_COL', '0.05'))
ROLLER_SHIFT = float(os.environ.get('V2_ROLLER_SHIFT', '5.0'))
# 海綿軟接觸 (舊車調給 27 g 球的值 kp 150 N/m, kd 4.2；實際海綿剛性沒量)
ROLLER_KP = float(os.environ.get('V2_ROLLER_KP', '150'))
ROLLER_KD = float(os.environ.get('V2_ROLLER_KD', '4.2'))   # 滾輪中心在出軸尖端往下 (沿軸) 幾 mm (CAD 沒畫滾輪，假設)
# 相機：CAD 相機方塊前面中心；俯角照實測擬合 21.3° (CAD 沒畫角度，新車可調：V2_CAM_PITCH_DEG)，左偏 3.96° 同舊車 (D 程式 image_cx=283)
CAM_STEP = np.array([-200.0, 187.0, 161.0])
CAM_PITCH = math.radians(float(os.environ.get('V2_CAM_PITCH_DEG', '21.3')))
CAM_YAW = 0.0691

# ---- 質量 (kg)。None = 3D 列印件，用體積算 ----
PLA = 1.24e-3            # g/mm^3 -> kg 用 /1000
PRINT_FILL = float(os.environ.get('V2_PRINT_FILL', '0.6'))
MASS = {
    'trunk_v2_sheetmetal': 3.255,   # A 的 Inventor (鋼)
    'trunk_v2_battery': 0.50,       # Sean 估
    'trunk_v2_jetsonnano': 0.14,    # 網路：開發板含散熱片約 138 g
    'trunk_v2_STM32': 0.035,        # Nucleo-64 板 (網路列 77 g 可能含包裝，估 35 g)
    'trunk_v2_camera': 0.05,        # 估
    'trunk_v2_motor1': 0.25, 'trunk_v2_motor2': 0.25,   # JGB37-555B (網路沒有可靠值，估)
    'trunk_v2_motor3': 0.20, 'trunk_v2_motor4': 0.20,   # JGB37-3530B 約 200 g
}
ROLLER_MASS, WHEEL_MASS, CASTER_MASS = 0.10, 0.20, 0.03
# 碰撞：球會經過或可能碰到的零件 (電池/板子/驅動馬達在箱子裡面，不給碰撞)
NO_COLLISION = {'trunk_v2_battery', 'trunk_v2_jetsonnano', 'trunk_v2_STM32', 'trunk_v2_motor1', 'trunk_v2_motor2'}
COLORS = {'sheetmetal': '0.45 0.5 0.55 1', 'battery': '0.2 0.6 0.2 1', 'jetsonnano': '0.2 0.6 0.2 1',
          'STM32': '0.2 0.6 0.2 1', 'camera': '0.5 0.2 0.6 1', 'motor': '0.8 0.2 0.2 1', 'part12': '0.95 0.55 0.1 1',
          '_01': '0.55 0.75 0.95 1', '_02': '0.55 0.75 0.95 1'}


def to_ros(p_mm):
    p = np.atleast_2d(p_mm)
    return np.c_[(p[:, 2] - AXLE_Z), (p[:, 0] + 200.0), (p[:, 1] + PLATE_TOP_H)] / 1000.0


def read_parts():
    doc = TDocStd_Document(TCollection_ExtendedString('doc'))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    assert r.ReadFile(str(STEP)) == IFSelect_RetDone
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels = TDF_LabelSequence()
    st.GetFreeShapes(labels)
    parts = []

    def name_of(lab):
        n = TDataStd_Name()
        return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'

    def walk(lab, loc):
        if st.IsAssembly_s(lab):
            comps = TDF_LabelSequence()
            st.GetComponents_s(lab, comps)
            for i in range(1, comps.Length() + 1):
                c = comps.Value(i)
                ref = c.__class__()
                st.GetReferredShape_s(c, ref)
                walk(ref, loc.Multiplied(st.GetLocation_s(c)))
        else:
            parts.append((name_of(lab), st.GetShape_s(lab).Moved(loc)))
    for i in range(1, labels.Length() + 1):
        walk(labels.Value(i), TopLoc_Location())
    return parts


def tessellate(shape):
    BRepMesh_IncrementalMesh(shape, 0.3, False, 0.25, True)
    V, F = [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(V)
            for k in range(1, tri.NbNodes() + 1):
                p = tri.Node(k).Transformed(trsf)
                V.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for k in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(k).Get()
                F.append((base + a - 1, base + c - 1, base + b - 1) if rev else (base + a - 1, base + b - 1, base + c - 1))
        exp.Next()
    return np.array(V), np.array(F)


def mass_props(V, F):
    """封閉三角網格的體積、重心、對重心的慣性張量 (密度 1)。有號四面體法。"""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    vol = vol6.sum() / 6.0
    cg = ((a + b + c) * vol6[:, None]).sum(0) / 24.0 / vol
    # 二次矩 ∫x x^T dV (對原點)
    C = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            C[i, j] = (vol6 / 120.0 * (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j])
                                      + a[:, i] * b[:, j] + a[:, j] * b[:, i] + a[:, i] * c[:, j]
                                      + a[:, j] * c[:, i] + b[:, i] * c[:, j] + b[:, j] * c[:, i])).sum()
    C = C - vol * np.outer(cg, cg)
    I = np.trace(C) * np.eye(3) - C
    return vol, cg, I


def write_stl(path, V, F):
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, 'wb') as f:
        f.write(b'tennis_bot v2'.ljust(80, b' '))
        f.write(np.uint32(len(F)).tobytes())
        rec = np.zeros(len(F), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        rec['n'] = n
        rec['v'] = tri
        f.write(rec.tobytes())


def color_of(name):
    for k, c in COLORS.items():
        if k in name:
            return c
    return '0.75 0.75 0.75 1'


def inertial_xml(m, cg, I, indent='    '):
    return (f'{indent}<inertial>\n{indent}  <origin xyz="{cg[0]:.5f} {cg[1]:.5f} {cg[2]:.5f}" rpy="0 0 0"/>\n'
            f'{indent}  <mass value="{m:.4f}"/>\n'
            f'{indent}  <inertia ixx="{I[0,0]:.6f}" ixy="{I[0,1]:.6f}" ixz="{I[0,2]:.6f}" '
            f'iyy="{I[1,1]:.6f}" iyz="{I[1,2]:.6f}" izz="{I[2,2]:.6f}"/>\n{indent}</inertial>\n')


def cyl_inertia(m, r, l):
    ia = 0.5 * m * r * r
    it = m * (3 * r * r + l * l) / 12.0
    return np.diag([it, it, ia])


def main():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    rows, visuals, collisions = [], [], []
    M_tot, S1, Itot_parts = 0.0, np.zeros(3), []
    for name, shape in read_parts():
        Vmm, F = tessellate(shape)
        V = to_ros(Vmm)
        vol, cg, I1 = mass_props(V, F)      # vol m^3, I1 對重心 (密度 1)
        if vol < 0:
            F = F[:, ::-1]
            vol, cg, I1 = mass_props(V, F)
        m = MASS.get(name)
        src = 'CAD/規格/估'
        if m is None:
            m = vol * 1e9 * PLA / 1000.0 * PRINT_FILL
            src = f'3D列印 體積x1.24x{PRINT_FILL}'
        rho = m / vol
        I = I1 * rho
        write_stl(MESH_DIR / f'{name}.stl', V, F)
        rows.append([name, f'{vol * 1e6:.1f}', f'{m:.3f}', src, *[f'{v:.4f}' for v in cg]])
        M_tot += m
        S1 += m * cg
        Itot_parts.append((m, cg, I))
        uri = f'package://tennis_bot/urdf/meshes/v2/{name}.stl'
        visuals.append(f'    <visual name="{name}">\n      <geometry><mesh filename="{uri}"/></geometry>\n'
                       f'      <material name="m_{name}"><color rgba="{color_of(name)}"/></material>\n    </visual>\n')
        if name not in NO_COLLISION:
            collisions.append(f'    <collision name="{name}_c">\n      <geometry><mesh filename="{uri}"/></geometry>\n'
                              f'    </collision>\n')
    cg_tot = S1 / M_tot
    I_tot = np.zeros((3, 3))
    for m, cg, I in Itot_parts:
        d = cg - cg_tot
        I_tot += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    with open(CAD_DIR / '質量表.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['part', 'volume_cm3', 'mass_kg', 'source', 'cg_x_m', 'cg_y_m', 'cg_z_m'])
        w.writerows(rows)
        w.writerow(['車體合計 (不含輪子/滾輪/萬向輪)', '', f'{M_tot:.3f}', '', *[f'{v:.4f}' for v in cg_tot]])

    # ---- 滾輪 (ROS 座標) ----
    ax = to_ros(np.array([[0, 0, 0]]))[0]
    axis_ros = (to_ros(ROLLER_AXIS_STEP * 1000.0) - ax)[0]
    axis_ros /= np.linalg.norm(axis_ros)
    roller_pitch = math.atan2(axis_ros[0], axis_ros[2])     # z 軸繞 y 轉到 axis
    roller_xml = ''
    for side in ('left', 'right'):
        c = to_ros(ROLLER_TIP_STEP[side] - ROLLER_SHIFT * ROLLER_AXIS_STEP)[0]
        Ir = cyl_inertia(ROLLER_MASS, ROLLER_R_VIS, ROLLER_L)
        roller_xml += f'''
  <!-- {side} 滾輪：CAD motor{'4' if side == 'left' else '3'} 出軸，軸往前上 48°；直徑/寬度/海綿同舊車 -->
  <link name="{side}_roller">
    <visual><geometry><cylinder radius="{ROLLER_R_VIS}" length="{ROLLER_L}"/></geometry>
      <material name="roller_y"><color rgba="0.8 0.8 0 1"/></material></visual>
    <visual><origin xyz="0.045 0 0" rpy="0 0 0"/><geometry><box size="0.01 0.01 0.01"/></geometry>
      <material name="roller_mark"><color rgba="1 0 0 1"/></material></visual>
    <collision><geometry><cylinder radius="{ROLLER_R_COL}" length="{ROLLER_L}"/></geometry></collision>
{inertial_xml(ROLLER_MASS, [0, 0, 0], Ir, '    ')}  </link>
  <joint name="{side}_roller_joint" type="continuous">
    <parent link="chassis"/><child link="{side}_roller"/>
    <origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" rpy="0 {roller_pitch:.4f} 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <gazebo reference="{side}_roller">
    <mu1>1.5</mu1><mu2>1.5</mu2><kp>{ROLLER_KP}</kp><kd>{ROLLER_KD}</kd><min_depth>0.001</min_depth><max_vel>10</max_vel>
  </gazebo>
'''
    wheel_xml = ''
    for side, sgn in (('left', 1), ('right', -1)):
        Iw = cyl_inertia(WHEEL_MASS, WHEEL_R, WHEEL_W)
        wheel_xml += f'''
  <link name="{side}_drive_wheel">
    <visual><geometry><cylinder radius="{WHEEL_R}" length="{WHEEL_W}"/></geometry>
      <material name="tyre"><color rgba="0.1 0.1 0.1 1"/></material></visual>
    <collision><geometry><cylinder radius="{WHEEL_R}" length="{WHEEL_W}"/></geometry></collision>
{inertial_xml(WHEEL_MASS, [0, 0, 0], Iw, '    ')}  </link>
  <joint name="{side}_wheel_joint" type="continuous">
    <parent link="chassis"/><child link="{side}_drive_wheel"/>
    <origin xyz="0 {sgn * TRACK / 2:.4f} {WHEEL_R}" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <gazebo reference="{side}_drive_wheel"><mu1>200.0</mu1><mu2>200.0</mu2><material>Gazebo/Black</material></gazebo>
'''
    caster_xml = ''
    for i, (cx, cy) in enumerate(CASTER_XY):
        caster_xml += f'''
  <link name="caster_{i}">
    <visual><geometry><sphere radius="{CASTER_R:.4f}"/></geometry><material name="caster"><color rgba="0.1 0.1 0.1 1"/></material></visual>
    <collision><geometry><sphere radius="{CASTER_R:.4f}"/></geometry>
      <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface></collision>
{inertial_xml(CASTER_MASS, [0, 0, 0], np.eye(3) * 2e-6, '    ')}  </link>
  <joint name="caster_{i}_joint" type="fixed">
    <parent link="chassis"/><child link="caster_{i}"/>
    <origin xyz="{cx} {cy} {CASTER_R:.4f}" rpy="0 0 0"/>
  </joint>
  <!-- 萬向輪 = 無摩擦球 (會自己轉向的輪子阻力很小)。摩擦要寫在 gazebo reference：URDF <collision> 裡的
       <surface> Gazebo 不會讀 (舊車 simple_bot.urdf 也是這樣寫，實際摩擦是預設值)。新車重心在輪軸後 16 cm，
       萬向輪撐 ~1/3 重量，有摩擦的話原地轉不動 (2026-10-26 實測：指令 86° 只轉 0°)。 -->
  <gazebo reference="caster_{i}"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
'''
    cam = to_ros(CAM_STEP)[0]
    tmpl = (ROOT / 'experiments' / 'tennis_bot_v2_template.urdf').read_text(encoding='utf-8')
    out = (tmpl.replace('@CHASSIS_VISUALS@', ''.join(visuals))
           .replace('@CHASSIS_COLLISIONS@', ''.join(collisions))
           .replace('@CHASSIS_INERTIAL@', inertial_xml(M_tot, cg_tot, I_tot))
           .replace('@WHEELS@', wheel_xml).replace('@ROLLERS@', roller_xml).replace('@CASTERS@', caster_xml)
           .replace('@TRACK@', f'{TRACK:.4f}').replace('@WHEEL_DIAM@', f'{2 * WHEEL_R:.4f}')
           .replace('@CAM_XYZ@', f'{cam[0]:.4f} {cam[1]:.4f} {cam[2]:.4f}')
           .replace('@CAM_RPY@', f'0 {CAM_PITCH} {CAM_YAW}'))
    URDF.write_text(out, encoding='utf-8')
    tot_all = M_tot + 2 * ROLLER_MASS + 2 * WHEEL_MASS + len(CASTER_XY) * CASTER_MASS
    print(f'車體 {M_tot:.3f} kg，重心 (m) {cg_tot.round(4)}；整台 {tot_all:.3f} kg；輪距 {TRACK} m；鋼板頂離地 {PLATE_TOP_H:.1f} mm')
    print(f'相機 (base_link) {cam.round(4)}；滾輪軸 {axis_ros.round(3)} pitch {math.degrees(roller_pitch):.1f}°')
    for r in rows:
        print('  ', r[0], r[1], 'cm3', r[2], 'kg', r[3])
    print('寫入', URDF)


if __name__ == '__main__':
    main()
