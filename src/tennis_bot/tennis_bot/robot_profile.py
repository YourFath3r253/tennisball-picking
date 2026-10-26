"""車體設定：TENNISBOT_ROBOT=v1 (舊車體，預設) / v2 (A 同學 2026-10-25 新車體，tennis_bot_v2.urdf)。

只放「跟車體幾何有關」的常數，各節點從這裡讀，換車體不用改程式：
  track_phys / radius_phys   車子實際的輪距/輪半徑 (馬達模型算車體速度用)。v1 用實體車 main.c 的值；
                             v2 輪距是 CAD 輪子缺口估的 (0.33~0.35)，還沒量。
  axle_in_base               兩驅動輪軸中點在 base_link 的位置 (x, y)
  basket_*                   「球進後車廂」判定範圍 (base_link 座標)
  cam_in_base                相機在 base_link 的 (x, y) (實驗工具擺球用)
STM32 韌體裡的輪距 (main.c WHEEL_TRACK = 0.243) 不在這裡：那是 B 同學程式寫死的常數，換車體後如果沒改，
實體車的編碼器轉角就會算錯，模擬照實體車一樣保留 (stm32_b_node，環境變數 TENNISBOT_FW_TRACK 可模擬改過的韌體)。
"""
import os

ROBOT = os.environ.get('TENNISBOT_ROBOT', 'v1')

PROFILES = {
    'v1': dict(track_phys=0.243, radius_phys=0.052, axle_in_base=(0.080, -0.135),
               basket_x=(-0.36, 0.0), basket_y_center=-0.135, basket_y_half=0.11,
               basket_min_z=0.05, basket_rest_max_z=0.15, cam_in_base=(0.20, -0.135)),
    # v2：base_link = 輪軸中點正下方地面；後車廂 (CAD trunk_v2_01/02) x -0.508~-0.312、寬 ±0.2、
    # 車廂底 = 鋼板頂 (離地 35 mm)，球躺著時中心約 0.07 m
    'v2': dict(track_phys=0.34, radius_phys=0.052, axle_in_base=(0.0, 0.0),
               basket_x=(-0.51, -0.31), basket_y_center=0.0, basket_y_half=0.19,
               basket_min_z=0.05, basket_rest_max_z=0.15, cam_in_base=(0.103, 0.0)),
}
P = PROFILES[ROBOT]
