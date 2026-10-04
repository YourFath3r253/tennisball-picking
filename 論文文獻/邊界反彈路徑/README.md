# 邊界反彈路徑（D 同學提案）參考文獻

D 同學的提案是：沒有地圖、也不用弓字路徑。車子直走，碰到邊界就轉一個固定角度，看到球就去撿。
這種策略在文獻裡叫 **bouncing robot**，或叫 **random bounce / 隨機碰撞式覆蓋**（掃地機器人 Roomba 的基本行為就是這種）。
下面每篇都已經下載到這個資料夾。

| 檔案 | 文獻 | 跟我們的關係 |
|---|---|---|
| `NilBecLav17.pdf` | A. Q. Nilles, I. Becerra, S. M. LaValle, "Periodic Trajectories of Mobile Robots," *IEEE/RSJ IROS*, 2017. | **最關鍵。** 他們證明：如果每次都用「固定角度」反彈（角度相對於邊界法線），在凸多邊形（矩形也算）裡，軌跡會收斂到一條穩定的**週期軌道**（limit cycle），之後就在同一條路線上一直繞。換句話說，固定角度反彈最後**不會走遍整個場地**。這就是我們 90° 繞外圈、180° 在同一條線來回的原因。 |
| `EriLav13.pdf` | L. H. Erickson, S. M. LaValle, "Toward the Design and Analysis of Blind, Bouncing Robots," *IEEE ICRA*, 2013, pp. 3233–3238. | 「直走、撞到邊界、照固定規則轉向」這種最簡單的機器人，把反彈規則當成一個動態系統來分析，討論它能不能走遍環境。 |
| `NilRenBecLav21.pdf` | A. Q. Nilles, Y. Ren, I. Becerra, S. M. LaValle, "A visibility-based approach to computing non-deterministic bouncing strategies," *IJRR* 40(10–11), 2021, pp. 1196–1211. | 2017 那篇的延伸。反彈角度如果在一個範圍裡（非固定）變動，要怎麼設計才能走到想去的地方。 |
| `KunBahLav24.pdf` | S. Kundu, Y. Bahoo, S. M. LaValle, "Systematic Escape Using Billiard Moves," *RoMoCo*, 2024, pp. 285–290. | 矩形房間裡的撞球式反彈路徑。Related work 有整理反彈機器人的覆蓋問題（包括矩形房間、π/4 反彈角）。 |
| `FS93-03-008.pdf` | K. L. Doty, R. R. Harrison, "Sweep Strategies for a Sensory-Driven, Behavior-Based Vacuum Cleaning Agent," *AAAI Fall Symposium* FS-93-03, 1993. | 掃地機器人實測：基準是 random sweep（碰到障礙就**隨機轉 ±180°**）。他們測到最好的是「隨機走，碰到障礙時有 5% 機率改成沿牆走」，20 分鐘覆蓋約 85%。 |
| `US6809490.pdf` | J. L. Jones, P. R. Mass (iRobot), "Method and System for Multi-Mode Coverage for an Autonomous Robot," US Patent 6,809,490 B2, 2004. | Roomba 的專利。它的 BOUNCE 行為是：撞到牆以後，從「相對於牆 90°~270°」的範圍裡**隨機**挑一個新方向（均勻分布），往轉比較少的那一邊轉。注意它用的是**隨機角度**，不是固定角度。 |

## 文獻告訴我們的結論（還要用模擬驗證）

1. **固定角度反彈會收斂到週期軌道**（Nilles 2017）。實際效果取決於角度：
   - 180°：原路折返，等於在一條線上來回。
   - 90°：繞外圈，變成沿著場地邊緣跑的矩形，中間走不到。
   - 110°、135°：週期軌道比較長、比較「斜」，掃過的範圍比較大。
2. 真正走遍整個場地的商用做法（Roomba、Doty 1993）都會加**隨機性**，避免卡在週期軌道裡。
3. 我們的情況多了一個好處：**每撿一顆球，方向就被打亂一次**（Sean 的觀察），等於免費多了一點隨機性。
   所以球多的時候，固定角度也可能全部撿完；但剩下最後幾顆球、而且剛好在週期軌道外面時，就會卡住。

## 下載不到、需要從清大圖書館找的

都下載到了，沒有缺。
