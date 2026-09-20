# 實體車轉向 PID 化計畫

> 這份檔案是給「之後在 Jetson 上安裝 Claude Code」的那個 session 看的背景資料，
> 也是 2026-09-18 跟 Sean 討論後的現況紀錄。裡面同時記錄「已確認的事實」跟「還沒確認、需要問 Sean 的事」，
> 兩者不要混為一談 — 標「[待確認]」的都還沒驗證過，不要當成事實使用。

## 目標

把撿球車現有「原地旋轉找球 → 直行 → 固定秒數盲抓」流程裡，同學寫的**離散式轉向決策**
換成**連續、平穩的 PID 轉向**，解決現在會轉過頭（發散）的問題。跟 Gazebo 模擬版的行為邏輯一致，
差別是實體車還沒做路徑規劃（開機後直接原地轉找最近的球）。

## 已決定的事（2026-09-18 討論後拍板）

- **Claude Code 架構**：裝在 Sean 的筆電上，SSH 遙控 Jetson（不是裝在 Jetson 本機）。
- **教室行程**：直接去教室接螢幕+鍵盤，把 Jetson 的 WiFi/SSH 環境一次設定好，之後開發不用再帶螢幕。
- **STM32 baseline**：`main.c`（改名前是 `USER CODE BEGIN Header.txt`）+ 這份 `PLAN.md` 已做 git baseline commit，之後改 PID 前後都能對比這個版本。

## 遠端存取設定（2026-09-18 已完成，教室現場做的）

- **SSH 金鑰登入已設定完成**，Jetson 帳號 `hp`（主機名稱顯示為 `123`），不用再打密碼
- 筆電 `~/.ssh/config` 已加好捷徑：
  - `ssh jetson` → 走 **Tailscale**（`100.104.92.104`），不管兩台在不在同個網路都能連，**平常請用這個**
  - `ssh jetson-lan` → 走教室 WiFi 直連 IP（`172.16.2.163`），這個 IP 離開教室後大概率會變，只是備用/除錯用
- **Tailscale 已裝好並登入**（筆電、Jetson 都用同一個帳號 `y31724005@`），兩邊互通已實測成功
- `hp` 帳號已設定 **passwordless sudo**（`/etc/sudoers.d/hp-nopasswd`），這樣 Claude Code 才能透過 SSH 直接執行需要 root 權限的指令（裝套件、燒錄等），不用每次手動輸密碼。
  取捨：這代表「能用 SSH 金鑰登入這台機器」=「能做任何系統層級操作」。因為本來就只有金鑰能登入（沒開密碼登入），對單人使用的機器人開發板來說這個風險可接受，但**這台以後不要拿來給不信任的人共用登入**。
- **已經把 sleep/suspend/hibernate 全部 mask 掉**（`systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target`），Jetson 系統層級不會再真的進入待機，不管是螢幕保護程式、電源管理設定還是什麼東西觸發都一樣被擋下來。真正的系統待機（跟螢幕變黑不一樣）本來會讓 SSH/網路整個斷掉，物理上碰它才能喚醒，所以直接鎖死比較保險。
- **已實測確認**：拔插電源重開機，不用按任何鍵、直接跳桌面（自動開機+自動登入都正常），Tailscale/SSH 開機後
  在數十秒內自己恢復連線，`eduroam` 自動重連，sudo 免密碼設定也在重開機後還在（因為是寫死在設定檔）。
  **這代表今天教室要處理的環境設定都已經驗證過、是穩固的**，之後可以純遠端（不用再帶螢幕）繼續開發。
- **已新增兩組 WiFi 設定檔**（`autoconnect: yes`，之後到範圍內會自動連上，密碼未寫進本檔案）：
  - `imoney`（手機熱點，一般 WPA2-PSK）
  - `nthupeap`（校園網路，WPA2-Enterprise/PEAP，帳號 `s112033247`）
  - 之後如果要加更多已知網路，指令模式：一般網路用 `sudo nmcli connection add type wifi con-name "<名稱>" ifname wlan0 ssid "<SSID>" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<密碼>"`；
    企業級（帳密驗證）用 PLAN 裡上面示範的 `802-1x.eap peap` 那組寫法。**密碼一律由 Sean 自己在終端機輸入，不會經過 Claude。**
- **[小提醒/之後收資料時注意]** Jetson 沒有內建電池校時的 RTC，剛開機時系統時間會先亂跳，等連上網路 NTP 校正後才準。
  之後做「開機沒多久就開始記錄角度-時間 CSV」的測試時，建議先等待約10幾秒讓時間校正完，避免最前面幾筆時間戳記不準。

## 硬體架構

- 深度相機 → Jetson Nano (32GB micro SD，可能是舊版 JetPack，繼承自學長實驗室) → UART → STM32 (Nucleo 板) → 兩顆底盤驅動馬達 + 兩顆撿球滾輪馬達
- 真實車另外有 9 軸陀螺儀，目前**沒有接上電路**（電路是別人接的，很亂，怕花時間）
- 團隊分工：Sean 負責視覺（已完成）+ 模擬；STM32 轉向控制原本是同學寫的離散版本

## 現有 STM32 程式碼分析（`main.c`，2026-09-18 讀取記錄）

**重要：不是從零開始。** 已經有的東西：

1. **左右輪各自已經有閉迴路轉速 PID**，靠編碼器回授（`TIM1`/`TIM4` encoder mode），
   在 `TIM6` 的 10ms 硬體中斷裡跑 (`PID_Calc`，`HAL_TIM_PeriodElapsedCallback`)。
   目前參數 `Kp=5.0, Ki=1.0, Kd=0.01`，有做積分分離跟低通濾波。
   → **輪子轉速本身不是同學說的「開迴路」控制，已經是 PID 了。**
2. **已經有里程計角度積分** `chassis_angle_deg`：靠差速輪距公式從左右輪編碼器算出來的相對轉角，
   不需要陀螺儀就已經有一個角度估計值可以用。
3. **已經有一個閉迴路精準轉向函式 `Chassis_TurnPrecise()`**（轉到目標角度、誤差 ±2° 才停），
   但目前主迴圈**沒有呼叫它**。
4. Jetson → STM32 UART 協定（115200 baud）：
   - Jetson 送 `"BALL,<距離cm>,<角度deg>\n"` 或 `"NOBALL\n"` / `"BALL_OFF\n"`
   - STM32 回 `"ACK:...\n"`
   - STM32 收到後**自己決定**怎麼動（不是 Jetson 端算好指令再送）

**同學寫的「離散」到底離散在哪裡**（主迴圈 `USER CODE BEGIN 3` 區塊）：

不是馬達轉速不連續（那已經是 PID），是「要不要轉、轉多快」這個**高層決策**是離散的 bang-bang：

- 用固定門檻角度（>30cm 時是 4°/6°，依距離切換）決定要不要轉
- 轉向轉速只有兩檔：`40 rpm` 或 `55 rpm`（角度 >14° 才用高檔）
- 轉完之後有 **1000ms 冷卻時間**才會再檢查一次角度（`last_turn_time`）

這種「衝一下、等一下、再看」配合相機辨識延遲，很容易轉過頭 — 跟 Sean 描述的發散現象吻合。

## 針對 Sean 提出的每一點的判斷

### 5. 轉向連續化怎麼做

低階馬達轉速 PID 不用重做。要做的是加一層「外環」轉向 PID：把 `ball_angle_deg` 當誤差，
連續算出轉向修正量（例如差速轉速），取代現在的兩檔位 if/else。可以放在 STM32 收到 `BALL` 封包時算，
邏輯跟 `PID_Calc` 類似，但輸出是連續值而不是 40/55 兩檔。

**資料收集順序建議**：先不要改 code，直接用現在的版本多跑幾次，把角度（`chassis_angle_deg` 或視覺角度）
對時間記錄下來當 baseline，畫圖存起來；PID 版本做出來後用同樣方式記錄，兩張圖疊在一起比較，
才有東西可以给教授看「有沒有變好、好多少」。

### 6/7. STM32 燒錄 + Jetson 裝 Claude Code

**[待確認]** 現在只有 `main.c` 一個檔案，沒有看到完整 STM32CubeIDE 專案
（`Core/`、`Drivers/`、連結腳本、`.ioc`、Makefile）。要能編譯燒錄，需要完整專案。

如果有完整專案，`arm-none-eabi-gcc`（交叉編譯工具鏈）跟 `openocd`（燒錄工具，支援 ST-Link）
在 aarch64（Jetson 的 CPU 架構）上都有現成版本，理論上可以整個編譯+燒錄流程都在 Jetson 上做，
不需要另外一台電腦（前提是 Nucleo 板的 ST-Link USB 線接到 Jetson 的 USB 孔）。
比較不確定能不能在 aarch64 上跑的是 ST 官方的 STM32CubeProgrammer（Java 介面工具），
所以建議走 `openocd` 這條路線比較穩。

**Claude Code 架構：建議重新考慮「裝在哪」這件事**

Sean 原本想法是把 Claude Code 直接裝在 Jetson 上，把 Jetson 當筆電用。但有兩個風險：

1. 這台 Jetson Nano 如果是舊版 JetPack（通常對應 Ubuntu 18.04, glibc 2.27），
   新版 Node.js（Claude Code 需要）官方 build 可能要求更新的 glibc，裝不裝得起來要先測試才知道，
   不是裝了就一定能跑。
2. 舊 Jetson Nano 本身資源就吃緊（要同時做視覺辨識），再跑一個 CLI 工具是額外負擔，
   雖然 Claude Code 本身不重，但如果為了裝它還要處理系統升級，會拖累進度。

**建議改成：Claude Code 裝在你的筆電上，透過 SSH 遙控 Jetson**（而不是裝在 Jetson 本機）：

- 完全繞開上面兩個風險，Jetson 系統版本再舊都沒差，只要能 SSH 進去就好
- 螢幕問題直接消失：Claude Code 本來就是純終端機工具，不需要顯示器；
  你描述的「接螢幕」「無線投影」都是為了解決 Claude Code 需要畫面這個假設，但這個假設不成立
- 具體做法：筆電用 `sshfs` 把 Jetson 上的程式資料夾掛載成本機資料夾，Claude Code 在筆電上對這個掛載路徑做讀寫（跟本機檔案一樣），需要「真的在 Jetson 上執行」的動作（跑視覺程式、透過 USB 燒錄 STM32）再用 `ssh jetson '指令'` 送過去執行
- 唯一需要碰到教室螢幕的情境，是 Jetson 現在完全沒連網路、沒開 SSH，那需要**一次性**接螢幕+鍵盤設定好網路跟 SSH（大概 10-15 分鐘），之後就再也不用接螢幕

如果你評估後還是想直接裝在 Jetson 本機（例如想要離線也能跑），也可以，只是要先測一下 Node.js 相容性。

### 8. 螢幕 vs 無線投影 vs SSH

見上一點，建議直接 SSH 遙控，理由是資源占用最小、不受 JetPack 版本限制、也不用大螢幕拖著車體旁邊。
如果之後真的需要「看畫面」除錯視覺辨識，可以用單張截圖 `scp` 回筆電看，或跑一個輕量 MJPEG 串流用瀏覽器看，
都比整個桌面環境或無線投影省資源。

### 9. 馬達轉速不準（70rpm 指令、量測 49rpm）

在懷疑硬體、決定要不要上陀螺儀之前，建議先做一個 5 分鐘就能測完、不用拆電路的檢查：

**編碼器解析度常數校正**。目前 code 裡寫死：
```
ENCODER_RESOLUTION = 11.0 * 18.8 * 4.0   // 基本解析度11 * 減速比18.8 * 四倍頻
```
PID 是「準確地追一個目標」沒錯，但如果這個常數本身跟馬達實際減速比對不起來，
PID 追到的 `target_rpm` 跟輪子實際物理轉速之間就會有一個固定比例的落差 —
看起來像「馬達不準」，其實是換算常數錯了，不需要換硬體就能修。

具體驗證方法：手動轉一顆論子剛好 10 圈（or 用馬克筆做記號數圈數），
同時從 UART 印出 `chassis_enc_left` 累積值（現在的 code 是每 10ms 讀完就歸零，
需要暫時加一行 debug 把每次讀到的值累加起來印出來），算出「轉 10 圈總共多少 count」，
除以 10 再除以 4（四倍頻），看看是不是等於 `11 * 18.8 = 206.8`。差很多才代表是機構/減速比問題。

**[待確認]** 同學量測「70rpm 指令、49rpm 實際」是在現在這版有編碼器 PID 的 code 上量的，
還是更早期開迴路版本量的？如果是舊版量的，這個數字可能已經不適用，不能直接當作現在的問題依據
（沒有量測依據前，先當作未知，不要假設現在還是這個誤差）。

**建議順序**：先做編碼器常數校正（不用碰電路）→ 跑連續 PID 轉向、收斂測試 → 如果還是不夠穩，
再加陀螺儀。這跟 Sean 原本想法一致，只是把「校正常數」插在最前面，因為成本最低、最可能解釋現象。

### 10. 儲存空間 / log 管理

Jetson 一旦能 SSH（或至少連網），建議比照現在模擬端 `run113/run114/...` 的資料夾慣例：
每次測試結束、CSV 檔案關閉後，自動 `rsync`/`scp` 整個 run 資料夾回筆電（或直接進這個 git repo 的
`experiments/` 底下），確認同步成功後才刪本機那份。加一個簡單的容量檢查（開始新測試前跑一下 `df`），
超過門檻（例如 26GB）就先警告、暫停產生新 log，而不是等到滿了才發現。

## Jetson 視覺程式分析（`realtime_camera_trt_distance_angle_uart.py`，2026-09-18 讀取記錄）

- 用 TensorRT engine (`best_fold1_opset9_sim_512x384.engine`) 做球偵測，輸入 512x384，相機抓 640x480 @30fps
- **UART 埠是 `/dev/ttyACM0`**（USB 虛擬序列埠，不是傳統 TX/RX 接線）→ 強烈佐證 STM32 是 Nucleo 內建 ST-Link 板：
  Nucleo 的 ST-Link 子板本身就是一個 USB CDC 虛擬 COM port，在 Linux 上會長這樣。也就是說 **Jetson↔STM32 現在已經是「一條 USB 線」在接**，
  跟 Sean 問的「Jetson↔筆電要不要插 USB-C」是完全不同的兩條線、兩個目的，不要搞混。
- **距離/角度傳送頻率是 5Hz**（每 0.2 秒送一次 `BALL,<dist>,<angle>`），不是每幀都送（相機是 30fps）
- 角度值在送出前已經做過 **EMA 平滑**（α=0.35），代表 STM32 收到的角度本身就已經有一段平滑延遲，
  之後設計外環轉向 PID 的時候，這個延遲要算進去（微分項especially容易被這種延遲搞得更抖）
- 角度正負號：**正值 = 球在畫面右側，負值 = 左側**，跟 STM32 那邊 `angle>0 → SpinRight` 的判斷一致，沒有反向問題
- 已經有「連續看到2幀才算偵測到、連續5幀沒看到才算真的丟球」的 hysteresis，跟模擬端做過的視覺雙門檻邏輯是同一個精神
- 這支程式本身就會存 CSV（每幀存 `bearing_deg` 等欄位），**這個 CSV 已經可以直接拿來當「角度 vs 時間」的資料來源**，
  不用另外寫記錄工具，測試時把 `TEST_NAME` 環境變數設好、跑完把 CSV 抓回來就有圖可以畫
- [待確認/次要] `.engine` 檔是針對特定 GPU/TensorRT 版本編譯出來的，不能跨機器直接搬，如果之後 Jetson 系統版本有更動需要重新從 ONNX 轉換，先記著，現階段不影響 PID 工作

## Jetson 免螢幕連線方式（2026-09-18 討論）

Sean 問「能不能直接 USB 線接 Jetson 跟筆電」——可以試，這是跟教室 WiFi 方案平行的另一條路，
在家就能試、完全不用出門，如果成功就不一定要去教室了：

- Jetson Nano 板子上有一個 **Micro-USB 孔**（如果 Jetson 現在不是靠這個孔供電，而是用另一個圓形電源孔供電），
  可以切換成「USB 裝置模式」，插上電腦後 Jetson 會偽裝成一張虛擬網路卡，自己會有固定 IP（通常 `192.168.55.1`），
  接上就能直接 SSH 進去，完全不需要 WiFi、不需要路由器、不需要去教室
- 需要一條**支援傳輸資料**的 Micro-USB 對 Type-C 連接線（很多線只能充電不能傳資料，先拿去手機上測試過比較保險）
- 這功能是否已經開啟不確定（重灌過系統，原廠預設可能被改掉），要實際試才知道
- 這條線的作用跟前面說的 `/dev/ttyACM0`（Jetson↔STM32）是兩回事，不要搞混
- 如果這招沒反應，退回教室 WiFi+螢幕方案（上次已經決定的備案）

## 團隊多人遠端存取（2026-09-19 討論）

- **現況**：B/D 同學昨晚去網球場測邊界偵測，D 同學也會 SSH 進 Jetson，但 Jetson 上沒有球場現場的網路
  （只有手機熱點，沒有 `eduroam`），導致連不上。
- **解法（不需要接螢幕）**：
  1. 把該次測試用的手機熱點，用跟 `imoney` 一樣的 `sudo nmcli connection add ... wifi-sec.psk` 指令加進 Jetson
     （誰在場、知道那個熱點密碼，就由誰在終端機打，密碼不透過 Claude）
  2. 需要遠端連線的隊友，在自己電腦裝 Tailscale，**用 Tailscale 後台「Invite user」邀請他用自己的帳號加入**，
     不要共用 Sean 的 Tailscale 登入帳密
  3. Tailscale 只解決「連不連得到 Jetson」，不解決「登不登得進 `hp` 這個帳號」——用密碼登入的隊友網路通了就能繼續用密碼；
     如果也想要免密碼登入，需要另外把他的 SSH 公鑰加進 `~/.ssh/authorized_keys`
- **根本建議**：與其每個新場地都臨時加一個 WiFi 設定檔，不如**固定每次測試都帶同一顆行動熱點開著**
  （例如固定用 Sean 的 `imoney`），Jetson 就永遠連同一個已知網路，不用每個場地都補登記一次。
- **範圍確認（2026-09-19）**：這台 Jetson 目前只有 Sean、D 同學兩人使用，D 同學的熱點是**最後一次**新增熱點設定檔。
- **D 同學不需要設定 SSH 金鑰**：D 同學本來就知道 `hp` 帳號密碼，Tailscale 只解決「連不連得到」，
  裝好 Tailscale、用他自己的帳號登入邀請後，直接 `ssh hp@100.104.92.104` 打密碼登入即可，
  跟金鑰（Sean 筆電在用的免密碼方式）是兩件獨立的事，金鑰是可省略的額外方便設定，不是必要條件。
  （風險提醒：`hp` 密碼兩人都知道、且該帳號 sudo 免密碼，等同知道密碼=完全控制權，僅適合互信隊友之間，
  密碼不要貼在公開群組/repo。）

## 視覺程式版本判斷方式（不要只看日期）

D 同學目前在同步開發球場邊界偵測（靠顏色區分邊界），會持續產生新日期的檔案，**「日期最新」不代表協定相容**。
比較可靠的判斷方式：打開該版本搜尋 `BALL,`（例如 `grep -n "BALL," 檔名.py`），確認它是送
`BALL,<距離>,<角度>` 連續座標（跟現在 `main.c` 相容），而不是 `BALL_ON`/`BALL_OFF` 開關格式（不相容）。

## 電源架構（2026-09-19，看 [撿球機7_29.pdf](../簡報/google簡報下載/撿球機7_29.pdf) 最後兩頁確認）

- **馬達**：獨立 24V(7串) 鋰電池，15.6Ah，輸出上限40A，特意抓大餘裕避免卡球時電壓塌陷重開機
- **Jetson**：獨立微雪 UPS 模組 + 18650 4顆(2串2並 7.4V/6.6Ah)，就是為了避免「用一般行動電源、感測器啟動瞬間電壓驟降
  導致 Jetson 無預警重開機」這個問題才特別設計的（架構本身是對的）
- **STM32 的電是從 Jetson 的 40-pin GPIO 拉 5V 供電**（順便兩板共地，穩定通訊用）——
  **代表 Jetson 只要瞬斷電，STM32 也會跟著斷電重開**，裡面累積的里程計角度 `chassis_angle_deg` 會歸零。
  之後測試中如果數據忽然異常/歸零，先檢查是不是發生過這種瞬斷，而不是先懷疑演算法。
- **2026-09-19 事故記錄**：Jetson 專屬電池的螺絲鬆脫導致間歇性斷電重開機（跟上面設計要避免的問題長得像，
  但根因是機構鎖固沒鎖緊，不是電路設計問題），鎖緊後恢復正常。

## Jetson 序列埠權限（2026-09-19 已修正，一勞永逸）

`/dev/ttyACM0`（STM32）預設權限只有 `root`/`dialout` 群組能讀寫，一般帳號跑視覺程式會 Permission denied。
**已經把 `hp` 加進 `dialout` 群組**（`sudo usermod -aG dialout hp`），這個設定寫在帳號本身，
不會因為重開機、重新插拔 USB 線而消失，之後不用再手動 `chmod 666 /dev/ttyACM0`。

## 舊視覺數據分析（2026-09-19，見 [舊視覺數據_20260919分析/](舊視覺數據_20260919分析/)）

今天實際測試因為電池問題被打斷，**沒有產生今天的離散轉向 baseline 資料**（確認過 Jetson 上今天沒有任何新 CSV）。
先把 Jetson 上既有的舊 CSV 全部抓回來分析，結論：

- 6個舊檔案裡，**只有 `distance_512x384_uart.csv`（9/9，230幀，31.9秒）有 `motor_on` 欄位**，
  是唯一一份「車子真的有在動」的紀錄；其他5個（6/21）都沒有 `motor_on`，是純視覺端的靜態距離校正測試
  （球固定放在已知位置測 `test_50cm_center/left/right` 等），不是轉向行為紀錄。
- 9/9 那份資料看得出來離散轉向的震盪特徵：`bearing_deg` 在 -29°~+29° 之間、
  `smooth_bearing_deg` 正負翻轉6次／32秒，圖存在 [discrete_baseline_20260909.png](舊視覺數據_20260919分析/discrete_baseline_20260909.png)。
  **但這份資料是9天前的，不確定跟現在的韌體版本、相機設定完全一致，不能直接當成「離散版 vs PID版」比較用的正式 baseline**，
  真正要用來跟教授報告改善幅度的數據，還是要等車子正常運作後，用今天講好的 `TEST_NAME=xxx python3 realtime_camera_trt_distance_uart_nodisplay.py`
  重新測，這樣才能保證跟未來 PID 版本是同一套軟硬體條件下比較。
- 靜態校正測試看出的視覺量測特性（之後調PID時的雜訊參考值）：
  球放在50cm時，量到的 `dist_cm` 平均 47-48cm（系統性低估約2-3cm，標準差 <0.5cm，量測穩定）；
  角度量測在左右50cm時標準差約0.3-0.6°，雜訊不大。

## STM32 完整專案已取得大半（2026-09-19，見 [ballpicker_STM32專案/](ballpicker_STM32專案/)）

B 同學給了4包壓縮檔（原本放在他 Windows 電腦 `C:\Users\User\Downloads\ballpicker\`），解壓後整理成
`ballpicker_STM32專案/` 資料夾，結構跟 STM32CubeIDE 標準專案一致：

- `Core/`（`Inc/`、`Src/`、`Startup/`）✓ 完整
- `Drivers/`（CMSIS 含 STM32F446xx device header、STM32F4xx_HAL_Driver）✓ 完整
- `Debug/`（CubeIDE 自動產生的 **Makefile 建置系統**：`makefile`、`sources.mk`、`objects.mk`，還有上次成功編譯的
  `ballpicker.elf`/`.map`）✓ 完整，但 makefile 裡連結腳本寫死 B 電腦的絕對路徑
  `C:\Users\User\Downloads\ballpicker\STM32F446RETX_FLASH.ld`，要換成相對路徑才能在 Jetson 上跑
- `.settings/` ✓（Eclipse偏好設定，非必要但拿到了）

**還缺 4 個專案根目錄檔案**：`ballpicker.ioc`、**`STM32F446RETX_FLASH.ld`（連結腳本，這個是編譯必需，最重要）**、
`.project`、`.cproject`。這些檔案很小（.ld通常幾KB的純文字），麻煩跟 B 同學要這幾個，
在他 `Downloads\ballpicker\` 資料夾最外層應該就看得到（不在 Core/Drivers/Debug 任何子資料夾裡面）。

**確認硬體是 STM32F446RE**（LQFP64），不是之前用時脈反推猜的 F401RE，這裡更正。

### 重大發現：main.c 已經加入「球場邊界」邏輯，跟之前分析的版本不同！

B 同學這份 main.c（1121行，比我們之前存的版本多了一大段）新增了 **`COURT,<狀態>,<方向>,<出界比例>`** 協定
（例如 `COURT,OUT,LEFT,0.8`），這應該是配合 D 同學正在做的球場邊界視覺偵測（[46]號待確認事項提過的那個）：

- 追球邏輯現在**被包在「場地狀態必須是 SAFE」的條件裡面**，不是 SAFE 就不會去追球
- `OUT`（真出界）：強力轉向迴避（65~75rpm，依方向）；`EDGE`（接近邊界）：溫和轉向迴避（45~55rpm）
- 找球用的原地旋轉邏輯也改了：從「一次性轉一下」變成「明確 1.5 秒計時、時間到自動煞停」的 `search_spin_active` 機制
- 盲衝逾時從 1500ms 改成 1000ms（在其中一處）

**這代表**：舊的 `main.c` 副本、9/9 那份 CSV 分析、目前 PLAN.md 前面幾節對「離散轉向邏輯」的描述，
都是基於**沒有邊界邏輯的舊版本**。現在的 [ballpicker_STM32專案/Core/Src/main.c](ballpicker_STM32專案/Core/Src/main.c)
才是目前真正的（或接近目前的）韌體邏輯，之後分析行為、設計PID，要以這份為準。舊的獨立 `main.c` 檔案已刪除，
避免兩份文件同時存在造成混淆。

**待確認**：Jetson 端視覺程式現在有沒有真的送出 `COURT,...` 這個新格式？如果還沒有（D同學邊界偵測還在開發中），
代表 `court_state` 會一直維持初始值 `"SAFE"`，追球邏輯實際上等於沒被邊界功能影響，現在球場測試應該還是舊行為。

## 2026-09-20 實測總結（重要，之後接續工作先看這裡）

**筆電端已經是完整可用的燒錄工作站**：`gcc-arm-none-eabi` + `openocd` 裝在 Sean 筆電上，
STM32 接 USB 到筆電就能編譯+燒錄（`cd Debug && make all -j4` 然後 `openocd -f interface/stlink.cfg
-f target/stm32f4x.cfg -c "program ballpicker.elf verify reset exit"`），今天來回燒錄十幾次都順利，
每次都在幾秒內完成。**STM32↔Jetson 之間的 USB 線需要手動切換**（不能同時接兩邊），
切換後 Jetson 上的裝置名稱可能從 `/dev/ttyACM0` 變成 `/dev/ttyACM1`（不固定），
每次切換後記得檢查 `ls /dev/ttyACM*` 並確認 `realtime_camera_trt_distance_uart_nodisplay.py`
裡的 `UART_PORT` 設定對不對。

**目前 STM32 上燒的是「安全測試版」，不是 B 同學原本要的完整版本**：
- 只保留原地轉向（B原本的離散兩檔邏輯），**完全禁用前進/盲衝**，避免暴衝撞牆
- 轉向轉速從 40/55rpm **降到 4/5.5rpm**（除以10）
- 之後要恢復完整功能，要記得把這兩處改回來，不要忘記現在是簡化版

**關鍵發現1：靜摩擦力/PWM太弱，導致原本40/55rpm轉向完全不會動**——降到4/5.5rpm後底盤馬達確認會動了，
Sean判斷**這個轉速適合拿來做接下來的連續平滑轉向PID**，可以當作起始參考值。

**關鍵發現2：YOLO誤判成穩定的假陽性，導致車子不管有沒有真的球都會持續慢慢右轉**——
這跟Sean說的「YOLO本來就會誤抓其他亮綠色物體」這個已知問題吻合，而且能解釋今天稍早
run4資料裡「連續191秒球的位置完全不變」的怪現象（其實是誤判成一個固定不動的背景物體，
不是真的球）。**這是視覺端問題，不是STM32邏輯或接線問題**——STM32收到什麼角度就照做，
沒辦法分辨是真球還是誤判。之後D同學處理視覺時，這個誤判問題值得一起看。

**新問題（今天稍早才發現）：滾輪馬達突然不轉了**——滾輪馬達的韌體邏輯今天完全沒有被改過
（初始化時設定死的PWM值，不受任何BALL/NOBALL邏輯影響），所以懷疑是硬體/接線問題，
不是我今天的修改造成的，需要肉眼檢查滾輪馬達那條線路。

**視覺端偵測率普遍偏低（今天多次測試都在0.3%~35%之間跳動，正常應該要接近100%持續追蹤同一顆球)**，
懷疑是新加的球場邊界色彩分類模型佔用資源，把主要球偵測的FPS從原本量到的~9Hz拖到只剩~4Hz，
這個之後也值得跟D同學討論要不要先關掉邊界偵測、把資源留給球偵測。

## 待確認事項（需要 Sean 回答，才能往下走）

1. **[Sean 表示不想問同學，交給 Claude 自己想辦法]** STM32 那邊除了 `main.c`，有沒有完整 STM32CubeIDE 專案
   （`Core/`、`Drivers/`、`.ioc`、連結腳本、Makefile）？→ **處理方式**：先檢查 Jetson 上、以及當初燒錄用的那台電腦
   （不管是誰的筆電或教室電腦）上有沒有殘留專案資料夾；如果真的完全找不到，退回方案是由 Claude 重新用
   STM32CubeMX 對應的標準檔案重建一個可編譯專案 —— 從 `main.c` 的時脈設定（HSI, PLLM=16, PLLN=336, PLLP=4 → 84MHz）
   加上 `B1_Pin`/`LD2_Pin` 命名慣例，高信心判斷這片板子是 **NUCLEO-F401RE**，週邊清單（TIM1/TIM4 encoder、
   TIM2/TIM3 PWM、TIM6 base+IT、USART2）也已經從程式碼讀出來了，重建專案是可行的，不需要問同學。
2. ~~Jetson 目前的系統版本~~ → **已確認：Ubuntu 18.04**（這代表新版 Node.js 相容性風險是真的存在，但反正 Claude Code
   決定跑在筆電端 SSH 遙控，這台 Jetson 完全不用裝 Node/Claude Code，這個風險已經不用擔心了）
3. ~~Jetson 現在有沒有 WiFi 網卡或接網路線？~~ → **已確認：有看到網路線，但幾乎確定沒開過 SSH**（重灌過，裡面基本是空的）
   → 這代表教室那趟或 USB 直連那趟，很可能需要先完成 Jetson 的「第一次開機設定精靈」（建立帳號密碼），
   這步驟過去很可能已經在某次重灌後做過一次（Jetson 開機精靈需要螢幕才能過），代表帳號密碼應該已經存在，
   只是 Sean 不一定記得 → 這個帳密还是要 Sean 自己想辦法找到/想起來，Claude 沒辦法用猜的
4. **[Sean 表示不想問同學，Claude 自己判斷]** 燒錄用的 ST-Link 是內建還是獨立？→ **高信心判斷：Nucleo 板內建 ST-Link**。
   證據：`/dev/ttyACM0`（Nucleo ST-Link 的 USB 虛擬序列埠特徵）+ `B1_Pin`/`LD2_Pin` 命名 + 84MHz 時脈剛好是
   CubeMX 對 F401RE 的預設配置。且**現在 Jetson↔STM32 那條 USB 線，很可能同一條就能拿來燒錄**
   （Nucleo 的 ST-Link 一條 USB 線同時提供虛擬序列埠跟 SWD 燒錄介面），不用另外找線或拆線。
   之後接上就能用 `lsusb` 看到 `STMicroelectronics ST-LINK` 字樣來雙重確認。
5. **[Sean 表示不想問同學]** 同學說的「70rpm/49rpm」是哪個版本測的？→ **不追究，直接用 PLAN 裡第9點的方法重新實測**，
   不管舊數字怎麼來的，反正都要重新校正編碼器常數，舊數字不影響現在的判斷。
6. ~~視覺程式版本~~ → **已解決（2026-09-19，靠 `~/.bash_history` 執行紀錄找到，不是靠檔名日期猜的）**：
   真正、重複跑最多次、目前實際在用的是 **`~/models/realtime_camera_trt_distance_uart_nodisplay.py`**，
   已經拉進 repo 存檔（跟 `main.c` 一樣）。特徵：
   - 送 `BALL,<距離>,<角度>` / `BALL_OFF`，跟 `main.c` 協定相容 ✓
   - `UART_PORT = "/dev/ttyACM0"`，跟 STM32 實際接的埠一致 ✓（已確認 `/dev/ttyACM0` 目前存在，STM32 有接著）
   - **沒有 5Hz 節流**（之前以為的 `UART_SEND_INTERVAL` 節流是舊版行為，這份是每一幀偵測到就送，
     送出頻率取決於推論速度，實際頻率待之後量測）
   - `CONF_THRES = 0.50`（比舊版 0.35 嚴格）
   - 沒有 `cv2.imshow`/`waitKey`，**純 SSH 就能跑，不需要任何螢幕/X顯示**
   - 已經內建 CSV 記錄（`frame, time, cx, cy, ..., dist_cm, bearing_deg, smooth_bearing_deg, motor_on` 等欄位），
     **這份就是角度/X座標-時間資料收集工具，不需要另外寫程式**，跑的時候設定 `TEST_NAME` 環境變數
     決定輸出檔名，例如：`TEST_NAME=baseline_discrete_run1 python3 realtime_camera_trt_distance_uart_nodisplay.py`
   - 之前 repo 裡誤判為 baseline 的 `realtime_camera_trt_distance_angle_uart.py`（7/16版）仍保留在 repo 中，
     但**不是實際在用的版本**，之後若要修改視覺端邏輯，應該以 `_nodisplay.py` 這份為準

## 版本紀錄

- 2026-09-18：初版。由 Claude 讀取 `main.c` 分析後與 Sean 討論撰寫，`main.c` 從
  `USER CODE BEGIN Header.txt`（原始檔名，STM32CubeIDE 匯出時貼錯檔名）改名而來。
- 2026-09-18（同日更新）：加入 Jetson 視覺程式 `realtime_camera_trt_distance_angle_uart.py` 分析、
  Jetson USB 裝置模式免螢幕連線方案，並整理待確認事項的處理方式（Sean 不想問同學，改由 Claude 自行判斷/驗證）。
