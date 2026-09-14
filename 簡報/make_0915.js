// 產生 撿球機9_15.pptx (白底黑字，基本版)。用法: cd 簡報 && node make_0915.js
// 實驗數據頁讀 data_0915.json (summarize_runs.py 產生)，沒有的話放待補字樣。
const pptxgen = require('pptxgenjs');
const fs = require('fs');
const path = require('path');

const pres = new pptxgen();
pres.layout = 'LAYOUT_16x9'; // 10 x 5.625 in
const FONT = 'Arial';
const BLACK = '000000';
const GRAY = '555555';
const IMG = p => path.join(__dirname, 'img', p);

function title(slide, text) {
  slide.addText(text, { x: 0.5, y: 0.3, w: 9, h: 0.7, fontFace: FONT, fontSize: 28, bold: true, color: BLACK, isTextBox: true, margin: 0 });
}
function bullets(slide, items, opt = {}) {
  const runs = items.map((t, i) => {
    const o = { text: typeof t === 'string' ? t : t.text, options: { bullet: true, breakLine: i < items.length - 1, paraSpaceAfter: 6 } };
    if (typeof t !== 'string' && t.indent) o.options.indentLevel = 1;
    if (typeof t !== 'string' && t.bold) o.options.bold = true;
    return o;
  });
  slide.addText(runs, Object.assign({ x: 0.5, y: 1.1, w: 9, h: 4.2, fontFace: FONT, fontSize: 15, color: BLACK, valign: 'top', isTextBox: true, margin: 0 }, opt));
}
function note(slide, text, opt = {}) {
  slide.addText(text, Object.assign({ x: 0.5, y: 5.0, w: 9, h: 0.4, fontFace: FONT, fontSize: 11, color: GRAY, isTextBox: true, margin: 0 }, opt));
}
function table(slide, rows, opt = {}) {
  const data = rows.map((r, i) => r.map(c => ({ text: String(c), options: { fontFace: FONT, fontSize: opt.fontSize || 12, bold: i === 0, color: BLACK, align: 'center', valign: 'middle' } })));
  slide.addTable(data, Object.assign({ x: 0.5, y: 1.1, w: 9, border: { type: 'solid', color: '888888', pt: 0.75 }, colW: opt.colW, rowH: opt.rowH || 0.32 }, opt.pos || {}));
}

// ---------- 1 封面 ----------
{
  const s = pres.addSlide();
  s.addText('撿球機', { x: 0.5, y: 1.8, w: 9, h: 1.2, fontFace: FONT, fontSize: 48, bold: true, color: BLACK, align: 'center', isTextBox: true });
  s.addText('9/15', { x: 0.5, y: 3.0, w: 9, h: 0.7, fontFace: FONT, fontSize: 28, color: GRAY, align: 'center', isTextBox: true });
}

// ---------- 2 進度總覽 ----------
{
  const s = pres.addSlide();
  title(s, '7/8 至今的進度總覽 (模擬端)');
  bullets(s, [
    '定位方案：光達 + AMCL 地圖 → 輪速編碼器 + 陀螺儀里程計 (教授建議；學長的光達是舊型室內用)',
    '弓字形巡邏 + 追球狀態機：格邊長由相機辨識距離與視角決定',
    '里程計誤差消除：找出 4 個疊加的原因，位置誤差 1.1 m → 0.03 m，航向 11° → 0.2°',
    '陀螺儀取樣：模擬 50 Hz → 1000 Hz 的原因與現實 50 Hz 的差別',
    '撿球機構整合：滾輪真的把球拋進後車廂 (不再用「碰到就消除」)',
    '場地：改成開放式網球場 (無牆) + 真實尺寸網子 (擋視線、不可通過)',
    '實驗數據：隨機 10 / 15 / 20 顆球各 5 次',
  ]);
}

// ---------- 3 定位方案改變 ----------
{
  const s = pres.addSlide();
  title(s, '為什麼從光達 + AMCL 改成陀螺儀里程計');
  bullets(s, [
    '7/8 的方案 C：先驗地圖 + AMCL 動態導航 + Nav2 (Smac2D + DWB)',
    '改變原因：',
    { text: '教授建議：網球場是開放、幾乎沒有特徵的空間，AMCL 靠雷射特徵比對校正，在空曠場地校正效果有限', indent: true },
    { text: '學長留下的 RPLIDAR A2M8 是室內用舊型光達，戶外陽光下與 12 m 級的場地尺度不適用', indent: true },
    { text: '實體車本來就有馬達編碼器 + IMU (BNO080)，不用再多一顆感測器', indent: true },
    '新方案：航位推算 (dead reckoning)',
    { text: '前進速度 = 輪半徑 × 左右輪角速度平均 (編碼器)；航向 = 陀螺儀三軸角速度積分', indent: true },
    { text: '位置 x, y 用當下航向積分；不用 /odom、不用光達、不用地圖', indent: true },
    { text: '模擬優勢：Gazebo 可以直接讀真實位置，拿來跟里程計比對，量化誤差', indent: true },
  ], { fontSize: 14 });
}

// ---------- 4 弓字形巡邏 + 追球邏輯 ----------
{
  const s = pres.addSlide();
  title(s, '弓字形巡邏 + 追球狀態機');
  bullets(s, [
    'PATROL：沿格子中心點走弓字形 (蛇形)，0.4 m/s，到點容差 0.3 m',
    'ALIGN：相機看到球 → 原地轉，P 控制把球對到畫面中央 (±5°)',
    'APPROACH：0.3 m/s 前進，邊走邊用像素誤差 PID 修正方向',
    'BLIND_DASH：球進相機死角後盲衝 0.3 m/s × 2 s，讓滾輪把球撈進去',
    '撿完 (球確認在後車廂) 直接回到巡邏路徑，繼續往下一格',
    '停止條件：球撿滿 / 走完路徑 / 出界 (真實座標)',
  ], { x: 0.5, y: 1.1, w: 4.6, h: 4.2, fontSize: 13 });
  s.addImage({ path: IMG('run86_net.png'), x: 5.2, y: 1.1, w: 4.5, h: 3.4 });
  note(s, '圖：run86，藍=PATROL、紅=APPROACH，虛線=里程計以為的位置 (幾乎完全重疊)', { x: 5.2, y: 4.55, w: 4.5, h: 0.5 });
}

// ---------- 5 格邊長決定 ----------
{
  const s = pres.addSlide();
  title(s, '格邊長怎麼決定');
  bullets(s, [
    { text: '格邊長上限 = 2 × 0.8 × R × sin θ', bold: true },
    { text: 'R = 相機能穩定辨識球的距離 = 3.0 m (視覺節點 55 px² 面積門檻的理論距離)', indent: true },
    { text: 'θ = 相機水平視角的一半 = 40° (視角 80°)', indent: true },
    { text: '0.8 = 安全係數：相鄰格子的視野重疊 20%，不漏球', indent: true },
    { text: '→ 上限 3.085 m；格數 = 場地邊長 ÷ 上限 無條件進位', indent: true },
    '開放場地 24 × 11 m：8 × 4 = 32 格 (每格 3.00 × 2.75 m)',
    '有網子時每個半場 (11.1 × 11 m)：4 × 4 = 16 格 (每格 2.775 × 2.75 m)，兩個半場共 32 格',
    '車走在格子中心線上，左右各看得到 0.8·R·sinθ = 1.54 m，剛好蓋滿半格 (1.5 m 或 1.39 m)',
    '文獻：Choset & Pignon (1998) Boustrophedon Cellular Decomposition；Galceran & Carreras (2013) CPP survey',
    '公式本身的原始出處待補 (見備註)',
  ], { fontSize: 13 });
  s.addNotes('格邊長公式 2×0.8×R×sinθ 的原始文獻還沒找到，目前是幾何推導：相機在距離 R 看到的橫向半寬 R sinθ、兩側 2R sinθ，乘 0.8 讓相鄰格視野重疊。');
}

// ---------- 6 里程計誤差消除 ----------
{
  const s = pres.addSlide();
  title(s, '里程計誤差消除：四個疊加的原因');
  table(s, [
    ['#', '原因', '證據', '修法', '效果'],
    ['1', 'dt 用真實時鐘，但模擬即時率 0.93', '每次 run 的 (真實/模擬時間) 都等於 (里程計/真實距離)，如 1.103 vs 1.105', 'dt 改用 /joint_states 封包的模擬時間戳 (實體車：用編碼器封包時間戳)', '距離比例 1.05 → 0.98，航向 11° → 3°'],
    ['2', '真實座標比錯點：里程計算的是輪軸中點，Gazebo 回報 base_link', '實測兩點差 (0.08, −0.135) m，原地轉時假差 0.31 m', '真實座標換算到輪軸中點', '距離比例 0.979 → 1.001'],
    ['3', '記錄時間差：兩個 timer 分開記，最多差 0.5 s', '車在動時看起來像誤差', '真實座標一到就同步記錄', '位置誤差 0.33 → 0.22 m'],
    ['4', '陀螺儀 50 Hz 取樣不夠', 'ODE 每步角速度抖動 ±8%，50 Hz 矩形積分每轉一段隨機差 ±0.5°', 'IMU 1000 Hz、在 IMU callback 用封包時間戳積分', '位置 0.026 m、航向 0.17°'],
  ], { fontSize: 10, colW: [0.3, 2.0, 2.6, 2.4, 1.7], rowH: 0.62 });
  note(s, '修正後 (run74)：93.7 m 路徑，位置誤差平均 0.026 m / 最大 0.078 m，航向平均 0.17° / 最大 0.34°', { y: 4.55 });
}

// ---------- 7 before / after 圖 ----------
{
  const s = pres.addSlide();
  title(s, '里程計 修正前 vs 修正後');
  s.addImage({ path: IMG('run69_before.png'), x: 0.3, y: 1.1, w: 4.7, h: 3.55 });
  s.addImage({ path: IMG('run74_after.png'), x: 5.1, y: 1.1, w: 4.7, h: 3.55 });
  note(s, '左：run69 (修正前) 平均 1.13 m / 最大 3.17 m / 航向 11.7°', { x: 0.3, y: 4.7, w: 4.7, h: 0.4 });
  note(s, '右：run74 (修正後) 平均 0.026 m / 最大 0.078 m / 航向 0.17°，虛線被實線蓋住', { x: 5.1, y: 4.7, w: 4.7, h: 0.4 });
}

// ---------- 8 決定性證據 RTF 0.7 ----------
{
  const s = pres.addSlide();
  title(s, '單變數驗證：把模擬即時率硬壓到 0.7');
  bullets(s, [
    '同一份程式、同一佈局，只把 Gazebo 物理更新率從 1000 降到 700 (即時率 0.700)',
    '同一次 run 裡並排記錄兩條里程計：',
    { text: '新版 (封包時間戳 dt)：位置誤差平均 0.029 m / 最大 0.070 m，航向 0.15°', indent: true },
    { text: '舊版 (真實時鐘 dt)：路徑長 ×1.431 (=1/0.7)，位置誤差平均 18 m，航向 113°，直接飛出球場', indent: true },
    '結論：之前 7~10% 的距離與航向誤差主因就是 dt 來源，不是摩擦、碰撞或質量',
  ], { x: 0.5, y: 1.1, w: 4.4, h: 4.0, fontSize: 13 });
  s.addImage({ path: IMG('run75_rtf07.png'), x: 5.0, y: 1.1, w: 4.7, h: 3.55 });
  note(s, 'run75：黑虛線 (新) 貼著實線；洋紅虛線 (舊) 出界', { x: 5.0, y: 4.7, w: 4.7, h: 0.4 });
}

// ---------- 9 陀螺儀取樣率 ----------
{
  const s = pres.addSlide();
  title(s, '陀螺儀取樣率：模擬 50 → 1000 Hz，現實 50 Hz 的意義');
  bullets(s, [
    '模擬裡發現的問題：Gazebo IMU 讀值跟真實角速度逐筆完全一致 (零誤差、零延遲)，但物理引擎每 1 ms 一步算出來的角速度本身抖動 ±8% (接觸求解雜訊)，姿態卻是平滑的',
    '50 Hz 只取到 1/20 的步，矩形積分抖動訊號，每轉一段隨機差 ±0.2~0.5°；乘上 40 m 直線就是 0.5~0.9 m 位置誤差',
    '改成 1000 Hz (= 物理步) 後每一步都算到：直走 0.003°、原地轉 246° 差 0.22°',
    '這是模擬的數值雜訊，現實的車體角速度是連續平滑的，50 Hz 取樣不會有這種抖動問題',
    '現實 50 Hz 要注意的反而是：(1) 用 IMU 封包自己的時間戳積分、每筆剛好算一次；(2) 三軸都要積分 (車體傾斜時只積 z 軸會少算，模擬實測少 5%)；(3) 真實陀螺儀有偏移與雜訊，模擬還沒加',
    '模擬與現實的對應：模擬 1000 Hz 是為了把積分誤差壓到只剩「演算法本身」，好單獨驗證路徑與撿球邏輯',
  ], { fontSize: 13 });
}

// ---------- 10 三軸積分 + 其他修正 ----------
{
  const s = pres.addSlide();
  title(s, '航向積分改三軸四元數');
  bullets(s, [
    '症狀：原地轉向 (ALIGN) 時偶爾一秒內少算 1~3° (實測真實轉 23°、陀螺儀 z 軸積分 21.8°)',
    '原因：車體一傾斜 (滾輪擦地、被撞)，世界座標的航向變化率 = ω_z·cosφ + ω_y·sinφ + …，只積 z 軸會少算 cosφ',
    '修法：三軸角速度做四元數積分 q ← q ⊗ exp(ω·dt/2)，再從 q 取 yaw',
    '單元檢查：先傾斜 20° 再繞世界 z 轉 90°，只積 z 軸得 84.6°，四元數得 90.0°',
    '效果：有網子的完整巡邏 (100 m 路徑) 航向誤差 5.3° → 0.1~0.6°，位置 1.1 m → 2~5 cm',
    '實體車 BNO080 本來就是三軸，直接套用同一套積分',
  ], { fontSize: 14 });
}

// ---------- 11 滾輪撿球整合 ----------
{
  const s = pres.addSlide();
  title(s, '撿球機構整合：滾輪真的把球拋進後車廂');
  bullets(s, [
    '之前為了專心做路徑，撿球是「滾輪碰到球就把球消除」；現在改回物理撿球',
    '關鍵：滾輪位置。5/21 影片能成功的設定 (5/13 筆記) 是滾輪在 x=0.2、z=0.03；後來改回 CAD 位置 (x=0.10)，離後車廂 17.7 cm 前擋牆只剩 10 cm，球被推到牆邊只能垂直彈起，永遠進不了車廂 (已跟設計組說明)',
    '滾輪常轉，轉速掃描 (右表)：W=40~45 (62~70 rad/s) 在盲衝 0.2~0.4 m/s 全部進車廂；≥50 拋過頭飛出車尾；採用 W=42.5 (約 66 rad/s)',
    '「撿到」判定改用真實座標：球躺在車廂地板高度 (z 0.05~0.15) 且連續 3 次都在車廂內；結束時稽核每顆球的位置',
  ], { x: 0.5, y: 1.1, w: 5.4, h: 4.2, fontSize: 12 });
  table(s, [
    ['W', 'rad/s', 'v=0.2', 'v=0.3', 'v=0.4'],
    ['35', '54', '進', '未過牆', '未過牆'],
    ['40', '62', '進', '進', '進'],
    ['45', '70', '進', '進', '進'],
    ['50', '78', '飛出', '飛出', '飛出'],
    ['55', '86', '—', '飛出', '飛出'],
  ], { fontSize: 11, colW: [0.6, 0.7, 0.75, 0.75, 0.75], rowH: 0.3, pos: { x: 6.1, y: 1.1, w: 3.55 } });
  s.addImage({ path: IMG('video47_pickup.jpg'), x: 6.1, y: 3.15, w: 3.55, h: 1.1 });
  note(s, '5/21 影片：7.5 s 球到滾輪，7.75 s 飛過車廂前緣', { x: 6.1, y: 4.3, w: 3.55, h: 0.4 });
}

// ---------- 12 撿球結果 ----------
{
  const s = pres.addSlide();
  title(s, '物理撿球 + 巡邏：完整測試');
  bullets(s, [
    '標準 9 顆球佈局：run76、run77 皆 9/9 進後車廂，結束稽核 9 顆全部躺在車廂地板 (一顆疊在第二層)',
    '後車廂載球對里程計的影響 (實測)：位置誤差 0.026 → 0.056~0.078 m，航向 0.17° → 0.33~0.63°；在前 60 秒追球期間累積後持平，撿球瞬間本身沒有跳動',
    '設計組：後車廂可放 20 顆；本次最多測 20 顆',
    '注意：滾輪轉速 ≥ 90 rad/s 時球卡住會讓滾輪關節在物理引擎裡失控，轉速要保持在 66 rad/s 附近',
  ], { x: 0.5, y: 1.1, w: 4.6, h: 4.2, fontSize: 13 });
  s.addImage({ path: IMG('run77_pickup.png'), x: 5.2, y: 1.1, w: 4.5, h: 3.4 });
  note(s, 'run77：9/9，位置誤差平均 0.078 m', { x: 5.2, y: 4.55, w: 4.5, h: 0.4 });
}

// ---------- 13 開放式網球場 ----------
{
  const s = pres.addSlide();
  title(s, '場地改成開放式網球場 (無牆)');
  bullets(s, [
    '現實場地是開放的：Gazebo 裡的牆只留半透明視覺當邊界，碰撞全部拿掉',
    '「撞牆停止」改成「出界停止」：真實座標離開場地 + 邊界外緣就停 (模擬才有的判定)',
    '巡邏範圍改成整個 24 × 11 m 雙打場地 (不再內縮 1 m)，格數由格邊長公式算：8 × 4 = 32 格',
    '隨機 9 顆球 × 3 次 (run78~80)：3/3 皆 9/9，稽核全部在車廂',
  ], { x: 0.5, y: 1.1, w: 4.6, h: 4.2, fontSize: 13 });
  s.addImage({ path: IMG('run80_open.png'), x: 5.2, y: 1.1, w: 4.5, h: 3.4 });
  note(s, 'run80：開放場地隨機佈局 9/9', { x: 5.2, y: 4.55, w: 4.5, h: 0.4 });
}

// ---------- 14 網子 ----------
{
  const s = pres.addSlide();
  title(s, '真實網子：擋視線、不可通過');
  bullets(s, [
    'ITF 規格：中央 0.914 m、網柱 1.07 m，網柱在雙打邊線外 0.914 m → 全長 12.8 m (網柱 y = ±6.4)',
    '模擬：x=0 處 1 m 高的深灰實心板 + 兩根網柱，有碰撞；從 15 cm 高的相機看過去完全擋住另一半場 (對應 YOLO 隔網辨識不可靠)',
    '路徑：Choset boustrophedon cellular decomposition 的最簡單特例——網子把場地切成兩個矩形，各自走弓字 (每半場 4 × 4 格，格子從離網 0.9 m 開始)，第一半場走完繞網柱外側 (y=±7.5) 到第二半場靠網的角落接上',
    '追球時的網子保護：車體中心離網 < 0.763 m (車身 0.5 m + 車頭 0.263 m) 又朝向網子就放棄這顆球，5 秒不理視覺',
    '隨機 9 顆球 (離網 ≥ 1 m) × 3 次 (run85~87)：3/3 皆 9/9，稽核全在車廂，零次觸發放棄；里程計位置誤差 2~5 cm、航向 ≤ 0.6°',
  ], { x: 0.5, y: 1.1, w: 5.0, h: 4.2, fontSize: 12 });
  s.addImage({ path: IMG('run86_net.png'), x: 5.6, y: 1.1, w: 4.1, h: 3.1 });
  note(s, 'run86：兩個半場 + 繞網柱', { x: 5.6, y: 4.25, w: 4.1, h: 0.4 });
}

// ---------- 15 實驗數據 ----------
{
  const s = pres.addSlide();
  title(s, '實驗數據：隨機 10 / 15 / 20 顆球 × 5 次');
  const dataPath = path.join(__dirname, 'data_0915.json');
  if (fs.existsSync(dataPath)) {
    const d = JSON.parse(fs.readFileSync(dataPath, 'utf8'));
    const rows = [['球數', '成功次數', '平均撿完時間 (s)', '平均每顆 (s)', '平均路徑 (m)', '里程計位置誤差 平均 (m)', '最終偏移 (m)', '航向誤差 平均 (°)']];
    for (const g of d.groups) rows.push([g.balls, `${g.success}/${g.runs}`, g.time, g.sec_per_ball, g.path, g.pos_err, g.pos_final, g.yaw_err]);
    table(s, rows, { fontSize: 11, colW: [0.8, 1.0, 1.4, 1.1, 1.2, 1.4, 1.0, 1.1], rowH: 0.4 });
    s.addChart(pres.ChartType.bar, [
      { name: '平均撿完時間 (s)', labels: d.groups.map(g => `${g.balls} 顆`), values: d.groups.map(g => g.time) },
    ], { x: 0.5, y: 2.9, w: 4.4, h: 2.3, barDir: 'col', chartColors: ['444444'], showValue: true, dataLabelPosition: 'outEnd', dataLabelFontSize: 10,
         showLegend: false, showTitle: true, title: '平均撿完時間 (s)', titleFontSize: 12, catAxisLabelFontSize: 11, valAxisLabelFontSize: 10, valGridLine: { color: 'DDDDDD', size: 0.5 }, catGridLine: { style: 'none' } });
    s.addChart(pres.ChartType.bar, [
      { name: '里程計最終偏移 (m)', labels: d.groups.map(g => `${g.balls} 顆`), values: d.groups.map(g => g.pos_final) },
    ], { x: 5.2, y: 2.9, w: 4.4, h: 2.3, barDir: 'col', chartColors: ['444444'], showValue: true, dataLabelPosition: 'outEnd', dataLabelFontSize: 10,
         showLegend: false, showTitle: true, title: '里程計最終偏移 (m)', titleFontSize: 12, catAxisLabelFontSize: 11, valAxisLabelFontSize: 10, valGridLine: { color: 'DDDDDD', size: 0.5 }, catGridLine: { style: 'none' } });
    if (d.note) note(s, d.note, { y: 5.2, h: 0.35 });
  } else {
    bullets(s, ['(數據跑完後由 summarize_runs.py 產生 data_0915.json 再重新產生本頁)']);
  }
}

// ---------- 16 每次 run 明細 ----------
{
  const dataPath = path.join(__dirname, 'data_0915.json');
  if (fs.existsSync(dataPath)) {
    const d = JSON.parse(fs.readFileSync(dataPath, 'utf8'));
    const s = pres.addSlide();
    title(s, '每次 run 明細');
    const rows = [['球數', 'run', '撿到', '撿完時間 (s)', '路徑 (m)', '位置誤差 平均/最大 (m)', '最終偏移 (m)', '航向 平均/最大 (°)']];
    for (const r of d.runs) rows.push([r.balls, r.run, `${r.picked}/${r.balls}`, r.time, r.path, `${r.pos_mean} / ${r.pos_max}`, r.pos_final, `${r.yaw_mean} / ${r.yaw_max}`]);
    table(s, rows, { fontSize: 9, colW: [0.6, 0.8, 0.7, 1.1, 0.9, 1.9, 1.0, 1.6], rowH: 0.24 });
  }
}

// ---------- 17 已知限制 / 後續 ----------
{
  const s = pres.addSlide();
  title(s, '已知限制與後續');
  bullets(s, [
    '模擬與實體的差異：陀螺儀還是理想 sensor (沒有偏移、雜訊)；滾輪模型位置與 CAD 不同 (設計組已知)；前車體無碰撞',
    '實體車要對應的三件事：編碼器/IMU 封包時間戳、三軸陀螺儀四元數積分、滾輪轉速約 66 rad/s',
    '格邊長公式的文獻出處待補',
    '後續：加入陀螺儀偏移/雜訊模型驗證現實可行性；多顆球載重下的行為；與實體車比對',
  ], { fontSize: 14 });
}

const out = path.join(__dirname, '撿球機9_15.pptx');
pres.writeFile({ fileName: out }).then(() => console.log('wrote', out));
