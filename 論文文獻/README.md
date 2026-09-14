# 論文文獻

## 已下載 (開放取用)
- `Choset_Pignon_1997_Coverage_Path_Planning_Boustrophedon_Cellular_Decomposition.pdf`
  Choset, H., Pignon, P. (1998). "Coverage Path Planning: The Boustrophedon Cellular Decomposition."
  Field and Service Robotics, Springer. (CMU 公開版)
  -> 弓字形 (boustrophedon) 全覆蓋路徑的原始論文：把有障礙物的區域切成沒有障礙物的
     梯形/矩形 cell，每個 cell 內走弓字，cell 之間用鄰接圖決定走訪順序。
     我們的網球場 = 用網子切成兩個矩形 cell，就是這個方法最簡單的特例。
- `Choset_2000_Coverage_of_Known_Spaces_Boustrophedon_AuRo.pdf`
  Choset, H. (2000). "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition."
  Autonomous Robots 9, 247-253.
  -> 同一方法的期刊版，多了 cell 走訪順序 (深度優先) 的說明。

- `A survey on coverage path planning for robotics.pdf` (Sean 由清大圖書館下載)
  Galceran, E., Carreras, M. (2013). Robotics and Autonomous Systems 61(12), 1258-1276.
  DOI: 10.1016/j.robot.2013.09.004
  -> 全覆蓋路徑規劃的總整理，包含「條帶寬度由感測器覆蓋範圍決定」的通則
     (sensor footprint / 條帶重疊)，以及 boustrophedon 在有障礙物時的分格方式。
- `Cabreira_2019_Survey_CPP_UAV_Drones.pdf` (MDPI 開放取用)
  Cabreira, T. M., Brisolara, L. B., Ferreira Jr., P. R. (2019). "Survey on Coverage Path Planning
  with Unmanned Aerial Vehicles." Drones 3(1), 4. DOI: 10.3390/drones3010004
  -> 第 6 頁 (2.2 節)：「用相機做覆蓋時，格子的大小正比於相機的 footprint (視野投影範圍)，
     格子解析度由影像需求 (解析度、重疊率) 與影像感測器特性決定」；第 9 頁：back-and-forth
     路徑的條帶間距與重疊率由 Di Franco & Buttazzo 的方法計算。
     這是「格邊長 = 視野寬度 × (1 − 重疊率)」最直接的文獻依據：
     視野寬度 = 2·R·sinθ (相機在辨識距離 R 看到的橫向寬度)，重疊率 20% -> 係數 0.8。

## 「格邊長 = 2 × 0.8 × R × sin θ」的出處
- 一模一樣寫法的原始論文沒有搜到 (英文/中文關鍵字、NDLTD、CNKI 方向都試過；台灣博碩士論文
  系統有兩篇崑山科大的「智能網球撿球車」(106KSUT0442010)、「智慧型網球撿球機之開發與設計」
  (108KSUT0442003)，需要登入才能看內容，可能是其中之一)。
- 可引用的依據 (由上面兩篇 survey 組合)：
  1. 條帶/格子寬度 = 感測器 footprint 寬度 × (1 − 重疊率)：Cabreira et al. 2019 (Drones) 2.2 節、
     Galceran & Carreras 2013 (RAS) 的 boustrophedon 段落；原始方法是
     Di Franco, D., Buttazzo, G. (2016). "Coverage Path Planning for UAVs Photogrammetry with
     Energy and Resolution Constraints." J. Intell. Robot. Syst. 83, 445-462.
     DOI: 10.1007/s10846-016-0348-x (Springer，需圖書館下載)。
  2. footprint 寬度 = 2·R·sinθ：相機在辨識距離 R、半視角 θ 時，畫面邊緣能看到的橫向半寬是
     R·sinθ (無人機版本是 2·h·tan(θ) ，因為是垂直往下看；我們是水平往前看、以辨識距離 R 為半徑)。
  3. 0.8 = 重疊率 20% (航拍常用 side overlap 20~30%)。
  目前程式 (grid_waypoints.py)：R=3.0 m (vision_node 55px² 門檻)、θ=40° (URDF 水平視角 80°)
  -> 格邊長上限 3.085 m。
