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

## 需要用清大圖書館下載 (被擋)
- Galceran, E., Carreras, M. (2013). "A survey on coverage path planning for robotics."
  Robotics and Autonomous Systems 61(12), 1258-1276. DOI: 10.1016/j.robot.2013.09.004
  -> 全覆蓋路徑規劃的總整理，包含「條帶寬度由感測器覆蓋範圍決定」的通則
     (sensor footprint / 條帶重疊)，以及 boustrophedon 在有障礙物時的分格方式。

## 沒找到的
- 「網格邊長 = 2 × 0.8 × R × sin θ」(R 相機辨識距離、θ 相機半視角、0.8 安全係數)
  這個公式的出處沒有搜到 (英文/中文關鍵字都試過，很可能是 CNKI 的中文碩士論文)。
  公式本身可以直接推導：相機在距離 R 處看到的橫向半寬 = R sin θ，兩側合起來 2R sin θ，
  乘 0.8 讓相鄰格子的視野重疊、不漏球。目前程式 (grid_waypoints.py) 用
  R=3.0 m (vision_node 55px² 門檻)、θ=40° (URDF 水平視角 80°) -> 格邊長上限 3.085 m。
