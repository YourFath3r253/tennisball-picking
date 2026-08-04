from PIL import Image, ImageDraw

def create_tennis_court_map():
    # 1. 設定解析度 (Resolution)：每個像素代表 0.05 公尺 (ROS 2 標準)
    resolution = 0.05
    
    # 2. 設定地圖畫布總大小 (保留場外的空間讓車子運作)
    # 我們設定畫布為 34公尺 x 20公尺
    width_m = 34.0
    height_m = 20.0
    
    # 換算成像素大小
    img_width = int(width_m / resolution)   # 34 / 0.05 = 680 px
    img_height = int(height_m / resolution) # 20 / 0.05 = 400 px
    
    # 建立純白畫布 (255代表無障礙物)
    img = Image.new('L', (img_width, img_height), color=255)
    draw = ImageDraw.Draw(img)
    
    # 3. 計算網球場在畫布上的座標
    # 網球場尺寸 24m x 11m
    court_length_m = 24.0
    court_width_m = 11.0
    
    # 將球場放在畫布正中央
    center_x = img_width / 2
    center_y = img_height / 2
    
    # 計算球場邊界的像素座標
    left_px = center_x - (court_length_m / 2 / resolution)
    right_px = center_x + (court_length_m / 2 / resolution)
    top_px = center_y - (court_width_m / 2 / resolution)
    bottom_px = center_y + (court_width_m / 2 / resolution)
    
    # 4. 畫出黑色的牆壁 (0代表致命障礙物)，線寬設為 2 像素
    draw.rectangle([left_px, top_px, right_px, bottom_px], outline=0, width=2)
    
    # 5. 存檔為 PGM 格式 (ROS 2 地圖標準格式)
    img.save('tennis_court_map.pgm')
    print("地圖 tennis_court_map.pgm 生成成功！大小：680x400 pixels")

if __name__ == '__main__':
    create_tennis_court_map()
