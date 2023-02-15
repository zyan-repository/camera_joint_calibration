import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import cv2
import numpy as np
from orbbec_mag_coordinates_transform import orbbec_to_mag, load_joint_parameter
from orbbec_mag_joint_calibration import find_chessboard_corners


K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = load_joint_parameter(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter"))

print("K1:", K1)
print("D1:", D1)
print("rvec1:", rvec1)
print("R1:", R1)
print("T1:", T1)
print("K2:", K2)
print("D2:", D2)
print("rvec2:", rvec2)
print("R2:", R2)
print("T2:", T2)

img_dir = r"D:\data\hand_camera\1676431921\1676436689.74_20_"

ori_orbbec_img = img_dir + "orbbec_ori_rgb.jpg"
ori_mag_img = img_dir + "MAG_ori_rgb.jpg"
ori_orbbec_img = cv2.imread(ori_orbbec_img)
gray = cv2.cvtColor(ori_orbbec_img, cv2.COLOR_BGR2GRAY)
# 获取灰度图规格
gray_size = gray.shape[::-1]
# 寻找棋盘格上的亚像素角点
ret, corners = find_chessboard_corners(gray, (6, 9))
corners = np.around(corners, 0).astype(np.int64)
tran_points = []
for corner in corners:
    x, y = corner.ravel()
    tran_points.append((x, y))
mag_pixel_coordinates = orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, tran_points, r"D:\data\hand_camera\1676431921\1676436689.74_20_orbbec_depth.pkl")

print(mag_pixel_coordinates)
