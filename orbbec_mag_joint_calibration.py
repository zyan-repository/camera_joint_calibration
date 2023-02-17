import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import argparse
import glob
import cv2
import numpy as np
from utils.calibrate import get_obj_points, cal_internal_monocular, cal_outside_image_monocular
from orbbec_camera.orbbec_astra_Sample import load_data, get_world_coordinate
from orbbec_mag_coordinates_transform import load_joint_parameter


# 寻找亚像素焦点
def find_chessboard_corners(gray, checker_board):
    """
    寻找标定板交点，在此基础上寻找亚像素焦点优化。
    :param gray: 需要求角点的灰度图
    :param checker_board: 角点的横纵数量，格式为元组
    :return: 角点寻找是否成功与像素角点， shape:[num, 2]
    """
    ret, corners = cv2.findChessboardCorners(gray, checker_board, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)
    if ret:
        temp_corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
        if [temp_corners]:
            corners = temp_corners
    return ret, corners


def cal_world_coordinates(img_list, checker_board, square_size, img_dir_lst, lib_path, camera_type, obj_orbbec=None, corner_success=[], tran_matrix=None):
    """
    计算内角点真实世界坐标，调用奥比中光sdk获得真实世界坐标，获得的世界坐标是基于奥比中光建立的世界坐标系
    如果深度相机没有对应sdk，应根据深度文件加外部测量人工测量世界坐标，单位一般是毫米
    """
    import copy
    # 存储3D点
    obj_points = []
    if camera_type == 'orbbec':
        corner_success = []
    cnt_mag = 0

    # 遍历图片列表
    for idx, img_ in enumerate(img_list):
        # 将图片转为灰度图
        gray = cv2.cvtColor(img_, cv2.COLOR_BGR2GRAY)
        # 寻找棋盘格上的亚像素角点
        ret, corners = find_chessboard_corners(gray, checker_board)
        depth_name = img_dir_lst[idx].replace("rgb.jpg", "depth.pkl")
        # 寻找成功
        if ret:
            if camera_type == 'orbbec':
                corners_int = np.around(copy.deepcopy(corners).reshape(1, checker_board[0] * checker_board[1], 2), 0).astype(np.int64)
                depth_stream, depth_data = load_data("", depth_name, lib_path)
                obj_p = get_obj_points(checker_board, square_size)
                flag = False
                for i, coordinate in enumerate(corners_int[0]):
                    if tran_matrix is not None:
                        coordinate = np.hstack((coordinate, 1)).reshape(1, 3)
                        coordinate = np.squeeze(np.around(coordinate.dot(tran_matrix.T), 0).astype(np.int64))
                    x_w, y_w, z_w = get_world_coordinate(depth_stream, depth_data, coordinate)
                    if x_w is None:
                        flag = True
                        break
                    obj_p[0, i, 0] = x_w
                    obj_p[0, i, 1] = y_w
                    obj_p[0, i, 2] = z_w
                if flag:
                    corner_success.append(False)
                    continue
                corner_success.append(True)
                # 将世界坐标添加到世界坐标列表中
                obj_points.append(obj_p)
            else:
                if corner_success[idx]:
                    # 将世界坐标添加到世界坐标列表中
                    obj_points.append(obj_orbbec[cnt_mag])
                    cnt_mag += 1
        else:
            if camera_type == 'orbbec':
                corner_success.append(False)
            else:
                if corner_success[idx]:
                    corner_success[idx] = False
                    cnt_mag += 1
    return obj_points, corner_success


def joint_calibration():
    """
    寻找标定板交点，在此基础上寻找亚像素焦点优化。
    :return: 奥比中光: 内参矩阵K1，畸变系数D1，旋转向量rvec1，旋转矩阵R1，平移向量T1 巨哥科技: 内参矩阵K2，畸变系数D2，旋转向量rvec2，旋转矩阵R2，平移向量T2
    """
    def parse_args():
        """
        :param checker_board: 棋盘格内角点数，格式为元组
        :param square_size: 棋盘方格真实长宽，格式为元组，单位毫米
        :param sample_path: 采集数据保存地址，采集数据可用sample/joint_calibration_sample.py脚本，
                            尽量选择其中角点检测成功且清晰的数据提高标定质量（不筛选也行，但可能影响结果精度），
                            名字带rendered画出了角点，红色是检测的，绿色是根据默认参数匹配的，标定后参数会自动更新
        :param lib_path: 奥比中光openni sdk路径,路径到/sdk/libs
        :param tran_matrix: 奥比中光rgb对齐到深度图像的变换矩阵文件路径
        """
        parser = argparse.ArgumentParser()
        parser.add_argument('--checker_board', nargs='+', type=int, help='checker board size')
        parser.add_argument('--square_size', nargs='+', type=int, help='checker board square real size')
        parser.add_argument('--lib_path', type=str, help='orbbec sdk lib path')
        parser.add_argument('--sample_path', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "sample"), help='sampling data saved dir')
        parser.add_argument('--tran_matrix', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/tran_matrix.npy"), help='transform matrix saved dir')
        return parser.parse_args()

    args = parse_args()
    checker_board = tuple(args.checker_board)
    square_size = tuple(args.square_size)
    sample_path = args.sample_path
    lib_path = args.lib_path
    tran_matrix = np.load(args.tran_matrix)
    orbbec_img_dir_lst = glob.glob(os.path.join(sample_path, "*orbbec_rgb.jpg"))
    mag_img_dir_lst = glob.glob(os.path.join(sample_path, "*MAG_rgb.jpg"))
    # calibrate orbbec
    orbbec_img_list = []
    for img_dir in orbbec_img_dir_lst:
        orbbec_img_list.append(cv2.imread(img_dir))
    obj_p = get_obj_points(checker_board, square_size)
    ret, K1, D1 = cal_internal_monocular(obj_p, orbbec_img_list, checker_board)

    # calibrate magnity
    mag_img_list = []
    for img_dir in mag_img_dir_lst:
        mag_img_list.append(cv2.imread(img_dir))
    ret, K2, D2 = cal_internal_monocular(obj_p, mag_img_list, checker_board)

    # joint calibrate outside
    obj_points, corner_success = cal_world_coordinates(orbbec_img_list, checker_board, square_size, orbbec_img_dir_lst, lib_path, camera_type='orbbec', tran_matrix=tran_matrix)
    img = []
    for idx, img_dir in enumerate(orbbec_img_dir_lst):
        if corner_success[idx]:
            img.append(cv2.imread(img_dir))
    ret, rvec1, R1, T1 = cal_outside_image_monocular(obj_points, img, checker_board, K1, D1)
    obj_points, corner_success = cal_world_coordinates(mag_img_list, checker_board, square_size, mag_img_dir_lst, lib_path, camera_type='mag', obj_orbbec=obj_points, corner_success=corner_success, tran_matrix=tran_matrix)
    img = []
    for idx, img_dir in enumerate(mag_img_dir_lst):
        if corner_success[idx]:
            img.append(cv2.imread(img_dir))

    ret, rvec2, R2, T2 = cal_outside_image_monocular(obj_points, img, checker_board, K2, D2)
    if not os.path.exists(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter")):
        os.makedirs(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter"))
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/K1.npy"), K1)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/D1.npy"), D1)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/rvec1.npy"), rvec1)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/R1.npy"), R1)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/T1.npy"), T1)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/K2.npy"), K2)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/D2.npy"), D2)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/rvec2.npy"), rvec2)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/R2.npy"), R2)
    np.save(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/T2.npy"), T2)

    return K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2


# python orbbec_mag_joint_calibration.py --checker_board 6 9 --square_size 28 28 --lib_path C:\Users\38698\work_space\OpenNI\Win64-Release\sdk\libs --sample_path D:\data\hand_camera\1676431921 --tran_matrix C:\Users\38698\work_space\camera_joint_calibration\joint_parameter\tran_matrix.npy
if __name__ == '__main__':
    # calibration
    K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = joint_calibration()

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
