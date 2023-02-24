import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import cv2
import glob
import argparse
import numpy as np
from utils.calibrate import find_chessboard_corners
from orbbec_mag_coordinates_transform import orbbec_to_mag, load_joint_parameter


def affine_transform():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sample_path', type=str, help='sampling data path')
    parser.add_argument('--checker_board', nargs='+', type=int, help='checker board size')
    parser.add_argument('--tran_matrix', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/tran_matrix.txt"), help='transform matrix saved dir')
    parser.add_argument('--calibration_matrix', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter"), help='calibration matrix saved dir')
    parser.add_argument('--fix_matrix_dir', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/fix_matrix.txt"), help='fix matrix saved dir')
    args = parser.parse_args()
    sample_path = args.sample_path
    checker_board = args.checker_board
    tran_matrix = args.tran_matrix
    calibration_matrix = args.calibration_matrix
    fix_matrix_dir = args.fix_matrix_dir

    orbbec_img_dir_lst = glob.glob(os.path.join(sample_path, "*orbbec_rgb.jpg"))
    mag_img_dir_lst = glob.glob(os.path.join(sample_path, "*MAG_rgb.jpg"))
    depth_dir_lst = glob.glob(os.path.join(sample_path, "*depth.pkl"))
    tran_matrix = np.loadtxt(tran_matrix)
    K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = load_joint_parameter(calibration_matrix)
    source = np.asarray([])
    target = np.asarray([])
    first = True
    for idx, orbbec_img_name in enumerate(orbbec_img_dir_lst):
        img_depth = cv2.imread(orbbec_img_name)
        gray = cv2.cvtColor(img_depth, cv2.COLOR_BGR2GRAY)
        ret_depth, corners_depth = find_chessboard_corners(gray, checker_board)
        if ret_depth:
            img_temp = cv2.imread(mag_img_dir_lst[idx])
            gray = cv2.cvtColor(img_temp, cv2.COLOR_BGR2GRAY)
            ret_temp, corners_temp = find_chessboard_corners(gray, checker_board)
            if ret_temp:
                corners_depth = np.squeeze(corners_depth)
                corners_temp = np.squeeze(corners_temp)
                if first:
                    first = False
                    source = orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, corners_depth, depth_dir_lst[idx], tran_matrix=tran_matrix)
                    target = corners_temp
                else:
                    source = np.vstack((source, orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, corners_depth, depth_dir_lst[idx], tran_matrix=tran_matrix)))
                    target = np.vstack((target, corners_temp))
    fix_matrix = cv2.estimateAffine2D(source, target, False)
    np.savetxt(fix_matrix_dir, fix_matrix[0])


# python utils/affine_transform.py --sample_path D:\data\hand_camera\1676959393 --checker_board 6 9
if __name__ == '__main__':
    affine_transform()
