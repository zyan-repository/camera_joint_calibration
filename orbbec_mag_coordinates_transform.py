import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import pickle
import cv2
import numpy as np
# from orbbec_camera.orbec_astra_Sample import load_data, get_world_coordinate


def load_joint_parameter(parameter_dir):
    K1 = np.loadtxt(os.path.join(parameter_dir, 'K1.txt'))
    D1 = np.loadtxt(os.path.join(parameter_dir, 'D1.txt')).reshape(1, -1)
    rvec1 = np.loadtxt(os.path.join(parameter_dir, 'rvec1.txt')).reshape(-1, 1)
    R1 = np.loadtxt(os.path.join(parameter_dir, 'R1.txt'))
    T1 = np.loadtxt(os.path.join(parameter_dir, 'T1.txt')).reshape(-1, 1)
    K2 = np.loadtxt(os.path.join(parameter_dir, 'K2.txt'))
    D2 = np.loadtxt(os.path.join(parameter_dir, 'D2.txt')).reshape(1, -1)
    rvec2 = np.loadtxt(os.path.join(parameter_dir, 'rvec2.txt')).reshape(-1, 1)
    R2 = np.loadtxt(os.path.join(parameter_dir, 'R2.txt'))
    T2 = np.loadtxt(os.path.join(parameter_dir, 'T2.txt')).reshape(-1, 1)
    return K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2


def orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, orbbec_pixel_coordinates, depth_data, use_sdk=False, lib_path="", tran_matrix=None):
    """
    奥比中光rgb图上像素坐标对应到巨哥科技rgb图上像素坐标，奥比中光rgb分辨率640*480，巨哥科技rgb分辨率1920*1080
    :param K1: 奥比中光深度相机内参矩阵
    :param R1: 奥比中光深度相机旋转矩阵
    :param T1: 奥比中光深度相机平移向量
    :param K2: 巨哥科技测温相机内参矩阵
    :param D2: 巨哥科技测温相机畸变系数
    :param rvec2: 巨哥科技测温相机旋转向量
    :param T2: 巨哥科技测温相机平移向量
    :param orbbec_pixel_coordinates: 奥比中光深度相机rgb图上像素坐标 shape(n, 2)
    :param depth_file: 奥比中光深度相机rgb图对应的深度pickle文件，字符串或数组
    :param use_sdk: 是否使用奥比中光sdk，偏差较大，默认为False
    :param lib_path: 奥比中光openni sdk路径,路径到/sdk/libs
    :param tran_matrix: 奥比中光rgb对齐到深度图像的变换矩阵
    :return: 巨哥科技测温相机rgb图中对应的像素坐标
    """
    orbbec_pixel_coordinates = np.asarray(orbbec_pixel_coordinates)
    if orbbec_pixel_coordinates.size == 0:
        return np.asarray([])
    if use_sdk:
        from openni import openni2
        from openni import _openni2 as c_api
        if isinstance(depth_data, str):
            depth_stream, depth_data = load_data("", depth_data, lib_path)
        else:
            openni2.initialize(os.path.join(os.path.dirname(__file__), lib_path))
            device = openni2.Device.open_any()
            depth_stream = device.create_depth_stream()
            depth_stream.set_video_mode(c_api.OniVideoMode(resolutionX=640, resolutionY=480, fps=30, pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM))  # 必须保证和采集时严格一致
        obj_points = []
        for point in orbbec_pixel_coordinates:
            x_w, y_w, z_w = get_world_coordinate(depth_stream, depth_data, point)
            obj_point = np.asarray([x_w, y_w, z_w])
            obj_points.append(obj_point)
    else:
        if isinstance(depth_data, str):
            depth_data = pickle.load(open(depth_data, 'rb'))
        if tran_matrix is not None:
            orbbec_depth_coordinates = np.hstack((orbbec_pixel_coordinates.reshape(-1, 2), np.ones(orbbec_pixel_coordinates.shape[0]).reshape(-1, 1)))
            orbbec_depth_coordinates = np.around(orbbec_depth_coordinates.dot(tran_matrix.T), 0).astype(np.int64)
            Zc = depth_data[orbbec_depth_coordinates[:, 1], orbbec_depth_coordinates[:, 0]]
        else:
            Zc = depth_data[orbbec_pixel_coordinates[:, 1], orbbec_pixel_coordinates[:, 0]]
        pixel_matrix = np.hstack((orbbec_pixel_coordinates, np.ones(orbbec_pixel_coordinates.shape[0]).reshape(-1, 1))).T
        Zc[:] = Zc * 0.75
        # pixel coordinate to world coordinate
        obj_points = (Zc * np.linalg.inv(R1).dot(np.linalg.inv(K1).dot(pixel_matrix)) - np.linalg.inv(R1).dot(T1)).T
    image_points, _ = cv2.projectPoints(obj_points, rvec2, T2, K2, D2)
    return np.around(image_points.squeeze(), 0).astype(np.int64)


if __name__ == '__main__':
    K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = load_joint_parameter(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter"))

    depth_data = r"D:\data\orbbec_mag_rgb_calibration\1676534300\1676534384.22_448_orbbec_depth.pkl"
    depth_data = pickle.load(open(depth_data, 'rb'))
    tran_matrix = np.loadtxt(os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/tran_matrix.txt"))
    mag_pixel_coordinate = orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, [(336, 108), (100, 100)], depth_data, tran_matrix=tran_matrix)
    print(mag_pixel_coordinate)
