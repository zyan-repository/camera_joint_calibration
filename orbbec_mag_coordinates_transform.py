import os
import pickle
import cv2
import numpy as np
# from orbbec_camera.orbec_astra_Sample import load_data, get_world_coordinate


def load_joint_parameter(parameter_dir):
    K1 = np.load(os.path.join(parameter_dir, 'K1.npy'))
    D1 = np.load(os.path.join(parameter_dir, 'D1.npy'))
    rvec1 = np.load(os.path.join(parameter_dir, 'rvec1.npy'))
    R1 = np.load(os.path.join(parameter_dir, 'R1.npy'))
    T1 = np.load(os.path.join(parameter_dir, 'T1.npy'))
    K2 = np.load(os.path.join(parameter_dir, 'K2.npy'))
    D2 = np.load(os.path.join(parameter_dir, 'D2.npy'))
    rvec2 = np.load(os.path.join(parameter_dir, 'rvec2.npy'))
    R2 = np.load(os.path.join(parameter_dir, 'R2.npy'))
    T2 = np.load(os.path.join(parameter_dir, 'T2.npy'))
    return K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2


def orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, orbbec_pixel_coordinates, depth_data, use_sdk=False, lib_path=""):
    """
    奥比中光rgb图上像素坐标对应到巨哥科技rgb图上像素坐标，奥比中光rgb分辨率640*480，巨哥科技rgb分辨率1920*1080
    :param K1: 奥比中光深度相机内参矩阵
    :param R1: 奥比中光深度相机旋转矩阵
    :param T1: 奥比中光深度相机平移向量
    :param K2: 巨哥科技测温相机内参矩阵
    :param D2: 巨哥科技测温相机畸变系数
    :param rvec2: 巨哥科技测温相机旋转向量
    :param T2: 巨哥科技测温相机平移向量
    :param orbbec_pixel_coordinates: 奥比中光深度相机rgb图上像素坐标
    :param depth_file: 奥比中光深度相机rgb图对应的深度pickle文件，字符串或数组
    :param use_sdk: 是否使用奥比中光sdk，偏差较大，默认为False
    :param lib_path: 奥比中光openni sdk路径,路径到/sdk/libs
    :param verbose: 是否输出额外信息
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
        Zc = depth_data[orbbec_pixel_coordinates[:, 1], orbbec_pixel_coordinates[:, 0]]
        pixel_matrix = np.hstack((orbbec_pixel_coordinates, np.ones(orbbec_pixel_coordinates.shape[0]).reshape(3, 1))).T
        # pixel coordinate to world coordinate
        obj_points = (Zc * np.linalg.inv(R1).dot(np.linalg.inv(K1).dot(pixel_matrix)) - np.linalg.inv(R1).dot(T1)).T
    image_points, _ = cv2.projectPoints(obj_points, rvec2, T2, K2, D2)
    return np.around(image_points.squeeze(), 0).astype(np.int64)


if __name__ == '__main__':
    K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = load_joint_parameter("./joint_parameter")

    depth_data = r"C:\Users\38698\work_space\data\hand_camera\1675135288_pig_0_0_xw_white_small_stand\0_0_xw_white_small_stand_120_orbbec_depth.pkl"
    depth_data = pickle.load(open(depth_data, 'rb'))
    mag_pixel_coordinate = orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, [(301, 260), (366, 405), (327, 294)], depth_data)
    print(mag_pixel_coordinate)
