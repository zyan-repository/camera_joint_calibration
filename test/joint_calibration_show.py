# coding: utf-8
# 读取奥比中广 astraS数据， 参考https://developer.orbbec.com.cn/technical_library.html?id=50

import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import cv2
import argparse
import numpy as np

from openni import openni2
from openni import _openni2 as c_api

from mag_camera.MagDevice import MagDevice
from mag_camera.live_ir import Infrared
from mag_camera.live_vis import Visible
from utils.calibrate import find_chessboard_corners
from orbbec_mag_coordinates_transform import orbbec_to_mag, load_joint_parameter


def read_orbbec_mag():
    """
    同时读奥比中光深度相机和巨哥科技温度相机，对比角点检测算法精度，红点表示检测出的角点，绿点表示通过深度相机rgb转换出的温度相机rgb角点
    奥比中光为Astra系列usb摄像头，巨哥科技MAG-AI系列MAG24AI网络摄像头
    参考https://developer.orbbec.com.cn/forum_plate_module_details.html?id=13476
    :return:
    """
    def parse_args():
        """PARAMETERS"""
        parser = argparse.ArgumentParser()
        parser.add_argument('--orbbec_lib', type=str, help='orbbec camera sdk lib path')
        parser.add_argument('--mag_ip', type=str, help='MAG camera ip')
        parser.add_argument('--checker_board', nargs='+', type=int, help='checker board size')
        parser.add_argument('--width_depth', type=int, default=640, help='resolutionX')
        parser.add_argument('--height_depth', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps_depth', type=int, default=30, help='frame per second')
        parser.add_argument('--width', type=int, default=640, help='resolutionX')
        parser.add_argument('--height', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps', type=int, default=30, help='frame per second')
        parser.add_argument('--mirroring', default=False)
        parser.add_argument('--tran_matrix', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter/tran_matrix.txt"), help='transform matrix saved dir')
        parser.add_argument('--calibration_matrix', type=str, default=os.path.join(PROJECT_ABSOLUTE_PATH, "joint_parameter"), help='calibration matrix saved dir')
        parser.add_argument('--single_image', type=bool, default=False, help='test single image')
        parser.add_argument('--single_path', type=str, default="None", help='test single image dir')
        return parser.parse_args()

    def get_orbbec(libpath):
        # 记载 openni
        try:
            print("library path is: ", os.path.join(os.path.dirname(__file__), libpath))
            openni2.initialize(os.path.join(os.path.dirname(__file__), libpath))
            print("OpenNI2 initialized \n")
        except Exception as ex:
            print("ERROR OpenNI2 not initialized", ex, " check library path..\n")
            return
        # 加载 orbbec 相机
        device = openni2.Device.open_any()
        return device

    def get_depth(depth_stream):
        """
        Returns numpy ndarrays representing the raw and ranged depth images.
        Outputs:
            dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1  # 不止4096
            d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255
        Note1:
            fromstring is faster than asarray or frombuffer
        Note2:
            .reshape(120,160) #smaller image for faster response
                    OMAP/ARM default video configuration
            .reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
                    Requires .set_video_mode
        """
        dmap = np.fromstring(depth_stream.read_frame().get_buffer_as_uint16(), dtype=np.uint16).reshape(
            args.height_depth, args.width_depth)  # Works & It's FAST
        dmap_float = dmap.astype(float)
        dmap_int = dmap.astype(np.int16)
        assert dmap_int.mean() == dmap_float.mean(), '警告一致性失败'
        d4d = np.uint8(dmap_float * 255 / dmap_float.max())  # Correct the range. Depth images are 12bits
        d4d = 255 - cv2.cvtColor(d4d, cv2.COLOR_GRAY2RGB)
        return dmap_int, d4d

    def img_cat(img1, img2, img3, img4):
        img1 = cv2.resize(img1, (640, 480))
        img2 = cv2.resize(img2, (640, 480))
        img3 = cv2.resize(img3, (640, 480))
        img4 = cv2.resize(img4, (640, 480))
        img12 = np.hstack((img1, img2))
        img34 = np.hstack((img3, img4))
        return np.vstack((img12, img34))

    args = parse_args()
    checker_board = tuple(args.checker_board)
    single_image = args.single_image
    single_path = args.single_path
    if not single_image:
        device = get_orbbec(args.orbbec_lib)
        print(device.get_device_info())
        # 创建流
        depth_stream = device.create_depth_stream()
        infra_stream = device.create_ir_stream()
        depth_stream.set_mirroring_enabled(args.mirroring)
        infra_stream.set_mirroring_enabled(args.mirroring)
        depth_stream.set_video_mode(
            c_api.OniVideoMode(resolutionX=args.width_depth, resolutionY=args.height_depth, fps=args.fps_depth,
                               pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM)
        )
        # 设置 镜像 帧同步
        device.set_image_registration_mode(True)
        device.set_depth_color_sync_enabled(True)
        device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
        depth_stream.start()
        # color_stream.start()
        # infra_stream.start()    # 硬件上infra和rgb不能同时工作

        cap = cv2.VideoCapture(0)
        cap.set(3, 1920)
        cap.set(4, 1080)
    ip = args.mag_ip
    device = MagDevice()
    # camera_info = device.GetCamInfoEx()
    infrared = Infrared(device)
    infrared.start(ip)
    visible = Visible(device)
    visible.start(ip)
    calibration_matrix = args.calibration_matrix
    K1, D1, rvec1, R1, T1, K2, D2, rvec2, R2, T2 = load_joint_parameter(calibration_matrix)
    tran_matrix = np.loadtxt(args.tran_matrix)
    while True:
        if single_image:
            import pickle
            depth_raw = pickle.load(open(single_path + '_orbbec_depth.pkl', 'rb'))
            depth_uint8 = cv2.imread(single_path + '_orbbec_depth.jpg')
            frame = cv2.imread(single_path + '_orbbec_rgb.jpg')
            ir_img = cv2.imread(single_path + '_MAG_ir_vis.jpg')
            vis_img = cv2.imread(single_path + '_MAG_rgb.jpg')
        else:
            depth_raw, depth_uint8 = get_depth(depth_stream)
            ret, frame = cap.read()
            if not ret:
                continue
            ir_img = infrared.get_frame(0.1)
            vis_img = visible.get_frame()
        mag_pixel_coordinates = []
        print("Farthest depth: %s m" % (depth_raw.max() / 1000))
        if depth_raw is None or frame is None or ir_img is None or vis_img is None:
            continue
        if depth_raw.size == 0 or frame.size == 0 or ir_img.size == 0 or vis_img.size == 0:
            continue
        tran_success = False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 寻找棋盘格上的亚像素角点
        ret, corners = find_chessboard_corners(gray, checker_board)
        if ret:
            tran_success = True
            corners = np.around(corners, 0).astype(np.int64)
            tran_points = []
            depth_points = []
            for corner in corners:
                x, y = corner.ravel()
                tran_points.append((x, y))
                cv2.circle(frame, (x, y), 15, (0, 0, 255), -1)
                depth_point = np.squeeze(np.around(np.hstack((np.asarray([x, y]), 1)).reshape(1, 3).dot(tran_matrix.T), 0).astype(np.int64))
                cv2.circle(depth_uint8, depth_point, 5, (0, 0, 255), -1)
                depth_points.append(depth_point)
            mag_pixel_coordinates = orbbec_to_mag(K1, R1, T1, K2, D2, rvec2, T2, tran_points, depth_raw, tran_matrix=tran_matrix)
            # for idx, p in enumerate(depth_points):
            #     print("%d号内角点，深度是%d毫米" % (idx, depth_raw[p[1], p[0]]))
        gray = cv2.cvtColor(vis_img, cv2.COLOR_BGR2GRAY)
        flag_bias = True
        if tran_success:
            for idx, point in enumerate(mag_pixel_coordinates):
                try:
                    point = np.around(point, 0).astype(np.int64)
                    cv2.circle(vis_img, point, 5, (0, 255, 0), -1)
                    x, y, _ = device.ConvertVisCorr2IrCorr(point[0], 1080 - point[1], depth_raw[depth_points[idx][1], depth_points[idx][0]])
                    # x, y, _ = device.ConvertVisCorr2IrCorr(point[0], 1080 - point[1], 0)
                    cv2.circle(ir_img, np.asarray([x.value, 240 - y.value]).astype(np.int64), 3, (0, 255, 0), -1)
                except Exception as e:
                    flag_bias = False
                    print("通过奥比中光转换出的角点坐标，在巨哥科技rgb上绘制失败。")
                    print(e)
        # 寻找棋盘格上的亚像素角点
        ret, corners = find_chessboard_corners(gray, checker_board)
        if ret:
            corners = np.around(corners, 0).astype(np.int64)
            sum_bias = 0
            for idx, corner in enumerate(corners):
                x, y = corner.ravel()
                cv2.circle(vis_img, (x, y), 4, (0, 0, 255), -1)
                if tran_success:
                    if flag_bias:
                        sum_bias += np.linalg.norm(np.asarray([x, y]) - mag_pixel_coordinates[idx])
                    x, y, _ = device.ConvertVisCorr2IrCorr(x, 1080 - y, depth_raw[depth_points[idx][1], depth_points[idx][0]])
                else:
                    x, y, _ = device.ConvertVisCorr2IrCorr(x, 1080 - y, 0)
                cv2.circle(ir_img, np.asarray([x.value, 240 - y.value]).astype(np.int64), 2, (0, 0, 255), -1)
            if flag_bias and tran_success is True:
                print("误差：", np.around(sum_bias / (checker_board[0] * checker_board[1]), 2))
        cv2.imshow('concat', img_cat(frame, depth_uint8, vis_img, ir_img))
        if single_image:
            cv2.waitKey(0)
            break
        else:
            cv2.waitKey(1)

    # 检测设备是否关闭（没什么用）
    try:
        openni2.unload()
        print("Device unloaded \n")
    except Exception as ex:
        print("Device not unloaded: ", ex, "\n")

    visible.stop()
    infrared.stop()


# python test/joint_calibration_show.py --orbbec_lib C:\Users\38698\work_space\OpenNI\Win64-Release\sdk\libs --mag_ip 10.100.24.60 --checker_board 6 9
# python test/joint_calibration_show.py --orbbec_lib C:\Users\38698\work_space\OpenNI\Win64-Release\sdk\libs --mag_ip 10.100.24.60 --checker_board 6 9 --single_image True --single_path D:\data\orbbec_mag_rgb_calibration\test\1676534894.92_4221
if __name__ == "__main__":
    read_orbbec_mag()
