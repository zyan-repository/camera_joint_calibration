# coding: utf-8
# 读取奥比中广 astraS数据， 参考https://developer.orbbec.com.cn/technical_library.html?id=50

import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import cv2
import time
import copy
import pickle
import argparse
import numpy as np

from openni import openni2
from openni import _openni2 as c_api

from mag_camera.MagDevice import MagDevice
from mag_camera.live_ir import Infrared
from mag_camera.live_vis import Visible
from utils.calibrate import find_chessboard_corners


def read_orbbec_mag():
    """
    同时读奥比中光深度相机和巨哥科技温度相机，对比角点检测算法精度，红点表示检测出的角点，绿点表示通过深度相机rgb转换出的温度相机rgb角点
    采集内容和对比图保存在指定路径
    奥比中光为Astra系列usb摄像头，巨哥科技MAG-AI系列MAG24AI网络摄像头
    参考https://developer.orbbec.com.cn/forum_plate_module_details.html?id=13476
    :return:
    """
    def parse_args():
        """PARAMETERS"""
        parser = argparse.ArgumentParser()
        parser.add_argument('--orbbec_lib', type=str, help='orbbec camera sdk lib path')
        parser.add_argument('--mag_ip', type=str, help='MAG camera ip')
        parser.add_argument('--save_dir', type=str, default='./', help='sampling data saved dir')
        parser.add_argument('--checker_board', nargs='+', type=int, help='checker board size')
        parser.add_argument('--width_depth', type=int, default=640, help='resolutionX')
        parser.add_argument('--height_depth', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps_depth', type=int, default=30, help='frame per second')
        parser.add_argument('--width', type=int, default=640, help='resolutionX')
        parser.add_argument('--height', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps', type=int, default=30, help='frame per second')
        parser.add_argument('--mirroring', default=False)
        parser.add_argument('--candid', type=bool, default=False, help='whether candid image or save sustaining images')
        parser.add_argument('--save_interval_fps', type=int, default=5, help='default save rate FPS')
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

    args = parse_args()
    checker_board = tuple(args.checker_board)
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

    n = 0
    dir_name = os.path.join(args.save_dir, str(time.time()).split('.')[0])
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
    while True:
        n += 1
        depth_raw, depth_uint8 = get_depth(depth_stream)
        ret, frame = cap.read()
        # rendered_orbbec_rgb = copy.deepcopy(frame)
        # try:
        #     gray = cv2.cvtColor(rendered_orbbec_rgb, cv2.COLOR_BGR2GRAY)
        #     # 寻找棋盘格上的亚像素角点
        #     ret, corners = find_chessboard_corners(gray, checker_board)
        #     corners = np.around(corners, 0).astype(np.int64)
        #     tran_points = []
        #     for corner in corners:
        #         x, y = corner.ravel()
        #         tran_points.append((x, y))
        #         cv2.circle(rendered_orbbec_rgb, (x, y), 15, (0, 0, 255), -1)
        # except Exception as e:
        #     print("奥比中光rgb寻找角点，转换到巨哥科技rgb失败。")
        #     print(e)
        #     pass
        color_uint8 = frame
        ir_img = infrared.get_frame(0.1)
        try:
            # cv2.imshow('mag_ir', ir_img)
            vis_img = visible.get_frame()
            # rendered_mag_rgb = copy.deepcopy(vis_img)
            # try:
            #     gray = cv2.cvtColor(rendered_mag_rgb, cv2.COLOR_BGR2GRAY)
            #     # 寻找棋盘格上的亚像素角点
            #     ret, corners = find_chessboard_corners(gray, checker_board)
            #     corners = np.around(corners, 0).astype(np.int64)
            #     for corner in corners:
            #         x, y = corner.ravel()
            #         cv2.circle(rendered_mag_rgb, (x, y), 15, (0, 0, 255), -1)
            # except Exception as e:
            #     print("巨哥科技rgb寻找角点，绘制到巨哥科技rgb失败。")
            #     print(e)
            #     pass
            print("Farthest depth: %s m" % (depth_raw.max() / 1000))
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)

            tt = str(time.time())
            if args.candid:
                if cv2.waitKey(1) == ord('k'):
                    print("单张抓取")
                    name = tt.split('.')[0] + '.' + tt.split('.')[1][:2]  # 必须截取时间戳，否则linux在翻阅时长文件名顺序错乱
                    print("\ts key detected. Saving image and distance map {}".format(name))
                    cv2.imwrite("%s/%s_%s_orbbec_rgb.jpg" % (dir_name, name, n), color_uint8)
                    cv2.imwrite("%s/%s_%s_orbbec_depth.jpg" % (dir_name, name, n), depth_uint8)
                    pickle.dump(depth_raw, open("%s/%s_%s_orbbec_depth.pkl" % (dir_name, name, n), 'wb'))
                    # cv2.imwrite("%s/%s_%s_orbbec_rendered_rgb.jpg" % (dir_name, name, n), rendered_orbbec_rgb)

                    if ir_img is not None:
                        ir_outdir1 = "%s/%s_%s_MAG_ir_vis.jpg" % (dir_name, name, n)
                        ir_outdir2 = "%s/%s_%s_MAG_ir_data.jpg" % (dir_name, name, n)
                        device.SaveDDT(ir_outdir2)
                        cv2.imwrite(ir_outdir1, ir_img)
                    if vis_img is not None:
                        cv2.imwrite("%s/%s_%s_MAG_rgb.jpg" % (dir_name, name, n), vis_img)
                        # cv2.imwrite("%s/%s_%s_MAG_rendered_rgb.jpg" % (dir_name, name, n), rendered_mag_rgb)
                    print("保存完成")
                if cv2.waitKey(1) == ord('q'):
                    # 关闭窗口 和 相机
                    depth_stream.stop()
                    cv2.destroyAllWindows()
                    ans = str(input('保留此次采集？输入n则不保留，默认保留 %s \n' % dir_name))
                    if 'N' in ans.upper():
                        os.system('rm %s -r' % dir_name)
                        print('已删除')
                    else:
                        print(ans)
                    break

            else:
                # 键盘监听
                if cv2.waitKey(1) == ord('q'):
                    # 关闭窗口 和 相机
                    depth_stream.stop()
                    cv2.destroyAllWindows()
                    ans = str(input('保留此次采集？输入n则不保留，默认保留 %s \n' % dir_name))
                    if 'N' in ans.upper():
                        os.system('rm %s -r' % dir_name)
                        print('已删除')
                    else:
                        print(ans)
                    break
                if n % int(args.save_interval_fps) == 0:  # 每隔N帧存一次
                    # if not os.path.exists(dir_name):
                    # 	os.mkdir(dir_name)
                    name = tt.split('.')[0] + '.' + tt.split('.')[1][:2]  # 必须截取时间戳，否则linux在翻阅时长文件名顺序错乱
                    print("\ts key detected. Saving image and distance map {}".format(name))
                    cv2.imwrite("%s/%s_%s_orbbec_rgb.jpg" % (dir_name, name, n), color_uint8)
                    cv2.imwrite("%s/%s_%s_orbbec_depth.jpg" % (dir_name, name, n), depth_uint8)
                    pickle.dump(depth_raw, open("%s/%s_%s_orbbec_depth.pkl" % (dir_name, name, n), 'wb'))
                    cv2.imwrite("%s/%s_%s_orbbec_rendered_rgb.jpg" % (dir_name, name, n), rendered_orbbec_rgb)
                    if ir_img is not None:
                        ir_outdir1 = "%s/%s_%s_MAG_ir_vis.jpg" % (dir_name, name, n)
                        ir_outdir2 = "%s/%s_%s_MAG_ir_data.jpg" % (dir_name, name, n)
                        device.SaveDDT(ir_outdir2)
                        cv2.imwrite(ir_outdir1, ir_img)
                    if vis_img is not None:
                        cv2.imwrite("%s/%s_%s_MAG_rgb.jpg" % (dir_name, name, n), vis_img)
                        cv2.imwrite("%s/%s_%s_MAG_rendered_rgb.jpg" % (dir_name, name, n), rendered_mag_rgb)
            resize_ir_img = cv2.resize(ir_img, (640, 480))
            vis_img_reize = cv2.resize(rendered_mag_rgb, (640, 480))
            concat_orbbec = np.hstack((cv2.resize(rendered_orbbec_rgb, (640, 480)), depth_uint8))
            concat_MAG = np.hstack((vis_img_reize, resize_ir_img))
            # concat_MAG = np.hstack((vis_img, ir_img))
            concat_uint8 = np.vstack((concat_orbbec, concat_MAG))
            cv2.imshow('concat', concat_uint8)
        except:
            continue

    # 检测设备是否关闭（没什么用）
    try:
        openni2.unload()
        print("Device unloaded \n")
    except Exception as ex:
        print("Device not unloaded: ", ex, "\n")

    visible.stop()
    infrared.stop()


# python sample/joint_calibration_sample.py --orbbec_lib C:\Users\38698\work_space\OpenNI\Win64-Release\sdk\libs --mag_ip 10.100.24.60 --save_dir D:\data\hand_camera --checker_board 6 9
if __name__ == "__main__":
    read_orbbec_mag()
