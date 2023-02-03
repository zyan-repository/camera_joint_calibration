# coding: utf-8
# 读取奥比中广 astraS数据， 参考https://developer.orbbec.com.cn/technical_library.html?id=50
from IPython import embed
def read_by_orbbecSDK():
    # 尝试1
    # 1：下载了OrbbecSDK_1.0.1_20220126_c0fef2ad_Release_Python.zip
    # 2：链接了对应py版本的c_lib
    # 3：然后append编译好的对应版本wraper 就可以用了
    # 4：不行，sudo也找不到设备，固件版本问题？ 客服说只有astra+系列可用

    # 安全append
    import sys
    sys.path.append("/home/lmw/leemengwei/hardwares/orbec_astra_s/OrbbecSDK_1.0.1_20220126_c0fef2ad_Release_Python/linux/python3.7/lib/python_lib/")
    import Version
    import Context
    import Device
    import Sensor
    from Error import ObException
    import sys, termios, tty
    try:
        #打印SDK的版本号，SDK版本号分为主版本号，副版本号和修订版本号
        version= Version.Version()
        print( "SDK version: %d.%d.%d" \
            %( version.getMajor(), version.getMinor(), version.getPatch()))

        #创建一个Context，与Pipeline不同，Context是底层API的入口，在开关流等常用操作上
        #使用低级会稍微复杂一些，但是底层API可以提供更多灵活的操作，如获取多个设备，读写
        #设备及相机的属性等
        ctx = Context.Context( None )

        #查询已经接入设备的列表
        devList = ctx.queryDeviceList()

        #获取接入设备的数量
        devCount = devList.deviceCount()
        if devCount == 0:
            print( "Device not found!" )
            sys.exit()

        #创建设备，0表示第一个设备的索引
        dev = devList.getDevice( 0 )

        #获取设备信息
        devInfo = dev.getDeviceInfo()

    except ObException as e:
        print( "function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %( e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus() ) )
        

def read_by_openni():
    '''
    # 尝试2
    参考https://developer.orbbec.com.cn/forum_plate_module_details.html?id=13476
    orbbec sdk找不到设备，转而使用基础openni sdk，可以读到
    注意必须sudo运行否则：AttributeError: 'OniStatus' object has no attribute 'val'
    '''
    from datetime import datetime
    import time
    import argparse
    import sys
    import configparser
    import os
    from openni import openni2
    from openni import _openni2 as c_api
    import cv2
    import numpy as np
    import pickle
    def parse_args():
        '''PARAMETERS'''
        parser = argparse.ArgumentParser()
        parser.add_argument('--width_depth', type=int, default=640, help='resolutionX')
        parser.add_argument('--height_depth', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps_depth', type=int, default=30, help='frame per second')
        parser.add_argument('--width', type=int, default=640, help='resolutionX')
        parser.add_argument('--height', type=int, default=480, help='resolutionY')
        parser.add_argument('--fps', type=int, default=30, help='frame per second')
        parser.add_argument('--mirroring', default=False)
        # parser.add_argument('--pig_dir_name', type=str, required=True, help='name (fatness) to save this pig data')
        parser.add_argument('--pig_dir_name', type=str, default="pigpig", help='name (fatness) to save this pig data')
        parser.add_argument('--save_interval_fps', type=int, default=5, help='default save rate FPS')
        return parser.parse_args()
    def getOrbbec():
        # 记载 openni
        try:
            # if sys.platform == "win32":
            #     libpath = "lib/Windows"
            # else:
                # libpath = "/home/lmw/leemengwei/hardwares/orbec_astra_s/OpenNI_2.3.0.81_202110281040_ff72b9bf_release_linux/sdk/libs"
            libpath = r"D:/third_party/OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release/Win64-Release/sdk/libs"
            print("library path is: ", os.path.join(os.path.dirname(__file__), libpath))
            openni2.initialize(os.path.join(os.path.dirname(__file__), libpath))
            print("OpenNI2 initialized \n")
        except Exception as ex:
            print("ERROR OpenNI2 not initialized", ex, " check library path..\n")
            return
        # 加载 orbbec 相机
        try:
            device = openni2.Device.open_any()
            return device
        except Exception as ex:
            print("ERROR Unable to open the device: \n", ex, " \ndevice disconnected? \n")
            return
    def get_rgb(color_stream):
        """
        Returns numpy 3L ndarray to represent the rgb image.
        """
        bgr = np.fromstring(color_stream.read_frame().get_buffer_as_uint8(), dtype=np.uint8).reshape(args.height, args.width, 3)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return rgb
    def get_infra(infra_stream):
        infra = np.fromstring(infra_stream.read_frame().get_buffer_as_uint8(), dtype=np.uint8).reshape(args.height, args.width)
        infra = cv2.cvtColor(infra, cv2.COLOR_GRAY2RGB)
        return infra
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
        dmap = np.fromstring(depth_stream.read_frame().get_buffer_as_uint16(), dtype=np.uint16).reshape(args.height_depth, args.width_depth)  # Works & It's FAST
        dmap_float = dmap.astype(float)
        dmap_int = dmap.astype(np.int16)
        assert dmap_int.mean() == dmap_float.mean() , '警告一致性失败'
        d4d = np.uint8(dmap_float * 255/ dmap_float.max()) # Correct the range. Depth images are 12bits
        d4d = 255 - cv2.cvtColor(d4d, cv2.COLOR_GRAY2RGB)
        return dmap_int, d4d
    def mask_rgbd(d4d, rgb, dmap, farthest=0):
        """
        Overlays images and uses some blur to slightly smooth the mask
        (3L ndarray, 3L ndarray) -> 3L ndarray
        th:= threshold
        """
        mask = d4d.copy()
        #mask = cv2.GaussianBlur(mask, (5,5),0)
        idx =(dmap > farthest)  # 距离太远了就盖住
        mask[idx] = rgb[idx]
        return mask
    def plot_world_point_cloud(depth_stream, depth_raw, plt):
        '''
        openni2 逐点转换到世界坐标系
        '''
        ax = plt.figure().add_subplot(111, projection='3d')
        for row in list(range(depth_raw.shape[0]))[::10]:
            for col in list(range(depth_raw.shape[1]))[::10]:
                print(row, col, depth_raw[row, col])
                tmp = openni2.convert_depth_to_world(depth_stream, row, col, depth_raw[row, col])
                print(tmp)
                ax.scatter3D(-tmp[1], -tmp[0], -tmp[2], s=1, color='k')
        plt.xlabel('x')
        plt.ylabel('y')
        return
    args = parse_args()
    device = getOrbbec()
    print(device.get_device_info())
    # 创建流
    depth_stream = device.create_depth_stream()
    # color_stream = device.create_color_stream() *****
    infra_stream = device.create_ir_stream()
    depth_stream.set_mirroring_enabled(args.mirroring)
    # color_stream.set_mirroring_enabled(args.mirroring)   *****
    infra_stream.set_mirroring_enabled(args.mirroring)
    depth_stream.set_video_mode(c_api.OniVideoMode(resolutionX=args.width_depth, resolutionY=args.height_depth, fps=args.fps_depth, pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM))
    # color_stream.set_video_mode(c_api.OniVideoMode(resolutionX=args.width, resolutionY=args.height, fps=args.fps, pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888)) *****
    infra_stream.set_video_mode(c_api.OniVideoMode(resolutionX=args.width, resolutionY=args.height, fps=args.fps, pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_GRAY8))
    # 设置 镜像 帧同步
    device.set_image_registration_mode(True)
    device.set_depth_color_sync_enabled(True)
    device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    depth_stream.start()
    # color_stream.start()
    # infra_stream.start()    # 硬件上infra和rgb不能同时工作

    n = 0
    pig_dir_name = r'E:/datasets/depth_data/3D_data/20220927/%s_'%str(time.time()).split('.')[0] + args.pig_dir_name
    cap = cv2.VideoCapture(0)
    while True:
        n += 1
        depth_raw, depth_uint8 = get_depth(depth_stream)
        ret, frame = cap.read()
        # color_uint8 = get_rgb(color_stream) ******
        color_uint8 = frame
        # infra_uint8 = get_infra(infra_stream)
        # rgbd  = mask_rgbd(depth_uint8, color_uint8, depth_raw, farthest=1000)
        concat_uint8 = np.hstack((color_uint8, depth_uint8))
        cv2.imshow('concat', concat_uint8)
        print("Farthest depth: %s m" % (depth_raw.max()/1000))

        # 键盘监听
        if cv2.waitKey(1) == ord('q'):
            # 关闭窗口 和 相机
            depth_stream.stop()
            # color_stream.stop() *****
            cv2.destroyAllWindows()
            ans = str(input('保留此次采集？输入n则不保留，默认保留 %s \n'%pig_dir_name))
            if 'N' in ans.upper():
                os.system('rm %s -r'%pig_dir_name)
                print('已删除')
            else:
                print(ans)
            break
        if n % int(args.save_interval_fps) == 0:   # 每隔N帧存一次
            if not os.path.exists(pig_dir_name):
                os.mkdir(pig_dir_name)
            name = str(time.time())[:13]   # 必须截取时间戳，否则linux在翻阅时长文件名顺序错乱
            print("\ts key detected. Saving image and distance map {}".format(name))
            cv2.imwrite("%s/%s_rgb.jpg"%(pig_dir_name, str(name)), color_uint8)
            pickle.dump(depth_raw, open("%s/%s_depth.pkl"%(pig_dir_name, str(name)), 'wb'))
    # 检测设备是否关闭（没什么用）
    try:
        openni2.unload()
        print("Device unloaded \n")
    except Exception as ex:
        print("Device not unloaded: ", ex, "\n")


def load_data(rgb_file, depth_file, lib_path):
    # rgb_file = "/home/lmw/leemengwei/dataset_others/3D数据/手持移动盘点数据/1646789379_length_73_width_14.5_weight_14.2/1646789384.92_rgb.png"
    # depth_file = "/home/lmw/leemengwei/dataset_others/3D数据/手持移动盘点数据/1646789379_length_73_width_14.5_weight_14.2/1646789384.92_depth.pkl"

    from openni import openni2
    import pickle
    import cv2
    import os, sys
    from openni import _openni2 as c_api

    def getOrbbec():
        # 记载 openni
        try:
            libpath = lib_path
            # print("library path is: ", os.path.join(os.path.dirname(__file__), libpath))
            openni2.initialize(os.path.join(os.path.dirname(__file__), libpath))
            # print("OpenNI2 initialized \n")
        except Exception as ex:
            print("ERROR OpenNI2 not initialized", ex, " check library path..\n")
            return
        # 加载 orbbec 相机
        try:
            device = openni2.Device.open_any()
            return device
        except Exception as ex:
            print("ERROR Unable to open the device: \n", ex, " \ndevice disconnected? \n")
            return
    # 读取数据文件
    # rgb_data = cv2.imread(rgb_file)
    depth_data = pickle.load(open(depth_file, 'rb'))
    # 实例化 deapth_stream
    device = getOrbbec()
    depth_stream = device.create_depth_stream()
    depth_stream.set_video_mode(c_api.OniVideoMode(resolutionX=640, resolutionY=480, fps=30, pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM))  # 必须保证和采集时严格一致
    return depth_stream, depth_data


def get_world_coordinate(depth_stream, depth_data, pixel_coordinate):
    from openni import openni2
    # 转换到世界坐标系
    pixel_x = pixel_coordinate[0]  # 640 -1
    pixel_y = pixel_coordinate[1]  # 480 -1
    if depth_data[pixel_y, pixel_x] == 0:
        return None, None, None
    world_x, world_y, world_z = openni2.convert_depth_to_world(depth_stream, pixel_y, pixel_x, depth_data[pixel_y, pixel_x])
    return world_x, world_y, world_z


if __name__ == "__main__":
    # read_by_orbbecSDK()  # 方式一读摄像头失败
    # read_by_openni()   # 方式二读摄像头ok
    # load_data()

    depth_stream, depth_data = load_data("", r"C:\Users\38698\work_space\data\hand_camera\1675127493_pig_0_0_xw_white_small_stand\0_0_xw_white_small_stand_50_orbbec_depth.pkl", r"C:\Users\38698\work_space\OpenNI\Win64-Release\sdk\libs")

    x_w, y_w, z_w = get_world_coordinate(depth_stream, depth_data, (320, 240))
    print(x_w, y_w, z_w)
