import cv2
import numpy as np
import mag_camera.MagSDK as MagSDK
import threading


class Infrared:
    def __init__(self, device):
        self.__new_infrared_callback = None
        self.__device = device
        self.__signaled = False
        self.__lock = threading.Lock()
        self.__condition = threading.Condition(self.__lock)

    def start(self, ip):
        return self.__start_infrared(ip)

    def stop(self):
        self.__stop_infrared()

    def __start_infrared(self, ip):
        if self.__device is None:
            return False

        if self.__device.Initialize():
            print("infrared: init success")
        else:
            print("infrared: init failed")

        if self.__device.LinkCamera(ip, 1000):
            print("infrared: login success")
        else:
            print("infrared: login failed")
            return False

        # print camera information
        camera_info = self.__device.GetCamInfoEx()
        print("infrared: "
              "serial_num:", camera_info.intCameraSN,
              ", name:", camera_info.BaseInfo.charName.decode("utf-8"),
              ", type:", camera_info.BaseInfo.charType.decode("utf-8"),
              ", resolution:", camera_info.BaseInfo.intFPAWidth, "x", camera_info.BaseInfo.intFPAHeight)

        if self.__device.IsProcessingImage():
            return True

        self.__new_infrared_callback = MagSDK.MAG_FRAMECALLBACK(self.new_infrared_frame)

        if self.__device.StartProcessImage_v2(self.__new_infrared_callback, MagSDK.STREAM_TEMPERATURE, self):
            self.__device.SetColorPalette(MagSDK.ColorPalette.IronBow.value)
            print("infrared: play success")
            return True
        else:
            print("infrared: play failed")
            return False

    def __stop_infrared(self):
        if self.__device is None:
            return

        if self.__device.IsProcessingImage():
            self.__device.StopProcessImage()
            print("infrared: stop")

        if self.__device.IsLinked():
            self.__device.DisLinkCamera()
            print("infrared: logout")

        if self.__device.IsInitialized():
            self.__device.Deinitialize()
            print("infrared: exit")

        self.__new_frame_callback = None
        self.__device = None
        self.__signaled = False

    def new_infrared_frame(self, channel_index, camera_temperature, ffc_counter_down, camera_state, stream_type,
                           user_data):
        self.__lock.acquire()
        self.__signaled = True
        self.__condition.notify()
        self.__lock.release()

    def get_frame(self, timeout):
        try:
            self.__lock.acquire()
            if not self.__condition.wait_for(lambda: self.__signaled, timeout):
                return None

            self.__signaled = False
            return self.__convert_to_cvdata()
        except ValueError:
            return None
        finally:
            self.__lock.release()

    def __convert_to_cvdata(self):
        self.__device.Lock()
        # get rgb image
        is_temperature, data = self.__device.GetOutputBMPDataRGB24_v2(True)
        is_video = False
        if not is_temperature:
            is_video, data = self.__device.GetOutputVideoDataRGB24_v2(True)

        if not is_temperature and not is_video:
            self.__device.Unlock()
            return None

        # get max & min temperature
        if is_temperature:
            state = self.__device.GetFrameStatisticalData()
            para, fix_option = self.__device.GetFixPara()
            max_x, max_y = self.__device.ConvertPos2XY(state.intPosMax)
            max_temperature = self.__device.FixTemperature(state.intMaxTemperature, para.fEmissivity, max_x, max_y)
            min_x, min_y = self.__device.ConvertPos2XY(state.intPosMin)
            min_temperature = self.__device.FixTemperature(state.intMinTemperature, para.fEmissivity, min_x, min_y)
            # print("infrared: max: %.1f C, min: %.1f C" % (max_temperature * 0.001, min_temperature * 0.001))

        self.__device.Unlock()

        # if not result:
        #     print("Fail to get infrared data")
        #     return None

        camInfo = self.__device.GetCamInfo()

        # convert ir_data to opencv readable format, a new array generated
        cvdata = cv2.flip(
            np.asarray(data).reshape((camInfo.intVideoHeight, camInfo.intVideoWidth, 3)).astype(np.uint8), 0)

        return cvdata

