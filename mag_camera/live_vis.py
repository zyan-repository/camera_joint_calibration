import numpy as np
import mag_camera.MagSDK as MagSDK


class Visible:
    def __init__(self, device):
        self.__vis_init_callback = None
        self.__new_visible_callback = None
        self.__device = device
        self.__signaled = False

    def start(self, ip):
        return self.__start_visible(ip)

    def stop(self):
        self.__stop_visible()

    def __start_visible(self, ip):
        if self.__device is None or not self.__device.IsLinked():
            return False

        if not self.__device.VisIsSupported():
            print("visible: visible not supported")
            return True

        if self.__device.VisIsPlaying():
            return True

        self.__vis_init_callback = MagSDK.MAG_INITCALLBACK(self.__vis_initialize)
        self.__device.VisSetInitCallback(self.__vis_init_callback, self)

        self.__new_visible_callback = MagSDK.MAG_VISFRAMECALLBACK(self.new_visible_frame)

        rtsp = "rtsp://" + ip + ":554/camera1"
        print(rtsp, type(rtsp))
        print(MagSDK.enumVideoPixFormat.pixFmtRGB24.value, type(MagSDK.enumVideoPixFormat.pixFmtRGB24.value))
        if self.__device.VisPlay(rtsp, MagSDK.enumVideoPixFormat.pixFmtRGB24.value, self.__new_visible_callback, self,
                                 1, 2000) == 0:
            print(self.__device, dir(self.__device), type(self.__device))
            # from IPython import embed;embed()
            print("visible: play success")
            return True
        else:
            print("visible: play failed")
            return False

    def __vis_initialize(self, channel, width, height, user):
        pass

    def __stop_visible(self):
        if self.__device is None:
            return

        if self.__device.VisIsPlaying():
            self.__device.VisStop()
            print("visible: stop")

        self.__new_frame_callback = None
        self.__device = None
        self.__ip = None
        self.__signaled = False

    def new_visible_frame(self, channel, data, width, height, pixel_format, user):
        pass

    def get_frame(self):
        if not self.__device.VisIsSupported():
            return None

        return self.__convert_to_cvdata()

    def __convert_to_cvdata(self):
        self.__device.Lock()
        result, data = self.__device.VisGetData_copy_v2()
        self.__device.Unlock()

        if not result:
            print("Fail to get visible data")
            return None

        width = self.__device.VisGetWidth()
        height = self.__device.VisGetHeight()

        # convert ir_data to opencv readable format, a new array generated
        cvdata = np.asarray(data).reshape((height, width, 3)).astype(np.uint8)

        return cvdata

