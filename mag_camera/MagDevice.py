﻿import os
import sys
PROJECT_ABSOLUTE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ABSOLUTE_PATH)
import mag_camera.MagSDK as MagSDK
import ctypes
import socket
import struct
from ctypes import *

MAX_CHANNELINDEX = 128
STREAM_TEMPERATURE = 2


class BITMAPINFOHEADER(ctypes.Structure):
    _pack_ = 1  # structure field byte alignment
    _fields_ = [
        ('biSize', ctypes.c_uint),
        ('biWidth', ctypes.c_uint),
        ('biHeight', ctypes.c_uint),
        ('biPLanes', ctypes.c_ushort),
        ('biBitCount', ctypes.c_ushort),
        ('biCompression', ctypes.c_uint),
        ('biSizeImage', ctypes.c_uint),
        ('biXPelsPerMeter', ctypes.c_uint),
        ('biYPelsPerMeter', ctypes.c_uint),
        ('biClrUsed', ctypes.c_uint),
        ('biClrImportant', ctypes.c_uint)
    ]


class MagDevice(object):
    def __init__(self):
        self.__initialized = False
        self.__channelIndex = -1
        self.__recordingAvi = False
        self.__recordingMGS = False
        self.__recordingLocalAvi = False
        self.__recordingLocalMgs = False
        self.__playingLocalMgs = False
        self.__frameIndex = -1
        self.__camIPAddr = 0
        self.__camInfo = MagSDK.CamInfo()
        self.__regContent = MagSDK.CeRegContent()
        self.__camInfoEx = MagSDK.CamInfoEx()
        self.__bar_width = 0
        self.__bar_height = 0
        self.__irRgbData = None
        self.__irRgbDataPtr = None
        self.__irRgbBarData = None
        self.__visRgbData = None
        self.__visRgbBits = 24

    def Initialize(self):
        if self.__initialized:
            return True

        if (self.__channelIndex <= 0 or self.__channelIndex > MAX_CHANNELINDEX):
            for i in range(1, MAX_CHANNELINDEX + 1):
                if not MagSDK.IsChannelAvailable(i):
                    bSuccess = MagSDK.NewChannel(i)
                    self.__channelIndex = i
                    break

        if (self.__channelIndex > 0 and self.__channelIndex <= MAX_CHANNELINDEX and MagSDK.IsLanConnected()):
            self.__initialized = MagSDK.Initialize(self.__channelIndex, None)

        return self.__initialized

    def Deinitialize(self):
        if MagSDK.IsInitialized(self.__channelIndex):
            MagSDK.Free(self.__channelIndex)
            self.__initialized = False

        if MagSDK.IsChannelAvailable(self.__channelIndex):
            MagSDK.DelChannel(self.__channelIndex)
            self.__channelIndex = -1

    def IsInitialized(self):
        return MagSDK.IsInitialized(self.__channelIndex)

    def IsLinked(self):
        return MagSDK.IsLinked(self.__channelIndex)

    def LinkCamera(self, ip, intTimeoutMS):
        if not self.Initialize():
            return False

        if isinstance(ip, int):  # network byte order
            pass
        elif isinstance(ip, str):
            try:
                ip = struct.unpack("=I", socket.inet_aton(ip))[0]
            except OSError:
                return False
        else:
            return False

        if MagSDK.LinkCamera(self.__channelIndex, ip, intTimeoutMS):
            self.__camIPAddr = ip
            MagSDK.GetCamInfo(self.__channelIndex, byref(self.__camInfo), sizeof(self.__camInfo))
            MagSDK.ReadCameraRegContent(self.__channelIndex, byref(self.__regContent), MagSDK.DEFAULT_TIMEOUT, False)
            return True
        else:
            return False

    def LinkCameraEx(self, ip, shortCmdPort, shortImgPort, charCloudUser, charCloudPwd, intCamSN, charCamUser,
                     charCamPwd, intTimeoutMS):
        if not self.Initialize():
            return False

        if isinstance(ip, int):  # network byte order
            pass
        elif isinstance(ip, str):
            try:
                ip = struct.unpack("=I", socket.inet_aton(ip))[0]
            except OSError:
                return False
        else:
            return False

        if MagSDK.LinkCameraEx(self.__channelIndex, ip, shortCmdPort, shortImgPort, charCloudUser, charCloudPwd,
                               intCamSN, charCamUser, charCamPwd, intTimeoutMS):
            self.__camIPAddr = ip
            MagSDK.GetCamInfo(self.__channelIndex, byref(self.__camInfo), sizeof(self.__camInfo))
            MagSDK.ReadCameraRegContent(self.__channelIndex, byref(self.__regContent), MagSDK.DEFAULT_TIMEOUT, False)
            return True
        else:
            return False

    def DisLinkCamera(self):
        if self.__recordingMGS:
            self.SDCardStorage(MagSDK.SDStorageFileType.SDFileMGS.value, 0)

        if self.__recordingAvi:
            self.SDCardStorage(MagSDK.SDStorageFileType.SDFileAVI.value, 0)

        MagSDK.DisLinkCamera(self.__channelIndex)

        self.__camIPAddr = 0
        self.__irRgbData = None
        self.__irRgbBarData = None

    def GetCamInfo(self):
        return self.__camInfo

    def GetCamInfoEx(self):
        MagSDK.GetCamInfoEx(self.__channelIndex, byref(self.__camInfoEx), sizeof(self.__camInfoEx))
        return self.__camInfoEx

    def GetColorbarSize(self):
        return self.__bar_width, self.__bar_height

    def GetRecentHeartBeat(self):
        return MagSDK.GetRecentHeartBeat(self.__channelIndex)

    def SetReConnectCallBack(self, pCallBack, pUserData):
        pUserData = ctypes.py_object(pUserData)
        return MagSDK.SetReConnectCallBack(self.__channelIndex, pCallBack, pUserData)

    def ResetCamera(self):
        if self.__recordingMGS:
            self.SDCardStorage(MagSDK.SDStorageFileType.SDFileMGS.value, 0)

        if self.__recordingAvi:
            self.SDCardStorage(MagSDK.SDStorageFileType.SDFileAVI.value, 0)

        return MagSDK.ResetCamera(self.__channelIndex)

    def TriggerFFC(self):
        return MagSDK.TriggerFFC(self.__channelIndex)

    def AutoFocus(self):
        return MagSDK.SetPTZCmd(self.__channelIndex, MagSDK.PTZCmd.PTZFocusAuto.value, 0)

    def SetIoAlarmState(self, bAlarm):
        return MagSDK.SetIoAlarmState(self.__channelIndex, bAlarm)

    def SetPTZCmd(self, cmd, dwPara):
        return MagSDK.SetPTZCmd(self.__channelIndex, cmd, dwPara)

    def QueryPTZState(self, query, intValue, intTimeoutMS):
        return MagSDK.QueryPTZState(self.__channelIndex, query, intValue, intTimeoutMS)

    def SetSerialCmd(self, buff, intBufferLen):
        return MagSDK.SetSerialCmd(self.__channelIndex, buff, intBufferLen)

    def SetSerialCallBack(self, pCallBack, pUserData):
        pUserData = ctypes.py_object(pUserData)
        return MagSDK.SetSerialCallBack(self.__channelIndex, pCallBack, pUserData)

    def GetCameraTemperature(self, intT, intTimeoutMS):
        return MagSDK.GetCameraTemperature(self.__channelIndex, intT, intTimeoutMS)

    def ReadCameraRegContent(self, pContent, intTimeoutMs):
        return MagSDK.ReadCameraRegContent(self.__channelIndex, byref(pContent), intTimeoutMs, False)
    
    def SetCameraRegContent(self, pContent):
        if not MagSDK.SetCameraRegContent(self.__channelIndex, pContent):
            MagSDK.ReadCameraRegContent(self.__channelIndex, byref(self.__regContent), MagSDK.DEFAULT_TIMEOUT, False)
            return True
        else:
            return False

    def ReadCameraRegContent2(self, pCfgPara, intTimeoutMs):
        return MagSDK.ReadCameraRegContent2(self.__channelIndex, byref(pCfgPara), intTimeoutMs)

    def SetCameraRegContent2(self, pCfgPara):
        return MagSDK.SetCameraRegContent2(self.__channelIndex, byref(pCfgPara))

    def ReadCamRegs(self, pContent, intTimeoutMs):
        return MagSDK.ReadCamRegs(self.__channelIndex, byref(pContent), intTimeoutMs)

    def SetCamRegs(self, pContent):
        return MagSDK.SetCamRegs(self.__channelIndex, byref(pContent))

    def SetIrregularROIs(self, pROIs, intROINum):
        return MagSDK.SetIrregularROIs(self.__channelIndex, pROIs, intROINum)

    def SetUserROIs(self, pROI):
        return MagSDK.SetUserROIs(self.__channelIndex, pROI)

    def SetUserROIsEx(self, pROIs, intROINum):
        return MagSDK.SetUserROIsEx(self.__channelIndex, pROIs, intROINum)

    def SetROIReportCallBack(self, pCallBack, pUserData):
        pUserData = ctypes.py_object(pUserData)
        return MagSDK.SetROIReportCallBack(self.__channelIndex, pCallBack, pUserData)

    def SetIrregularROIReportExCallBack(self, pCallBack, pUserData):
        pUserData = ctypes.py_object(pUserData)
        return MagSDK.SetIrregularROIReportExCallBack(self.__channelIndex, pCallBack, pUserData)

    def SetObjRecoCallBack(self, pCallBack, pUserData):
        return MagSDK.SetObjRecoCallBack(self.__channelIndex, pCallBack, pUserData)

    def IsProcessingImage(self):
        return MagSDK.IsProcessingImage(self.__channelIndex)

    def StartProcessImage(self, paraOut, funcFrame, dwStreamType, pUserData):
        if MagSDK.IsProcessingImage(self.__channelIndex):
            return False

        self.__bar_width = paraOut.dwColorBarWidth
        self.__bar_height = paraOut.dwColorBarHeight

        pUserData = ctypes.py_object(pUserData)
        return MagSDK.StartProcessImage(self.__channelIndex, paraOut, funcFrame, dwStreamType, pUserData)

    def StartProcessImage_v2(self, funcFrame, dwStreamType, pUserData):
        para = MagSDK.OutputPara()
        para.dwFPAWidth = self.__camInfo.intFPAWidth
        para.dwFPAHeight = self.__camInfo.intFPAHeight
        para.dwBMPWidth = self.__camInfo.intVideoWidth
        para.dwBMPHeight = self.__camInfo.intVideoHeight
        para.dwColorBarWidth = 16
        para.dwColorBarHeight = para.dwBMPHeight

        return self.StartProcessImage(para, funcFrame, dwStreamType, pUserData)

    def StartProcessPulseImage(self, paraOut, funcFrame, dwStreamType, pUserData):
        if MagSDK.IsProcessingImage(self.__channelIndex):
            return False

        self.__bar_width = paraOut.dwColorBarWidth
        self.__bar_height = paraOut.dwColorBarHeight

        pUserData = ctypes.py_object(pUserData)
        return MagSDK.StartProcessPulseImage(self.__channelIndex, paraOut, funcFrame, dwStreamType, pUserData)

    def StartProcessPulseImage_v2(self, funcFrame, dwStreamType, pUserData):
        para = MagSDK.OutputPara()
        para.dwFPAWidth = self.__camInfo.intFPAWidth
        para.dwFPAHeight = self.__camInfo.intFPAHeight
        para.dwBMPWidth = self.__camInfo.intVideoWidth
        para.dwBMPHeight = self.__camInfo.intVideoHeight
        para.dwColorBarWidth = 16
        para.dwColorBarHeight = para.dwBMPHeight

        return self.StartProcessPulseImage(para, funcFrame, dwStreamType, pUserData)

    def TransferPulseImage(self):
        return MagSDK.TransferPulseImage(self.__channelIndex)

    def TransferPulseImageWait(self, intTimeoutMS):
        return MagSDK.TransferPulseImageWait(self.__channelIndex, intTimeoutMS)

    def StopProcessImage(self):
        if self.__recordingLocalAvi:
            self.LocalStorageAviStop()

        return MagSDK.StopProcessImage(self.__channelIndex)

    def SetColorPalette(self, ColorPaletteIndex):
        return MagSDK.SetColorPalette(self.__channelIndex, ColorPaletteIndex)

    def SetSubsectionEnlargePara(self, intX1, intX2, byteY1, byteY2):
        return MagSDK.SetSubsectionEnlargePara(self.__channelIndex, intX1, intX2, byteY1, byteY2)

    def SetIsothermalPara(self, intLowerLimit, intUpperLimit):
        return MagSDK.SetIsothermalPara(self.__channelIndex, intLowerLimit, intUpperLimit)

    def SetEnhancedROI(self, intChannelIndex, intEnhancedRatio, x0, y0, x1, y1):
        return MagSDK.SetEnhancedROI(self.__channelIndex, intEnhancedRatio, x0, y0, x1, y1)

    def SetAutoEnlargePara(self, dwAutoEnlargeRange, intBrightOffset, intContrastOffset):
        return MagSDK.SetAutoEnlargePara(self.__channelIndex, dwAutoEnlargeRange, intBrightOffset, intContrastOffset)

    def SetEXLevel(self, ExLevel, intCenterX, intCenterY):
        return MagSDK.SetEXLevel(self.__channelIndex, ExLevel, intCenterX, intCenterY)

    def GetEXLevel(self):
        return MagSDK.GetEXLevel(self.__channelIndex)

    def SetDetailEnhancement(self, intDDE, bQuickDDE):
        return MagSDK.SetDetailEnhancement(self.__channelIndex, intDDE, bQuickDDE)

    def SetVideoContrast(self, intContrastOffset):
        return MagSDK.SetVideoContrast(self.__channelIndex, intContrastOffset)

    def SetVideoBrightness(self, intBrightnessOffset):
        return MagSDK.SetVideoBrightness(self.__channelIndex, intBrightnessOffset)

    def GetFixPara(self):
        para = MagSDK.FixPara()
        resnum = MagSDK.GetFixPara(self.__channelIndex, para)
        fixOption = MagSDK.FixSelection(resnum)
        return para, fixOption

    def SetFixPara(self, pPara, enumFixOption):
        """
        Should call FixTemperature to make it effect
        """
        return MagSDK.SetFixPara(self.__channelIndex, pPara, enumFixOption)

    def FixTemperature(self, intT, fEmissivity, dwPosX, dwPosY):
        return MagSDK.FixTemperature(self.__channelIndex, intT, fEmissivity, dwPosX, dwPosY)

    def GetFilteredRaw(self):
        return MagSDK.GetFilteredRaw(self.__channelIndex)

    def _bmp_info_to_index_table(self, bmp_info):
        colorTable = []
        offset = ctypes.sizeof(BITMAPINFOHEADER)
        idx = 0

        for i in range(256):
            argbVal = 0xff << 24 | bmp_info[offset + idx + 2] << 16 | bmp_info[offset + idx + 1] << 8 | bmp_info[
                offset + idx]
            idx += 4
            colorTable.append(argbVal)

        return colorTable

    def _ubyte_ptr_cast_to_ubyte_array(self, data, info):
        infoHeader = ctypes.cast(info, ctypes.POINTER(BITMAPINFOHEADER)).contents
        length = infoHeader.biWidth * infoHeader.biHeight * infoHeader.biBitCount // 8
        return ctypes.cast(data, ctypes.POINTER(ctypes.c_ubyte*length)).contents

    def GetOutputBMPData(self):
        """
        Get 8 bits pseudo-color image, data buffer managed by native SDK, read only
        """
        irData = ctypes.POINTER(ctypes.c_ubyte)()
        irInfo = ctypes.POINTER(ctypes.c_ubyte)()

        if MagSDK.GetOutputBMPdata(self.__channelIndex, irData, irInfo):
            colorTable = self._bmp_info_to_index_table(irInfo)
            return True, self._ubyte_ptr_cast_to_ubyte_array(irData, irInfo), colorTable
        else:
            return False, None, None

    def GetOutputBMPDataRGB24(self, data, size, bOrderBGR=False):
        """
        Get 24 bits RGB image, data buffer managed by user
        """
        if MagSDK.GetOutputBMPDataRGB24(self.__channelIndex, data, size, bOrderBGR):
            return True, data
        else:
            return False, None

    def GetOutputBMPDataRGB24_v2(self, bOrderBGR=False):
        """
        Simple method to get 24 bits RGB image, data buffer managed by MagDevice class
        """
        length = self.__camInfo.intVideoWidth * self.__camInfo.intVideoHeight * 3  # rgb24

        if self.__irRgbData is None or length != sizeof(self.__irRgbData):
            self.__irRgbData = (ctypes.c_ubyte * length)()

        return self.GetOutputBMPDataRGB24(self.__irRgbData, sizeof(self.__irRgbData), bOrderBGR)

    def GetOutputVideoData(self):
        """
        Get 24 bits decoded RGB image, data buffer managed by native SDK, read only
        """
        irData = ctypes.POINTER(ctypes.c_ubyte)()
        irInfo = ctypes.POINTER(ctypes.c_ubyte)()

        if MagSDK.GetOutputVideoData(self.__channelIndex, irData, irInfo):
            return True, irData
        else:
            return False, None

    def GetOutputVideoDataRGB24(self, data, size, bOrderBGR=False):
        """
        Get 24 bits decoded RGB image, data buffer managed by user
        """
        if MagSDK.GetOutputVideoDataRGB24(self.__channelIndex, data, size, bOrderBGR):
            return True, data
        else:
            return False, None

    def GetOutputVideoDataRGB24_v2(self, bOrderBGR=False):
        """
        Simple method to get 24 bits decoded RGB image, data buffer managed by MagDevice class
        """
        length = self.__camInfo.intVideoWidth * self.__camInfo.intVideoHeight * 3  # rgb24
        if self.__irRgbData is None or length != sizeof(self.__irRgbData):
            self.__irRgbData = (ctypes.c_ubyte * length)()

        return self.GetOutputVideoDataRGB24(self.__irRgbData, sizeof(self.__irRgbData), bOrderBGR)

    def GetOutputColorBarData(self):
        """
        Get 8 bits pseudo-color bar image, data buffer managed by native SDK, read only
        """
        barData = ctypes.POINTER(ctypes.c_ubyte)()
        barInfo = ctypes.POINTER(ctypes.c_ubyte)()

        if MagSDK.GetOutputColorBardata(self.__channelIndex, barData, barInfo):
            if self.__bar_width == 0 or self.__bar_height == 0:
                info = ctypes.cast(barInfo, ctypes.POINTER(BITMAPINFOHEADER)).contents
                self.__bar_width = info.biWidth
                self.__bar_height = info.biHeight
            colorTable = self._bmp_info_to_index_table(barInfo)
            return True, self._ubyte_ptr_cast_to_ubyte_array(barData, barInfo), colorTable
        else:
            return False, None, None

    def GetOutputColorBardataRGB24(self, data, size, bOrderBGR=False):
        """
        Get 24 bits color bar image, data buffer managed by user
        """
        if MagSDK.GetOutputColorBardataRGB24(self.__channelIndex, data, size, bOrderBGR):
            return True, data
        else:
            return False, None

    def GetOutputColorBardataRGB24_v2(self, bOrderBGR=False):
        """
        Simple method to get 24 bits color bar image, data buffer managed by MagDevice class
        """
        length = self.__bar_width * self.__bar_height * 3  # rgb24

        if self.__irRgbBarData is None or length != sizeof(self.__irRgbBarData):
            self.__irRgbBarData = (ctypes.c_ubyte * length)()

        return self.GetOutputColorBardataRGB24(self.__irRgbBarData, sizeof(self.__irRgbBarData), bOrderBGR)

    def GetOutputVideoYV12(self):
        return MagSDK.GetOutputVideoYV12(self.__channelIndex)

    def GetFrameStatisticalData(self):
        """
        Get current frame temperature statistical data.

        Returns:
            A MagSDK.State(Structure) object.

            The return value has following attributes:
                value.intMaxTemperature: c_int
                value.intMinTemperature: c_int
                value.intAveTemperature: c_int
                value.intSTDTemperature: c_int
                value.intPosMax: c_uint
                value.intPosMin: c_uint
        """
        data = MagSDK.GetFrameStatisticalData(self.__channelIndex)
        return data.contents

    def GetTemperatureData(self, bEnableExtCorrect):
        """
        Get current frame temperature data.

        Args：
            bEnableExtCorrect: bool, Whether to enable emissivity correction.

        Returns:
            If success, return a ctypes.Pointer which points to ctypes.c_int array.
            You can regard it as a python list to visit. The length of the return value is FPAWidth * FPAHeight.
        
        Raises:
            Return None.
        """
        length = self.__camInfo.intFPAWidth * self.__camInfo.intFPAHeight
        data = (ctypes.c_int * length)()
        status = MagSDK.GetTemperatureData(self.__channelIndex, data, sizeof(data), bEnableExtCorrect)
        return data if status else None

    def GetTemperatureData_Raw(self, bEnableExtCorrect):
        """
        The parameters and effect are consistent with 'GetTemperatureData', this function computes faster, but maybe the precision is beyond the range.
        """
        length = self.__camInfo.intFPAWidth * self.__camInfo.intFPAHeight
        data = (ctypes.c_int * length)()
        status = MagSDK.GetTemperatureData_Raw(self.__channelIndex, data, sizeof(data), bEnableExtCorrect)
        return data if status else None

    def GetTemperatureProbe(self, dwPosX, dwPosY, intSize):
        return MagSDK.GetTemperatureProbe(self.__channelIndex, dwPosX, dwPosY, intSize)

    def GetLineTemperatureInfo(self, buff, intBufferSizeByte, info, x0, y0, x1, y1):
        return MagSDK.GetLineTemperatureInfo(self.__channelIndex, buff, intBufferSizeByte, info, x0, y0, x1, y1)

    def GetLineTemperatureInfo2(self, x0, y0, x1, y1):
        """
        Get line temperature statistical info.

        Args:
            x0, y0, x1, y1: int

        Returns:
            status: bool
            info: ctypes.c_int array, length is 5.
            
            info[0]: Min Temperature
            info[1]: Max Temperature
            info[2]: Ave Temperature
            info[3]: Min Temperature Postion
            info[4]: Max Temperature Position
        """
        info = (ctypes.c_int * 5)()
        status = MagSDK.GetLineTemperatureInfo2(self.__channelIndex, x0, y0, x1, y1, info)
        return status, info

    def GetRectTemperatureInfo(self, x0, y0, x1, y1):
        """
        Get rect temperature statistical info.

        Args:
            x0, y0, x1, y1: int

        Returns:
            status: bool
            info: ctypes.c_int array, length is 5.

            info[0]: Min Temperature
            info[1]: Max Temperature
            info[2]: Ave Temperature
            info[3]: Min Temperature Postion
            info[4]: Max Temperature Position
        """
        info = (ctypes.c_int * 5)()
        status = MagSDK.GetRectTemperatureInfo(self.__channelIndex, x0, y0, x1, y1, info)
        return status, info

    def GetEllipseTemperatureInfo(self, x0, y0, x1, y1):
        """
        Get ellipse temperature statistical info.

        Args:
            x0, y0, x1, y1: int

        Returns:
            status: bool
            info: ctypes.c_int array, length is 5.

            info[0]: Min Temperature
            info[1]: Max Temperature
            info[2]: Ave Temperature
            info[3]: Min Temperature Postion
            info[4]: Max Temperature Position
        """
        info = (ctypes.c_int * 5)()
        status = MagSDK.GetEllipseTemperatureInfo(self.__channelIndex, x0, y0, x1, y1, info)
        return status, info

    def GetRgnTemperatureInfo(self, Pos, intPosNumber, info):
        return MagSDK.GetRgnTemperatureInfo(self.__channelIndex, Pos, intPosNumber, info)

    def UseTemperatureMask(self, bUse):
        return MagSDK.UseTemperatureMask(self.__channelIndex, bUse)

    def IsUsingTemperatureMask(self):
        return MagSDK.IsUsingTemperatureMask(self.__channelIndex)

    def SaveBMP(self, dwIndex, charFilename):
        return MagSDK.SaveBMP(self.__channelIndex, dwIndex, charFilename)

    def SaveJpg(self, charFilename, ext, intTimeoutMS):
        return MagSDK.SaveJPG(self.__channelIndex, charFilename, ext, intTimeoutMS)

    def SaveMGT(self, charFilename):
        return MagSDK.SaveMGT(self.__channelIndex, charFilename)

    def SaveDDT(self, charFilename):
        return MagSDK.SaveDDT(self.__channelIndex, charFilename)

    def SaveDDT2Buffer(self, pBuffer, intBufferSize):
        return MagSDK.SaveDDT2Buffer(self.__channelIndex, pBuffer, intBufferSize)

    def LoadDDT(self, paraOut, charFilename, funcFrame, pUserData):
        if MagSDK.IsLinked(self.__channelIndex):
            return False

        if not self.IsInitialized() and not self.Initialize():
            return False

        if not MagSDK.LoadDDT(self.__channelIndex, paraOut, charFilename, funcFrame, pUserData):
            return False

        MagSDK.GetCamInfo(self.__channelIndex, byref(self.__camInfo), sizeof(self.__camInfo))
        self.__bar_width = paraOut.dwColorBarWidth
        self.__bar_height = paraOut.dwColorBarHeight
        return True

    def LoadDDT_v2(self, charFilename, funcFrame, pUserData):
        para = MagSDK.OutputPara()
        para.dwFPAWidth = 384
        para.dwFPAHeight = 288
        para.dwBMPWidth = 384
        para.dwBMPHeight = 288
        para.dwColorBarWidth = 16
        para.dwColorBarHeight = para.dwBMPHeight

        return self.LoadDDT(para, charFilename, funcFrame, pUserData)

    def LoadBufferedDDT(self, paraOut, pBuffer, intBufferSize, funcFrame, pUserData):
        if MagSDK.IsLinked(self.__channelIndex):
            return False

        if not self.IsInitialized() and not self.Initialize():
            return False

        if not MagSDK.LoadBufferedDDT(self.__channelIndex, paraOut, pBuffer, intBufferSize, funcFrame, pUserData):
            return False

        MagSDK.GetCamInfo(self.__channelIndex, byref(self.__camInfo), sizeof(self.__camInfo))
        self.__bar_width = paraOut.dwColorBarWidth
        self.__bar_height = paraOut.dwColorBarHeight
        return True

    def LoadBufferedDDT_v2(self, pBuffer, intBufferSize, funcFrame, pUserData):
        para = MagSDK.OutputPara()
        para.dwFPAWidth = 384
        para.dwFPAHeight = 288
        para.dwBMPWidth = 384
        para.dwBMPHeight = 288
        para.dwColorBarWidth = 16
        para.dwColorBarHeight = para.dwBMPHeight

        return self.LoadBufferedDDT(para, pBuffer, intBufferSize, funcFrame, pUserData)

    def SetAsyncCompressCallBack(self, pCallBack, intQuality):
        return MagSDK.SetAsyncCompressCallBack(self.__channelIndex, pCallBack, intQuality)

    def GrabAndAsyncCompressDDT(self, pUserData):
        return MagSDK.GrabAndAsyncCompressDDT(self.__channelIndex, pUserData)

    def SDCardStorage(self, filetype, para):
        bReturn = MagSDK.SDCardStorage(self.__channelIndex, filetype, para)

        if bReturn and filetype == MagSDK.SDStorageFileType.SDFileMGS.value:
            if para == 1:
                self.__recordingMGS = True
            else:
                self.__recordingMGS = False

        if bReturn and filetype == MagSDK.SDStorageFileType.SDFileAVI.value:
            if para == 1:
                self.__recordingAvi = True
            else:
                self.__recordingAvi = False

        return bReturn

    def SDStorageMGT(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileMGT.value, 0)

    def SDStorageBMP(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileBMP.value, 0)

    def SDStorageDDT(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileDDT.value, 0)

    def SDStorageMGSStart(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileMGS.value, 1)

    def SDStorageMGSStop(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileMGS.value, 0)

    def SDStorageAviStart(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileAVI.value, 1)

    def SDStorageAviStop(self):
        return MagSDK.SDCardStorage(self.__channelIndex, MagSDK.SDStorageFileType.SDFileAVI.value, 0)

    def LocalStorageAviStart(self, charFilename, intSamplePeriod):
        self.__recordingLocalAvi = self.__recordingLocalAvi or MagSDK.LocalStorageAviStart(self.__channelIndex,
                                                                                           charFilename,
                                                                                           intSamplePeriod)
        return self.__recordingLocalAvi

    def LocalStorageAviStop(self):
        MagSDK.LocalStorageAviStop(self.__channelIndex)
        self.__recordingLocalAvi = False

    def IsLocalAviRecording(self):
        return self.__recordingLocalAvi

    def LocalStorageMgsRecord(self, charFilename, intSamplePeriod):
        self.__recordingLocalMgs = self.__recordingLocalMgs or MagSDK.LocalStorageMgsRecord(self.__channelIndex,
                                                                                            charFilename,
                                                                                            intSamplePeriod)
        return self.__recordingLocalMgs

    def LocalStorageMgsPlay(self, charFilename, funcFrame, pUserData):
        if MagSDK.IsProcessingImage(self.__channelIndex):
            MagSDK.LocalStorageMgsStop(self.__channelIndex)

        intTotalFrames = MagSDK.LocalStorageMgsPlay(self.__channelIndex, charFilename, funcFrame, pUserData)
        if intTotalFrames > 0:
            self.__playingLocalMgs = True
        if self.__playingLocalMgs:
            MagSDK.GetCamInfo(self.__channelIndex, byref(self.__camInfo), sizeof(self.__camInfo))
        return intTotalFrames

    def LocalStorageMgsPopFrame(self):
        return MagSDK.LocalStorageMgsPopFrame(self.__channelIndex)

    def LocalStorageMgsSeekFrame(self, intFrameIndex):
        return MagSDK.LocalStorageMgsSeekFrame(self.__channelIndex, self.__frameIndex)

    def LocalStorageMgsStop(self):
        MagSDK.LocalStorageMgsStop(self.__channelIndex)
        self.__recordingLocalMgs = False
        self.__playingLocalMgs = False

    def IsLocalMgsRecording(self):
        return self.__recordingLocalMgs

    def IsLocalMgsPlaying(self):
        return self.__playingLocalMgs

    def Lock(self):
        return MagSDK.LockFrame(self.__channelIndex)

    def Unlock(self):
        return MagSDK.UnLockFrame(self.__channelIndex)

    # def ConvertIrCorr2VisCorr(self, intIrX, intIrY, fDistance, intVisX, intVisY):
        # return MagSDK.ConvertIrCorr2VisCorr(self.__channelIndex, intIrX, intIrY, fDistance, intVisX, intVisY)

    # def ConvertVisCorr2IrCorr(self, intVisX, intVisY, fDistance, intIrX, intIrY):
    #     return MagSDK.ConvertVisCorr2IrCorr(self.__channelIndex, intVisX, intVisY, fDistance, intIrX, intIrY)

    def ConvertIrCorr2VisCorr(self, intIrX, intIrY, fDistance):
        vis_x = ctypes.c_int(0)
        vis_y = ctypes.c_int(0)

        b = MagSDK.ConvertIrCorr2VisCorr(self.__channelIndex, intIrX, intIrY, fDistance, ctypes.byref(vis_x), ctypes.byref(vis_y))
        return (vis_x, vis_y, b)

    def ConvertVisCorr2IrCorr(self, intVisX, intVisY, fDistance):
        ir_x = ctypes.c_int(0)
        ir_y = ctypes.c_int(0)
        b = MagSDK.ConvertVisCorr2IrCorr(self.__channelIndex, intVisX, intVisY, fDistance, ctypes.byref(ir_x),
                                         ctypes.byref(ir_y))
        return (ir_x, ir_y, b)

    # 使用二元数组
    def ConvertPos2XY(self, intPos):
        """
        Convert MAG Position to cartesian coordinate system.

        Args:
            intPos: int, MAG Position
        
        Returns:
            Tuple(x, y)
        
        Raises:
            Tuple(None, None)
        """
        w = self.__camInfo.intFPAWidth
        if w:
            pY = intPos // w
            pX = intPos - pY * w
            return (pX, pY)
        else:
            return (None, None)

    def ConvertXY2Pos(self, x, y):
        return (y * self.__camInfo.intFPAWidth + x)

    # NPU correlation function
    def NPUSetNetWork(self, pSrcPara, wcharNetworkFile, intTimeoutMS):
        return MagSDK.NPUSetNetwork(self.__channelIndex, pSrcPara, wcharNetworkFile, intTimeoutMS)

    def NPUSetWeight(self, wcharNetworkFile, intTimeoutMS):
        return MagSDK.NPUSetWeight(self.__channelIndex, wcharNetworkFile, intTimeoutMS)

    def NPUUpdateTestImage(self, wcharNetworkFile, intTimeoutMS):
        return MagSDK.NPUUpdateTestImage(self.__channelIndex, wcharNetworkFile, intTimeoutMS)

    def NPUStop(self):
        return MagSDK.NPUStop(self.__channelIndex)

    def NPUStart(self, intInferencePriod, funcNPU, pUserData):
        return MagSDK.NPUStart(self.__channelIndex, intInferencePriod, funcNPU, pUserData)

    def VisIsSupported(self):
        """
        Is this camera support visible module
        """
        return 0 != self.__regContent.dwPartnerVisibleIp

    def VisPlay(self, rtspUrl, pixelFormat, frameCallback, pUserData, protocol, timeout):
        """
        Must be called after LinkCamera of infrared camera
        :param rtspUrl: for example rtsp://192.168.1.100:554/camera1
        :param pixelFormat: output image pixel format.
                must be MagSDK.enumVideoPixFormat.pixFmtARGB.value or MagSDK.enumVideoPixFormat.pixFmtRGB24.value
        :param frameCallback: callback when new frame arrived
        :param pUserData:
        :param protocol: 0 - RTSP over UDP; 1 - RTSP over TCP
        :param timeout: milliseconds
        :return: 0 - success; < 0 - errcode
        """
        if pixelFormat== MagSDK.enumVideoPixFormat.pixFmtARGB.value:
            self.__visRgbBits = 32
        else:
            self.__visRgbBits = 24
        return MagSDK.StartVis(self.__channelIndex, rtspUrl, pixelFormat, frameCallback, pUserData, protocol, timeout)

    def VisStop(self):
        MagSDK.StopVis(self.__channelIndex)
        self.__visRgbData = None

    def VisIsPlaying(self):
        return MagSDK.IsVisStarted(self.__channelIndex)

    def VisSetInitCallback(self, initCallback, pUserData):
        """
        Called when RTSP handshake finished but data not come. Do some initialize usually
        """
        return MagSDK.SetVisInitCallback(self.__channelIndex, initCallback, pUserData)

    def VisSetReconnectCallback(self, reconnectCallback, pUserData):
        """
        Called when camera lost or recovered
        """
        MagSDK.SetVisReconnectCallback(self.__channelIndex, reconnectCallback, pUserData)

    def VisGetWidth(self):
        return MagSDK.VisWidth(self.__channelIndex)

    def VisGetHeight(self):
        return MagSDK.VisHeight(self.__channelIndex)

    def VisSaveBMP(self, charFileName):
        return MagSDK.SaveVis(self.__channelIndex, charFileName)

    def VisLock(self):
        MagSDK.LockVisFrame(self.__channelIndex)

    def VisUnlock(self):
        MagSDK.UnlockVisFrame(self.__channelIndex)

    def VisGetData(self):
        """
        Get 24 or 32 bits visible image, data buffer managed by native SDK, read only
        """
        visData = ctypes.POINTER(ctypes.c_ubyte)()
        visInfo = ctypes.POINTER(ctypes.c_ubyte)()

        if MagSDK.GetVisData(self.__channelIndex, visData, visInfo) == 0:
            return True, self._ubyte_ptr_cast_to_ubyte_array(visData, visInfo)
        else:
            return False, None

    def VisGetData_copy(self, data, size):
        """
        Get 24 or 32 bits RGB image, data buffer managed by user
        """
        return MagSDK.GetVisData_copy(self.__channelIndex, data, size)

    def VisGetData_copy_v2(self):
        """
        Simple method to get 24 or 32 bits RGB image, data buffer managed by MagDevice class
        """
        if not self.VisIsPlaying():
            return False, None

        if self.__visRgbData is None:
            bufferSize = self.VisGetWidth() * self.VisGetHeight() * self.__visRgbBits // 8 # alloc enough buffer size
            self.__visRgbData = (ctypes.c_ubyte * bufferSize)()

        if self.__visRgbData and self.VisGetData_copy(self.__visRgbData, sizeof(self.__visRgbData)):
            return True, self.__visRgbData
        else:
            return False, None
