# camera_joint_calibration  
奥比中光、巨哥科技深度、温度四摄对齐  
做法通用：分别标定内参，联立标定外参，外参通过深度摄像头获取  
转换过程：深度像素坐标系->深度相机坐标系->深度世界坐标系 == 温度世界坐标系 -> 温度像素坐标系，深度相机坐标系下Zc通过深度文件获得  
sample/joint_calibration_sample.py 采集脚本，可设置模式（按键k采集或者间隔固定帧采集），由于图片保存是同步的，如果用于标定建议按键采集模式避免图片不同步，或进行人工筛选去掉不同步的图片       
orbbec_mag_joint_calibration.py 联合标定脚本，需要奥比中光sdk，参数文件保存在 joint_parameter  
orbbec_mag_coordinates_transform.py 转换脚本，可不用sdk，调用orbbec_to_mag，需要读取标定文件   
test/joint_calibration_show.py 对比测试脚本，用棋盘格实时观察对齐效果  
