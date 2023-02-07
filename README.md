# camera_joint_calibration  
test/joint_calibration_sample.py 采集、对比测试脚本，可设置模式（按键k采集或者间隔固定帧采集），由于图片保存是同步的，如果用于标定建议按键采集模式避免图片不同步     
orbbec_mag_joint_calibration.py 联合标定脚本，需要奥比中光sdk，参数文件保存在 joint_parameter  
orbbec_mag_coordinates_transform.py 转换脚本，可不用sdk，调用orbbec_to_mag，需要读取标定文件   
