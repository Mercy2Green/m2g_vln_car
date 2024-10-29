roslaunch livox_ros_driver rviz_MID360.launch 

rosbag record /livox/lidar -O /home/uav/m2g_vln_car/vln_car/src/vln/data/4

roslaunch livox_camera_calib bag_to_pcd.launch 

roslaunch livox_camera_calib calib.launch

roslaunch livox_camera_calib multi_calib.launch


rosrun camera_calibration cameracalibrator.py --size 7x10 --square 0.015 image:=/orbbec_camera/color/image_raw
# rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.020 image:=/camera/color/image_raw

cd /home/uav/m2g_vln_car/SensorsCalibration/lidar2camera/manual_calib/

./bin/run_lidar2camera data/2.png data/2.pcd data/center_camera-intrinsic.json data/top_center_lidar-to-center_camera-extrinsic.json

