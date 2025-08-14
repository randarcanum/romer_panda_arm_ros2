source install/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true align_depth.enable:=true enable_sync:=true init_reset:=true pointcloud.enable:=true camera_namespace:=/

