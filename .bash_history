ls
rviz2
ros2 topic list
export ROS_LOCALHOST_ONLY
export ROS_LOCALHOST_ONLY=1
ros2 topic list
printenv | grep -i ROS
rviz2
export ROS_LOCALHOST_ONLY=0
rviz2
cd 
ls
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
sudo apt install unzip -y
man unzip
unzip -h
unzip opencv.zip -d opencv
cd opencv/
ls
mv opencv-4.x/ ..
cd ..
rm opencv -r
mv opencv-4.x/ opencv
cd opencv/
ls
mkdir build
cd build/
cmake ..
make -j4
sudo make install
cd
cd turtlebot3_ws/
colcon build --symlink-install
sudo apt install libopencv-dev
colcon build --symlink-install
cd /usr/local/include/
ls
cd 
cd /usr/include/
ls
cd 
clear
export ROS_LOCALHOST_ONLY=1
ros2 run nbv_calculator nbv_action_server
ros2 run nbv_calculator nbv_action_server --ros-args --params-file turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
ros2 run nbv_calculator nbv_action_server --ros-args --params-file turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
ros2 topic list
export ROS_LOCALHOST_ONLY=0
ros2 topic list
ros2 run nbv_calculator nbv_action_server --ros-args --params-file turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
cd turtlebot3_ws/
colcon build --symlink-install
colcon build --symlink-install
cd /usr/local/include/opencv4/opencv2/
ls
cd turtlebot3_ws/src/
code .
tmux
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
cd build/
ls
cd nbv_
cd nbv_calculator/
ls
cd ../../install/nbv_calculator/
ls
cd lib/
ls
cd nbv_calculator/
ls
cd ..
cd ..
cd share/
ls
c d..
cd ..
ls
cd ..
ls
cd vdb_mapping
ls
cd include/
ls
cd ..
cd ..
cd vdb_mapping_ros2/
ls
cd include/
ls
cd vdb_mapping_ros2/
ls
ls
cd 
cd turtlebot3_ws/
colcon build --symlink-insta
rm -rf build/ log/ install/
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ls
cd install/
ls
cd ..
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 topic list
ros2 topic echo /camera/image_mouse_left
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
rqt
ros2 run nbv_calculator nbv_action_server --ros-args --params-file turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
ros2 run nbv_calculator nbv_action_server --ros-args --params-file ~/turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
ros2 run nbv_calculator nbv_action_server --ros-args --params-file ~/turtlebot3_ws/src/vi_to_nav/config/vdb_params_virtual.yaml 
ros2 run nbv_calculator nbv_action_server
ros2 run nbv_calculator nbv_action_server
ros2 run nbv_calculator nbv_action_server
ros2 run nbv_calculator nbv_action_server
ros2 action lst
ros2 action list
rviz2
ls
cd turtlebot3_ws/
cd src/
code .
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cd ..
rm -rf opencv/
cd turtlebot3_ws/
colcon build --symlink-install
colcon build --symlink-install
tmux
ros2 launch vi_to_nav scan_world.launch.py
cd turtlebot3_ws/
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 run tf2_tools view_frames
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 173.0, origin_y: 213.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
htop
sudo apt install -y htop
htop
htop
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
rqt
ros2 topic echo /camera/image_mouse_left
ros2 topic echo /camera_2/image_mouse_left
ls
cd turtlebot3_ws/src/
code .
cd ..
cd ..
ls
tmux
sudo apt install htop -y
sudo apt update
apt list --upgradable
sudo apt upgrade
cd turtlebot3_ws/
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 interface show sensor_msgs/msg/CameraInfo
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
gz
ign 
ign gazebo 
ign gazebo sdf -p ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf 
ign gazebo sdf -p ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf > waffle.sdf
ign gazebo sdf ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf > waffle.sdf
ign gazebo sdf ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf
ign gazebo ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf
gz sdf -p ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_potato.urdf > ./my_sdf.sdf
diff ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_potato.urdf 
diff ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_potato.urdf 
colcon build --symlink-install
vim ../.bashrc 
export TURTLEBOT3_MODEL=crepe
ros2 launch vi_to_nav scan_world.launch.py
gz sdf -p ./src/turtlebot3/turtlebot3_gazebo/urdf/turtlebot3_crepe.urdf > ./my_sdf.sdf
vim my_sdf.sdf 
ros2 launch vi_to_nav scan_world.launch.py
vim my_sdf.sdf 
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 topic list
ros2 topic echo /camera_2/depth_image
ros2 topic list
ros2 topic echo [200~/camera_2/depth_image/compressedDepth~
ros2 topic echo /camera_2/depth_image/compressedDepth
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 181.0, origin_y: 229.0, frame_name: camera_2_frame, camera_name: camera_2}"
rqt
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 171.0, origin_y: 240.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 171.0, origin_y: 240.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 171.0, origin_y: 240.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 interface show sensor_msgs/msg/CameraInfo
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 171.0, origin_y: 240.0, frame_name: camera_2_frame, camera_name: camera_2}"
ros2 topic list
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 171.0, origin_y: 240.0, frame_name: camera_frame, camera_name: camera}"
ros2 topic /camera/image_mouse_left
ros2 topic echo /camera/image_mouse_left
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 17147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 topic echo /camera/image_mouse_left
ros2 topic echo /camera_2/image_mouse_left
ros2 topic list
rviz2
rqt
ls
cd turtlebot3_ws/
cd src/
ls
code .
cd ..
colcon build --symlink-install
tmux
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
rviz2
ros2 topic list
ros2 topic echo /camera/depth_image
ros2 interface show /sensor_msgs/msg/image
ros2 interface show sensor_msgs/msg/image
ros2 interface show sensor_msgs/msg/Image
ros2 interface show sensor_msgs/msg/CameraInfo
ros2 topic echo /camera/camera_info
ros2 interface show sensor_msgs/msg/CameraInfo
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_rgb_optical_frame, camera_name: camera}"
ros2 topic list
ros2 topic echo /camera/camera_info
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ls
cd turtlebot3_ws/src/
code .
code .
cd ..
ros2 launch vi_to_nav scan_world.launch.py
export TURTLEBOT3_MODEL=crepe
tmux
tmux
tmux
ros2 interface show nbv_interfaces/srv/ViewPointSampling
ros2 service 
ros2 service list
ros2 service /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call
ros2 service call -h
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
python3
clear
python3
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
python3
ros2 launch vi_to_nav scan_world.launch.py
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run tf2_tools view_frames
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
cd turtlebot3_ws/src/
code .
tmux
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
cd turtlebot3_ws/src/
code .
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
tmux
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
export TURTLEBOT3_MODEL=crepe
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 interface show nav2_msgs/action/ComputePathToGoal
ros2 interface show nav2_msgs/action/ComputePathToPose
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_optical_rgb_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_rgb_optical_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_rgb_optical_frame, camera_name: camera}"
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 action list
ros2 action list -t
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
cd turtlebot3_ws/src/
code .
tmux
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
export TURTLEBOT3_MODEL=crepe
ros2 launch vi_to_nav scan_world.launch.py
export TURTLEBOT3_MODEL=custom
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 run vi_to_nav vi_to_tb
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_optical_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
htop
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
ros2 action send_goal /nbv nbv_interfaces/action/Nbv "{origin_x: 147, origin_y: 225.0, frame_name: camera_2_optical_frame, camera_name: camera_2}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_start: False, planner_id: "GridBased"}" 
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_start: False, planner_id: 'GridBased'}" 
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_start: False, planner_id: 'GridBased'}" 
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_start: False, planner_id: GridBased}" 
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_start: False, planner_id: GridBased }" 
ros2 action send_goal /compute_path_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 action send_goal /compute_path_p_to_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 action send_goal /compute_path_po_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 action send_goal /compute_path_potpose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, use_start: false, planner_id: GridBased}" 
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 0.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action list -t
ros2 interface show nav2_msgs/action/NavigateToPose
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: 0.38, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: 0.38, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -0.38, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: 0.38, y: 0.0, z: -0.38, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.38, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -0.38, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -0.76, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -1.2, y: 0.0, z: 0.0, w: 0.92}}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -1.2, y: 0.0, z: -0.38, w: 0.92}}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.94, y: -1.27, z: 0.0}, orientation: {x: -1.2, y: 0.0, z: 0.38, w: 0.92}}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 service call /vi_to_nav/view_point_sampling nbv_interfaces/srv/ViewPointSampling "{centroids: [{x: 2.0, y: 0.5, z: 0.05}], cam_info: {header: {frame_id: "camera_frame"}}}"
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.26, y: -0.74, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.38, w: 0.92}}}}"
ros2 topic list
ros2 topic echo /camera_2/points
cd turtlebot3_ws/src/
code .
cd ..
tmux
