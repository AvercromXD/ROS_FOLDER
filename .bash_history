colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 interface show std_srvs/srv/Trigger
ros2 service list
ros2 service info nbv/setup_nbv
ros2 service type /nbv/setup_nbv
ros2 interface show nbv_interfaces/srv/SetupNBV
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x:= 307, pixel_y:= 288}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 288}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ls
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 action send_goal /nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217, optical_frame_name: camera_optical_frame, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217, camera_name: camera}"
ros2 action send_goal /nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action send_goal /nbv n/find_nbv bv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action list
ros2 action list
ros2 service list
ros2 action list
ros2 run vi_to_nav vi_to_nav_tb
ros2 action list
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 service call /nbv/setup_nbv nbv_interfaces/srv/SetupNBV "{pixel_x: 307, pixel_y: 217}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action list
ros2 action list -t
ros2 interface show nbv_interfaces/action/NBV
ros2 interface show nbv_interfaces/action/MoveToPose
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 1.14, y: 1.0, z: 0.1}, orientation: {z: -0.41, w: 0.912}}, camera_frame: camera_frame}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 0.91, y: -0.22, z: 0.1}, orientation: {z: 0.195, w: 0.981}}, camera_frame: camera_frame}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 1.513, y: -0.399, z: 0.1}, orientation: {z: 0.450, w: 0.893}}, camera_frame: camera_frame}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 1.323, y: -0.328, z: 0.1}, orientation: {z: -0.230, w: 0.973}}, camera_frame: camera_frame}"
ros2 action send_goal /nbv/find_nbv nbv_interfaces/action/NBV "{origin_x: 307, origin_y: 217}"
rviz2
ros2 topic echo /vdb_mapping_lifecycle/vdb_map_updates
ros2 action list
ros2 action list
ros2 service list
ros2 service type vi_to_nav/view_point_sampling
ros2 service info vi_to_nav/view_point_sampling
ros2 service find vi_to_nav/view_point_sampling
ros2 service list
rviz2
rqt
cd nbv_ws/src/
code .
tmux
cd nbv_ws/
source install/setup.bash
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
cd nbv_ws/
ls
colcon build --symlink-install
cd ..
cd vdb_ws/
ls
colcon build --symlink-install
cd ..
cd nbv_ws/
colcon build --symlink-install
source ../vdb_ws/install/setup.bash
colcon build --symlink-install
cd src/
code .
ros2 interface show geometry_msgs/msg/Point
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
source install/setup.bash
ros2 launch vi_to_nav scan_world.launch.py
rqt
ros2 action list -t
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
clear
tmux
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
rqt
rqt
rqt
rqt
cd nbv_ws/src/
code .
ls
cd ..
tmux
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 interface show sensor_msgs/msg/CameraInfo
ros2 run nbv_controller controller
rqt
rqt
rqt
rqt
rqt
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
rviz2
cd nbv_ws/src/
code .
tmux
cd vdb_ws/src/
code .
cd nbv_ws/
cd src/
code .
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
cd ../vdb_ws/
code .
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
cd ../nbv_ws/
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ls
cd ..
ls
cd nbv_ws/src/nbv/vi_to_nav/config/
ls
rviz2 -d marker.rviz 
ros2 launch vi_to_nav nbv.launch.py
cd ../../..
c d..
cd ..
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
colcon build --symlink-install --packages-select vi_to_nav --cmake-clean-first
colcon build --symlink-install --packages-select vi_to_nav --cmake-clear-cache
colcon build --symlink-install --packages-select vi_to_nav --cmake-clean-cache
cd build/vi_to_nav/
ls
cd ..
rm -rf vi_to_nav/
cd ..
colcon build --symlink-install --packages-select vi_to_nav --cmake-clean-cache
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
sudo apt install openmp
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
rqt
rqt
rqt
rqt
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 1.323, y: -0.328, z: 0.1}, orientation: {z: -0.230, w: 0.973}}, camera_frame: camera_frame}"
ros2 action send_goal vi_to_nav/move_to_pose nbv_interfaces/action/MoveToPose "{camera_pose: {position: {x: 1.323, y: -0.328, z: 0.1}, orientation: {z: -0.230, w: 0.973}}, camera_frame: camera_frame}"
ros2 action list
ros2 service list
rqt
rqt
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
man colcon
colcon --help
colcon build --help
colcon build --cmake-args --help
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
ros2 run nbv_controller controller
rviz2
htop
htop
tmix
tmux
cd nbv_ws/src/
code .
cd ..
colcon build --cmake-args --help
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 launch vi_to_nav nbv.launch.py
tmux
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 lifecycle set /vdb_mapping_lifecycle activate
ros2 lifecycle list
ros2 lifecycle -h
ros2 lifecycle nodes
ros2 lifecycle nodes
ros2 lifecycle set /vdb_mapping_lifecycle activate
ros2 lifecycle set /vdb_mapping_lifecycle deactivate
ros2 lifecycle set /vdb_mapping_lifecycle activate
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 launch vi_to_nav nbv.launch.py
mkdir images
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
cd nbv_ws/
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 run nbv_controller controller
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
cd images/
ls
ls
ls -a
cd ..
ls
cd images/
ls
ls
ls
ls
tmux
ros2 launch vi_to_nav scan_world.launch.py
colcon build --symlink-install
colcon build --symlink-install
ros2 launch vi_to_nav scan_world.launch.py
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 launch vi_to_nav nbv.launch.py
ros2 launch vi_to_nav nbv.launch.py
cd nbv_ws/src/
code .
cd ..
colcon build --symlink-install
tmux
cd nbv_ws/src/
code .
cd ../../vdb_ws/src/
code .
ls -a
rm -rf .vscode/
cd vdb_mapping/include/vdb_mapping/
ls
vim VDBMapping.h
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
ros2 action send_goal /nbv/start_procedure nbv_interfaces/action/ScanObject "{}"
cd nbv_ws/build/vi_to_nav/config/
ls
rm vdb_params_virtual.yaml 
cd ../../..
cd ..
ls
cd nbv_ws/src/nbv/vi_to_nav/config/
ls
nvim nbv_params.yaml 
vim nbv_params.yaml 
cd 
mkdir images
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav scan_world.launch.py
ros2 launch vi_to_nav nbv.launch.py
cd nbv_ws/
cd ../vdb_ws/
ls
cd ../nbv_ws/
colcon build --symlink-install
colcon build --symlink-install
source install/setup.bash
ros2 launch vi_to_nav nbv.launch.py
cd src/nbv/nbv_calculator/
ls
cd include/nbv_calculator/
ls
vim nbv_utils.hpp 
cd 
cd nbv_ws/
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
ros2 interface show std_msgs/msg/ColorRGBA
colcon build --symlink-install
ros2 launch vi_to_nav nbv.launch.py
tmux
