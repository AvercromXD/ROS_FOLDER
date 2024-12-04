cd turtlebot3_ws/
ls
cd ..
ls
cd openvdb/
ls
mkdir build
cd build/
ls
cmake ..
make -j4
sudo make install
cd ../..
cd turtlebot3_ws/
colcon build --symlink-install
exit
