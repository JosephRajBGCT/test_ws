git clone https://github.com/JosephRajBGCT/test_ws.git
cd ~/test_ws

# Automatically install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt #install python packages


# Build the workspace
colcon build
source install/setup.bash

echo "source ~/test_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 run multi_ultrasonic_sensor multi_sensor_node
