Start the n64 controller setup  
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='n64'

Start the micro-ros agent  
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
