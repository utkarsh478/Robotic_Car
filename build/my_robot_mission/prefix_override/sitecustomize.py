import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/utkarsh/Robotic_car/Robotic_Car/install/my_robot_mission'
