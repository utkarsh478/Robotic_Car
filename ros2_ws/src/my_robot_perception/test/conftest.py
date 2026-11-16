import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)                        # synthetic.py
sys.path.insert(0, os.path.dirname(HERE))       # my_robot_perception (source tree)
