"""Green home-box detector alone (SPEC §9.2).  ros2 run my_robot_perception box_detector"""
from .detector_node import DetectorNode, run
from .pipeline import BOX_DEFAULTS, detect_box


def make(subscribe=True):
    return DetectorNode("box_detector", "box", BOX_DEFAULTS, detect_box, (0.1, 0.8, 0.2), subscribe)


def main():
    return run(make)


if __name__ == "__main__":
    main()
