"""Red target cylinder detector alone (SPEC §9.1).  ros2 run my_robot_perception target_detector"""
from .detector_node import DetectorNode, run
from .pipeline import TARGET_DEFAULTS, detect_target


def make(subscribe=True):
    return DetectorNode("target_detector", "target", TARGET_DEFAULTS, detect_target, (1.0, 0.1, 0.1), subscribe)


def main():
    return run(make)


if __name__ == "__main__":
    main()
