"""Both detectors in ONE process sharing one camera subscription (DC-31). The nodes keep their
own names, parameters, services and topics (/target_detector/..., /box_detector/...).
  ros2 run my_robot_perception perception_node --ros-args --params-file perception_params.yaml"""
from . import box_detector, target_detector
from .detector_node import run


def main():
    return run(target_detector.make, box_detector.make)


if __name__ == "__main__":
    main()
