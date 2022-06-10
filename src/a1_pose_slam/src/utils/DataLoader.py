from typing import List

# import bagpy
import rosbag

def get_sensor_measurements(bag_name: str, topics: List[str]):
    """Extract the sensor measurements from a given ROSbag.

    Args:
        bag_name: The name of the ROSbag with the data measurements.
        topics: A list of topics that should be read from in the ROSbag.
    Returns:
        data: A generator of the sensor measurements.
    """
    bag = rosbag.Bag(bag_name)
    return bag.read_messages(topics)

def get_bagpy_sensor_measurements(bag_name: str, topics: List[str]):
    """Extract the sensor measurements from a given ROSbag using bagpy.
    This is needed when ROS installation is unavailable (Windows and MacOS).

    Args:
        bag_name: The name of the ROSbag with the data measurements.
        topics: A list of topics that should be read from in the ROSbag.
    Returns:
        data: A generator of the sensor measurements.
    """
    bag = bagpy.bagreader(bag_name)