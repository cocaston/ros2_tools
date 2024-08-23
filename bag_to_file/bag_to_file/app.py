import rclpy
import os
import yaml

from ament_index_python.packages import get_package_share_path
from bag_to_file.topic_to_file import Transfer


def main():
    rclpy.init()
    transfer = Transfer()
    rclpy.spin(transfer)
    rclpy.shutdown()