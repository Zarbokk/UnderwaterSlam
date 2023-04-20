import os
import sys

import rosbag
import rospy
def print_usage():
    print("Usage: fixbag.py <ros bag> <target topic>")
    print("Usage: fixbag.py recorded.bag /robot/imu")


if __name__ == "__main__":
    # Check CLI args
    # if len(sys.argv) != 3:
    #     print_usage()
    #     exit(-1)

    bag_path = "/home/tim-external/dataFolder/ValentinBunkerKeller/diveKellerValentinConstantSonarNOEKF.bag"  # Path to ROS bag you want to repair
    target_topic = ['sonar/intensity']  # Target topic you want to repair

    # Open bag
    bag = rosbag.Bag(bag_path, 'r')
    # fix_bag = rosbag.Bag(repair_path, "w")  # Create a repaired bag

    print(bag.get_message_count(target_topic))