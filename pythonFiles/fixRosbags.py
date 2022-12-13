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
    target_topic = ['mavros/imu/data_frd','height_baro','camera/camera_info','camera/image_raw/compressed','/mavros/imu/static_pressure','/mavros/imu/data']  # Target topic you want to repair
    repair_path = bag_path.replace(".bag", "-repaired.bag")  # Output bag path

    # Open bag
    bag = rosbag.Bag(bag_path, 'r')
    fix_bag = rosbag.Bag(repair_path, "w")  # Create a repaired bag

    # Iterate through bag
    for topic, msg, t in bag.read_messages():
        # Add time back to the target message header


        if topic in target_topic:
            ourCurrentTime = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            ourCurrentTime = ourCurrentTime + rospy.Duration(3.6)
            msg.header.stamp.secs = ourCurrentTime.secs
            msg.header.stamp.nsecs = ourCurrentTime.nsecs

        # Write message to bag
        fix_bag.write(topic, msg, t)

    # Close bag - Very important else you'll have to reindex it
    fix_bag.close()