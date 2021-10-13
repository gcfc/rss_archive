import sys
import os
import csv
import rosbag
import rospy

##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# <navigate to current directory>
# python bag2csv.py . <name_of_rosbag.bag>
# ##################

filename = sys.argv[2]
directory = sys.argv[1]
print("Reading the rosbag file")
if not directory.endswith("/"):
  directory += "/"
extension = ""
if not filename.endswith(".bag"):
  extension = ".bag"
bag = rosbag.Bag(directory + filename + extension)

# Create directory with name filename (without extension)
results_dir = directory + filename[:-4] + "_results"
if not os.path.exists(results_dir):
  os.makedirs(results_dir)

print("Writing robot joint state data to CSV")

with open(results_dir +"/"+filename+'_odom.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time_secs', 'time_nsecs', 'odom_x', 'odom_y'])
  # Get all message on specified topics
  for topic, msg, t in bag.read_messages(topics=['/odom']):
    # Only write to CSV if the message is for our robot
    data_writer.writerow([msg.header.stamp.secs, 
                          msg.header.stamp.nsecs, 
                          msg.pose.pose.position.x, 
                          msg.pose.pose.position.y])

with open(results_dir +"/"+filename+'_pose.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time_secs', 'time_nsecs', 'odom_x', 'odom_y'])
  # Get all message on specified topics
  for topic, msg, t in bag.read_messages(topics=['/pf/pose/odom']):
    # Only write to CSV if the message is for our robot
    data_writer.writerow([msg.header.stamp.secs, 
                          msg.header.stamp.nsecs, 
                          msg.pose.pose.position.x, 
                          msg.pose.pose.position.y])

with open(results_dir +"/"+filename+'_posearray.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time_secs', 'time_nsecs', 'poses'])
  # Get all message on specified topics
  for topic, msg, t in bag.read_messages(topics=['/posearray']):
    # Only write to CSV if the message is for our robot
    t_poses = []
    for pose in msg.poses:
        t_poses.append((pose.position.x, pose.position.y))
    data_writer.writerow([msg.header.stamp.secs, 
                          msg.header.stamp.nsecs, 
                          str(t_poses)])

with open(results_dir +"/"+filename+'_avg.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['time_secs', 'time_nsecs', 'x_avg','y_avg'])
  # Get all message on specified topics
  for topic, msg, t in bag.read_messages(topics=['/avg_marker']):
    # Only write to CSV if the message is for our robot
    data_writer.writerow([msg.header.stamp.secs, 
                          msg.header.stamp.nsecs, 
                          msg.points[0].x, 
                          msg.points[0].y])

print("Finished creating csv file!")
bag.close()