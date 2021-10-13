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
    data_writer.writerow([msg.header.stamp.secs, 
                          msg.header.stamp.nsecs, 
                          msg.pose.pose.position.x, 
                          msg.pose.pose.position.y])

with open(results_dir +"/"+filename+'_traj.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['x', 'y'])
  # Get all message on specified topics
  for topic, msg, t in bag.read_messages(topics=['/planned_trajectory/path']):
    for point in msg.points:
        data_writer.writerow([point.x, point.y]) 


print("Finished creating csv file!")
bag.close()