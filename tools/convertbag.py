#! /usr/bin/python3
import rosbag
from nav_msgs.msg import Odometry
import os
import sys
import argparse
example_cmd = """
%(prog)s -b seq.bag -o output.bag 
"""
parser = argparse.ArgumentParser(
    description="Evaluate the error of the slam/vio output.",
    usage=example_cmd,
)

parser.add_argument(
    "-b", "--bag", action="store", required=True, help="Path of the raw bag"
)
parser.add_argument(
    "-o", "--output_bag", action="store", required=True, help="Path of the output bag"
)
parser.add_argument(
    "-g", "--generate_tum", action="store_true", help="Enable generate tum file"
)
args = parser.parse_args()
delete_topics = {"/mavros/rc/in"}
bag_file_path = args.bag.strip('"').strip('\'')
# 检查输入的文件是否存在
if not os.path.isfile(bag_file_path):
    print("输入的文件不存在。")
    exit(1)

# 获取文件名和文件目录
file_name = os.path.basename(bag_file_path)
file_dir = os.path.dirname(bag_file_path)

# 构建tum文件路径
output_tum = os.path.join(file_dir, os.path.splitext(file_name)[0] + ".tum")
output_bag = args.output_bag
if not output_bag.endswith(".bag"):
    print(f"Error: output bag: {output_bag} is not a *bag* file.")
if args.generate_tum:
    tum_file = open(output_tum, 'w')
# 创建输出文件并打开以供写入
with rosbag.Bag(bag_file_path) as in_bag, rosbag.Bag(output_bag, 'w') as bag_file:
    # 遍历rosbag中的消息
    for topic, msg, t in in_bag.read_messages():
        if topic == "/motion_capture/odom" and args.generate_tum:
        # 提取odom消息的数据并将其写入到txt文件中
            timestamp = msg.header.stamp.to_sec()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

                # 将数据以TUM格式写入txt文件
            tum_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
        if not topic in delete_topics:
            bag_file.write(topic, msg, t)            
if args.generate_tum:
    tum_file.close()
