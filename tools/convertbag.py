import rosbag
from nav_msgs.msg import Odometry
import os
import sys
if len(sys.argv) > 1 :
	bag_file = sys.argv[1]
else: 
	bag_file = input("请输入bag文件的地址: ")
bag_file = bag_file.strip('"')
bag_file = bag_file.strip('\'')
# 检查输入的文件是否存在
if not os.path.isfile(bag_file):
    print("输入的文件不存在。")
    exit(1)

# 获取文件名和文件目录
file_name = os.path.basename(bag_file)
file_dir = os.path.dirname(bag_file)

# 构建tum文件路径
output_file = os.path.join(file_dir, os.path.splitext(file_name)[0] + ".tum")

# 创建输出文件并打开以供写入
with open(output_file, 'w') as f:
    # 打开rosbag文件
    bag = rosbag.Bag(bag_file)

    # 遍历rosbag中的消息
    for topic, msg, t in bag.read_messages(topics=["/motion_capture/odom"]):
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
        f.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

    # 关闭rosbag文件
    bag.close()

