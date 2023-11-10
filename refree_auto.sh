#! /bin/bash
seq="seq$1"
team="$2"
output_path=$(pwd)/result/${team}/${seq}
rosbag_path=$(pwd)/dataset/IntelligentUAV2023_${seq}.bag
if [ -d "${output_path}" ]; then rm -rf ${output_path}; fi
mkdir -p ${output_path}
source ../../devel/setup.bash;
screen -S roscore_session -d -m roscore
sleep 1s
screen -S roslaunch_session  -d -m bash ./tools/start_judge.sh ${output_path} ${team} ${rosbag_path}
#roslaunch orb_slam3 start_slam_stereo_i_OAK.launch # 将此处修改为位姿估计节点启动程序
roslaunch vins oak.launch # 将此处修改为位姿估计节点启动程序
session_list=$(screen -ls | grep -o '[0-9]\+\.[^\t]*')
for session in $session_list; do
    session_id=$(echo "$session" | cut -d. -f1)
    screen -S "$session_id" -X quit
done
python3 tools/evo_stage4.py -r ./ground_truth/IntelligentUAV2023_${seq}.tum -o ${output_path} 