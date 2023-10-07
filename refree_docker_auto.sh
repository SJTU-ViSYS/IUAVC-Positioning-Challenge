#! /bin/bash
seq="seq$1"
team="$2"
image="$3"
output_path=$(pwd)/result/${team}/${seq}
rosbag_path=$(pwd)/dataset/IntelligentUAV2023_${seq}.bag
if [ -d "${output_path}" ]; then rm -rf ${output_path}; fi
mkdir -p ${output_path}
source ../../devel/setup.bash;
screen -S roscore_session -d -m bash ./tools/start_roscore.sh
sleep 1s
screen -S docker_session -d -m  docker run --rm -i -t -e ROS_IP='172.17.0.2' -e ROS_MASTER_URI='http://172.17.0.1:11311' ${image}
bash ./tools/start_judge.sh ${output_path} ${team} ${rosbag_path}
session_list=$(screen -ls | grep -o '[0-9]\+\.[^\t]*')
for session in $session_list; do
    session_id=$(echo "$session" | cut -d. -f1)
    screen -S "$session_id" -X quit
done
python3 tools/evo_stage4.py -r ./ground_truth/IntelligentUAV2023_${seq}.tum -o ${output_path} -p