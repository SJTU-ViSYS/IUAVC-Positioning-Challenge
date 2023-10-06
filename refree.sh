#! /bin/bash
echo "Start your pose estimation node and press ENTER to begin data playback."
read start
seq="seq$1"
team="$2"
output_path=$(pwd)/result/${team}/${seq}
rosbag_path=$(pwd)/dataset/IntelligentUAV2023_${seq}.bag
if [ -d "${output_path}" ]; then rm -rf ${output_path}; fi
mkdir -p ${output_path}
source ../../devel/setup.bash;
bash ./tools/start_judge.sh ${output_path} ${team} ${rosbag_path}
python3 tools/evo_stage4.py -r ./ground_truth/IntelligentUAV2023_${seq}.tum -o ${output_path} 