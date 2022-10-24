#!/bin/bash
settings_path=config/settings.json
data=$(cat ${settings_path} | sed -r 's/",/"/' | egrep -v '^[{}]' | sed 's/"//g' | sed 's/:/=/1')
declare $data;
output_path=$(pwd)/result/${team}
if [ -d "${output_path}" ]; then rm -rf ${output_path}; fi
mkdir -p ${output_path}
source ../../devel/setup.bash;
roslaunch stage4_refree refree.launch output_path_base:=${output_path} team:=${team} sensors:=${sensors}
if [ -f "${output_path}/${team}_result.json" ]; then
    if [ -f "${output_path}/${team}_result_traj.txt" ]; then 
        python3 thirdparty/stage4_evo/evo/main_traj.py euroc ${output_path}/${team}_result_traj.txt --save_as_tum
        mv ${team}_result_traj.tum ${output_path};
        if [ -f ${output_path}/${team}_result.zip ]; then rm ${output_path}/${team}_result.zip; fi
        python3 thirdparty/stage4_evo/evo/main_rpe.py tum ./GroundTruth/seq3_gt.tum ${output_path}/${team}_result_traj.tum -a --save_results ${output_path}/${team} -p 
    fi
fi