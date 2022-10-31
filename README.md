#  无人机智能感知技术竞赛-赛项四-精准定位
### 更新说明
#### 2022/10.31
1.  修复了ubuntu18.04兼容问题;
2.  优化了评分节点加载rosbag方式;
3.  按照最终比赛要求更新了实时性阈值。
### 介绍
[无人飞行器智能感知技术竞赛](https://www.robomaster.com/zh-CN/robo/drone?djifrom=nav_drone)以“智在飞翔”为主题，致力于打造智能感知与控制领域具有全球影响力的技术赛事。竞赛遵循“创新、合作、开放、开源”的指导思想，通过开放、开源和建立竞赛联盟等形式，旨在加速推动智能感知、定位导航与自主控制等领域的技术创新，并积极促进相关创新成果在无人飞行器领域开展转化与应用，发现和挖掘一批优质潜力项目和创新人才，为无人智能产业培养更多的未来技术领军人才。<br>
在无人飞行器智能感知技术竞赛-赛项四-精准定位中，组委会控制一架真实无人机在室内快速飞行，同时录制一段机载相机采集的RGB-D、双目图像以及机载IMU采集的加速度、角速度数据。参赛队伍从Rosbag数据包中读取所需传感器数据，运行状态估计算法，实现无人机状态估计。参赛队伍需利用组委会提供的传感器数据序列，恢复无人机各个时刻相对起飞位置的位姿。组委会将通过对比无人机轨迹真值（通过动捕系统获得）和队伍提交的轨迹，选出精度较高的参赛队伍。
<div align=center>
<img src="doc/seq2_capture2.gif" width="70%"/>
</div>
<p align="center">Figure 1.  Seq2 部分数据</p>

### 竞赛流程与规则
#### 任务
赛前，组委会会公布部分采集到的机载数据传感器数据序列以及通过动捕系统获取到的无人机姿态真值，并且会公布各个传感器的外参和标定数据。参赛学生可以通过公布出来的数据集对自身的定位算法进行验证和调优。线下实体赛中，组委会会公布测评用的机载数据传感器的序列。各参赛队伍下载数据之后，运用自己的算法计算出无人机的位姿和轨迹，并将程序打包成docker上传到服务器，以供评分。
####  评分规则
1.  参赛队将代码封装为docker并上传。组委会在评分时首先启动评分节点和选手提供的roslaunch文件，间隔2s后再播放rosbag；
2.  在 rosbag 播放后，参赛队根据图像、深度与IMU数据，计算输出当前时刻图像帧的位置姿态。注意，输出位姿的时间戳应和当前时刻图像帧的时间戳保持一致，时间戳与图像时间戳不同的位姿数据将被忽略。
3. 	评分节点将接收选手发送的位姿信息，如果该帧位姿的时间戳满足下列两个条件，将被设置为
有效帧，并且记录该帧数据：<br>
    a. 所填写的时间戳和当前图像的时间戳一致;<br>
    b. 位姿发布的系统时间晚于当前图像帧发布的系统时间，且小于40ms（要求参赛队在收到图像后的40ms内计算并发布位姿）.<br>
4.  在 rosbag 播放完成之后，评分节点根据真实轨迹得到每个有效帧的理论真实位姿，并将理论真实位姿和选手的输出位姿进行对比，通过计算 RPE 的 RMSE 进行评分：
$$E_{i,j} = \delta_{est_{i,j}} \ominus \delta_{ref_{i,j}} = (P_{ref,i}^{-1}P_{ref,j})^{-1} (P_{est,i}^{-1}P_{est,j}) \in \mathrm{SE}(3)\$$
$$\mathrm{RMSE} = \sqrt{ \frac{1}{N} \sum_{\forall ~i,j} RPE_{i,j}^2 } \$$
#### 排名方式
1.  有效帧和图像总帧数的比值必须大于80%，否则无排名资格；
2.  定位轨迹与真值更接近（RMSE 越小）的队伍排名靠前；
3.  如果 RMSE 差值绝对值小于 0.0001，则有效帧数较多的队伍排名靠前。
4.  若上述条件均无法排出先后，则比较所有有效帧数的平均延时（精确到 10us），平均延时低者排名靠前

### 评分节点安装教程
1.  cd YOUR_ROS_WORKSPACE/src
2.  git clone https://github.com/SJTU-ViSYS/IntelligentUAVChampionshipStage4.git
3.  cd IntelligentUAVChampionshipStage4/thirdparty/rapidjson
4.  mkdir build && cd build
5.  cmake .. && make
6.  cd YOUR_ROS_WORKSPACE/src/IntelligentUAVChampionshipStage4/thirdparty/stage4_evo
7.  pip install --editable . --upgrade --no-binary evo
8.  cd YOUR_ROS_WORKSPACE
9.  catkin build

### 评分节点使用说明
1.  修改config/settings.json文件，指定队伍名称，采用的图像类型(/front/stereo 或者 /front/rgbd 或者 /back 或者 /all)；
2.  将数据集下载到dataset目录下;
3.  启动位姿估计节点；
4.  执行./refree.sh，输入要测试的数据集(seq1或者seq2);
5.  在result/${team}目录下查看评分结果。

### 数据集说明
#### 数据下载地址：
seq1: https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/dataset/IntelligentUAV_seq1.bag<br>
seq2: https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/dataset/IntelligentUAV_seq2.bag
#### 数据说明
用于测试的数据包含两个sequence。其中seq1较简单，主要挑战点包含快速运动与转向。seq2较难，主要挑战点包含快速运动与转向、较暗环境的特征检测与跟踪，以及场景的明暗变化。<br>
最终测评所用的seq3（线下实体赛时发布）难度介于seq1和seq2之间，场景和seq1和seq2相比会有较大变化（请选手不要以提前建图的方式完成比赛。组委会将对疑似提前建图的队伍进行技术审查,并取消提前建图的队伍的成绩）。
#### 话题说明
##### 前视相机(realsense d455，正视前方)
1.  /front/imu：前视相机imu(帧率200hz)；
2.  /front/color/image_raw：前视相机rgb彩色图像(像素：640x480，帧率30hz)；
3.  /front/infra1/image_rect_raw：前视相机左目相机图像(像素：640x480，帧率30hz)；
4.  /front/infra2/image_rect_raw：前视相机右目相机图像(像素：640x480，帧率30hz)；
5.  /front/aligned_depth_to_color/image_raw：前视相机深度图像(像素：640x480，帧率30hz，和彩色图像对齐)。
##### 后视相机(realsense t265，斜向下观测)
1.  /back/imu：后视相机imu(帧率200hz)。
1.  /back/fisheye1/image_raw：后视相机左目图像(像素：848x800，帧率30hz，鱼眼相机模型)；
2.  /back/fisheye2/image_raw：后视相机右目图像(像素：848x800，帧率30hz，鱼眼相机模型)；

### 真值数据说明
真值由vicon动作捕捉系统捕获得到，真值以txt和tum的格式存储在了GroundTruth目录下。

### 标定文件说明
组委会提供了标定结果，以及标定所用的原始rosbag。
#### 标定结果
组委会采用[allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)标定了前后相机的imu参数，相机标定结果和原始数据可以通过 https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/imu_cal.zip 下载。<br>
[kalibr](https://github.com/ethz-asl/kalibr)标定了前后相机的相机内参、前视相机外参、后视相机外参，标定结果可以通过 https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/cam_cal_res.zip 下载。
#### 相机标定原始数据
##### 前视相机cam标定
https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/nuc_d455_01_cali.bag
##### 前视相机cam-imu标定
https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/nuc_d455_01_cam_imu_cali.bag
##### 后视相机cam标定
https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/nuc_t265_01_cali.bag
##### 后视相机cam-imu标定
https://intelligent-uav-championship.oss-cn-shanghai.aliyuncs.com/calibrate/nuc_t265_01_cam_imu_cali.bag
### 选手输出话题说明
1.  选手的估计结果要以话题名为 /player/odom 的形式实时发送出来，/player/odom 的类型为nav_msgs::Odometry；
2.  赛队在赛项四中可选的信息组合有以下几种：/front/stereo ，/front/rgbd ， /back 以及 /all，每种组合评分节点参考时间戳标准如下: <br>
    a. /front/stereo:  评分节点只会记录和 /front/infra1/image_rect_raw 时间戳一致的/player/odom信息；<br>
    b. /front/rgbd:  评分节点只会记录和/front/color/image_rect_raw时间戳一致的/player/odom信息；<br>
    c. /back/stereo:  评分节点只会记录和/back/fisheye1/image_raw时间戳一致的/player/odom信息；<br>
    d. /all:  评分节点只会记录和 /front/infra1/image_rect_raw 时间戳一致的/player/odom信息。

### 提交说明
1.  按照 https://github.com/RoboMaster/IntelligentUAVChampionshipBase 中的相关说明将评分节点打包成docker镜像的模式；
2.  将doccker镜像重命名为 队名_图像类型.tar (如 sjtu_front_stereo.tar, sjtu_back.tar)的形式并上传.
