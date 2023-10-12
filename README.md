#  无人机智能感知技术竞赛-精准定位
## 1. 介绍
>[无人飞行器智能感知技术竞赛](https://www.robomaster.com/zh-CN/robo/drone?djifrom=nav_drone)以“智在飞翔”为主题，致力于打造智能感知与控制领域具有全球影响力的技术赛事。竞赛遵循“创新、合作、开放、开源”的指导思想，通过开放、开源和建立竞赛联盟等形式，旨在加速推动智能感知、定位导航与自主控制等领域的技术创新，并积极促进相关创新成果在无人飞行器领域开展转化与应用，发现和挖掘一批优质潜力项目和创新人才，为无人智能产业培养更多的未来技术领军人才。<br>
>在无人飞行器智能感知技术竞赛-赛项四-精准定位中，组委会控制一架真实无人机在室内快速飞行，同时录制一段机载相机采集的多目图像以及机载IMU采集的加速度、角速度数据。参赛队伍从Rosbag数据包中读取所需传感器数据，运行状态估计算法，实现无人机状态估计。参赛队伍需利用组委会提供的传感器数据序列，恢复无人机各个时刻相对起飞位置的位姿。组委会将通过对比无人机轨迹真值（通过动捕系统获得）和队伍提交的轨迹，选出精度较高的参赛队伍。
<div align=center>
<img src="doc/seq3parten.gif"/>
</div>
<p align="center">Figure 1.  Seq3 部分数据</p>

## 2. 竞赛流程与规则
### 2.1 任务
>赛前，组委会会公布部分采集到的机载数据传感器数据序列以及通过动捕系统获取到的无人机姿态真值，并且会公布各个传感器的外参和标定数据。参赛学生可以通过公布出来的数据集对自身的定位算法进行验证和调优。实际比赛期间,各参赛队伍将程序打包成docker上传到服务器，以供评分。
###  2.2 评分规则
>+  参赛队将代码封装为docker并上传。组委会在评分时首先启动评分节点和选手提供的roslaunch文件，间隔2s后再播放rosbag；
>+  在 rosbag 播放后，参赛队根据图像与IMU数据，计算输出当前时刻图像帧的位置姿态。注意，输出位姿的时间戳应和当前时刻图像帧的时间戳保持一致，时间戳与图像时间戳不同的位姿数据将被忽略。
>+	评分节点将接收选手发送的位姿信息，如果该帧位姿的时间戳满足下列两个条件，将被设置为有效帧，并且记录该帧数据：<br>
    a. 所填写的时间戳和当前图像的时间戳一致;<br>
    b. 位姿发布的系统时间晚于当前图像帧发布的系统时间，且小于40ms（要求参赛队在收到图像后的40ms内计算并发布位姿）.<br>
>+  在 bag包播放完成之后，评分节点采用记录下来的位姿中前1/3的轨迹与真实轨迹进行对齐；然后将理论真实位姿和对齐后的选手的输出位姿进行对比，通过计算APE的RMSE进行评分：
>$$E_{i,j} = (P_{ref,i}^{-1}P_{est,j})\$$
>$$\mathrm{RMSE} = \sqrt{ \frac{1}{N} \sum_{\forall ~i,j} E_{i,j}^2 } \$$
>+ 	每支参赛队的提交的代码将反复进行5次评分，最终的RMSE成绩为5次评分的最低值；
>+  如果参赛队的APE的RMSE最低值大于5m，那么该参赛队列为A类排名，否则将被列为B类排名。在B类排名中，根据计算RPE的RMSE对参赛队重新排名：
>$$E_{i,j} = \delta_{est_{i,j}} \ominus \delta_{ref_{i,j}} = (P_{ref,i}^{-1}P_{ref,j})^{-1} (P_{est,i}^{-1}P_{est,j}) \in \mathrm{SE}(3)\$$
>$$\mathrm{RMSE} = \sqrt{ \frac{1}{N} \sum_{\forall ~i,j} E_{i,j}^2 } \$$
>+ 关于APE和RPE，可以详细参考：[EVO文档](https://github.com/MichaelGrupp/evo/blob/v1.23.0/notebooks/metrics.py_API_Documentation.ipynb)
### 2.3 排名方式
>+  有效帧和图像总帧数的比值必须大于80%，否则无排名资格；
>+  如果APE 的 RMSE 小于 5m,那么定位轨迹与真值更接近 (APE 的 RMSE越小) 的队伍排名靠前 (A 类排名)；
>+  如果 APE 的 RMSE 均大于 5m,那么 RPE 的 RMSE 越小的队伍排名靠前(B类排名);
>+  如果 RMSE 差值绝对值小于 0.0001，则有效帧数较多的队伍排名靠前:
>+  若上述条件均无法排出先后，则比较所有有效帧数的平均延时(精确到10us)，平均延时低者排名靠前。
## 3. 评分节点编译使用
> 编译评分节点
>+  `cd YOUR_ROS_WORKSPACE/src`
>+  git clone （待定）
>+  `cd YOUR_ROS_WORKSPACE`
>+  `catkin build`
----
> 运行评分节点
>+  将数据集下载到dataset目录下;
>+  启动位姿估计节点；
>+  在*IntelligentUAVChampoinshipStage4*目录下打开终端，输入指令：`./refree.sh  [sequence_number] [team]`，其中***sequence_number***是seq编号，***team***是自定义队伍名称， 如：`./refree.sh  1 sjtu`;
>+  在评分节点终端输入回车键，开始播放数据；
>+  数据播放结束后，在*result/[team]/[seq]* 目录下查看评分结果。

## 4. 数据集
### 4.1 数据下载地址：
<div align=center>

seq|大小/GB|长度/s|百度网盘|谷歌云盘|阿里云|
:--:|:--:|:--:|:--:|:--:|:--:|
1|1.5|50|[提取码：f490](https://pan.baidu.com/s/16MQNRhdljvrc-Td_oj09CA)|[链接](https://drive.google.com/file/d/1oWS038f2ckRZygXCYVXzWnqU_79Cwu8q/view?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/datasets/IntelligentUAV2023_seq1.bag)|
2|2.5|87|[提取码：0id0](https://pan.baidu.com/s/1TbrjrDt0aS-gpmyI9sZ8gg)|[链接](https://drive.google.com/file/d/1aGHneHb9FYjYX1R7qILsKXqeeCjrF7lX/view?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/datasets/IntelligentUAV2023_seq2.bag)|
3|2.0|68|[提取码：pair](https://pan.baidu.com/s/1uIGWzsoj8wi631GD-411tg)|[链接](https://drive.google.com/file/d/1-9tpv_7gPNrxjD14PK8q890fXl-Zdy9p/view?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/datasets/IntelligentUAV2023_seq3.bag)|

</div>

### 4.2 数据说明
>用于测试的数据包含3个sequence，三个数据集的特点如下：
<div align="center">

seq|速度|快速转向|场景明暗变化|黑暗片段|动态物体|
:--:|:--:|:--:|:--:|:--:|:--:|
1|慢|包含|不包含|不包含|包含|
2|慢|包含|不包含|不包含|包含|
3|较快|包含|包含|不包含|包含|
</div>

## 5. 传感器
### 5.1 环视相机
>组委会采用OAK FFC4p相机进行数据采集，相机组由四个相机以及一个板载IMU组成，各个相机的名称以及分布如下：<br>
<div align=center>
<img src="doc/drone.jpg" width="70%"/>
</div>
<p align="center">Figure 2.  相机分布</p>

>OAK FFC4p相关话题名称如下：

>+  `/oak_ffc/front_left/image`，前视左侧相机，像素：680x400，帧率30hz
>+  `/oak_ffc/front_right/image`，前视右侧相机，像素：680x400，帧率30hz
>+  `/oak_ffc/left/image`，侧边左视相机，像素：680x400，帧率30hz
>+  `/oak_ffc/right/image`，侧边右视相机，像素：680x400，帧率30hz
>+  `/oak_ffc/imu`，板载IMU，帧率100hz
### 5.2 飞控IMU
>数据集中包含了飞控IMU信息，帧率230hz，话题名称为`/mavros/imu/data`

## 6. 参考真值
>参考真值由动作捕捉系统捕获得到，以`/motion_capture/odom`的ros话题形式存储在bag数据集中（正式比赛的bag包不包含该话题）。同时，参考真值也以tum格式存储在*ground_truth* 目录下。

## 7. 输出话题
>+  参赛队的估计结果要以话题名为 `/player/odom` 的形式实时发送出来，`/player/odom` 的类型为*nav_msgs::Odometry*；
>+ **注意**：请在发布话题时，要手动填写header中的时间戳，评分节点只会记录和 `/oak_ffc/front_left/image` 时间戳一致的`/player/odom`信息: <br>

## 8. 标定文件
>组委会提供了标定结果，以及标定所用的原始rosbag。

<div align="center">

说明|百度网盘|谷歌云盘|阿里云|
:--:|:--:|:--:|:--:|
飞控IMU-相机内外参结果|[提取码：qmtt](https://pan.baidu.com/s/1g_a2EEymg7jZLqxfTICRnQ)|[链接](https://drive.google.com/drive/folders/1-PBsh7Dq1YBWdkvlD7YUuZhamPmfLDYl?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/cali_data/mavros_imu.zip)|
板载IMU-相机内外参结果|[提取码：gbip](https://pan.baidu.com/s/1Q9HyWzW-HuhAtNEVE_BaZQ)|[链接](https://drive.google.com/drive/folders/1-LQGFShkuSb_ylQiElDkwP_dcOR2egFp?usp=sharing)| [链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/cali_data/oak_imu.zip)|
IMU标定结果|[提取码：2idj](https://pan.baidu.com/s/15T7JDsTVm56YTILaXCuaQw)|[链接](https://drive.google.com/drive/folders/1-Qwb8bZEX9uUjdjo3jjm7Mywo8_JC0fD?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/cali_data/imu_cali.zip)|
内外参标定原始数据|[提取码：4zm1](https://pan.baidu.com/s/1cJj-WcmwjrTcIe4nfuf3Pw)|[链接](https://drive.google.com/drive/folders/1-0DcLUZ4vPFvZcCSN5A6QA4QTSbGLpD3?usp=sharing)|[链接](https://intelligentuav2023.oss-cn-hangzhou.aliyuncs.com/cali_data/orin_data.zip)|
</div>


## 9. 提交

>下载并导入镜像  
>+ `wget https://stg-robomasters-hz-q0o2.oss-cn-hangzhou.aliyuncs.com/student_image/student_basic_dev_0825.tar`  
>+ `docker load < student_basic_dev_0825.tar`
----
> 将程序封装入镜像中
>+ 使用如下指令打开一个容器，*-v* 指令会将主机中的 *workspace*  文件夹挂载到容器中的 */home/tmp* 中，使得容器可以访问主机文件夹的文件  
 `docker run -it -v /path/to/workspace/:/home/tmp   student_basic_dev_0825`  
>+ 在容器终端中进入 ***/home*** 目录  
`cd /home`  
>+ 在容器终端中用 ***/home/tmp*** 中的src文件夹覆盖 ***/home/student_basic_dev*** 的src文件夹  
`rm -r ./student_basic_dev/*`  
`cp -r ./tmp/src ./student_basic_dev/`  
>+ 在容器终端中删除已编译的内容后重新编译    
`cd student_basic_dev/`  
`rm -r devel/ build/ logs/`  
`source /opt/ros/noetic/setup.bash `  
`catkin build`   
>+ 在容器终端中添加自动启动命令  
`sudo apt install vim`  
`vim /etc/bash.bashrc `   
按 i 建进入编辑模式， 在最下面添加程序启动命令，例如：  
`source /home/student_basic_dev/devel/setup.bash`  
`roslaunch orb_slam3 start.launch`  
按 esc 退出编辑模式后，输入 `:wq` 退出编辑器
>+ 在主机中打开另一个终端，查看容器号并导出新镜像  
`docker ps -aq`  
`docker commit [OPTIONS] CONTAINER [REPOSITORY[:TAG]]`  
其中 ***[OPTIONS]*** 是可选项， ***CONTAINER*** 输入上一步中获得的容器ID, ***[REPOSITORY:TAG]*** 是导出的镜像名称及版本号，可随意填写， 例如：
`docker commit 694861df819c myimages:v0.1`
----
> 在主机中测试镜像
>+ 将如下指令中的 [镜像：TAG] 换成自己的对应名称即可启动测试  
`docker run --rm -i -t -e ROS_IP='172.17.0.2' -e ROS_MASTER_URI='http://172.17.0.1:11311'  [镜像：TAG]`
>+ 启动评分节点
----
>当容器启动后，评分节点运作正常时，一个完整的可提交的镜像制作完成,导出镜像即可   
`docker image save [镜像：TAG] > test.tar`  
在主机工作目录下会出现 *test.tar* 文件，该文件即为可提交镜像
>将 *test.tar*重新命名为 *[school]_[team].tar*，例如*sjtu_visys.tar*  
### 注意:  
>+ 服务器会在外部随机分配ip给容器，不能在镜像中的启动文件中提供 *ROS_IP* 和 *ROS_MASTER_URI* 这两个环境变量，否则服务器与容器将无法连接;   
