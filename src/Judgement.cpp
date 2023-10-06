#include "Judgement.h"

Judgement::Judgement(ros::NodeHandle &_nh) : mNodeHandle(_nh)
{
    ros::param::get("/refree/realtime_performance_th", mdRealtimePerformanceTh);
    ros::param::get("/refree/realtime_performance_per_th", mdRealtimePerformancePerTh);
    ros::param::get("/refree/rosbag_path", mstrRosbagPathBase);
    ros::param::get("/refree/output_path_base", mstrOutFilePathBase);
    
    if(mstrOutFilePathBase.back()!='/'){
        mstrOutFilePathBase+="/";
    }
    ROS_INFO("output_path_base: %s.", mstrOutFilePathBase.c_str());
    std::string jsonFilePath;
    ros::param::get("/refree/team", mstrTeam);

    mstrMajorImgTopic="/oak_ffc/front_left/image";
    std::string outFilePath = mstrOutFilePathBase + mstrTeam + "_result_traj.tum";
    mfOutFile.open(outFilePath);
    
    configFile.GetParam("major_image_topic", mstrMajorImgTopic);

    odomSub = mNodeHandle.subscribe("/player/odom", 100, &Judgement::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    rosbag::PlayerOptions opts;
    opts.bags.push_back(mstrRosbagPathBase);
    opts.quiet = false;
    mBagPlayer = rosbag::Player(opts);
    mBagPlayer.setMajorTopic(mstrMajorImgTopic);
    mlMsgRec.reserve(1500);
    mlImageStamp_ReciveStamp.reserve(1500);
}
void Judgement::Loop()
{
    double waitSec;
    ros::param::get("/refree/waitfor_player_sec", waitSec);
    ROS_INFO("Waiting %lf s...", waitSec);
    ros::Duration(waitSec).sleep();
    ROS_INFO("Bag is playing ...");
    mBagPlayer.publish();
    ROS_INFO("Bag play finished. Start judge realtime performance...");
    double realtimePercentage = JudgeRealTime();
    ROS_INFO("Percentage of frames meeting real-time requirements is %lf.", realtimePercentage);
    std::string resJsonFilePath;
    resJsonFilePath = mstrOutFilePathBase + "/" + mstrTeam + "_result.json";
    ROS_DEBUG("res write in %s", resJsonFilePath.c_str());
    std::ofstream resJsonFile;
    resJsonFile.open(resJsonFilePath); 
    if (realtimePercentage > mdRealtimePerformancePerTh)
    {
        resJsonFile << "{ \"Realtime\": \"qualified\", ";
        ROS_INFO("Meeting real-time requirements. Start calculate RPE.");
    }
    else
    {
        resJsonFile << "{ \"Realtime\": \"unqualified\", ";
        ROS_INFO("Not meeting real-time requirements.");
    }
    resJsonFile << "\"delay\": " << mfDelayAvg <<", \"ValidFramePercentage\": " <<realtimePercentage<<" }" << std::endl;  

}
void Judgement::OdomCallback(const nav_msgs::OdometryConstPtr &odomPtr)
{
    mlMsgRec.emplace_back(ros::Time::now().toNSec(), odomPtr);
}
double Judgement::JudgeRealTime()
{
    mlImageStamp_ReciveStamp = mBagPlayer.iImageStamp_ReciveStamp_;
    std::size_t totalFrame = mlImageStamp_ReciveStamp.size();
    if (totalFrame == 0)
    {
        ROS_ERROR("Refree Node recive 0 odom msg.");
        return 0.0;
    }
    std::size_t countValidFrame = 0;
    double validDelaySum = 0;

    for (auto &msg : mlMsgRec)
    {
        uint64_t odomGetStamp = static_cast<uint64_t>(msg.first); //获取到该帧位姿的时间
        uint64_t imageStamp = msg.second->header.stamp.toNSec();  //该帧位姿代表的图片时间戳
        int startIndex = 0;
        int endIndex = mlImageStamp_ReciveStamp.size();
        int midIndex = (startIndex + endIndex) / 2;
        while (StampDuration(mlImageStamp_ReciveStamp[midIndex].first, imageStamp) > msec2nsec(1) && startIndex < endIndex - 1)
        {
            if (mlImageStamp_ReciveStamp[midIndex].first > imageStamp)
            {
                endIndex = midIndex;
            }
            else
            {
                startIndex = midIndex;
            }
            midIndex = (startIndex + endIndex) / 2;
        }
        uint64_t publisherImageStamp = mlImageStamp_ReciveStamp[midIndex].first;
        uint64_t publishedStamp = mlImageStamp_ReciveStamp[midIndex].second;
        uint64_t duration = StampDuration(odomGetStamp, publishedStamp);

        ROS_DEBUG("Recive a pose at %lf, timestamp is %lf, and publish it at %lf. Delay = %lf ms.",
                  nsec2msec(odomGetStamp), nsec2msec(imageStamp), nsec2msec(publishedStamp), nsec2msec(duration));
        if (StampDuration(imageStamp, publisherImageStamp) < msec2nsec(1) && duration < msec2nsec(mdRealtimePerformanceTh))
        {
            nav_msgs::OdometryConstPtr msgPtr = msg.second;
            mfOutFile << std::fixed << std::setprecision(15) << nsec2sec(imageStamp) << " " 
                      << msgPtr->pose.pose.position.x << " " << msgPtr->pose.pose.position.y << " " << msgPtr->pose.pose.position.z << " "
                      << msgPtr->pose.pose.orientation.x << " " << msgPtr->pose.pose.orientation.y << " " << msgPtr->pose.pose.orientation.z << " "
                      << msgPtr->pose.pose.orientation.w<< std::endl;
            countValidFrame++;
            validDelaySum += nsec2msec(duration);
            startIndex = midIndex;
        }
    }
    if (countValidFrame == 0)
    {
        return 0;
    }
    mfDelayAvg = validDelaySum / countValidFrame;
    ROS_INFO("Average delay = %lf ms", mfDelayAvg);
    return static_cast<double>(countValidFrame) / totalFrame;
}