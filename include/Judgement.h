#include <ros/ros.h>
#include <vector>
#include "JsonInterface.h"
#include <string>
#include "stage4_rosbag/player.h"
#include <nav_msgs/Odometry.h>
#include <fstream>
class Judgement
{
public:
    Judgement(ros::NodeHandle &_nh);
    void Loop();

private:
    double mdRealtimePerformanceTh;
    double mdRealtimePerformancePerTh;
    std::string msResultPath;
    rosbag::Player mBagPlayer;
    JsonInterface configFile;
    double mfDelayAvg;
    ros::NodeHandle mNodeHandle;
    ros::Subscriber odomSub;
    std::vector<std::pair<uint64_t, uint64_t>> mlImageStamp_ReciveStamp;
    std::vector<std::pair<uint64_t, const nav_msgs::OdometryConstPtr>> mlMsgRec;
    std::ofstream mfOutFile;
private:
    void OdomCallback(const nav_msgs::OdometryConstPtr &odomPtr);
    double JudgeRealTime();
    int GetMsgSize();
    inline uint64_t StampDuration(uint64_t &a, uint64_t &b)
    {
        return (a < b ? b - a : a - b);
    }
    inline double nsec2sec(uint64_t nsec)
    {
        return static_cast<double>(nsec) / 1e9;
    }
    inline double nsec2msec(uint64_t nsec)
    {
        return static_cast<double>(nsec) / 1e6;
    }
    inline uint64_t msec2nsec(double msec)
    {
        return static_cast<uint64_t>(msec) * 1e6;
    }
private:
    std::string mstrMajorImgTopic;
    std::string mstrRosbagPathBase;
    std::string mstrOutFilePathBase;
    std::string mstrTeam;
};