#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include "JsonInterface.h"
class SimPlayer
{
public:
    SimPlayer(ros::NodeHandle &_nh, double _delaySec, std::string &majImgTopic) : mNodeHandle(_nh), duration(_delaySec)
    {
        odomPub = mNodeHandle.advertise<nav_msgs::Odometry>("/player/odom", 100, false);
        imageSub = mNodeHandle.subscribe(majImgTopic, 100, &SimPlayer::ImageCallback, this, ros::TransportHints().tcpNoDelay());
    }
    void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        odom.header.stamp = msg->header.stamp;
        duration.sleep();
        odomPub.publish(odom);
    }

private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber imageSub;
    ros::Publisher odomPub;
    nav_msgs::Odometry odom;
    ros::Duration duration;
};
int main(int argc, char **argv)
{
    JsonInterface configFile;
    if (argc < 2)
    {
        ROS_ERROR("Usage: rosrun stage4_refree refree_node PATH_TO_CONFIG");
        return -1;
    }
    configFile.OpenJsonfile(argv[1]);
    std::string majorImgTopic;
    configFile.GetParam("major_image_topic", majorImgTopic);

    ros::init(argc, argv, "player");
    ros::NodeHandle nh("player");
    ros::Rate rate(100);
    double delaySec = 0;

    ros::param::get("/player/delaySec", delaySec);
    ROS_INFO("Delay sec is %f.", delaySec);
    SimPlayer player(nh, delaySec, majorImgTopic);
    ros::spin();
}