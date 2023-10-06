#include <ros/ros.h>
#include "Judgement.h"
int main(int argc, char **argv)
{
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
    ros::init(argc, argv, "stage4_judgement");
    ros::NodeHandle nh("stage4_judgement");
    Judgement judgement(nh);
    judgement.Loop();
    ros::shutdown();
    return 0;
}