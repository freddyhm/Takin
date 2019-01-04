#include "ros/ros.h"
#include "std_msgs/String.h"

void areaCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "area_triangle_3d_sub");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("area_triangle_3d", 1000, areaCallback);

    ros::spin();

    return 0;
}