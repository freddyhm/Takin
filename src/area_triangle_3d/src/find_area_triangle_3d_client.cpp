#include "ros/ros.h"
#include "std_msgs/String.h"
#include "area_triangle_3d/AreaTriangle3D.h"
#include <geometry_msgs/Point.h>

using namespace std;
using namespace geometry_msgs;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_area_triangle_3d_client");
   
    ros::NodeHandle n;
    // create client for area_triange_3d service
    ros::ServiceClient client = n.serviceClient<area_triangle_3d::AreaTriangle3D>("find_area_triangle");
    area_triangle_3d::AreaTriangle3D srv;

    Point p1, p2, p3;
    p1.x = -5;
    p1.y = 5;
    p1.z = -5;

    p2.x = 1;
    p2.y = -6;
    p2.z = 6;

    p3.x = 2;
    p3.y = -3;
    p3.z = 4;

    // assigning values to request members of the service class
    srv.request.point1 = p1;
    srv.request.point2 = p2;
    srv.request.point3 = p3;

    while(!client.call(srv)){
       ROS_INFO("WAITING FOR SERVER TO RUN!");
       ROS_INFO("Sending to server: Point 1 x=%f, y=%f, z=%f", p1.x, p1.y, p1.z);
       ROS_INFO("Sending to server: Point 2 x=%f, y=%f, z=%f", p2.x, p2.y, p2.z);
       ROS_INFO("Sending to server: Point 3 x=%f, y=%f, z=%f", p3.x, p3.y, p3.z);
    }

     ROS_INFO("Server answered with area: %f", srv.response.triangleArea);

    // call to master node to publish our message on a given topic
    // advertise our message
    ros::Publisher area_triangle_3d_pub = n.advertise<std_msgs::String>("area_triangle_3d", 1000);
    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok()){

        std_msgs::String msg;
        std::stringstream ss;
        ss << "The area of the 3d triange is: " << srv.response.triangleArea;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        area_triangle_3d_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    } 

    return 0;
}
