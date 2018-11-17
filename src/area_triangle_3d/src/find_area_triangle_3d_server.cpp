#include "ros/ros.h"
#include "area_triangle_3d/AreaTriangle3D.h"
#include <numeric>
#include <vector>

using namespace std;

// service for calculating
bool area(area_triangle_3d::AreaTriangle3D::Request &req, area_triangle_3d::AreaTriangle3D::Response &res){
    
    // get vector p1p2 and p1p3
    float p1p2[] = {req.point2.x - req.point1.x, req.point2.y - req.point1.y, req.point2.z - req.point1.z};
    float p1p3[] = {req.point3.x - req.point1.x, req.point3.y - req.point1.y, req.point3.z - req.point1.z};

    // find dot product
    float dot_prod = inner_product(p1p2, p1p2 + 3, p1p3, 0);
    
    // get magnitude of vectors
    float p1p2_magn = sqrt(pow(p1p2[0], 2) + pow(p1p2[1], 2) + pow(p1p2[2], 2));
    float p1p3_magn = sqrt(pow(p1p3[0], 2) + pow(p1p3[1], 2) + pow(p1p3[2], 2));

    // find theta
    float cos_theta = dot_prod / (p1p2_magn * p1p3_magn);
    float theta = acos(cos_theta); 

    // find area
    float answer =  (p1p2_magn * p1p3_magn * sin(theta))/2; 

    ROS_INFO("request: Point 1 x=%f, y=%f, z=%f", req.point1.x, req.point1.y, req.point1.z);
    ROS_INFO("request: Point 2 x=%f, y=%f, z=%f", req.point2.x, req.point2.y, req.point2.z);
    ROS_INFO("request: Point 3 x=%f, y=%f, z=%f", req.point3.x, req.point3.y, req.point3.z);
    ROS_INFO("sending back response: %f", answer);

    // find area
    res.triangleArea = answer;
   
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_area_triangle_3d_server");
    ros::NodeHandle n;

    // service is created and advertised over ROS
    ros::ServiceServer service = n.advertiseService("find_area_triangle", area);
    ROS_INFO("Ready to find area of 3d triangle");
    ros::spin();

    return 0;
}
