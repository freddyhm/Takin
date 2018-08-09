#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

#include "PolygonDetector.h"

using namespace cv;

using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "capra_hazardous_materials");

    ros::NodeHandle n;

   

    ros::spin();
    
    return 0;

   // if (argc != 2)
 //   {
  //      printf("usage: DisplayImage.out <Image_Path>\n");
   //     return -1;
  //  }
   // PolygonDetector detec
   //     PolygonDetector detector = PolygonDetector(argv[1]);

  //  detector.detect();

    // Mat image;
    // Mat hsv;
    // Mat threshold;
    // image = imread( argv[1], 1 );

    //  cvtColor( image,  hsv, CV_BGR2HSV );
    // inRange(hsv,Scalar(0,0,230),Scalar(255,230,255),threshold);

    // namedWindow("Display Image", WINDOW_AUTOSIZE );
    // imshow("Display Image",threshold);
    // waitKey(0);
}