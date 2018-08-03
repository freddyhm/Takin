#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>

#include "PolygonDetector.h"

using namespace cv;

using namespace std;

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    PolygonDetector  detector = PolygonDetector(argv[1]);
    
    detector.detect();
    
   // Mat image;
   // Mat hsv;
   // Mat threshold;
   // image = imread( argv[1], 1 );

   //  cvtColor( image,  hsv, CV_BGR2HSV );
  // inRange(hsv,Scalar(0,0,230),Scalar(255,230,255),threshold);

   // namedWindow("Display Image", WINDOW_AUTOSIZE );
   // imshow("Display Image",threshold);
   // waitKey(0);
    return 0;
}