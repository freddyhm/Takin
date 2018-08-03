


#include "PolygonDetector.h"


PolygonDetector::PolygonDetector(string file)
{
 thresh = 100;
 max_thresh = 255;
 srcImage = imread(file,1);
}

int PolygonDetector::detect()
{

 RNG rng(12345); 
 convertToGrayScale();
 Mat canny_output;
 vector<vector<Point> > contours;
 vector<Vec4i> hierarchy;

 Canny(grayScaleImage, canny_output, thresh, thresh*2, 3 );

 findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE , Point(0, 0) );
 
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
      
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image",drawing);
    waitKey(0);

 return 0;
}
void  PolygonDetector::drawContour(Mat output)
{
  Mat drawing = Mat::zeros(output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
      
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }
}
void  PolygonDetector::drawResult()
{
   // namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image",threshold);
   // waitKey(0);

}
void  PolygonDetector::convertToGrayScale()
{
  cvtColor( srcImage, grayScaleImage, CV_BGR2GRAY );
}

void  PolygonDetector::applyTreshold()
{

}


