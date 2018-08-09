


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


using namespace std;
using namespace cv;

class PolygonDetector
{

  public:
    PolygonDetector(string file);
    void drawResult();

    int detect();
    //void drawContour();
    void convertToGrayScale();
    void applyTreshold();

  private:
    Mat srcImage;
    Mat grayScaleImage;
    Mat thresholdImage;
    int thresh;
    int max_thresh;
};
