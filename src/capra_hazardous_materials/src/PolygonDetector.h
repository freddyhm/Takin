#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string.h>

using namespace std;
using namespace cv;

class PolygonDetector {

public:
PolygonDetector(string file);
void drawResult();


int detect();
void drawContour();
void convertToGrayScale();
void applyTreshold();

private:
Mat srcImage;
Mat grayScaleImage;
Mat thresholdImage;
int thresh;
int max_thresh;


};
