#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 25;
//we'll have just one object to search for
//and keep track of its position.
int theObject[2] = {0,0};
//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0,0,0,0);
 
 
//int to string helper function
string intToString(int number){
 
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}
 
void searchForMovement(Mat thresholdImage, Mat &cameraFeed){
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.
    bool objectDetected = false;
    Mat temp;
    thresholdImage.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    //findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
 
    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)objectDetected=true;
    else objectDetected = false;
 
    if(objectDetected){
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));
        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        for(auto& contour:contours){
            objectBoundingRectangle = boundingRect(contour);
            int xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
            int ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
    
            /*circle(cameraFeed,Point(xpos,ypos),20,Scalar(0,255,0),2);
            line(cameraFeed,Point(xpos,ypos),Point(xpos,ypos-25),Scalar(0,255,0),2);
            line(cameraFeed,Point(xpos,ypos),Point(xpos,ypos+25),Scalar(0,255,0),2);
            line(cameraFeed,Point(xpos,ypos),Point(xpos-25,ypos),Scalar(0,255,0),2);
            line(cameraFeed,Point(xpos,ypos),Point(xpos+25,ypos),Scalar(0,255,0),2);*/
            //rectangle(cameraFeed,objectBoundingRectangle,Scalar(0,255,0),2);

            // get the min area rect
            cv::RotatedRect rrect = minAreaRect(contour);


            cv::Point2f vertices[4];
            rrect.points(vertices);
            for (int i = 0; i < 4; ++i)
            {
                    cv::line(cameraFeed, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2, CV_AA);
            }
            //cv::imshow("box", cameraFeed);


            /*Mat box;
            //vector<Mat> boxs(1);
            boxPoints(rect, box);
            for (size_t idx = 0; idx < contours.size(); idx++) 
            {
                cv::drawContours(cameraFeed, box, idx, cv::Scalar(255, 0, 0));
            }*/ 
            // draw a red 'nghien' rectangle
            //drawContours(cameraFeed,boxs[0],0,Scalar(0,0,255));
            // convert all coordinates floating point values to int

        } 
        
        //update the objects positions by changing the 'theObject' array values
        //theObject[0] = xpos , theObject[1] = ypos;
    }
    //make some temp x and y variables so we dont have to type out so much
    /*int x = theObject[0];
    int y = theObject[1];
     
    //draw some crosshairs around the object
    circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
    line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
 
    //write the position of the object to the screen
    putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
 */
     
 
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"capra_motion_detection_static_publisher");

    ros::NodeHandle nh;
    //cv::namedWindow("view");
    //cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_motion = it.advertise( "capra/motion_detection/motion",1);
    image_transport::Publisher pub_difference = it.advertise( "capra/motion_detection/difference",1);
    image_transport::Publisher pub_threshold = it.advertise( "capra/motion_detection/threshold",1);
    image_transport::Publisher pub_blurred_threshold = it.advertise( "capra/motion_detection/blurred_threshold",1);

    image_transport::Subscriber sub = it.subscribe("/capra/camera_3d/rgb/image_raw", 1, [&](const sensor_msgs::ImageConstPtr& msg){
        static Mat last_frame;
        Mat current_frame;
        try
        {
            // Set first image as last_frame
            if(last_frame.empty()){
                last_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
                return;
            }
            current_frame=cv_bridge::toCvShare(msg, "bgr8")->image;
             
        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(1);
        //this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        
        //some boolean variables for added functionality
        //bool objectDetected = false;
        //these two can be toggled by pressing 'd' or 't'
        bool debugMode = false;
        //bool trackingEnabled = false;
        //pause and resume code
        //bool pause = false;
        //set up the matrices that we will need
        //the two frames we will be comparing
        //their grayscale images (needed for absdiff() function)
        Mat grayImage1,grayImage2;
        //resulting difference image
        Mat differenceImage;
        //thresholded difference image (for use in findContours() function)
        Mat thresholdImage;
        //video capture object.
        //VideoCapture capture;

        //capture.open(0);
    
        /*if(!capture.isOpened()){
            cout<<"ERROR ACQUIRING VIDEO FEED\n";
            getchar();
            return -1;
        }
        while(1){*/
    
            //we can loop the video by re-opening the capture every time the video reaches its last frame
    
            
    
            //check if the video has reach its last frame.
            //we add '-1' because we are reading two frames from the video at a time.
            //if this is not included, we get a memory error!
    
            //read first frame
            //capture.read(last_frame);
            //convert last_frame to gray scale for frame differencing
            cv::cvtColor(last_frame,grayImage1,COLOR_BGR2GRAY);
            //copy second frame
            //capture.read(current_frame);
            //convert current_frame to gray scale for frame differencing
            cv::cvtColor(current_frame,grayImage2,COLOR_BGR2GRAY);
            //perform frame differencing with the sequential images. This will output an "intensity image"
            //do not confuse this with a threshold image, we will need to perform thresholding afterwards.
            cv::absdiff(grayImage1,grayImage2,differenceImage);
            //threshold intensity image at a given sensitivity value
            cv::threshold(differenceImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);

            pub_difference.publish(cv_bridge::CvImage(
                std_msgs::Header() /* empty header */,
                sensor_msgs::image_encodings::MONO8 /* image format */,
                differenceImage /* the opencv image object */
            ).toImageMsg());
            pub_threshold.publish(cv_bridge::CvImage(
                std_msgs::Header() /* empty header */,
                sensor_msgs::image_encodings::MONO8 /* image format */,
                thresholdImage /* the opencv image object */
            ).toImageMsg());
            if(debugMode==true){
                //show the difference image and threshold image
                //cv::imshow("Difference Image",differenceImage);
                //cv::imshow("Threshold Image", thresholdImage);
            }else{
                //if not in debug mode, destroy the windows so we don't see them anymore
                //cv::destroyWindow("Difference Image");
                //cv::destroyWindow("Threshold Image");
            }
            //blur the image to get rid of the noise. This will output an intensity image
            cv::blur(thresholdImage,thresholdImage,cv::Size(BLUR_SIZE,BLUR_SIZE));
            //threshold again to obtain binary image from blur output
            cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);

            pub_blurred_threshold.publish(cv_bridge::CvImage(
                std_msgs::Header() /* empty header */,
                sensor_msgs::image_encodings::MONO8 /* image format */,
                thresholdImage /* the opencv image object */
            ).toImageMsg());
            if(debugMode==true){
                //show the threshold image after it's been "blurred"

                //imshow("Final Threshold Image",thresholdImage);

            }
            else {
                //if not in debug mode, destroy the windows so we don't see them anymore
                //cv::destroyWindow("Final Threshold Image");
            }

            //if tracking enabled, search for contours in our thresholded image
            //if(trackingEnabled){

            searchForMovement(thresholdImage,last_frame);
            //}

            //show our captured frame
            //imshow("Frame1",last_frame);
            //check to see if a button has been pressed.
            //this 10ms delay is necessary for proper operation of this program
            //if removed, frames will not have enough time to referesh and a blank 
            //image will appear.
            /*switch(waitKey(10)){

            case 27: //'esc' key has been pressed, exit program.
                return ;
            case 116: //'t' has been pressed. this will toggle tracking
                trackingEnabled = !trackingEnabled;
                if(trackingEnabled == false) cout<<"Tracking disabled."<<endl;
                else cout<<"Tracking enabled."<<endl;
                break;
            case 100: //'d' has been pressed. this will debug mode
                debugMode = !debugMode;
                if(debugMode == false){
                    cout<<"Debug mode disabled."<<endl;
                    cv::destroyWindow("Difference Image");
                    cv::destroyWindow("Threshold Image");
                    cv::destroyWindow("Final Threshold Image");
                } 
                else cout<<"Debug mode enabled."<<endl;
                break;
            case 112: //'p' has been pressed. this will pause/resume the code.
                pause = !pause;
                if(pause == true){ cout<<"Code paused, press 'p' again to resume"<<endl;
                while (pause == true){
                    //stay in this loop until 
                    switch (waitKey()){
                        //a switch statement inside a switch statement? Mind blown.
                    case 112: 
                        //change pause back to false
                        pause = false;
                        cout<<"Code Resumed"<<endl;
                        break;
                    }
                }
                }
            }*/
        //}
        // Convert opencv image to ROS sensor_msgs image
        // Publish the ROS sensor_msgs image);
        pub_motion.publish(cv_bridge::CvImage(
            std_msgs::Header() /* empty header */,
            sensor_msgs::image_encodings::BGR8 /* image format */,
            last_frame /* the opencv image object */
        ).toImageMsg());
        
        //set current_frame to last_frame for next time
        last_frame = current_frame;
    
    });
    ros::spin();
    //cv::destroyWindow("view");


    return 0;
}

