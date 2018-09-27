#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <thread>
#include <string>
#include <ros/console.h>

using namespace cv;
using namespace std;

//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 30;
//we'll have just one object to search for
//and keep track of its position.
int theObject[2] = {0,0};
//bounding rectangle of the object, we will use the center of this as its position.

 
//int to string helper function
string intToString(int number){
 
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void focusSearchForMovement(cv::Rect roi, cv::Mat differenceImage, cv::Mat &last_frame){


	cv::Mat diff_img_roi = differenceImage(roi);
	cv::Mat thresh_img, blur_img;

	cv::threshold(diff_img_roi, thresh_img,SENSITIVITY_VALUE,255,THRESH_BINARY);

	int erosion_size =2;
	Mat element = getStructuringElement( MORPH_ELLIPSE,
                                   Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                   Point( erosion_size, erosion_size ) );

	    /// Apply the erosion operation
	erode( thresh_img, blur_img, element );

	int dilatation_size =3;
	element = getStructuringElement( MORPH_ELLIPSE,
                                   Size( 2*dilatation_size + 1, 2*dilatation_size+1 ),
                                   Point( dilatation_size, dilatation_size ) );
	dilate( blur_img, thresh_img, element );


    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(thresh_img,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
	

    if(contours.size()>0){
        for(auto& contour:contours){
	    Rect objectBoundingRectangle = Rect(0,0,0,0);
            objectBoundingRectangle = boundingRect(contour);
            int xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
            int ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
    

            // get the min area rect
            cv::RotatedRect rrect = minAreaRect(contour);


            cv::Point2f vertices[4];
            rrect.points(vertices);

            for (int i = 0; i < 4; ++i) {
		cv::Point2f start_point(vertices[i].x+roi.x,vertices[i].y+roi.y);
		cv::Point2f end_point(vertices[(i+1)%4].x+roi.x,vertices[(i+1)%4].y+roi.y);
                    cv::line(last_frame, start_point, end_point, cv::Scalar(0, 255, 0), 2, CV_AA);
            }
	}
    }

	
} 


cv::Rect searchForMovement(Mat thresholdImage, Mat &cameraFeed){
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
    objectDetected=contours.size()>0;

    if(objectDetected){
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));
        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        for(auto& contour:largestContourVec){
	    Rect objectBoundingRectangle = Rect(0,0,0,0);
            objectBoundingRectangle = boundingRect(contour);
            int xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
            int ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;
    

            // get the min area rect
            cv::RotatedRect rrect = minAreaRect(contour);


            cv::Point2f vertices[4];
            rrect.points(vertices);
            for (int i = 0; i < 4; ++i)
            {
                    cv::line(cameraFeed, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2, CV_AA);
            }

	    return objectBoundingRectangle;

        } 
        
    }
     
 
}

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"capra_motion_detection_static_publisher");

    ros::NodeHandle nh("~");

    Mat last_frame;
    Mat current_frame;
    
    string source;
    nh.param<string>("source",source,"");
    string output = source.substr(0,source.find_first_of("/",1))+"/motion_detection"+source.substr(source.find_first_of("/",1));
    //cv::namedWindow("view");
    //cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_motion = it.advertise( output+"/motion",1);
    image_transport::Publisher pub_motion_focus = it.advertise( output+"/motion_focus",1);
    image_transport::Publisher pub_difference = it.advertise( output+"/difference",1);
    image_transport::Publisher pub_threshold = it.advertise( output+"/threshold",1);
    image_transport::Publisher pub_blurred_threshold = it.advertise( output+"/blurred_threshold",1);

    string source_encoding;
    
    nh.param<string>("source_encoding",source_encoding,"");
    //ROS_DEBUG("Source: %s",source);
    image_transport::Subscriber sub = it.subscribe(source, 1, [&](const sensor_msgs::ImageConstPtr& msg)
    {

        try
        {
            // Set first image as last_frame
            if(last_frame.empty()){
                //ROS_DEBUG("Encoding: %s trying %s",msg->encoding.c_str(),source_encoding);
                last_frame = cv_bridge::toCvShare(msg, source_encoding)->image;
                return;
            }
            current_frame=cv_bridge::toCvShare(msg, source_encoding)->image;

        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(1);
        //this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        catch (cv_bridge::Exception& e)
        {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        
        Mat grayImage1,grayImage2;
        //resulting difference image
        Mat differenceImage;
        //thresholded difference image (for use in findContours() function)
        Mat thresholdImage;

    
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

	//blur the image to get rid of the noise. This will output an intensity image
	cv::blur(thresholdImage,thresholdImage,cv::Size(BLUR_SIZE,BLUR_SIZE));

	int erosion_size = 10;
	Mat element = getStructuringElement( MORPH_ELLIPSE,
                                   Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                   Point( erosion_size, erosion_size ) );

	    /// Apply the erosion operation
	erode( thresholdImage, thresholdImage, element );
	dilate( thresholdImage, thresholdImage, element );



        //threshold again to obtain binary image from blur output
        cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);

        pub_blurred_threshold.publish(cv_bridge::CvImage(
            std_msgs::Header() /* empty header */,
            sensor_msgs::image_encodings::MONO8 /* image format */,
            thresholdImage /* the opencv image object */
        ).toImageMsg());

        //if tracking enabled, search for contours in our thresholded image
        //if(trackingEnabled){

        cv::Rect new_roi = searchForMovement(thresholdImage,last_frame);
        // Convert opencv image to ROS sensor_msgs image
        // Publish the ROS sensor_msgs image);

	if(new_roi.y > 0 && new_roi.y < thresholdImage.rows && new_roi.x >0 && new_roi.x < thresholdImage.cols ){
	    focusSearchForMovement(new_roi, differenceImage, last_frame);
	}

	
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

