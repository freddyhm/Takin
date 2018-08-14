#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
//#include <opencv/highgui.h>

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
 
        circle(cameraFeed,Point(xpos,ypos),20,Scalar(0,255,0),2);
        line(cameraFeed,Point(xpos,ypos),Point(xpos,ypos-25),Scalar(0,255,0),2);
        line(cameraFeed,Point(xpos,ypos),Point(xpos,ypos+25),Scalar(0,255,0),2);
        line(cameraFeed,Point(xpos,ypos),Point(xpos-25,ypos),Scalar(0,255,0),2);
        line(cameraFeed,Point(xpos,ypos),Point(xpos+25,ypos),Scalar(0,255,0),2);
        rectangle(cameraFeed,objectBoundingRectangle,Scalar(0,255,0));
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


//some boolean variables for added functionality
    bool objectDetected = false;
    //these two can be toggled by pressing 'd' or 't'
    bool debugMode = false;
    bool trackingEnabled = false;
    //pause and resume code
    bool pause = false;
    //set up the matrices that we will need
    //the two frames we will be comparing
    Mat frame1,frame2;
    //their grayscale images (needed for absdiff() function)
    Mat grayImage1,grayImage2;
    //resulting difference image
    Mat differenceImage;
    //thresholded difference image (for use in findContours() function)
    Mat thresholdImage;
    //video capture object.
    VideoCapture capture;

    capture.open(0);
 
    if(!capture.isOpened()){
        cout<<"ERROR ACQUIRING VIDEO FEED\n";
        getchar();
        return -1;
    }
    while(1){
 
        //we can loop the video by re-opening the capture every time the video reaches its last frame
 
        
 
        //check if the video has reach its last frame.
        //we add '-1' because we are reading two frames from the video at a time.
        //if this is not included, we get a memory error!
 
        //read first frame
        capture.read(frame1);
        //convert frame1 to gray scale for frame differencing
        cv::cvtColor(frame1,grayImage1,COLOR_BGR2GRAY);
        //copy second frame
        capture.read(frame2);
        //convert frame2 to gray scale for frame differencing
        cv::cvtColor(frame2,grayImage2,COLOR_BGR2GRAY);
        //perform frame differencing with the sequential images. This will output an "intensity image"
        //do not confuse this with a threshold image, we will need to perform thresholding afterwards.
        cv::absdiff(grayImage1,grayImage2,differenceImage);
        //threshold intensity image at a given sensitivity value
        cv::threshold(differenceImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
        if(debugMode==true){
            //show the difference image and threshold image
            cv::imshow("Difference Image",differenceImage);
            cv::imshow("Threshold Image", thresholdImage);
        }else{
            //if not in debug mode, destroy the windows so we don't see them anymore
            //cv::destroyWindow("Difference Image");
            //cv::destroyWindow("Threshold Image");
        }
        //blur the image to get rid of the noise. This will output an intensity image
        cv::blur(thresholdImage,thresholdImage,cv::Size(BLUR_SIZE,BLUR_SIZE));
        //threshold again to obtain binary image from blur output
        cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
        if(debugMode==true){
            //show the threshold image after it's been "blurred"

            imshow("Final Threshold Image",thresholdImage);

        }
        else {
            //if not in debug mode, destroy the windows so we don't see them anymore
            //cv::destroyWindow("Final Threshold Image");
        }

        //if tracking enabled, search for contours in our thresholded image
        if(trackingEnabled){

            searchForMovement(thresholdImage,frame1);
        }

        //show our captured frame
        imshow("Frame1",frame1);
        //check to see if a button has been pressed.
        //this 10ms delay is necessary for proper operation of this program
        //if removed, frames will not have enough time to referesh and a blank 
        //image will appear.
        switch(waitKey(10)){

        case 27: //'esc' key has been pressed, exit program.
            return 0;
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
        }
    }

/*
// declares all required variables
  Rect2d roi;
  Mat frame;
  // create a tracker object
  Ptr<Tracker> tracker = TrackerKCF::create();
  // set input video
  std::string video = argv[1];
  VideoCapture cap(0);
  // get bounding box
  cap >> frame;
  roi=selectROI("tracker",frame);
  //quit if ROI was not selected
  if(roi.width==0 || roi.height==0)
    return 0;
  // initialize the tracker
  tracker->init(frame,roi);
  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for ( ;; ){
    // get frame from the video
    cap >> frame;
    // stop the program if no more images
    if(frame.rows==0 || frame.cols==0)
      break;
    // update the tracking result
    tracker->update(frame,roi);
    // draw the tracked object
    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
    // show image with the tracked object
    imshow("tracker",frame);
    //quit on ESC button
    if(waitKey(1)==27)break;
  }
*/
    /* // List of tracker types in OpenCV 3.4.1
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));
 
    // Create a tracker
    string trackerType = trackerTypes[2];
 
    Ptr<Tracker> tracker;

    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "MOSSE")
            tracker = TrackerMOSSE::create();
        if (trackerType == "CSRT")
            tracker = TrackerCSRT::create();
    }
    #endif
    // Read video
    VideoCapture video(0);
    //VideoCapture video("videos/chaplin.mp4");
     
    // Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl; 
        return 1; 
    } 
 
    // Read first frame 
    Mat frame; 
    bool ok = video.read(frame); 
 
    // Define initial bounding box 
    Rect2d bbox(287, 23, 86, 320); 
 
    // Uncomment the line below to select a different bounding box 
    bbox = selectROI(frame, false); 
    // Display bounding box. 
    //rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
 
    imshow("Tracking", frame); 
    tracker->init(frame, bbox);
     
    while(video.read(frame))
    {     
        // Start timer
        double timer = (double)getTickCount();
         
        // Update the tracking result
        bool ok = tracker->update(frame, bbox);
         
        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);
         
        if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
         
        // Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
 
        // Display frame.
        imshow("Tracking", frame);
         
        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27)
        {
            break;
        }
 
    }*/
   /* Mat frame, gray, frameDelta, thresh, firstFrame;
    vector<vector<Point> > cnts;
    VideoCapture camera(0); //open camera
    
    //set the video size to 512x288 to process faster
    camera.set(3, 512);
    camera.set(4, 288);

    sleep(3);
    camera.read(frame);

    //convert to grayscale and set the first frame
    cvtColor(frame, firstFrame, COLOR_BGR2GRAY);
    GaussianBlur(firstFrame, firstFrame, Size(21, 21), 0);

    while(camera.read(frame)) {

        //convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(21, 21), 0);

        //compute difference between first frame and current frame
        absdiff(firstFrame, gray, frameDelta);
        threshold(frameDelta, thresh, 25, 255, THRESH_BINARY);
        
        dilate(thresh, thresh, Mat(), Point(-1,-1), 2);
        findContours(thresh, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for(int i = 0; i< cnts.size(); i++) {
            if(contourArea(cnts[i]) < 500) {
                continue;
            }

            putText(frame, "Motion Detected", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }
    
        imshow("Camera", frame);
        firstFrame = frame;
        if(waitKey(1) == 27){
            //exit if ESC is pressed
            break;
        }
    
    }*/
    return 0;
}

