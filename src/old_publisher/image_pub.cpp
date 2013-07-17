
/* 
 * 
 * AUTHOR:MATTEO MASCIOTTA 
 * UNIVERSITA' DI ROMA TRE, DIPARTIMENTO DI INGEGNERIA DELL'AUTOMAZIONE, ITALY
 *  
 */


/*This code is write to have an executable ros node that 
 * can pubish over the network an elaborated video (with OpenCV) of a webcam. 
 * 
 */

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <algorithm>    // std::min
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
 #include "std_msgs/String.h"
#include <sstream>
#include "map.h"
#include <cv.h>
#include <highgui.h>

using namespace std;
 namespace enc = sensor_msgs::image_encodings;


//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher text_pub;

int main(int argc, char **argv)
{
    //name of the ros node
     ros::init(argc, argv, "webcam_pub_labrob14");
    
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
        ros::NodeHandle nh;
     
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
       
      
   // variable for webcam
    CvCapture* capture = 0;
    IplImage* frame = 0;
       
   //TODO loading calibration map
    char *name = "Map.txt";
  // Load Map
 //  InitPixelMap(name);
 
 //start capturing from webcam, 0 is for the default webcam /dev/video0
  capture = cvCaptureFromCAM(0);
    if (!capture) {
        printf("Capture failure\n");
        return -1;
    }


  //creating an output video and setting it to be handly resizable
   // cvNamedWindow("Output",CV_WINDOW_NORMAL);
   // cvResizeWindow("Output", 640, 480);    
  
   
  frame = cvQueryFrame(capture);
  frame = cvQueryFrame(capture);
  frame = cvQueryFrame(capture);

  
  //advertise the video on the path below
   pub = it.advertise("camera_labrob14/image_processed", 1);   
   text_pub = nh.advertise<std_msgs::String>("text", 1000);
   

   
    std_msgs::String msg;

    std::stringstream ss;
    ss << "nodo camera su labrob14 attivo";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    
    text_pub.publish(msg);
    
   
   
   
  while (1) {

      frame = cvQueryFrame(capture);
    
        if (frame == 0) {
            cout << "Error in frame querying" << endl;
            exit(0);
        } //ENDIF

 frame = cvCloneImage(frame);
        cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3); //smooth the original image using Gaussian kernel


 IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);	
     
        cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV



     //   IplImage* imgThreshRed = GetThresholdedImage(imgHSV, lowRed, highRed);

     //   IplImage* imgThreshGreen = GetThresholdedImage(imgHSV, lowGreen, highGreen);

     //   cvSmooth(imgThreshRed, imgThreshRed, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

     //   cvSmooth(imgThreshGreen, imgThreshGreen, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel


    //    IplImage* imgFinal = blobDetection2(imgThreshRed, imgThreshGreen);

    //    cvSmooth(imgFinal, imgFinal, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

      // cvShowImage("Output", frame);
       
              //cv::Mat image(imgHSV); 
        
            cv::Mat image(frame);
            
        cv_bridge::CvImage out_msg;
       // out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = image; // Your cv::Mat
	
      
        
        pub.publish(out_msg.toImageMsg());
        // ros::spin();
 
        //Clean up used images
        cvReleaseImage(&imgHSV);
     //   cvReleaseImage(&imgThreshRed);
    //    cvReleaseImage(&imgThreshGreen);
    //    cvReleaseImage(&imgFinal);
        cvReleaseImage(&frame);
        
        
  //   cvReleaseImage(&imgThresh);
//	cvReleaseImage(&frame1);
        
    //    cvReleaseImage(&imgHSV1);
        
        
     //Wait 10mS
        int c = cvWaitKey(10);
        //If 'ESC' is pressed, break the loop
        if ((char) c == 27) break;
        if(!(ros::ok())) {  
            break; //to exit by control-c 
        }
          
 
        
        //Display the image using OpenCV
   // cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
 //   cv::waitKey(3);
       
  
//cvDestroyAllWindows();

       
} //end while

         
   
}
