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
#define MAX_LOST  100000000
#define DEBUG_H  1
#include "map.h"
// Main blob library include
#include "BlobResult.h"
//#include <compressed_image_transport/compressed_publisher.h>


#include <cv.h>
#include <highgui.h>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher

image_transport::Publisher pub;
  
  int areaBlob = 100;
  int epsilon = 50;

int lowerH=0;
int lowerS=0;
int lowerV=0;

int upperH=180;
int upperS=256;
int upperV=256;



int gLowerH=43;
int gLowerS=60;
int gLowerV=247;

int gUpperH=94;
int gUpperS=256;
int gUpperV=256;
            
int rLowerH=0;
int rLowerS=72;
int rLowerV=248;

int rUpperH=28;
int rUpperS=154;
int rUpperV=256; 
 
//int rLowerH=0;
//int rLowerS=0;
//int rLowerV=0;
//
//int rUpperH=180;
//int rUpperS=256;
//int rUpperV=256;
//
//int gLowerH=0;
//int gLowerS=0;
//int gLowerV=0;
//
//int gUpperH=180;
//int gUpperS=256;
//int gUpperV=256;


//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImageRange(IplImage* imgHSV){
	
	IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
	cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);	
	
	return imgThresh;

}


//per immagine HSV
    CvScalar lowRed = cvScalar(rLowerH, rLowerS, rLowerV);
    CvScalar highRed = cvScalar(rUpperH, rUpperS, rUpperV);

    CvScalar lowGreen = cvScalar(gLowerH, gLowerS, gLowerV);
    CvScalar highGreen = cvScalar(gUpperH, gUpperS,gUpperV);
    //****************************************   
    


int color = 2;
int g_switch_value = 2;

// Trackbar/switch callback
void switch_callback( int position ){
	if( position == 0 ){
		 color = 0; //green
                 
 
 gLowerH=lowerH;
 gLowerS=lowerS;
 gLowerV=lowerV;

 gUpperH=upperH;  
 gUpperS=upperS;  
 gUpperV=upperV;
             
	}
        else if(position ==1){
		 color = 1; //red
                 
rLowerH=lowerH;
rLowerS=lowerS;
rLowerV=lowerV;

 rUpperH=upperH;
 rUpperS=upperS;
 rUpperV=upperV;

	}
        else { //default parameter 
            color = 2;
 gLowerH=43;
 gLowerS=37;
 gLowerV=195;

 gUpperH=78;
 gUpperS=175;
 gUpperV=256;
            
 rLowerH=0;
 rLowerS=33;
 rLowerV=233;

 rUpperH=33;
 rUpperS=256;
 rUpperV=256; 
            
        }
}

//This function create two windows and 6 trackbars for the "Ball" window
//void setwindowSettings(){
//	cvNamedWindow("Video");
//	cvNamedWindow("Ball", CV_WINDOW_NORMAL);
//        cvResizeWindow("Ball",300,500);
//	
//	cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
//    cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);
//
//	cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
//    cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);
//
//	cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
//    cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);	
//    
//    
//  cvCreateTrackbar( "0 (green) 1 (red)", "Ball", &g_switch_value, 2, switch_callback );
//    
//}


void setwindowSettings(){
	cvNamedWindow("Video");
	cvNamedWindow("Ball", CV_WINDOW_NORMAL);
        cvResizeWindow("Ball",300,700);
	
	cvCreateTrackbar("REDLowerH", "Ball", &rLowerH, 180, NULL);
       // cout<<"rLowerH: "<< rLowerH << "\n";
    cvCreateTrackbar("REDUpperH", "Ball", &rUpperH, 180, NULL);

	cvCreateTrackbar("REDLowerS", "Ball", &rLowerS, 256, NULL);
    cvCreateTrackbar("REDUpperS", "Ball", &rUpperS, 256, NULL);

	cvCreateTrackbar("REDLowerV", "Ball", &rLowerV, 256, NULL);
    cvCreateTrackbar("REDUpperV", "Ball", &rUpperV, 256, NULL);	
    
    cvCreateTrackbar("GREENLowerH", "Ball", &gLowerH, 180, NULL);
    cvCreateTrackbar("GREENUpperH", "Ball", &gUpperH, 180, NULL);

	cvCreateTrackbar("GREENLowerS", "Ball", &gLowerS, 256, NULL);
    cvCreateTrackbar("GREENUpperS", "Ball", &gUpperS, 256, NULL);

	cvCreateTrackbar("GREENLowerV", "Ball", &gLowerV, 256, NULL);
    cvCreateTrackbar("GREENUpperV", "Ball", &gUpperV, 256, NULL);	
    
      cvCreateTrackbar("epsilon", "Ball", &epsilon, 500, NULL);	
    cvCreateTrackbar("areaBlob", "Ball", &areaBlob, 300, NULL);	
    

    
}
     
      

    // 1*************codice da blob tracking*******************
    
using namespace std;

typedef struct {
    CvPoint2D32f head;
    CvPoint2D32f tail;
    CvPoint2D32f center;
    CvPoint2D32f coord;

    float orientation;
    int active;
    int lost;
    int id;
} Robot;

const int robMax = 2;

typedef struct {
    // Current List of Robots
    Robot robList[robMax];
    int robNum;
    int init;
} RobotList;

// Currently Available Robots
RobotList avRobList;



float distMatrix[robMax][robMax];
float minDistMatrix[robMax][robMax];

Robot createRobot(CvPoint2D32f h, CvPoint2D32f t) {

    Robot rob;
    rob.head = h;
    rob.tail = t;
    rob.center = cvPoint2D32f((h.x + t.x) / 2.0, (h.y + t.y) / 2.0);



    rob.active = 1;
    rob.id = -1;
    
     CvPoint2D32f h_c = getGlobalCoord(h.x, h.y);
    CvPoint2D32f t_c = getGlobalCoord(t.x, t.y);
    
    rob.orientation = cvFastArctan((h_c.y - t_c.y), h_c.x - t_c.x);

    rob.coord = getGlobalCoord(rob.center.x, rob.center.y);


#ifdef DEBUG_H    
  //  printf("Head: %2.2f %2.2f\n", h.x, h.y);
  //  printf("Tail: %2.2f %2.2f\n", t.x, t.y);
  //  printf("Center: %2.2f %2.2f\n", rob.center.x, rob.center.y);
#endif

    return rob;

}

void printRobot(Robot rob) {

    printf("Rob[%d]:%2.2f %2.2f %2.2f; coords:%2.2f %2.2f \n", rob.id, rob.center.x, rob.center.y, rob.orientation, rob.coord.x, rob.coord.y);
}


// This function compute the euclidean distance

float computeDist(CvPoint2D32f p, CvPoint2D32f q) {
    return cv::sqrt((float) cv::pow(p.x - q.x, 2) + cv::pow(p.y - q.y, 2));
}

void computeDistMatrix(RobotList *avRobList, RobotList potRobList) {
    int i, j;
    for (i = 0; i < robMax; i++)
        for (j = 0; j < robMax; j++)
            if (potRobList.robList[j].active == 1)
                distMatrix[i][j] = computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord);
            else
                distMatrix[i][j] = INFINITY;
}

int findNearestMeasure(int from) {
    float min_dist = INFINITY;
    int minId = -1;
    int i = 0;
    for (i = 0; i < robMax; i++)
        if (distMatrix[from][i] < min_dist) {
            min_dist = distMatrix[from][i];
            minId = i;
        }
    return minId;
}

int findNearestRobot(int from) {
    float min_dist = INFINITY;
    int minId = -1;
    int i = 0;
    for (i = 0; i < robMax; i++)
        if (distMatrix[i][from] < min_dist) {
            min_dist = distMatrix[i][from];
            minId = i;
        }
    return minId;
}

void updateDistMatrix(int idAv, int idPot) {
    int i;
    for (i = 0; i < robMax; i++)
        distMatrix[idAv][i] = INFINITY;

    if (idPot != -1) //se idPot==-1 sto solo disattivando idAv altrimenti sto anche disattivando la misura 
        for (i = 0; i < robMax; i++)
            distMatrix[i][idPot] = INFINITY;
}

void associateRob2Measure(int idAv, int idPot, RobotList *avRobList, RobotList *potRobList) {

    avRobList->robList[idAv] = potRobList->robList[idPot]; //aggiorno misura    
    avRobList->robList[idAv].active = 1; //attivo avRob
    avRobList->robList[idAv].lost = 0;
    avRobList->robNum++;
    potRobList->robList[idPot].active = 0; //disattivo potRob
    updateDistMatrix(idAv, idPot);


}

void deactivateRob(int id, RobotList *avRobList) {
    avRobList->robList[id].active = 0;
    if (avRobList->robList[id].lost < MAX_LOST)
        avRobList->robList[id].lost++;
    updateDistMatrix(id, -1);
}

void updateRobotList(RobotList *avRobList, RobotList potRobList) {
    int i, j;
    // Allowed movement
    float distTh = 20; //mm
    
    // If any potential robot  has been detected... 
    if (potRobList.robNum > 0) {
        // If first step let's create the data structure
        if (avRobList->init == 1) {
            // Fist let's create the identified robots
            for (i = 0; i < potRobList.robNum; i++) {
                //potRobList.robList[i].active = 1;
                avRobList->robList[i] = potRobList.robList[i];
                avRobList->robList[i].active = 1;
                avRobList->robNum++;
                avRobList->robList[i].id = i;
                avRobList->init = 0;
                avRobList->robList[i].lost = 0;
                //                printf("Robot %d : (%f, %f)\n", i, avRobList->robList[i].center.x, avRobList->robList[i].center.y);
            }
            // Then let's fill the array till the maximum number of potential robots
            for (i = potRobList.robNum; i < robMax; i++) {
                avRobList->robList[i].lost = MAX_LOST;
            }
        }// Real step
        else {
            // Let's fill in the matrix of distances for this step
            computeDistMatrix(avRobList, potRobList);

            // Try to associate to each robot id the next position
            avRobList->robNum = 0;
            int associated = 0;
            int deactivated = 0;
            int assigned[robMax]; // 0 if i-th is not assigned to any new location, 1 if it is assigned
            for (i = 0; i < robMax; i++)
                assigned[i] = 0;


            while (associated + deactivated < robMax) {

                for (i = 0; i < robMax; i++) {
                    // If the i-th robot has not been assigned to any new location yet
                    if (assigned[i] == 0) {
                        float dist;
                        int idPot, idAv;
                        float minDist_av_pot, minDist_pot_av;

                        idPot;
                        idAv;
                        //                        dist = INFINITY;

                        // Compute the distance between the i-th robot id and the new locations
                        idPot = findNearestMeasure(i);

                        // The only case it would get -1 is if all distances are infinity
                        // When can this happen?
                        if (idPot>-1)
                            minDist_av_pot = distMatrix[i][idPot];
                        else
                            minDist_av_pot = INFINITY;

                        //printf("ROB %d SCEGLIE POT %d\n",i,idPot);
                        //se la mindistap è accettabile
                        if (minDist_av_pot < distTh + (distTh * avRobList->robList[i].lost) / 4) {

                            //Compute the distance between the selected location and the closest robot
                            idAv = findNearestRobot(idPot);
                            // When does this happen?
                            if (idAv>-1)
                                minDist_pot_av = distMatrix[idAv][idPot];
                            else
                                minDist_pot_av = INFINITY;


                            //printf("POT %d SCEGLIE ROB %d dist %f\n",i,idPot,minDist_pot_av);
                            // If the match is accomplished then associate the i-th robot to the location
                            if (idAv == i) { //se corrispondono
                                associateRob2Measure(idAv, idPot, avRobList, &potRobList);

                                assigned[i] = 1; //aggiorno rob associati
                                associated++;
                            
                                printf("Robot %d : (%f, %f, %f)\n", i, avRobList->robList[i].coord.x, avRobList->robList[i].coord.y, avRobList->robList[i].orientation);
                           
                            }

                        } else {
                            // Deactivate the i-th robot
                            deactivateRob(i, avRobList);
                           
                            printf("Robot %d lost da %d step\n", i, avRobList->robList[i].lost);
                            deactivated++;
                            // This robot is deactivated (then is it marked as assigned)
                            assigned[i] = 1;
                        }
                    }
                }
                //               printf("ASS %d DEACT %d\n", associated, deactivated);
            }

        }
    }
}

int checkDistMarker(float a, float b) {
   // float epsilon = 50;
    return (fabs(a - b) < epsilon);
}



IplImage* imgTracking;

/* This function update the robot List */
void updateRobotListAndrea(RobotList *avRobList, RobotList potRobList) {
    int i, j;
    // Allowed movement
    float distTh = 100;

    if (potRobList.robNum > 0) {



        if (avRobList->init == 1) {
            for (i = 0; i < potRobList.robNum; i++) {
                avRobList->robList[i] = potRobList.robList[i];
                avRobList->robList[i].active = 1;
                avRobList->robNum++;
                avRobList->robList[i].id = i;
                avRobList->init = 0;

            }

        } else {

            // Try to associate to each robot id the next position
            avRobList->robNum = 0;

            for (i = 0; i < robMax; i++) {
                float dist;
                int idx;

                idx = -1;
                dist = INFINITY;

                // Compute the distance between the i-th robot id and the new locations
                for (j = 0; j < potRobList.robNum; j++) {
                    if ((computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord) < distTh) && (potRobList.robList[j].active == 1)) {
                        if (computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord) < dist) {
                            if (idx != -1)
                                potRobList.robList[idx].active = 1;

                            idx = j;
                            dist = computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord);
                            printf("Dist: %f\n", dist);
                            // Deactive this robot for further associations
                            potRobList.robList[j].active = 0;
                        }
                    }
                }

                // Assign to the i-th robot the new location
                if (idx > -1) {
                    avRobList->robList[i] = potRobList.robList[idx];
                    avRobList->robList[i].active = 1;
                    avRobList->robList[i].id = i;
                    avRobList->robNum++;
                } else {
                    // Deactivate the i-th robot
                    avRobList->robList[i].active = 0;
                }

            }
        }
    }

    for (i = 0; i < robMax; i++) {
        if (avRobList->robList[i].active == 1)
            printRobot(avRobList->robList[i]);
    }


}



//This function threshold the HSV image and create a binary image

IplImage* GetThresholdedImage(IplImage* imgHSV, CvScalar Low, CvScalar High) {
    IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, Low, High, imgThresh);
    return imgThresh;
}

// This function detects two different types of blobs

IplImage* blobDetection2(IplImage* imgThreshRed, IplImage* imgThreshGreen) {
    // get blobs and filter them using its area
    int i, j;
  //  int areaBlob = 100;
    float distMark = 10;
    CBlobResult blobsRed, blobsGreen, whiteRedBlobs, whiteGreenBlobs;
    CBlob *currentBlob;
    double px, py;

    // Create Image
    IplImage* displayedImage = cvCreateImage(cvGetSize(imgThreshRed), IPL_DEPTH_8U, 3);

    // find all the RED related blobs in the image
    blobsRed = CBlobResult(imgThreshRed, NULL, 0);
    // find all the GREEN related blobs in the image
    blobsGreen = CBlobResult(imgThreshGreen, NULL, 0);

    // select the ones with mean gray-level equal to 255 (white) and put
    // them in the whiteBlobs variable
    blobsRed.Filter(whiteRedBlobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 1.0);
    blobsGreen.Filter(whiteGreenBlobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 1.0);

#ifdef DEBUG_PRINT    
    printf("White Blobs: %d\n", whiteBlobs.GetNumBlobs());
#endif

    // display filtered blobs
    cvMerge(imgThreshRed, imgThreshRed, imgThreshRed, NULL, displayedImage);

    // RED
    CvPoint2D32f redCenter[whiteRedBlobs.GetNumBlobs()];

    for (i = 0; i < whiteRedBlobs.GetNumBlobs(); i++) {
        currentBlob = whiteRedBlobs.GetBlob(i);
        px = (currentBlob->MaxX() + currentBlob->MinX()) / 2.0;
        py = (currentBlob->MaxY() + currentBlob->MinY()) / 2.0;
        redCenter[i] = cvPoint2D32f(px, py);

#ifdef DEBUG_PRINT    
        printf("%2.2f\t%2.2f\n", px, py);
#endif

        if (currentBlob->Area() > areaBlob) {
            // Add Cross to the image
            currentBlob->FillBlob(displayedImage, CV_RGB(255, 0, 0));
            cvCircle(displayedImage, cvPointFrom32f(redCenter[i]), 2, cvScalar(255, 0, 0), 10, 8, 0);
        }
    }

    // GREEN
    CvPoint2D32f greenCenter[whiteGreenBlobs.GetNumBlobs()];

    for (i = 0; i < whiteGreenBlobs.GetNumBlobs(); i++) {
        currentBlob = whiteGreenBlobs.GetBlob(i);
        px = (currentBlob->MaxX() + currentBlob->MinX()) / 2.0;
        py = (currentBlob->MaxY() + currentBlob->MinY()) / 2.0;
        greenCenter[i] = cvPoint2D32f(px, py);

#ifdef DEBUG_PRINT    
        printf("%2.2f\t%2.2f\n", px, py);
#endif

        if (currentBlob->Area() > areaBlob) {
            // Add Cross to the image
            currentBlob->FillBlob(displayedImage, CV_RGB(255, 0, 0));
            cvCircle(displayedImage, cvPointFrom32f(greenCenter[i]), 2, cvScalar(0, 255, 0), 10, 8, 0);
        }
    }

    // Populating the list of potential robots
    RobotList potRobList;
    potRobList.robNum = 0;

    for (i = 0; i < robMax; i++)
        potRobList.robList[i].active = 0;

    int redUsage[whiteRedBlobs.GetNumBlobs()];
    int greenUsage[whiteGreenBlobs.GetNumBlobs()];

    for (i = 0; i < whiteRedBlobs.GetNumBlobs(); i++)
        redUsage[i] = 0;

    for (j = 0; j < whiteGreenBlobs.GetNumBlobs(); j++)
        greenUsage[j] = 0;



    // Detect Robots
    float distCenter[whiteRedBlobs.GetNumBlobs()][whiteGreenBlobs.GetNumBlobs()];
    for (i = 0; i < min(whiteRedBlobs.GetNumBlobs(),robMax); i++) {
        currentBlob = whiteRedBlobs.GetBlob(i);
        if (currentBlob->Area() > areaBlob) {
            for (j = 0; j < min(whiteGreenBlobs.GetNumBlobs(),robMax); j++) {
                currentBlob = whiteGreenBlobs.GetBlob(j);
                if (currentBlob->Area() > areaBlob) {
                    distCenter[i][j] = computeDist(redCenter[i], greenCenter[j]);
                    //printf("[%d] - [%d]: %2.2f\n", i, j, distCenter[i][j]);
                    printf("[%d] - [%d]: %2.2f\n", i, j, distCenter[i][j]);
                    // Print a connection line if this could be a robot
                    if (redUsage[i] == 0 && greenUsage[j] == 0 && checkDistMarker(distCenter[i][j], distMark)) {
                        cvLine(displayedImage, cvPointFrom32f(redCenter[i]), cvPointFrom32f(greenCenter[j]), cvScalar(0, 255, 255), 2, 8, 0);
                        // Check Robot
                        potRobList.robList[potRobList.robNum] = createRobot(redCenter[i], greenCenter[j]);

                        potRobList.robNum++;
                        redUsage[i] = 1;
                        greenUsage[j] = 1;
                        //                        printRobot(potRobList.robList[potRobList.robNum - 1]);


                        CvBox2D tmp;
                        tmp.angle = potRobList.robList[potRobList.robNum - 1].orientation;
                        tmp.center = potRobList.robList[potRobList.robNum - 1].center;
                        tmp.size = cvSize2D32f(30, 50);
                        cvEllipseBox(displayedImage, tmp, cvScalar(255, 255, 0), 4, 3, 0);
                        //			printRobot(potRobList.robList[potRobList.robNum-1]);

                    }

                }
            }
        }
    }


    // Matching The List of Potential Robots with previous List of Robots
    //    updateRobotListAndrea(&avRobList, potRobList);
    updateRobotList(&avRobList, potRobList);
    /*
        // Print robots
        for (i = 0; i < robMax; i++) {
            if (avRobList.robList[i].active == 1) {
                CvBox2D tmp;
                tmp.angle = avRobList.robList[i].orientation;
                tmp.center = avRobList.robList[i].center;
                tmp.size = cvSize2D32f(50, 30);
                cvEllipseBox(displayedImage, tmp, cvScalar(255, 255, 0), 4, 3, 0);
                printRobot(avRobList.robList[i]);
            }
        }
     */



    /* Control Law */

    return displayedImage;

}

/*
 * 
 */



    
// 1 *************************************************
 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    

    //Convert from the ROS image message to a CvImage suitable for
    //working with OpenCV for processing
 
 cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
  
    
    
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    //                                   codice opencv
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    
    
    //cvSetCaptureProperty(cv_ptr->image, CV_CAP_PROP_FRAME_WIDTH, 1280);
    //cvSetCaptureProperty(cv_ptr->image, CV_CAP_PROP_FRAME_HEIGHT, 1024);

    cvNamedWindow("Video",CV_WINDOW_NORMAL);
     cvNamedWindow("Binary Red",CV_WINDOW_NORMAL);
      cvNamedWindow("Binary Green",CV_WINDOW_NORMAL);
#ifdef DEBUG_H   
    cvNamedWindow("Binary Red");
    cvNamedWindow("Binary Green");
#endif    
    cvNamedWindow("Output",CV_WINDOW_NORMAL);

    cvResizeWindow("Video",320, 240);
    cvResizeWindow("Output", 320, 240);
     cvResizeWindow("Binary Red",320, 240);
    cvResizeWindow("Binary Green", 320, 240);

    

   //per immagine HSV
    CvScalar lowRed = cvScalar(rLowerH, rLowerS, rLowerV);
    CvScalar highRed = cvScalar(rUpperH, rUpperS, rUpperV);

    CvScalar lowGreen = cvScalar(gLowerH, gLowerS, gLowerV);
    CvScalar highGreen = cvScalar(gUpperH, gUpperS,gUpperV);
    //****************************************   
    
//    frame = cvQueryFrame(capture);
//    frame = cvQueryFrame(capture);
//    frame = cvQueryFrame(capture);

//    while (1) {


    //    frame = cvQueryFrame(capture);
      
        
     IplImage*  frame = cvCloneImage(&(IplImage)cv_ptr->image);
     
      
        if (frame == 0) {
            cout << "Error in frame querying" << endl;
            exit(0);
        }
     
     
     /* codice ricerca hsv*/
     	setwindowSettings();
         IplImage* frame1=0;
     frame1=cvCloneImage(frame);	
     IplImage* imgHSV1 = cvCreateImage(cvGetSize(frame1), IPL_DEPTH_8U, 3);	
		cvCvtColor(frame1, imgHSV1, CV_BGR2HSV); //Change the color format from BGR to HSV
			
		IplImage* imgThresh = GetThresholdedImageRange(imgHSV1);
		
               
		//cvShowImage("Ball", imgThresh);
      
      //  cvShowImage("Video", frame1);
     
     /***************************************************+*/
        //frame = cvCloneImage(frame);
        cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3); //smooth the original image using Gaussian kernel


  IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);	
        //IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
        cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV



        IplImage* imgThreshRed = GetThresholdedImage(imgHSV, lowRed, highRed);

        IplImage* imgThreshGreen = GetThresholdedImage(imgHSV, lowGreen, highGreen);

        cvSmooth(imgThreshRed, imgThreshRed, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

        cvSmooth(imgThreshGreen, imgThreshGreen, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel


        IplImage* imgFinal = blobDetection2(imgThreshRed, imgThreshGreen);

        cvSmooth(imgFinal, imgFinal, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel


        cvShowImage("Video", frame);
#ifdef DEBUG_H
    //    cvShowImage("Binary Red", imgThreshRed);
       // cvShowImage("Binary Green", imgThreshGreen);
#endif
        cvShowImage("Output", imgFinal);

       
/**
        
        * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
       //
        
        //***************************************
        // per pubblicare l'immagine finale in un nodo ros
        // decommentare anche nel main
        
        cv::Mat image(imgFinal); 
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = image; // Your cv::Mat

        pub.publish(out_msg.toImageMsg());


        
        
        
  // pub.publish(cv_ptr->toImageMsg()); //image processed
     //  pub.publish(cv_ptr->toImageMsg(image, "bgr8"));
        //Clean up used images
        cvReleaseImage(&imgHSV);
        cvReleaseImage(&imgThreshRed);
        cvReleaseImage(&imgThreshGreen);
        cvReleaseImage(&imgFinal);
        cvReleaseImage(&frame);
        
        
     cvReleaseImage(&imgThresh);
	cvReleaseImage(&frame1);
        
        cvReleaseImage(&imgHSV1);
        

//        //Wait 10mS
//        int c = cvWaitKey(10);
//        //If 'ESC' is pressed, break the loop
//        if ((char) c == 27) break;

        
        //Display the image using OpenCV
   // cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
      
   
    

  
       
//    }

 
//cvDestroyAllWindows();
       
    //************************************************************************
    
}                

int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
     ros::init(argc, argv, "image_processor");
    
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
        ros::NodeHandle nh;
     
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
 //   cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    
    //2 codice blob ********************************************
    avRobList.robNum = 0;
   avRobList.init = 1;
    
     char *name = "Map.txt";
  // Load Map
   InitPixelMap(name);


    // 2------------------------------------------------------------
    
       
                      
    
    /**
     * 
     * 
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used. 
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call 
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. 
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
     
    
     //ros::Subscriber sub = nh.subscribe(imagetopic, 1, imageCallback);
     image_transport::Subscriber sub = it.subscribe("/camera_labrob14/image_processed", 1, imageCallback, image_transport::TransportHints::TransportHints("compressed"));
   

        //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    
    //********************************************************
    // DECOMMENTARE LA SEGUENTE RIGA SE SI VUOLE PUBBLICARE L'IMMAGINE ELABORATA
    //IN UN NODO ROS 
     
      pub = it.advertise("usb_cam/image_processed", 1);
        
    //    pub = nh.advertise<sensor_msgs::CompressedImage>("jpeg/image/compressed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call. 
    * ros::spin() will not return until the node has been shutdown, either through a call 
    * to ros::shutdown() or a Ctrl-C.
    */
     
        ros::spinOnce();
    //  ros::spin();
    //ROS_INFO is the replacement for printf/cout.
  //  ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
