
/* 
 * 
 * AUTHOR:MATTEO MASCIOTTA 
 * UNIVERSITA' DI ROMA TRE, DIPARTIMENTO DI INGEGNERIA DELL'AUTOMAZIONE, ITALY
 *  
 */


/*This code is write to have an executable ros node that 
 * can publish over the network an elaborated video (with OpenCV) of a webcam. 
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
#include "map_1280_1024.h"
#include <string>
#include <cv.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <vector>
#include <highgui.h>
// Main blob library include
#include "BlobResult.h"
#define MAX_LOST  100000000
#include "std_msgs/Header.h"
#include <rosgraph_msgs/Clock.h>
#include <tutorialROSOpenCV/Stringts.h>


const int robMax = 6;
using namespace std;
using namespace boost;
namespace enc = sensor_msgs::image_encodings;


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

typedef struct {
    // Current List of Robots
    Robot robList[robMax];
    int robNum;
    int init;
} RobotList;

// Currently Available Robots
RobotList avRobList;
RobotList potRobList;
int areaBlob = 50;
int epsilon = 50;


float distMatrix[robMax][robMax];
float minDistMatrix[robMax][robMax];

rosgraph_msgs::Clock framets;
tutorialROSOpenCV::Stringts msg;
string ss, ss1, ss2, ss3, ss4, ss5, ss6;




void makelistRobot() {

    ss1 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f,%f,") % potRobList.robList[0].id % potRobList.robList[0].coord.x % potRobList.robList[0].coord.y % potRobList.robList[0].lost % potRobList.robList[0].active % potRobList.robList[0].center.x % potRobList.robList[0].center.y % potRobList.robList[0].orientation);
    ss2 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f,%f,") % potRobList.robList[1].id % potRobList.robList[1].coord.x % potRobList.robList[1].coord.y % potRobList.robList[1].lost % potRobList.robList[1].active % potRobList.robList[1].center.x % potRobList.robList[1].center.y % potRobList.robList[1].orientation);
    ss3 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f,%f,") % potRobList.robList[2].id % potRobList.robList[2].coord.x % potRobList.robList[2].coord.y % potRobList.robList[2].lost % potRobList.robList[2].active % potRobList.robList[2].center.x % potRobList.robList[2].center.y % potRobList.robList[2].orientation);
    ss4 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f,%f,") % potRobList.robList[3].id % potRobList.robList[3].coord.x % potRobList.robList[3].coord.y % potRobList.robList[3].lost % potRobList.robList[3].active % potRobList.robList[3].center.x % potRobList.robList[3].center.y % potRobList.robList[3].orientation);
    ss5 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f,%f,") % potRobList.robList[4].id % potRobList.robList[4].coord.x % potRobList.robList[4].coord.y % potRobList.robList[4].lost % potRobList.robList[4].active % potRobList.robList[4].center.x % potRobList.robList[4].center.y % potRobList.robList[4].orientation);
    ss6 = boost::str(boost::format("%f,%f,%f,%f,%f,%f,%f%f") % potRobList.robList[5].id % potRobList.robList[5].coord.x % potRobList.robList[5].coord.y % potRobList.robList[5].lost % potRobList.robList[5].active % potRobList.robList[5].center.x % potRobList.robList[5].center.y % potRobList.robList[5].orientation);

    ss = ss1 + ss2 + ss3 + ss4 + ss5 + ss6; //dati dei robot 1 e 2 nella stessa stringa per poterli trasmettere
    msg.data = ss;

    msg.stamp = framets.clock; //associo alla lista robot il timestamp del frame relativo

    // %Tag(ROSCONSOLE)%
   // ROS_INFO("%s", msg.data.c_str());
    // %EndTag(ROSCONSOLE)%
printf("---------------------------------------------------\n"); 
   for (int i = 0; i < robMax; i++){
        printf("Robot %d : (x:%2.2f, y:%2.2f,a:%d)\n", i, potRobList.robList[i].coord.x, potRobList.robList[i].coord.y, potRobList.robList[i].active);

    }
    
}

// %Tag(CALLBACK)%

//void roblistCallback(const std_msgs::String::ConstPtr& msg) {
//
//
//    vector <string> fields;
//
//    split_regex(fields, msg->data.c_str(), regex(","));
//
//    avRobList.robList[0].id = atoi(fields[0].c_str());
//    avRobList.robList[0].coord.x = atof(fields[1].c_str());
//    avRobList.robList[0].coord.y = atof(fields[2].c_str());
//    avRobList.robList[0].lost = atoi(fields[3].c_str());
//    avRobList.robList[0].active = atoi(fields[4].c_str());
//    avRobList.robList[1].id = atoi(fields[5].c_str());
//    avRobList.robList[1].coord.x = atof(fields[6].c_str());
//    avRobList.robList[1].coord.y = atof(fields[7].c_str());
//    avRobList.robList[1].lost = atoi(fields[8].c_str());
//    avRobList.robList[1].active = atoi(fields[9].c_str());
//    avRobList.robList[2].id = atoi(fields[10].c_str());
//    avRobList.robList[2].coord.x = atof(fields[11].c_str());
//    avRobList.robList[2].coord.y = atof(fields[12].c_str());
//    avRobList.robList[2].lost = atoi(fields[13].c_str());
//    avRobList.robList[2].active = atoi(fields[14].c_str());
//    avRobList.robList[3].id = atoi(fields[15].c_str());
//    avRobList.robList[3].coord.x = atof(fields[16].c_str());
//    avRobList.robList[3].coord.y = atof(fields[17].c_str());
//    avRobList.robList[3].lost = atoi(fields[18].c_str());
//    avRobList.robList[3].active = atoi(fields[19].c_str());
//    avRobList.robList[4].id = atoi(fields[20].c_str());
//    avRobList.robList[4].coord.x = atof(fields[21].c_str());
//    avRobList.robList[4].coord.y = atof(fields[22].c_str());
//    avRobList.robList[4].lost = atoi(fields[23].c_str());
//    avRobList.robList[4].active = atoi(fields[24].c_str());
//    avRobList.robList[5].id = atoi(fields[25].c_str());
//    avRobList.robList[5].coord.x = atof(fields[26].c_str());
//    avRobList.robList[5].coord.y = atof(fields[27].c_str());
//    avRobList.robList[5].lost = atoi(fields[28].c_str());
//    avRobList.robList[5].active = atoi(fields[29].c_str());
//
//
//    //      for (int i=0; i<robMax;i++)
//    //    printf("Robot %d : (%f, %f, %f)\n", i, avRobList.robList[i].coord.x, avRobList.robList[i].coord.y, avRobList.robList[i].lost);
//    //   
//}
//// %EndTag(CALLBACK)%

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

//void computeDistMatrix(RobotList *avRobList, RobotList potRobList) {
//    int i, j;
//    for (i = 0; i < robMax; i++)
//        for (j = 0; j < robMax; j++)
//            if (potRobList.robList[j].active == 1)
//                distMatrix[i][j] = computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord);
//            else
//                distMatrix[i][j] = INFINITY;
//}

//int findNearestMeasure(int from) {
//    float min_dist = INFINITY;
//    int minId = -1;
//    int i = 0;
//    for (i = 0; i < robMax; i++)
//        if (distMatrix[from][i] < min_dist) {
//            min_dist = distMatrix[from][i];
//            minId = i;
//        }
//    return minId;
//}

//int findNearestRobot(int from) {
//    float min_dist = INFINITY;
//    int minId = -1;
//    int i = 0;
//    for (i = 0; i < robMax; i++)
//        if (distMatrix[i][from] < min_dist) {
//            min_dist = distMatrix[i][from];
//            minId = i;
//        }
//    return minId;
//}
//
//void updateDistMatrix(int idAv, int idPot) {
//    int i;
//    for (i = 0; i < robMax; i++)
//        distMatrix[idAv][i] = INFINITY;
//
//    if (idPot != -1) //se idPot==-1 sto solo disattivando idAv altrimenti sto anche disattivando la misura 
//        for (i = 0; i < robMax; i++)
//            distMatrix[i][idPot] = INFINITY;
//}

//void associateRob2Measure(int idAv, int idPot, RobotList *avRobList, RobotList *potRobList) {
//
//    avRobList->robList[idAv] = potRobList->robList[idPot]; //aggiorno misura   
//    avRobList->robList[idAv].id=idAv;
//    avRobList->robList[idAv].active = 1; //attivo avRob
//    avRobList->robList[idAv].lost = 0;
//    avRobList->robNum++;
//    potRobList->robList[idPot].active = 0; //disattivo potRob
//    
//    // printf("idAv: %f idPot: %f \n", idAv, idPot);
//    updateDistMatrix(idAv, idPot);
//   
//
//
//}

//void deactivateRob(int id, RobotList *avRobList) {
//    avRobList->robList[id].active = 0;
//    if (avRobList->robList[id].lost < MAX_LOST)
//        avRobList->robList[id].lost++;
//    updateDistMatrix(id, -1);
//      
//}

//void updateRobotList(RobotList *avRobList, RobotList potRobList) {
//    int i, j;
//    // Allowed movement
//    float distTh = 20; //mm
//    
//    // If any potential robot  has been detected... 
//    if (potRobList.robNum > 0) {
//        // If first step let's create the data structure
//        if (avRobList->init == 1) {
//            // Fist let's create the identified robots
//            for (i = 0; i < potRobList.robNum; i++) {
//                //potRobList.robList[i].active = 1;
//                avRobList->robList[i] = potRobList.robList[i];
//                avRobList->robList[i].active = 1;
//                avRobList->robNum++;
//                avRobList->robList[i].id = i;
//                avRobList->init = 0;
//                avRobList->robList[i].lost = 0;
//             
//                //                printf("Robot %d : (%f, %f)\n", i, avRobList->robList[i].center.x, avRobList->robList[i].center.y);
//            }
//            // Then let's fill the array till the maximum number of potential robots
//            for (i = potRobList.robNum; i < robMax; i++) {
//                avRobList->robList[i].id = i;
//                avRobList->robList[i].lost = MAX_LOST;
//                
//            }
//        }// Real step
//        else {
//            // Let's fill in the matrix of distances for this step
//            computeDistMatrix(avRobList, potRobList);
//
//            // Try to associate to each robot id the next position
//            avRobList->robNum = 0;
//            int associated = 0;
//            int deactivated = 0;
//            int assigned[robMax]; // 0 if i-th is not assigned to any new location, 1 if it is assigned
//            for (i = 0; i < robMax; i++)
//                assigned[i] = 0;
//
//
//            while (associated + deactivated < robMax) {
//
//                for (i = 0; i < robMax; i++) {
//                    // If the i-th robot has not been assigned to any new location yet
//                    if (assigned[i] == 0) {
//                        float dist;
//                        int idPot, idAv;
//                        float minDist_av_pot, minDist_pot_av;
//
//                        idPot;
//                        idAv;
//                        //                        dist = INFINITY;
//
//                        // Compute the distance between the i-th robot id and the new locations
//                        idPot = findNearestMeasure(i);
//
//                        // The only case it would get -1 is if all distances are infinity
//                        // When can this happen?
//                        if (idPot>-1)
//                            minDist_av_pot = distMatrix[i][idPot];
//                        else
//                            minDist_av_pot = INFINITY;
//
//                        //printf("ROB %d SCEGLIE POT %d\n",i,idPot);
//                        //se la mindistap è accettabile
//                        if (minDist_av_pot < distTh + (distTh * avRobList->robList[i].lost) / 4) {
//
//                            //Compute the distance between the selected location and the closest robot
//                            idAv = findNearestRobot(idPot);
//
//                            // When does this happen?
//                            if (idAv>-1)
//                                minDist_pot_av = distMatrix[idAv][idPot];
//                            else
//                                minDist_pot_av = INFINITY;
//
//
//                            //printf("POT %d SCEGLIE ROB %d dist %f\n",i,idPot,minDist_pot_av);
//                            // If the match is accomplished then associate the i-th robot to the location
//                            if (idAv == i) { //se corrispondono
//                                associateRob2Measure(idAv, idPot, avRobList, &potRobList);
//
//                                assigned[i] = 1; //aggiorno rob associati
//                                associated++;
//                             //   printf("Robot %d : (%f, %f, %f)\n", i, avRobList->robList[i].coord.x, avRobList->robList[i].coord.y, avRobList->robList[i].orientation);
//
//                            }
//
//                        } else {
//                            // Deactivate the i-th robot
//                            deactivateRob(i, avRobList);
//
//                        //    printf("Robot %d lost da %d step\n", i, avRobList->robList[i].lost);
//
//                            deactivated++;
//                            // This robot is deactivated (then is it marked as assigned)
//                            assigned[i] = 1;
//
//
//                        }
//                    }
//                }
//                //               printf("ASS %d DEACT %d\n", associated, deactivated);
//            }
//
//        }
//    }
//}

int checkDistMarker(float a, float b) {
    // float epsilon = 50;
    return (fabs(a - b) < epsilon);
}






//per immagine HSV
CvScalar lowRed = cvScalar(0, 72, 253);
CvScalar highRed = cvScalar(23, 256, 256);

CvScalar lowGreen = cvScalar(57, 52, 244);
CvScalar highGreen = cvScalar(104, 256, 256);
//****************************************  



//This function threshold the HSV image and create a binary image

IplImage* GetThresholdedImage(IplImage* imgHSV, CvScalar Low, CvScalar High) {
    IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, Low, High, imgThresh);
    return imgThresh;
}

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
    for (i = 0; i < min(whiteRedBlobs.GetNumBlobs(), robMax); i++) {
        currentBlob = whiteRedBlobs.GetBlob(i);
        if (currentBlob->Area() > areaBlob) {
            for (j = 0; j < min(whiteGreenBlobs.GetNumBlobs(), robMax); j++) {
                currentBlob = whiteGreenBlobs.GetBlob(j);
                if (currentBlob->Area() > areaBlob) {
                    distCenter[i][j] = computeDist(redCenter[i], greenCenter[j]);
                    //printf("[%d] - [%d]: %2.2f\n", i, j, distCenter[i][j]);
                    //printf("[%d] - [%d]: %2.2f\n", i, j, distCenter[i][j]);
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
    //  updateRobotList(&avRobList, potRobList);
   
    for(int i=0;i<robMax;i++){
        if(potRobList.robList[i].center.y<320){
            potRobList.robList[i].center.y=0;
            potRobList.robList[i].center.x=0;
            potRobList.robList[i].active=0;
            potRobList.robList[0].id=0;
            potRobList.robList[0].lost=0;
            potRobList.robList[0].orientation=0;
        }
    }
    makelistRobot();

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

int main(int argc, char **argv) {
    //name of the ros node
    ros::init(argc, argv, "webcam_pub_labrob14");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;



    //Use method of ImageTransport to create image publisher

    image_transport::Publisher pub_f;
    image_transport::Publisher pub_or;


    //Create an ImageTransport instance, initializing it with our NodeHandle.

    image_transport::ImageTransport it_f(nh);
    image_transport::ImageTransport it_or(nh);
    //    // %Tag(SUBSCRIBER)%
    //  ros::Subscriber sub = nh.subscribe("robList", 100, roblistCallback);
    //// %EndTag(SUBSCRIBER)%

    ros::Publisher list_pub = nh.advertise<tutorialROSOpenCV::Stringts > ("robList_labrob14_c1", 10);
    // ros::Publisher timestamp_pub = nh.advertise<rosgraph_msgs::Clock > ("timestamp", 10);

    //advertise the video on the path below

    pub_f = it_f.advertise("camera1_labrob14/blobs", 10);
    pub_or = it_or.advertise("camera1_labrob14/RGB", 10);

    avRobList.robNum = 0;
    avRobList.init = 1;


    // %Tag(SUBSCRIBER)%
    // ros::Subscriber sub = nh.subscribe("robList", 100, roblistCallback);
    // %EndTag(SUBSCRIBER)%

    // variable for webcam
    CvCapture* capture = 0;
    IplImage* frame = 0;
    IplImage* frame1 = 0;

    //TODO loading calibration map
    char *name = "Map_c1.txt";
    // Load Map
    InitPixelMap(name);

    //start capturing from webcam, 0 is for the default webcam /dev/video0
    capture = cvCaptureFromCAM(0);
    if (!capture) {
        printf("Capture failure\n");
        return -1;
    }

    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1280);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 1024);



    //creating an output video and setting it to be handly resizable
    cvNamedWindow("Output cam1", CV_WINDOW_NORMAL);
    cvResizeWindow("Output cam1", 320, 280);
  //  cvNamedWindow("input", CV_WINDOW_NORMAL);
 //   cvResizeWindow("input", 320, 280);


    frame = cvQueryFrame(capture);
    frame = cvQueryFrame(capture);
    frame = cvQueryFrame(capture);

    ros::Rate r(10);


    while (1) {

        frame = cvQueryFrame(capture);

        framets.clock = ros::Time::now();
        if (frame == 0) {
            cout << "Error in frame querying" << endl;
            exit(0);
        } //ENDIF

        frame = cvCloneImage(frame);
        frame1 = cvCloneImage(frame);

        cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3); //smooth the original image using Gaussian kernel


        IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);

        cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV



        IplImage* imgThreshRed = GetThresholdedImage(imgHSV, lowRed, highRed);

        IplImage* imgThreshGreen = GetThresholdedImage(imgHSV, lowGreen, highGreen);

        cvSmooth(imgThreshRed, imgThreshRed, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

        cvSmooth(imgThreshGreen, imgThreshGreen, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel


        IplImage* imgFinal = blobDetection2(imgThreshRed, imgThreshGreen);

        //        int px = 253;
        //        int py =757;
        //      CvPoint2D32f cordin= getGlobalCoord(px,py);
        //        
        //       CvPoint point = cvPointFrom32f(cordin);
        //           printf("stampa pallino real(rosso) %f,%f,pixel %d,%d \n", cordin.x,cordin.y,px,py);
        //    cvCircle(frame1,point,10, cvScalar(255, 0, 0), 10, 8, 0);
        //    cvCircle(frame1, cvPoint(px,py),10, cvScalar(0, 0, 255), 10, 8, 0);


        cvSmooth(imgFinal, imgFinal, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

       cvShowImage("Output cam1", imgFinal);
  //      cvShowImage("input", frame1);

        //cv::Mat image(imgHSV); 


        /*publishing the final image with robots*/
        cv::Mat image(imgFinal);

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = framets.clock;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image = image; // Your cv::Mat

        pub_f.publish(out_msg.toImageMsg());

      
        cv::Mat image2(frame);
        // out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image = image2; // Your cv::Mat

        pub_or.publish(out_msg.toImageMsg());



      list_pub.publish(msg);


        //Clean up used images
        cvReleaseImage(&imgHSV);
        cvReleaseImage(&imgThreshRed);
        cvReleaseImage(&imgThreshGreen);
        cvReleaseImage(&imgFinal);
        cvReleaseImage(&frame);
        cvReleaseImage(&frame1);


        //   cvReleaseImage(&imgThresh);
        //	cvReleaseImage(&frame1);

        //    cvReleaseImage(&imgHSV1);


        //Wait 10mS
        int c = cvWaitKey(10);
        //If 'ESC' is pressed, break the loop
        if ((char) c == 27) break;
        if (!(ros::ok())) {
            break; //to exit by control-c 
        }

        r.sleep();
        ros::spinOnce();
        //  loop_rate.sleep();




        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        //   cv::waitKey(3);


        //cvDestroyAllWindows();


    } //end while



}
