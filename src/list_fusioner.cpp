
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
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <algorithm>    // std::min
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include "std_msgs/String.h"
#include <sstream>
#include <image_transport/image_transport.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <cv.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <vector>
#include <highgui.h>
#include "opencv2/stitching/stitcher.hpp"

#include <rosgraph_msgs/Clock.h>

#include <tutorialROSOpenCV/Stringts.h>
#include <tf/transform_broadcaster.h>
#define MAX_LOST  100000000
using namespace std;
using namespace boost;
using namespace cv;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
const int robMax = 6;

typedef struct {
    CvPoint2D32f head;
    CvPoint2D32f tail;
    CvPoint2D32f center;
    CvPoint2D32f coord;

    float orientation;
    int active;
    int lost;
    int id;   
    int hasDuplicate;
} Robot;

typedef struct {
    // Current List of Robots
    Robot robList[robMax];
    int robNum;
    int init;

  

} RobotList;



rosgraph_msgs::Clock framets;
tutorialROSOpenCV::Stringts msg;
RobotList avRobList;
RobotList potRobList; //merge

RobotList potRobList1;
RobotList potRobList2;
RobotList potRobList3;
int robot_is_in_cam1[robMax];
int robot_is_in_cam2[robMax];
int robot_is_in_cam3[robMax];

RobotList robList_temp[3]; //3 è il numero di webcam del sistema

//RobotList avRobList3;
ros::Time timestamp[3];


  int areaBlob = 50;
  int epsilon = 50;

float distMatrix[robMax][robMax];
float minDistMatrix[robMax][robMax];

string ss, ss1, ss2, ss3, ss4, ss5, ss6;


IplImage* frame_c[3] ={ 0,0,0};
//IplImage* frame_c2 = 0;
//IplImage* frame_c3 = 0;

int exec_c[3] = {0,0,0};
//int exec_c1 =0 ;
//int exec_c2=0;
//int exec_c3=0;

int node_number =5;  //Numero di telecamere del sistema

// %Tag(CALLBACK)%

void roblistCallback(const tutorialROSOpenCV::Stringts::ConstPtr& msg, int n) {
  vector <string> fields;
    split_regex(fields, msg->data.c_str(), regex(","));
     int i=0;
     for (int j=0;j<node_number;j++){
    robList_temp[n].robList[j].id = atoi(fields[i++].c_str());
    robList_temp[n].robList[j].coord.x = atof(fields[i++].c_str());
    robList_temp[n].robList[j].coord.y = atof(fields[i++].c_str());
    robList_temp[n].robList[j].lost = atoi(fields[i++].c_str());
    robList_temp[n].robList[j].active = atoi(fields[i++].c_str());
    robList_temp[n].robList[j].center.x = atof(fields[i++].c_str());
    robList_temp[n].robList[j].center.y = atof(fields[i++].c_str());
    robList_temp[n].robList[j].orientation = atof(fields[i++].c_str());
    }
    timestamp[n] = msg->stamp;
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
    avRobList->robList[idAv].id=idAv;
    avRobList->robList[idAv].active = 1; //attivo avRob
    avRobList->robList[idAv].lost = 0;
    avRobList->robNum++;
    potRobList->robList[idPot].active = 0; //disattivo potRob
    
    
    //aggiorno anche l'id dei robot potenziali
    //potRobList->robList[idPot].id = idAv;
    
    // printf("antes idAv: %d idPot: %d \n", idAv, idPot);
    updateDistMatrix(idAv, idPot);
   


}

void deactivateRob(int id, RobotList *avRobList) {
    avRobList->robList[id].active = 0;
    if (avRobList->robList[id].lost < MAX_LOST)
        avRobList->robList[id].lost++;
    updateDistMatrix(id, -1);
      
}


int findNearestMeasure(int from) {
    float min_dist = INFINITY;
    int minId = -1;
    int i = 0; 
   // printf("findNearestMeasrure\n", from);
    for (i = 0; i < robMax; i++)
        if (distMatrix[from][i] < min_dist) {
            min_dist = distMatrix[from][i];
            minId = i;
        //     printf("FNMd distMatrix[%d][%d], id:%d \n",from,i,minId);
           
            
            
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

float computeDist(CvPoint2D32f p, CvPoint2D32f q) {
    return cv::sqrt((float) cv::pow(p.x - q.x, 2) + cv::pow(p.y - q.y, 2));
}

void computeDistMatrix(RobotList *avRobList, RobotList potRobList) {
    int i, j;
    for (i = 0; i < robMax; i++)
        for (j = 0; j < robMax; j++)
            if (potRobList.robList[j].active == 1){
               
                distMatrix[i][j] = computeDist(avRobList->robList[i].coord, potRobList.robList[j].coord);
//         printf("distMatrix[%d][%d] = %2.2f\n",i,j,distMatrix[i][j] );
//              printf("avRobList->robList[%d].coord.x=%2.2f  y=%2.2f\n",i,avRobList->robList[i].coord.x,avRobList->robList[i].coord.y);
//             
            }else
                distMatrix[i][j] = INFINITY;
}


void updateRobotList(RobotList *avRobList, RobotList potRobList) {
    int i, j;
    // Allowed movement
    float distTh = 400; //mm
  
    // If any potential robot  has been detected... 
    if (potRobList.robNum > 0) {
      
        // If first step let's create the data structure
        if (avRobList->init == 1) {
           
            // Fist let's create the identified robots
            for (i = 0; i < potRobList.robNum; i++) {
           
             //   potRobList.robList[i].active = 1;
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
                 avRobList->robList[i].id = i;
              
                avRobList->robList[i].lost = MAX_LOST;
                
            }
               
        }// Real step
        else {

            //            for (int i = 0; i < robMax; i++) {
            //                printf("AV_Robot %d : (%f, %f, %d)\n", avRobList->robList[i].id, avRobList->robList[i].coord.x, avRobList->robList[i].coord.y, avRobList->robList[i].lost);
            //            }

            // printf("+++++++++++++++else+++++++++++++\n");
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

                        //                        dist = INFINITY;

                        // Compute the distance between the i-th robot id and the new locations
                        idPot = findNearestMeasure(i);
                        //  printf("+++++++++++++++idPot i: %d +++++++++++++\n",idPot);
                        // The only case it would get -1 is if all distances are infinity
                        // When can this happen?
                        if (idPot>-1)
                            minDist_av_pot = distMatrix[i][idPot];
                        else
                            minDist_av_pot = INFINITY;

                        //    printf("ROB %d SCEGLIE POT %d....", i, idPot);

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

                                //      printf("assRob2mes: (idAv:%d,idPot: %d)\n", idAv, idPot);

                                associateRob2Measure(idAv, idPot, avRobList, &potRobList);

                                assigned[i] = 1; //aggiorno rob associati
                                associated++;
                                //  printf("Robot %d : (%f, %f, %f)\n", i, avRobList->robList[i].coord.x, avRobList->robList[i].coord.y, avRobList->robList[i].orientation);

                            }

                        } else {
                            // Deactivate the i-th robot
                            deactivateRob(i, avRobList);

                            //  printf("Robot %d lost da %d step\n", i, avRobList->robList[i].lost);

                            deactivated++;
                            // This robot is deactivated (then is it marked as assigned)
                            assigned[i] = 1;


                        }
                    }
                }
                // printf("\n");
                //               printf("pr %d DEACT %d\n", associated, deactivated);
            }

        }
    }
}

void lists_merging(RobotList potRobList1, RobotList potRobList2,RobotList potRobList3) {


    potRobList.robNum = 0;
    
    int n = 0; 
    
   // printf("azzero presenza robot in cam \n");
    
    for (int i = 0; i < robMax; i++) {
    robot_is_in_cam1[i] = 0;
    robot_is_in_cam2[i] = 0;
    robot_is_in_cam3[i] = 0;
   
        }
    
    for (int i = 0; i < robMax; i++) {
        // printf("for %d \n",i); 
        if ((potRobList1.robList[i].active) == 1) {
            //  printf("active\n"); 
            //  if (potRobList1.robList[i].coord.y <= 1200) {
            //   printf("in margin: %d\n",i); 
            potRobList.robList[n].id = potRobList1.robList[i].id;
            potRobList.robList[n].center.x = potRobList1.robList[i].center.x;
            potRobList.robList[n].center.y = potRobList1.robList[i].center.y;
            potRobList.robList[n].coord.x = potRobList1.robList[i].coord.x;
            potRobList.robList[n].coord.y = potRobList1.robList[i].coord.y;
            potRobList.robList[n].lost = potRobList1.robList[i].lost;
            potRobList.robList[n].active = potRobList1.robList[i].active;
            potRobList.robList[n].orientation = potRobList1.robList[i].orientation;
            potRobList.robList[n].hasDuplicate = 0;
            robot_is_in_cam1[i] = 1;
            potRobList.robNum++;

            n++;

            //  }
        } 

    }

    for (int i = 0; i < robMax; i++) {


        if ((potRobList2.robList[i].active) == 1) {
            // if (potRobList2.robList[i].coord.y > 1200) {

            potRobList.robList[n].id = potRobList2.robList[i].id;
            potRobList.robList[n].center.x = potRobList2.robList[i].center.x;
            potRobList.robList[n].center.y = potRobList2.robList[i].center.y;
            // printf("prova: %f",potRobList.robList[n].center.y );
            potRobList.robList[n].coord.x = potRobList2.robList[i].coord.x;
            potRobList.robList[n].coord.y = potRobList2.robList[i].coord.y;
            potRobList.robList[n].lost = potRobList2.robList[i].lost;
            potRobList.robList[n].active = potRobList2.robList[i].active;
            potRobList.robList[n].orientation = potRobList2.robList[i].orientation;
            potRobList.robList[n].hasDuplicate = 0;
            robot_is_in_cam2[i] = 1;
            potRobList.robNum++;

            n++;
            //   }
        } 
    }
    
    for (int i = 0; i < robMax; i++) {


        if ((potRobList3.robList[i].active) == 1) {
            // if (potRobList3.robList[i].coord.y > 1200) {

            potRobList.robList[n].id = potRobList3.robList[i].id;
            potRobList.robList[n].center.x = potRobList3.robList[i].center.x;
            potRobList.robList[n].center.y = potRobList3.robList[i].center.y;
            // printf("prova: %f",potRobList.robList[n].center.y );
            potRobList.robList[n].coord.x = potRobList3.robList[i].coord.x;
            potRobList.robList[n].coord.y = potRobList3.robList[i].coord.y;
            potRobList.robList[n].lost = potRobList3.robList[i].lost;
            potRobList.robList[n].active = potRobList3.robList[i].active;
            potRobList.robList[n].orientation = potRobList3.robList[i].orientation;
            potRobList.robList[n].hasDuplicate = 0;
            robot_is_in_cam2[i] = 1;
            potRobList.robNum++;

            n++;
            //   }
        } 
    }
    //    printf("n_out: %d-",n);
    //    
    //    while(n<=robMax){
    //         printf("n_in: %d-",n);
    //             //   potRobList.robList[n].id = 0;
    //              //  potRobList.robList[n].coord.x = 0;
    //            //   potRobList.robList[n].coord.y = 0;
    //            //  potRobList.robList[n].lost = 0;
    //               potRobList.robList[n].active = 0;
    //                printf("%d;",n);
    //             n++;  
    //                 
    //    }
    //    printf("\n");


}

//this function is called everytime a new image is published

void imageCallback(const sensor_msgs::ImageConstPtr& original_image, int n) {

    if (exec_c[n] == 0) {


        //Convert from the ROS image message to a CvImage suitable for
        //working with OpenCV for processing

        cv_bridge::CvImagePtr cv_ptr;
        try {
            //Always copy, returning a mutable CvImage
            //OpenCV expects color images to use BGR channel order.
            cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
        } catch (cv_bridge::Exception& e) {
            //if there is an error during conversion, display it
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        IplImage* frame = cvCloneImage(&(IplImage) cv_ptr->image);



        //print robot id in immagine

        //          CvPoint point  = cvPointFrom32f(cordin);

        // cvNamedWindow("id2", CV_WINDOW_NORMAL);
        //cvResizeWindow("id2", 320, 280);


        //    CvPoint point;
        //
        //    point.x = avRobList.robList[0].center.x;
        //    point.y = avRobList.robList[0].center.y;
        //
        // //   printf("cord: %d, %d\n", point.x, point.y);
        //
        //    //char str[200];
        //  //  sprintf(str, "ciao");
        //  //  cv::Mat image11(frame);
        //  //  putText(image11, str, point, FONT_HERSHEY_SIMPLEX, 5, Scalar(0, 0, 255), 4);
        // //   cvShowImage("id2", frame);

        /*decommentare se merge immagini decommentato*/
        frame_c[n] = cvCloneImage(frame);
        //  printf("ok_C1\n");
        //   printf("callback1\n");
        cvReleaseImage(&frame);
    }
}


void gui_builder() {

    //    cvNamedWindow("id", CV_WINDOW_AUTOSIZE);
    //        // cvResizeWindow("id", 320, 280);
    //    cv::Mat *working_frame;
    //     cv::Mat *merged_frame = Mat(Size(640, 480), CV_8UC3);
    //      cv::Mat image1(frame_c1);
    //  //  resize(image1, working_frame, Size(), 0.5, 0.5); //320 256
    //     
    //  cv::Mat roi = Mat(merged_frame, Rect(0, 0, 320, 256));
    //    working_frame.copyTo(roi);
    //    
    ////roi = Mat(merged_frame, Rect(320, 0, 320, 240));
    ////detected_frame.copyTo(roi);
    ////roi = Mat(merged_frame, Rect(0, 240, 320, 240));
    ////threshold_color_frame.copyTo(roi);
    ////roi = Mat(merged_frame, Rect(320, 240, 320, 240));
    ////threshold_grayscale_frame.copyTo(roi);
    //         
    //         
    //         
    //        cv::imshow("id", merged_frame);
    //   

}

//void mergeImage() {
//
//    /* le immagini in ingresso non sono sufficientemente simili quindi non 
//     * riesce ad unirle
//     */
//
//    bool try_use_gpu = true;
//
//    vector<Mat> imgs;
//
//    if (frame_c1 != 0 && frame_c2 != 0) {
//        //
//        printf("immagini acquisite\n");
//        Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
//        //
//        //
//
//        //    to rotate image
//        Mat source(frame_c2);
//
//
//        Point2f src_center(source.cols / 2.0, source.rows / 2.0);
//        Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
//        Mat dst;
//        warpAffine(source, dst, rot_mat, source.size());
//
//        //Open the window
//        cv::namedWindow("rotated");
//        //IplImage* im_rotated = &dst.operator IplImage(); 
//        cv::imshow("rotated", dst);
//
//
//        cv::Mat image_c1(frame_c1);
//        cv::Mat image_c2(frame_c2);
//        //  imgs[0] = image_c1;
//        // imgs[1] = image_c2;
//        imgs.assign(1, image_c1);
//        imgs.assign(1, image_c2);
//
//        //
//        Mat pano;
//        //
//        Stitcher::Status status = stitcher.stitch(imgs, pano);
//        //
//        if (status != Stitcher::OK) {
//            cout << "Error stitching - Code: " << int(status) << endl;
//            //       cout << "Error in frame querying" << endl;
//            //            //   exit(0);
//            //      //   return -1;
//        } else {
//            cvNamedWindow("fus", CV_WINDOW_NORMAL);
//            cvResizeWindow("fus", 320, 240);
//            // cvShowImage("fus",frame_c1);
//            // cv::imshow("id", pano);
//            cv::imshow("fus", pano);
//
//        }
//        //        
//    }
//    //
//
//
//}

 int bestMeasure(int x_i, int y_i, int x_j, int y_j,int i,int j){
    
    int x_c= 1280/2;
    int y_c= 1024; //720
   float result_i = 0;
   float result_j = 0;
   result_i = sqrt((x_c-x_i)^2  + (y_c-y_i)^2);
   result_j = sqrt((x_c-x_j)^2  + (y_c-y_j)^2);
  // printf("distanza i: %.2f  \n distanza j:%.2f \n",result_i,result_j);
   if(result_i<=result_j){
       //il robot è più vicino alla telecamera 1
       return i;
   } else
       return j;
       
    
}

RobotList deleteDuplicate() {

    RobotList fusion; 
    // fusion.init = 0;


    float diff_x = 0;
    float diff_y = 0;
    for (int i = 0; i < robMax; i++) {


        if ((potRobList.robList[i].active) == 1) {
            //   printf("ricerca duplicati robot id: %d\n", i);
            for (int j = i + 1; j < robMax; j++) {

                if (potRobList.robList[j].hasDuplicate == 0) {

                    diff_x = abs(potRobList.robList[i].coord.x - potRobList.robList[j].coord.x);
                    diff_y = abs(potRobList.robList[i].coord.y - potRobList.robList[j].coord.y);

                    //   printf("diff [%d][%d], x: %2.2f y=%2.2f  ", i, j, diff_x, diff_y);

                    if ((diff_x <= 200) && (diff_y <= 200)) {
                        
                       j= bestMeasure(potRobList.robList[i].center.x,potRobList.robList[i].center.y,potRobList.robList[j].center.x,potRobList.robList[j].center.y,i,j);
                        
                        potRobList.robList[j].hasDuplicate = 1;
                       // printf("i,j: %d, %d\n",i,j);
                        //   printf(".....");
                        //   printf("duplicato = %d\n ", j);


                        //                     printf("potRobList I.x [%f] \n", potRobList.robList[i].coord.x);
                        //                     printf("potRobList J.x [%f] \n", potRobList.robList[j].coord.x);


                    } else {
                        potRobList.robList[j].hasDuplicate = 0;
                        //       printf("NO %d\n", j);
                    }

                }
            }

        }


    }


    //    for (int i = 0; i < robMax; i++) {
    //
    //        printf("id:[%d]  duplicato[%d]\n ", i, potRobList.robList[i].hasDuplicate);
    //
    //    }


    // printf("elimino eventuali rob doppi \n");
    int n = 0;
    fusion.robNum = 0;

    for (int i = 0; i < robMax; i++) {
        //   printf("potRobList.robList[%d].active= %d \n ", i, potRobList.robList[i].active);
        //   printf("potRobList.robList[%d].hasDuplicate= %d \n ", i, potRobList.robList[i].hasDuplicate);
        if (potRobList.robList[i].hasDuplicate == 0 && potRobList.robList[i].active == 1) { //
            //  printf("-->copio potRobList[%d] in fusion.robList[%d]\n ", i, n);
            fusion.robList[n].id = potRobList.robList[i].id;
            fusion.robList[n].coord.x = potRobList.robList[i].coord.x;
            fusion.robList[n].coord.y = potRobList.robList[i].coord.y;
            fusion.robList[n].center.y = potRobList.robList[i].center.y;
            fusion.robList[n].center.x = potRobList.robList[i].center.x;
            fusion.robList[n].lost = potRobList.robList[i].lost;
            fusion.robList[n].active = potRobList.robList[i].active;
            fusion.robList[n].orientation = potRobList.robList[i].orientation;


            n++;
            fusion.robNum++;

            //             if(fusion.robList[n].active == 1){
            //              fusion.robNum++;
            //                // 
            //            }
        }
    }
    //  printf("fusion.robNum %d\n", fusion.robNum);
    //  



    for (int i = fusion.robNum; i < robMax; i++) {
        //   printf("potRobList.robList[%d].hasDuplicate= %d ", i, potRobList.robList[i].hasDuplicate);

        fusion.robList[n].id = 0;
        fusion.robList[n].coord.x = 0;
        fusion.robList[n].coord.y = 0;
        fusion.robList[n].center.x = 0;
        fusion.robList[n].center.y = 0;
        fusion.robList[n].lost = 0;
        fusion.robList[n].active = 0;
        fusion.robList[n].orientation = 0;
        n++;
    }




    return fusion;
}

void putIdOnImages() {

//
//    /*
//     
//     funziona solo per la prima immagine perchè in avroblist ho solo le coordinate
//     provenienti da una sola telecamera, quindi quando faccio il confronto 
//     tra le coordinate dei potenziali e degli av trovo solo quelli della prima
//     telecamera (perchè quando elimino i duplicati  elimino i dati della seconda camera
//     (se ho le coordinate da entrambe le webcam*/
//
//    CvPoint point;
//
//   // cvNamedWindow("id1", CV_WINDOW_NORMAL);
//   // cvResizeWindow("id1", 320, 280);
//   // cvNamedWindow("id2", CV_WINDOW_NORMAL);
//   // cvResizeWindow("id2", 320, 280);
//
//
//    int idtemp;
//    int p = 0;
//
//    float diff_x;
//    float diff_y;
//
//    for (int i = 0; i < robMax; i++) {
//        point.x = 0;
//        point.y = 0;
//
//           if (potRobList.robList[i].active) {
//        point.x = potRobList.robList[i].center.x;
//        point.y = potRobList.robList[i].center.y;
//
//        p = 0;
//
//        while (p < robMax) {
//            //     printf(" potRobList %d -- avRobList %d --", i,p);
//            idtemp = 0;
//            diff_x = abs(potRobList.robList[i].coord.x - avRobList.robList[p].coord.x);
//            diff_y = abs(potRobList.robList[i].coord.y - avRobList.robList[p].coord.y);
//
//            //   printf("diff %f, %f\n",diff_x,diff_y);
//            if ((diff_x <= 200) && (diff_y <= 200)) {
//                idtemp = avRobList.robList[p].id;
//                //  printf("id:%d \n", idtemp);
//                break;
//            } else p++;
//        }
//
//
//
//
//        std::stringstream s;
//        s << idtemp;
//
//        if (robot_is_in_cam1[i] == 1) {
//       //      printf("rob %d in cam 1 point: %d %d \n", idtemp, point.x, point.y);
//            cv::Mat image11(frame_c[1]);
//            putText(image11, s.str(), point, FONT_HERSHEY_SIMPLEX, 5, Scalar(0, 0, 255), 4);
//
//        } else  {
//          //  printf("rob %d in cam 2 point: %d %d\n", idtemp, point.x, point.y);
//            cv::Mat image11(frame_c[2]);
//            putText(image11, s.str(), point, FONT_HERSHEY_SIMPLEX, 5, Scalar(0, 0, 255), 4);
//
//        }
//
//
//        }
//
//        //      }
//
//    }
//  //  cvShowImage("id1", frame_c[1]);
//   // cvShowImage("id2", frame_c[2]);
//    
}

int main(int argc, char **argv) {


    //name of the ros node
    ros::init(argc, argv, "fusioner");


    ros::NodeHandle nh;


    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);


    ros::Subscriber sub1 = nh.subscribe<tutorialROSOpenCV::Stringts>("robList_labrob14_c1", 10, boost::bind(roblistCallback, _1, 1) );
ros::Subscriber sub2 = nh.subscribe<tutorialROSOpenCV::Stringts>("robList_labrob14_c2", 10, boost::bind(roblistCallback, _1, 2) );
//ros::Subscriber sub3 = nh.subscribe<tutorialROSOpenCV::Stringts>("robList_labrob14_c3", 100, boost::bind(roblistCallback, _1, 3) );

    // %Tag(SUBSCRIBER)%
   // ros::Subscriber sub1 = nh.subscribe("robList_labrob14_c1", 100, roblistCallback1);
   // ros::Subscriber sub2 = nh.subscribe("robList_labrob14_c2", 100, roblistCallback2);
   // ros::Subscriber sub3 = nh.subscribe("robList_labrob14_c2", 100, roblistCallback3);

    //ros::Subscriber sub = nh.subscribe(imagetopic, 1, imageCallback);
    // image_transport::Subscriber sub = it.subscribe("/camera2_labrob14/RGB", 1, c2imageCallback, image_transport::TransportHints::TransportHints("compressed"));
  //  image_transport::Subscriber sub_C1 = it.subscribe("/camera1_labrob14/RGB", 1, boost::bind(imageCallback, _1, 1));
  //  image_transport::Subscriber sub_C2 = it.subscribe("/camera2_labrob14/RGB", 1, boost::bind(imageCallback, _1, 2));
  ///  image_transport::Subscriber sub_C3 = it.subscribe("/camera2_labrob14/RGB", 10, c3imageCallback);

    // %EndTag(SUBSCRIBER)%

    // ros::Publisher list_pub = nh.advertise<tutorialROSOpenCV::Stringts > ("robList", 100);






    avRobList.robNum = 0;
    avRobList.init = 1;
    ros::Rate r(10); // 20 hz
    
       int i=0;
    // int c = 0;
    RobotList fusion1;
    while (ros::ok()) {
        //   printf("*********c:[%d]\n", c);
        //c++;

        //  ros::Publisher list_pub = nh.advertise<std_msgs::String > ("robList", 100);


        //unisco le liste con stesso timestamp e pubblico la lista risultante
        lists_merging(robList_temp[1], robList_temp[2], robList_temp[3]);
        printf("---------------------------\n");
        fusion1 = deleteDuplicate();





        //                for (int i = 0; i < robMax; i++) {

        //                    //  printf("[%f] ", timestamp1.toSec());
        //                    printf("POT_Robot %d : (%f, %f, %d)\n", potRobList.robList[i].id, potRobList.robList[i].coord.x, potRobList.robList[i].coord.y, potRobList.robList[i].lost);
        //                }
        //        for (int i = 0; i < robMax; i++) {
        //            printf("FUS_Robot %d : (%f, %f)\n", fusion1.robList[i].id, fusion1.robList[i].coord.x, fusion1.robList[i].coord.y);
        //        }

        //  fusion1.init = 0;


        updateRobotList(&avRobList, fusion1);

        for (int i = 0; i < robMax; i++) {
            printf("POT_Robot %d : (%2.2f, %2.2f)\n", i+1, potRobList.robList[i].coord.x, potRobList.robList[i].coord.y);
        }

        //        for (int i = 0; i < robMax; i++) {
        //
        //            printf("FUS_Robot %d : (%f, %f)\n", fusion1.robList[i].id, fusion1.robList[i].coord.x, fusion1.robList[i].coord.y);
        //        }
        //
        //
        for (int i = 0; i < robMax; i++) {
            printf("AV_ROB %d : (%2.2f, %2.2f, %d)\n", i+1, avRobList.robList[i].coord.x, avRobList.robList[i].coord.y, avRobList.robList[i].lost);
        }

      //  putIdOnImages();
        
        
    
         static tf::TransformBroadcaster br;
         
        tf::Transform transform;

        std::stringstream ss;
       
       
       for(int index=1;index<=3;index++){
       
        ss.str("");
        ss << "/robot/" << index << "/base_link";
                              
        transform.setOrigin(tf::Vector3(avRobList.robList[index-1].coord.x / 1000, avRobList.robList[index-1].coord.y / 1000, 0));
                transform.setRotation(tf::Quaternion(0, 0, avRobList.robList[index-1].orientation / 360 * 2 * M_PI));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", ss.str()));
               
       }       
            cout << i++ <<endl;    
               
    
 
        // gui_builder();
        //mergeImage(); //vedere commento nel metodo
        //print robot id in immagine
        //        
        //          CvPoint point  = cvPointFrom32f(cordin); 
        //           //printf("pix %.2f,%.2f\n", potRobList.robList[0].center.x,potRobList.robList[0].center.y);
        //           // printf("pix %.2f,%.2f\n", potRobList.robList[1].center.x,potRobList.robList[1].center.y);
        //    cvCircle(frame1,point,10, cvScalar(0, 0, 255), 10, 8, 0);
        ////    cvCircle(imgFinal, point,10, cvScalar(0, 0, 255), 10, 8, 0);
        //
        cv::waitKey(10);

        //  if (frame_c1 != 0){
        for (int i=1;i<=3;i++){
        cvReleaseImage(&frame_c[i]);
        exec_c[i] = 0;
        }
        //  printf("rilasciato f1\n");
        //   }

//        // if (frame_c2 != 0){
//        cvReleaseImage(&frame_c2);
//        exec_c2 = 0;
//        //  printf("rilasciato f2\n");
//        //  }
//        
//        cvReleaseImage(&frame_c3);
//        exec_c3 = 0;

        r.sleep();

        ros::spinOnce();


    }


}