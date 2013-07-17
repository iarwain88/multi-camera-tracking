
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

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <algorithm>    // std::min
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include "std_msgs/String.h"
#include <sstream>

#include <string>
#include <cv.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <vector>
#include <highgui.h>

#include <rosgraph_msgs/Clock.h>
#include <tutorialROSOpenCV/Stringts.h>
#define MAX_LOST  100000000
using namespace std;
using namespace boost;
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
//RobotList avRobList3;
ros::Time timestamp1;
ros::Time timestamp2;
//ros::Time timestamp3;

  int areaBlob = 50;
  int epsilon = 50;


float distMatrix[robMax][robMax];
float minDistMatrix[robMax][robMax];


string ss, ss1, ss2, ss3, ss4, ss5, ss6;





// %Tag(CALLBACK)%

void roblistCallback1(const tutorialROSOpenCV::Stringts::ConstPtr& msg) {


    vector <string> fields;

    split_regex(fields, msg->data.c_str(), regex(","));

    potRobList1.robList[0].id = atoi(fields[0].c_str());
    potRobList1.robList[0].coord.x = atof(fields[1].c_str());
    potRobList1.robList[0].coord.y = atof(fields[2].c_str());
    potRobList1.robList[0].lost = atoi(fields[3].c_str());
    potRobList1.robList[0].active = atoi(fields[4].c_str());
    potRobList1.robList[1].id = atoi(fields[5].c_str());
    potRobList1.robList[1].coord.x = atof(fields[6].c_str());
    potRobList1.robList[1].coord.y = atof(fields[7].c_str());
    potRobList1.robList[1].lost = atoi(fields[8].c_str());
    potRobList1.robList[1].active = atoi(fields[9].c_str());
    potRobList1.robList[2].id = atoi(fields[10].c_str());
    potRobList1.robList[2].coord.x = atof(fields[11].c_str());
    potRobList1.robList[2].coord.y = atof(fields[12].c_str());
    potRobList1.robList[2].lost = atoi(fields[13].c_str());
    potRobList1.robList[2].active = atoi(fields[14].c_str());
    potRobList1.robList[3].id = atoi(fields[15].c_str());
    potRobList1.robList[3].coord.x = atof(fields[16].c_str());
    potRobList1.robList[3].coord.y = atof(fields[17].c_str());
    potRobList1.robList[3].lost = atoi(fields[18].c_str());
    potRobList1.robList[3].active = atoi(fields[19].c_str());
    potRobList1.robList[4].id = atoi(fields[20].c_str());
    potRobList1.robList[4].coord.x = atof(fields[21].c_str());
    potRobList1.robList[4].coord.y = atof(fields[22].c_str());
    potRobList1.robList[4].lost = atoi(fields[23].c_str());
    potRobList1.robList[4].active = atoi(fields[24].c_str());
    potRobList1.robList[5].id = atoi(fields[25].c_str());
    potRobList1.robList[5].coord.x = atof(fields[26].c_str());
    potRobList1.robList[5].coord.y = atof(fields[27].c_str());
    potRobList1.robList[5].lost = atoi(fields[28].c_str());
    potRobList1.robList[5].active = atoi(fields[29].c_str());

    timestamp1 = msg->stamp;


//          for (int i=0; i<robMax;i++){
//        printf("--Robot cal1 %d : (%f, %f, %d)\n", i, potRobList1.robList[i].coord.x, potRobList1.robList[i].coord.y, potRobList1.robList[i].lost);
//          }
}

// %Tag(CALLBACK)%

void roblistCallback2(const tutorialROSOpenCV::Stringts::ConstPtr& msg) {


    vector <string> fields;

    split_regex(fields, msg->data.c_str(), regex(","));

    potRobList2.robList[0].id = atoi(fields[0].c_str());
    potRobList2.robList[0].coord.x = atof(fields[1].c_str());
    potRobList2.robList[0].coord.y = atof(fields[2].c_str());
    potRobList2.robList[0].lost = atoi(fields[3].c_str());
    potRobList2.robList[0].active = atoi(fields[4].c_str());
    potRobList2.robList[1].id = atoi(fields[5].c_str());
    potRobList2.robList[1].coord.x = atof(fields[6].c_str());
    potRobList2.robList[1].coord.y = atof(fields[7].c_str());
    potRobList2.robList[1].lost = atoi(fields[8].c_str());
    potRobList2.robList[1].active = atoi(fields[9].c_str());
    potRobList2.robList[2].id = atoi(fields[10].c_str());
    potRobList2.robList[2].coord.x = atof(fields[11].c_str());
    potRobList2.robList[2].coord.y = atof(fields[12].c_str());
    potRobList2.robList[2].lost = atoi(fields[13].c_str());
    potRobList2.robList[2].active = atoi(fields[14].c_str());
    potRobList2.robList[3].id = atoi(fields[15].c_str());
    potRobList2.robList[3].coord.x = atof(fields[16].c_str());
    potRobList2.robList[3].coord.y = atof(fields[17].c_str());
    potRobList2.robList[3].lost = atoi(fields[18].c_str());
    potRobList2.robList[3].active = atoi(fields[19].c_str());
    potRobList2.robList[4].id = atoi(fields[20].c_str());
    potRobList2.robList[4].coord.x = atof(fields[21].c_str());
    potRobList2.robList[4].coord.y = atof(fields[22].c_str());
    potRobList2.robList[4].lost = atoi(fields[23].c_str());
    potRobList2.robList[4].active = atoi(fields[24].c_str());
    potRobList2.robList[5].id = atoi(fields[25].c_str());
    potRobList2.robList[5].coord.x = atof(fields[26].c_str());
    potRobList2.robList[5].coord.y = atof(fields[27].c_str());
    potRobList2.robList[5].lost = atoi(fields[28].c_str());
    potRobList2.robList[5].active = atoi(fields[29].c_str());

    timestamp2 = msg->stamp;

    //      for (int i=0; i<robMax;i++)
    //    printf("Robot cal2 %d : (%f, %f, %f)\n", i, avRobList2.robList[i].coord.x, avRobList2.robList[i].coord.y, avRobList2.robList[i].lost);
    //   
}

//void makelistRobot(avRobList) {
//
//    ss1 = boost::str(boost::format("%f,%f,%f,%f,%f,") % avRobList.robList[0].id % avRobList.robList[0].coord.x % avRobList.robList[0].coord.y % avRobList.robList[0].lost % avRobList.robList[0].active);
//    ss2 = boost::str(boost::format("%f,%f,%f,%f,%f,") % avRobList.robList[1].id % avRobList.robList[1].coord.x % avRobList.robList[1].coord.y % avRobList.robList[1].lost % avRobList.robList[1].active);
//    ss3 = boost::str(boost::format("%f,%f,%f,%f,%f,") % avRobList.robList[2].id % avRobList.robList[2].coord.x % avRobList.robList[2].coord.y % avRobList.robList[2].lost % avRobList.robList[2].active);
//    ss4 = boost::str(boost::format("%f,%f,%f,%f,%f,") % avRobList.robList[3].id % avRobList.robList[3].coord.x % avRobList.robList[3].coord.y % avRobList.robList[3].lost % avRobList.robList[3].active);
//    ss5 = boost::str(boost::format("%f,%f,%f,%f,%f,") % avRobList.robList[4].id % avRobList.robList[4].coord.x % avRobList.robList[4].coord.y % avRobList.robList[4].lost % avRobList.robList[4].active);
//    ss6 = boost::str(boost::format("%f,%f,%f,%f,%f") % avRobList.robList[5].id % avRobList.robList[5].coord.x % avRobList.robList[5].coord.y % avRobList.robList[5].lost % avRobList.robList[5].active);
//
//
//    ss = ss1 + ss2 + ss3 + ss4 + ss5 + ss6; //dati dei robot 1 e 2 nella stessa stringa per poterli trasmettere
//    msg.data = ss;
//
//    msg.stamp = framets.clock; //associo alla lista robot il timestamp del frame relativo
//
////    // %Tag(ROSCONSOLE)%
////    ROS_INFO("%s", msg.data.c_str());
////    // %EndTag(ROSCONSOLE)%
//}



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
    
 //    printf("idAv: %f idPot: %f \n", idAv, idPot);
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
    float distTh = 200; //mm
  
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
                        // printf("+++++++++++++++idPot i: %d +++++++++++++\n",idPot);
                        // The only case it would get -1 is if all distances are infinity
                        // When can this happen?
                        if (idPot>-1)
                            minDist_av_pot = distMatrix[i][idPot];
                        else
                            minDist_av_pot = INFINITY;

                        printf("ROB %d SCEGLIE POT %d....", i, idPot);

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
                                //non fa la print!  
                                //  printf("assRob2mes: (idAv:%d,idPot: %d)\n", idAv, idPot);

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
                printf("\n");
                //               printf("pr %d DEACT %d\n", associated, deactivated);
            }

        }
    }
}

void lists_merging(RobotList potRobList1, RobotList potRobList2) {


    potRobList.robNum = 0;
    //    for (int i = 0; i < robMax; i++) {
    //
    //        potRobList.robList[i].active = 0;
    //        potRobList.robList[i].center.x = 0;
    //        potRobList.robList[i].center.y = 0;
    //        potRobList.robList[i].coord.x = 0;
    //        potRobList.robList[i].coord.y = 0;
    //        potRobList.robList[i].hasDuplicate = 0;
    //        potRobList.robList[i].head.x = 0;
    //        potRobList.robList[i].head.y = 0;
    //        potRobList.robList[i].id = i;
    //        potRobList.robList[i].lost = 0;
    //        potRobList.robList[i].orientation = 0;
    //        potRobList.robList[i].tail.x = 0;
    //        potRobList.robList[i].tail.y = 0;
    //
    //    }
    //    


    //    printf("[diff: %f] ", timestamp1.toSec() - timestamp2.toSec());
    //    printf("[%f] ", timestamp1.toSec());
    //    printf("[%f]\n ", timestamp2.toSec());

    //printf("potenziali \n"); 

    //    for (int i = 0; i < robMax; i++) {
    //
    //            //  printf("[%f] ", timestamp1.toSec());
    //            printf("pot1_Robot %d : (%f, %f)\n", potRobList1.robList[i].id, potRobList1.robList[i].coord.x, potRobList1.robList[i].coord.y);
    //        }
    //     for (int i = 0; i < robMax; i++) {
    //
    //            //  printf("[%f] ", timestamp1.toSec());
    //            printf("pot2_Robot %d : (%f, %f)\n", potRobList2.robList[i].id, potRobList2.robList[i].coord.x, potRobList2.robList[i].coord.y);
    //        }

    //    for (int i = 0; i < robMax; i++) {
    //
    //        printf("%d;", potRobList1.robList[i].active);
    //
    //    }
    //    printf("\n");
    int n = 0;
    for (int i = 0; i < robMax; i++) {
        // printf("for %d \n",i); 
        if ((potRobList1.robList[i].active) == 1) {
            //  printf("active\n"); 
            //  if (potRobList1.robList[i].coord.y <= 1200) {
            //   printf("in margin: %d\n",i); 
            potRobList.robList[n].id = potRobList1.robList[i].id;

            potRobList.robList[n].coord.x = potRobList1.robList[i].coord.x;
            potRobList.robList[n].coord.y = potRobList1.robList[i].coord.y;
            potRobList.robList[n].lost = potRobList1.robList[i].lost;
            potRobList.robList[n].active = potRobList1.robList[i].active;
            potRobList.robList[n].hasDuplicate = 0;
            potRobList.robNum++;

            n++;

            //  }
        }

    }

    for (int i = 0; i < robMax; i++) {


        if ((potRobList2.robList[i].active) == 1) {
            // if (potRobList2.robList[i].coord.y > 1200) {

            potRobList.robList[n].id = potRobList2.robList[i].id;
            potRobList.robList[n].coord.x = potRobList2.robList[i].coord.x;
            potRobList.robList[n].coord.y = potRobList2.robList[i].coord.y;
            potRobList.robList[n].lost = potRobList2.robList[i].lost;
            potRobList.robList[n].active = potRobList2.robList[i].active;
            potRobList.robList[n].hasDuplicate = 0;
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

RobotList deleteDuplicate() {

    RobotList fusion; //copio roblist1 e inserisco in coda i 
    // robot non doppi presenti nella seconda lista
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

                    printf("diff [%d][%d], x: %2.2f y=%2.2f  ", i, j, diff_x, diff_y);

                    if ((diff_x <= 200) && (diff_y <= 200)) {
                        potRobList.robList[j].hasDuplicate = 1;
                        printf(".....");
                        printf("duplicato = %d\n ", j);


                        //                     printf("potRobList I.x [%f] \n", potRobList.robList[i].coord.x);
                        //                     printf("potRobList J.x [%f] \n", potRobList.robList[j].coord.x);


                    } else {
                        potRobList.robList[j].hasDuplicate = 0;
                        printf("NO %d\n", j);
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


    printf("elimino eventuali rob doppi \n");
    int n = 0;
    fusion.robNum = 0;

    for (int i = 0; i < robMax; i++) {
        //   printf("potRobList.robList[%d].active= %d \n ", i, potRobList.robList[i].active);
        //   printf("potRobList.robList[%d].hasDuplicate= %d \n ", i, potRobList.robList[i].hasDuplicate);
        if (potRobList.robList[i].hasDuplicate == 0 && potRobList.robList[i].active == 1) { //
            printf("-->copio potRobList[%d] in fusion.robList[%d]\n ", i, n);
            fusion.robList[n].id = potRobList.robList[i].id;
            fusion.robList[n].coord.x = potRobList.robList[i].coord.x;
            fusion.robList[n].coord.y = potRobList.robList[i].coord.y;
            fusion.robList[n].lost = potRobList.robList[i].lost;
            fusion.robList[n].active = potRobList.robList[i].active;
            n++;
            fusion.robNum++;

            //             if(fusion.robList[n].active == 1){
            //              fusion.robNum++;
            //                // 
            //            }
        }
    }
    printf("fusion.robNum %d\n", fusion.robNum);
    //  



    for (int i = fusion.robNum; i < robMax; i++) {
        //   printf("potRobList.robList[%d].hasDuplicate= %d ", i, potRobList.robList[i].hasDuplicate);

        fusion.robList[n].id = 0;
        fusion.robList[n].coord.x = 0;
        fusion.robList[n].coord.y = 0;
        fusion.robList[n].lost = 0;
        fusion.robList[n].active = 0;
        n++;
    }


    for (int i = 0; i < robMax; i++) {
        printf("POT_Robot %d : (%f, %f)\n", i, potRobList.robList[i].coord.x, potRobList.robList[i].coord.y);
    }

    for (int i = 0; i < robMax; i++) {
       
        printf("FUS_Robot %d : (%f, %f)\n", fusion.robList[i].id, fusion.robList[i].coord.x, fusion.robList[i].coord.y);
    }

    return fusion;
}

int main(int argc, char **argv) {


    //name of the ros node
    ros::init(argc, argv, "fusioner");


    ros::NodeHandle nh;




    // %Tag(SUBSCRIBER)%
    ros::Subscriber sub1 = nh.subscribe("robList_labrob14_c1", 100, roblistCallback1);
    ros::Subscriber sub2 = nh.subscribe("robList_labrob14_c2", 100, roblistCallback2);
    // %EndTag(SUBSCRIBER)%

    // ros::Publisher list_pub = nh.advertise<tutorialROSOpenCV::Stringts > ("robList", 100);





    avRobList.robNum = 0;
    avRobList.init = 1;
    ros::Rate r(20); // 10 hz
    int c = 0;
    while (ros::ok()) {
        printf("*********c:[%d]\n", c);
        c++;

        //  ros::Publisher list_pub = nh.advertise<std_msgs::String > ("robList", 100);


        //unisco le liste con stesso timestamp e pubblico la lista risultante
        lists_merging(potRobList1, potRobList2);
        printf("---------------------------\n");
        RobotList fusion1 = deleteDuplicate();





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
            printf("AV_ROB %d : (%f, %f, %d)\n", i, avRobList.robList[i].coord.x, avRobList.robList[i].coord.y, avRobList.robList[i].lost);
        }



        r.sleep();

        ros::spinOnce();


    }

}
