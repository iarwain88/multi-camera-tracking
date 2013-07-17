//not used, just an attempt

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>

ros::Publisher clockPub;



    
   
int main(int argc, char** argv){
   ros::init(argc,argv,"clock");
   ros::NodeHandle node_handle;
      
    clockPub = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
     rosgraph_msgs::Clock clockmsg;
     
     
    while(1){
       
    clockmsg.clock = ros::Time::now();
    clockPub.publish( clockmsg );
    
    // %Tag(ROSCONSOLE)%
  ROS_INFO("%f",clockmsg.clock.toSec());
// %EndTag(ROSCONSOLE)%
      
  

   if (!(ros::ok())) {
            break; //to exit by control-c 
        }


        ros::spinOnce();
    
}
     
}
