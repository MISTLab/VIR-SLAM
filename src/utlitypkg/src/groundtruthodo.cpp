#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <random>
#include <iostream>
#include <string>
#include "math.h"
#include <vector>
#include <algorithm>
#include <time.h>


double gtx, gty, gtz, stx, sty, stz;
bool getStartPos = false;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0,0.03);

ros::Publisher pub_path;
ros::Publisher pub_sim_uwb;

nav_msgs::Path path;

int totalTime = 186; // 186s for Euroc01, 151s for Euroc02, 135s for Euroc03, 102s for Euroc04,114s for Euroc05,
int cycle = totalTime/2; // The pose frequency is 10hz. 20 cycles in total. Each cycle is totalTime*10/20.

std::vector<int> blockStageIndex = { };
int blockStageNum = 0; 
int sendIdx = 0;

void receivPos(const nav_msgs::OdometryPtr &odo_msg)
{
  int curStage = sendIdx/cycle;
  if (std::find(blockStageIndex.begin(), blockStageIndex.end(), curStage ) == blockStageIndex.end()){
    geometry_msgs::PointStamped dist;
    dist.header = odo_msg->header;
    dist.header.frame_id = "world";

    float x, y, z;
    x =  odo_msg->pose.pose.position.x;
    y =  odo_msg->pose.pose.position.y;
    z =  odo_msg->pose.pose.position.z;

    dist.point.x = sqrt(x*x + y*y + z*z)+distribution(generator);
    //dist.point.x = sqrt(x*x + y*y + z*z);
    //std::cout<<"Simulated UWB "<<dist.point.x<<std::endl;
    pub_sim_uwb.publish(dist);
  }
  sendIdx += 1;
  
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "groundtruth");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
  for (int i = 0; i<blockStageNum; i++ ){
      srand (time(NULL));
      int key = rand()%20;
      if(std::find(blockStageIndex.begin(), blockStageIndex.end(), key) == blockStageIndex.end()){
          blockStageIndex.push_back(key);
      }
      else{
          i--;
      }
  }

  sort(blockStageIndex.begin(), blockStageIndex.end()); 

  std::cout<< " BlockStageIndex ";
  for (auto i = blockStageIndex.begin(); i != blockStageIndex.end(); ++i)
  std::cout << *i << ' '; 

  std::cout << std::endl;
  //pub_path = n.advertise<nav_msgs::Path>("path", 100);
  pub_sim_uwb = n.advertise<geometry_msgs::PointStamped>("/uwb/corrected_range", 100);
  ros::Subscriber sub_gt = n.subscribe("/benchmark_publisher/odometry", 200, receivPos);
  ros::spin();
  return 0;
}

