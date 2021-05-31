#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <ceres/ceres.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <math.h> 
#include <ros/ros.h>
#include "initial_anchor.h"


using namespace std;

class pg_standalone
{
public:
  deque<nav_msgs::Odometry> raw_poses;
  deque<geometry_msgs::PointStamped> raw_uwbs;
  vector<double> ranges;
  vector<double> smoothedRanges;
  vector<double> matchedRange;
  vector<nav_msgs::Odometry> matchedPose;
  bool gotAnchorPos = false;



  void getPoseData(deque<nav_msgs::Odometry::ConstPtr> pose_buf);
  void getUWBData(deque<geometry_msgs::PointStamped::ConstPtr> uwb_buf);
  void alignPosesUWB();
  void smoothRange();
  void findMatchedRange();
  void optimization();

  void initAnchorPos();
  void initPG();

  double anchor_pos[3];
  
  

  pg_standalone(/* args */);
  ~pg_standalone();


};
