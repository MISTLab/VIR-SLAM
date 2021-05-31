#include "pg_standalone.h"
#include "GaussSmoothen.h"


pg_standalone::pg_standalone(/* args */){
}

pg_standalone::~pg_standalone(){
}

void pg_standalone::getPoseData(deque<nav_msgs::Odometry::ConstPtr> pose_buf){
  for(deque<nav_msgs::Odometry::ConstPtr>::iterator it = pose_buf.begin(); it != pose_buf.end(); it++){
    raw_poses.push_back(**it);
  }
}

void pg_standalone::getUWBData(deque<geometry_msgs::PointStamped::ConstPtr> uwb_buf){
  for(deque<geometry_msgs::PointStamped::ConstPtr>::iterator it = uwb_buf.begin(); it != uwb_buf.end(); it++){
    raw_uwbs.push_back(**it);
  } 
}

void pg_standalone::alignPosesUWB(){
  while(raw_poses.front().header.stamp.toSec()<raw_uwbs.front().header.stamp.toSec()){
    raw_poses.pop_front();
  }

  cout<< " start of poses "<<raw_poses.front().header.stamp.toSec() << "  diff with (uwb - pose) "<<  raw_uwbs.front().header.stamp.toSec() - raw_poses.front().header.stamp.toSec()<< endl;
  
  for(deque<geometry_msgs::PointStamped>::iterator it = raw_uwbs.begin(); it != raw_uwbs.end(); it++){
    ranges.push_back(it->point.x);
  }

  cout<< " range size "<< ranges.size()<<endl;


}


void pg_standalone::smoothRange(){

  double sigma = 0.2;
  int samples = 15; // Need to be odd number.


  // Add half of samples for gaussian smooth for edge values
  for(int i=0; i<samples/2; i++){
    ranges.insert(ranges.begin(),ranges.front());
    ranges.insert(ranges.end(),ranges.back());
  }

  smoothedRanges = gaussSmoothen(ranges, sigma, samples);

  // Remove the add values.
  for(int i=0; i<samples/2; i++){
    smoothedRanges.erase(smoothedRanges.begin());
    smoothedRanges.pop_back();
  }

  cout<< " smooth range size "<< smoothedRanges.size()<<endl;

}

void pg_standalone::findMatchedRange(){
// Based on timem of raw_poses and raw_uwbs, calculate correspoinding uwb ranges from smoothRanges.
  int idx = 0;
  deque<geometry_msgs::PointStamped>::iterator itUWB = raw_uwbs.begin();
  double uwbTime = itUWB->header.stamp.toSec();
  double leftTime = uwbTime;
    //for(deque<geometry_msgs::PointStamped>::iterator itUWB = raw_uwbs.begin(); itUWB != raw_uwbs.end(); itUWB++){
  for(deque<nav_msgs::Odometry>::iterator itPose = raw_poses.begin(); itPose < raw_poses.end(); itPose++){

    if(itUWB == raw_uwbs.end()-1){
      raw_poses.erase(itPose);
      break;
    }    
    while (uwbTime < itPose->header.stamp.toSec()){ // Find the uwb range pass pose
      itUWB++;

      leftTime = uwbTime;
      uwbTime = itUWB->header.stamp.toSec();
      idx ++;
    }
     
    double rightTime = uwbTime;
    double poseTime = itPose->header.stamp.toSec();

    double leftRange = smoothedRanges.at(idx-1);
    double rightRange = smoothedRanges.at(idx);

    double midRange = leftRange+(rightRange-leftRange)*(poseTime-leftTime)/(rightTime-leftTime); // Linear middle value of the pose time in uwb range time
    matchedRange.push_back(midRange);
    matchedPose.push_back(*itPose);



    //cout<< " before check pose time "<< raw_poses.size() <<endl;
    
    while((itPose+1)->header.stamp.toSec() < uwbTime){
      raw_poses.erase(itPose);
      itPose++;
    }
    //cout<< " idx "<< idx <<endl;
      

  }

  cout<< " findMatchedRange pose size "<< matchedPose.size() << " range size "<< matchedRange.size()<<endl;

}


void pg_standalone::optimization(){

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 3;
  options.max_num_iterations = 50*3;
  ceres::Solver::Summary summary;
  
}


void pg_standalone::initAnchorPos(){

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 3;
  options.max_num_iterations = 50*3;
  ceres::Solver::Summary summary;

  // Add residual blocks, which are uwb factors.
  int idx = 0;

  cout<<" Init Size matchedPose "<< matchedPose.size() << " range "<< matchedRange.size()<< endl; 

  for(vector<nav_msgs::Odometry>::iterator it = matchedPose.begin(); it != matchedPose.end(); it++){
  
    double uwbmeas = matchedRange.at(idx);

    intialAnchorFactor* oneFactor = new intialAnchorFactor(uwbmeas);
    Eigen::Vector3d rsPos(it->pose.pose.position.x,it->pose.pose.position.y,it->pose.pose.position.z);
    oneFactor->setPos(rsPos);
    ceres::CostFunction* cost_function
      = new ceres::AutoDiffCostFunction<intialAnchorFactor, 1, 3>(oneFactor);
    problem.AddResidualBlock(cost_function, NULL, anchor_pos);
    
    idx ++;

  }

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport()<< "\n";
  cout<< " Anchor "<< anchor_pos[0] << " "<< anchor_pos[1] << " "<< anchor_pos[2] << " " <<endl;
}


