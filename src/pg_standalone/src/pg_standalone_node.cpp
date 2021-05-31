#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <ceres/ceres.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <math.h> 
#include "pg_standalone.h"


using namespace std;

pg_standalone posegraph;

deque<nav_msgs::Odometry::ConstPtr> pose_buf;
deque<geometry_msgs::PointStamped::ConstPtr> uwb_buf;

std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;


int UWB_POSEGRAPH = 1;

Eigen::Vector3d tic;
Eigen::Matrix3d qic;

double distThreshold = 0.05;
bool poseGot = false;


void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_buf.lock();
    pose_buf.push_back(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}


void uwb_callback(const geometry_msgs::PointStamped::ConstPtr &pos_msg)
{
    m_buf.lock();
    uwb_buf.push_back(pos_msg);        
    m_buf.unlock();
}

void process()
{
    cout<<" in process "<< pose_buf.size() << endl; 
    while (!posegraph.gotAnchorPos ){
        //cout<<" To intilization "<< endl;  
        if (pose_buf.size()>3000)
        {
            m_buf.lock();
            posegraph.getPoseData(pose_buf);
            posegraph.getUWBData(uwb_buf);
            pose_buf.clear();
            uwb_buf.clear();
            m_buf.unlock();

            posegraph.alignPosesUWB();
            posegraph.smoothRange();
            posegraph.findMatchedRange();

            cout<<" start initialization "<< endl;   
            //initialization to estimate anchor position.
            posegraph.initAnchorPos();

        }
        
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }

    while (false)
    {

        // poseStartTime
        // poseEndTime
        // uwbStartId
        // uwbEndId
        // smoothUWB
        // coorespondingUWB
        // FactorCreate
        // MotionLink
        // CreatePoseGraph
        // Optimimzation

        m_buf.lock();
        posegraph.getPoseData(pose_buf);
        pose_buf.clear();
        uwb_buf.clear();
        posegraph.getUWBData(uwb_buf);
        m_buf.unlock();

        posegraph.alignPosesUWB();
        posegraph.smoothRange();
        posegraph.findMatchedRange();



        geometry_msgs::PointStampedPtr uwb_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        double uwb_curRange = -1;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!pose_buf.empty())
        {
            pose_msg = pose_buf.front();
            while (!pose_buf.empty())
                pose_buf.pop_front();

        }

        m_buf.unlock();

        Eigen::Vector3d T = Eigen::Vector3d(pose_msg->pose.pose.position.x,
                                pose_msg->pose.pose.position.y,
                                pose_msg->pose.pose.position.z);
        Eigen::Matrix3d R = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                    pose_msg->pose.pose.orientation.x,
                                    pose_msg->pose.pose.orientation.y,
                                    pose_msg->pose.pose.orientation.z).toRotationMatrix();
        
        //KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R);   


        //printf(" ------process thread \n");

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }

}

int main(int argc, char **argv)
{
    printf("start uwb_posgraph \n");

    ros::init(argc, argv, "uwb_pose_graph");
    ros::NodeHandle n("~");

    // read param

    ros::Subscriber sub_pose = n.subscribe("/camera/odom/sample", 2000, pose_callback);
    ros::Subscriber sub_uwb = n.subscribe("/uwb/range", 2000, uwb_callback);

    std::thread measurement_process;
    measurement_process = std::thread(process);

    ros::spin();

    return 0;
}
