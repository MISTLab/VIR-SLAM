#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "uwb_posegraph.h"
using namespace std;


queue<nav_msgs::Odometry::ConstPtr> pose_buf;
deque<geometry_msgs::PointStamped> uwb_buf;
deque<geometry_msgs::PointStamped>::reverse_iterator lastUwbIt = uwb_buf.rend();

std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;

uwbPoseGraph posegraph;

int UWB_POSEGRAPH = 1;

Eigen::Vector3d tic;
Eigen::Matrix3d qic;


void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
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


void uwb_callback(const geometry_msgs::PointStamped &pos_msg){
    m_buf.lock();
    uwb_buf.push_back(pos_msg);
    //printf(" callback uwb %f end %f \n",pos_msg.header.stamp.toSec(),uwb_buf.back().header.stamp.toSec());
    m_buf.unlock();
}

void process()
{
    if (!UWB_POSEGRAPH)
        return;
    while (true)
    {
        geometry_msgs::PointStampedPtr uwb_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        double uwb_curRange = -1;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!pose_buf.empty())
        {
            pose_msg = pose_buf.front();
            while (!pose_buf.empty())
                pose_buf.pop();

        }

        m_buf.unlock();

        if (pose_msg != NULL)// when receive the key pose topic, means this is the keyFrame.
        {
            m_buf.lock();
            //printf(" received pose_msg %f uwb %f \n",pose_msg->header.stamp.toSec(),uwb_buf.back().header.stamp.toSec());
            double t_time;
            t_time = pose_msg->header.stamp.toSec();

            // Find the uwb measurement for the pose
            if (!uwb_buf.empty() && uwb_buf.back().header.stamp.toSec()>=t_time)
            {
                //find the first time smaller than pose time
                for (std::deque<geometry_msgs::PointStamped>::reverse_iterator it = uwb_buf.rbegin(); it!=lastUwbIt; it++)
                {
                    //printf(" received pose_msg %f uwb %f \n",pose_msg->header.stamp.toSec(),it->header.stamp.toSec());
                    if (it->header.stamp.toSec()<t_time && it->header.stamp.toSec()>t_time-1)
                    {
                        uwb_curRange = it->point.x;
                        lastUwbIt = it;
                        printf(" received uwb_curRange %f \n", uwb_curRange);
                        break;
                    }
                    else if (it->header.stamp.toSec()<t_time-1)
                    {
                        printf(" No approriate UWB measure matched \n");
                        lastUwbIt = it;
                        break;
                    }
                }
            }
            else
            {
                printf(" No UWB measure received or no new \n");
            }
            
            m_buf.unlock();
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            
            KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R);   
            if(uwb_curRange != -1)
            {
                // add uwb meas to keyframe.
                keyframe->hasUwbMeas = true;
                keyframe->uwbMeas = lastUwbIt->point.x;
                printf(" uwb meas %f \n", lastUwbIt->point.x);
            }

            m_process.lock();
            posegraph.addKeyFrame(keyframe);
            m_process.unlock();
            frame_index++;
       
        }
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
    posegraph.registerPub(n);

    // read param

    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_uwb = n.subscribe("/uwb/corrected_range", 2000, uwb_callback);

    std::thread measurement_process;
    measurement_process = std::thread(process);

    ros::spin();

    return 0;
}
