#include "uwb_posegraph.h"

uwbPoseGraph::uwbPoseGraph()
{

	t_optimization = std::thread(&uwbPoseGraph::optimize4DoF, this);
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    global_index = 0;
    nav_msgs::Path uwbpg_path;

    anchor_pos[0]=1.0;
    anchor_pos[1]=1.0;
    anchor_pos[2]=1.0;

}

uwbPoseGraph::~uwbPoseGraph()
{
	t_optimization.join();
}

void uwbPoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_uwbpg_path = n.advertise<nav_msgs::Path>("uwbpg_path", 1000);
    pub_current_pose = n.advertise<geometry_msgs::PoseStamped>("uwb_cur_pose",1000);

}

void uwbPoseGraph::pub_pose(KeyFrame* it)
{

    Vector3d P;
    Matrix3d R;
    it->getPose(P, R);
    Quaterniond Q;
    Q = R;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(it->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() ;
    pose_stamped.pose.position.y = P.y() ;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();

    pub_current_pose.publish(pose_stamped);


}


void uwbPoseGraph::addKeyFrame(KeyFrame* cur_kf)
{
	m_keyframelist.lock();
	keyframelist.push_back(cur_kf);
    optimize_buf.push(cur_kf->index); // if is not empty(got loop), do optimization.
	m_keyframelist.unlock();
}

KeyFrame* uwbPoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}


void uwbPoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front(); // Triger of real optimization work.
            while (!optimize_buf.empty())
            {
                optimize_buf.pop();
            }
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            options.max_solver_time_in_seconds = 3;
            options.max_num_iterations = 50*3;
            ceres::Solver::Summary summary;
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;

            // Add anchor position as parameter in the problem.
            
            problem.AddParameterBlock(anchor_pos, 3);

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();


                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);




                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0)// Frame i and previous frame[i-j]
                  {
                      // Calculate the RT from frame i to i-j: last 5 frame.; hardly calculated.
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }

                //add loop edge(removed)

                // add uwb edge
                if((*it)->hasUwbMeas && (*it)->uwbMeas>1)
                {
                    //printf("Add UWB factor");
                    float rawdist = (*it)->uwbMeas;
                    int noise = 0.0 ; //rand()%200;
                    //printf("noise is %d",noise);
                    UWBFactor* uwb_factor = new UWBFactor(rawdist+noise/1000.0);
                    problem.AddResidualBlock(uwb_factor, NULL, t_array[i], anchor_pos);     
                }
                
                if ((*it)->index == cur_index)// add frames untile cur_index.
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport()<< "\n";
            if(cur_kf->hasUwbMeas)
                printf("Anchor: %f, %f, %f \n",anchor_pos[0],anchor_pos[1],anchor_pos[2]);
            
            m_keyframelist.lock();
            i = 0;
            geometry_msgs::PoseStamped curPose;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;//update the frames after cur_index that has not optimimzed with drifts.
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
            pub_pose(cur_kf);
        }

        //std::chrono::milliseconds dura(2000);
        //std::this_thread::sleep_for(dura);
    }
}

void uwbPoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    nav_msgs::Path tempPath;

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() ;
        pose_stamped.pose.position.y = P.y() ;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        tempPath.header = pose_stamped.header;
        tempPath.header.frame_id = "world";
        tempPath.poses.push_back(pose_stamped);
    }

    m_keyframelist.unlock();
    pub_uwbpg_path.publish(tempPath);
    
    
}
