#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/uwb_factor.h"

#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>




struct uwbPose
{
    Vector3d pos;
    Matrix3d ori;
    double ypr[3];
    double range;
    std_msgs::Header header;
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T>
void Quaternion2Matrix(const T q[7], T R[9])
{
    T s = q[6];
    T x = q[3];
    T y = q[4];
    T z = q[5];
    R[0] = T(1) - T(2)*y*y - T(2)*z*z; R[1] = T(2)*x*y - T(2)*s*z;        R[2] = T(2)*x*z + T(2)*s*y;
    R[3] = T(2)*x*y + T(2)*s*z;        R[4] = T(1) - T(2)*x*x - T(2)*z*z; R[5] = T(2)*y*z - T(2)*s*x;
    R[6] = T(2)*x*z - T(2)*s*y;        R[7] = T(2)*y*z + T(2)*s*x;        R[8] = T(1) - T(2)*x*x - T(2)*y*y;
}

template <typename T> 
void YawFromQuaternion(const T q[7], T yaw)
{
    T w = q[6];
    T x = q[3];
    T y = q[4];
    T z = q[5];
    yaw = atan2(T(2.0) * (w * z + x * y), w * w + x * x - y * y - z * z);
};

template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};


struct LongWindowError
{
	LongWindowError(Vector3d t_i, Vector3d t_j, Matrix3d r_i, Matrix3d r_j, double weight)
				  :t_i(t_i), t_j(t_j), r_i(r_i), r_j(r_j), weight(weight){
                    Vector3d euler_i = Utility::R2ypr(r_i);
                    Vector3d euler_j = Utility::R2ypr(r_j);
                    relative_t = t_i - t_j;
                    t_x = relative_t.x();
                    t_y = relative_t.y();
                    t_z = relative_t.z();
                    pitch_i = euler_i.y();
                    roll_i =  euler_i.z();
                    relative_yaw = euler_i.x() - euler_j.z();
                  }

	template <typename T>
	bool operator()( const T* pose_i, const T* pose_j, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = pose_i[0] - pose_j[0];
		t_w_ij[1] = pose_i[1] - pose_j[1];
		t_w_ij[2] = pose_i[2] - pose_j[2];

		// // Quaternion to rotation matrix
		// T w_R_i[9];
        // Quaternion2Matrix(pose_i, w_R_i);
		// // rotation transpose
		// T i_R_w[9];
		// RotationMatrixTranspose(w_R_i, i_R_w);
		// // rotation matrix rotate point
		// T t_i_ij[3];
		// RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

        T yaw_i;
        T yaw_j;
        YawFromQuaternion(pose_i, yaw_i);
        YawFromQuaternion(pose_j, yaw_j);
        T w = T(weight);
		residuals[0] = w*(t_w_ij[0] - T(t_x));
		residuals[1] = w*(t_w_ij[1] - T(t_y));
		residuals[2] = w*(t_w_ij[2] - T(t_z));
		//residuals[3] = T(0);// 
        residuals[3] = NormalizeAngle(yaw_i - yaw_j   - T(relative_yaw));
        //cout<<" residual[0print]"<<residuals[0]<<" ;  pose_i "<< pose_i[0]<< " pose_j " << pose_j[0]<<endl;

		return true;
	}

	static ceres::CostFunction* Create(const Vector3d t_i, const Vector3d t_j, const Matrix3d r_i, const Matrix3d r_j, const double weight) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          LongWindowError, 4, 7, 7>(
	          	new LongWindowError(t_i, t_j, r_i, r_j, weight)));
	}


    Vector3d t_i;
    Vector3d t_j;
    Matrix3d r_i;
    Matrix3d r_j;
    Vector3d relative_t;
    double t_x, t_y, t_z, weight;
	double relative_yaw, pitch_i, roll_i;

};


struct movingError
{
	movingError(Vector3d t_i, double weight)
				  :t_i(t_i),  weight(weight){
                  }

	template <typename T>
	bool operator()( const T* pose_i, T* residuals) const
	{

        T w = T(weight);
		residuals[0] = w*(T(t_i[0]) - pose_i[0]);
		residuals[1] = w*(T(t_i[1]) - pose_i[1]);
		residuals[2] = w*(T(t_i[2]) - pose_i[2]);
		return true;
	}

	static ceres::CostFunction* Create(const Vector3d t_i, const double weight) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          movingError, 3, 7>(
	          	new movingError(t_i, weight)));
	}


    Vector3d t_i;
    double weight;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);


    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();

    double anchor_pos[3];

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];

    deque<Vector3d> PsLong;
    deque<Matrix3d> RsLong;
    deque<Vector3d> PsLong_result;
    deque<Matrix3d> RsLong_result;

    Vector3d virPos;
    Matrix3d virOri;
    Vector3d virVel;


    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    deque<double> uwb_keymeas;
    deque<double> uwb_buf;

    int frame_count;
    int frame_count_pasted;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];
    
    double** para_Pose_Long;
    // [WINDOW_SIZE_LONG][SIZE_POSE];


    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;

    //Estimate anchor position 
    deque<double> uwbMeas4AnchorEst;
    deque<Vector3d> pose4AnchorEst;
    void estimateAnchorPos();
};


