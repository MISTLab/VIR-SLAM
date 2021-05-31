/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class UWBFactor : public ceres::SizedCostFunction<1,7,3>
{
  public:
    UWBFactor() = delete;
    double uwbmeas;
    double weight;
    UWBFactor(double dist, double _weight)
    {
      uwbmeas = dist;
      weight = _weight;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
      Eigen::Vector3d pos(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Vector3d anchor_pos(parameters[1][0], parameters[1][1], parameters[1][2]);
      Eigen::Vector3d tmp = pos-anchor_pos;
      double predist = tmp.norm();
      residuals[0] = weight*(uwbmeas - predist );
      if(jacobians)
      {
            jacobians[0][0] = weight*(parameters[1][0]-parameters[0][0])/predist;
            jacobians[0][1] = weight*(parameters[1][1]-parameters[0][1])/predist;
            //jacobians[0][2] = weight*(parameters[1][2]-parameters[0][2])/predist;
            jacobians[0][2] = 0;//-weight*parameters[0][2]/predist;
            jacobians[0][3] = 0;//-weight*parameters[0][2]/predist;
            jacobians[0][4] = 0;//-weight*parameters[0][2]/predist;
            jacobians[0][5] = 0;//-weight*parameters[0][2]/predist;
            jacobians[0][6] = 0;//-weight*parameters[0][2]/predist;

            if (KNOWN_ANCHOR == 1)
            {
              jacobians[1][0] = 0.;//
              jacobians[1][1] = 0.;//
              jacobians[1][2] = 0.;//
            }
            else
            {      
              jacobians[1][0] = -weight*(parameters[1][0]-parameters[0][0])/predist;//0.;//
              jacobians[1][1] = -weight*(parameters[1][1]-parameters[0][1])/predist;//0.;//
              jacobians[1][2] = -weight*(parameters[1][2]-parameters[0][2])/predist;//0;//
            }
      }
      //printf(" Meas of uwb is : %f, residual: %f \n", uwbmeas, residuals[0]);
      return true;
    }
};


class UWBAnchorFactor : public ceres::SizedCostFunction<1,3>
{
  public:
    UWBAnchorFactor() = delete;
    double uwbmeas;
    double weight;
    Eigen::Vector3d position;
    
    UWBAnchorFactor(double dist, double _weight, Eigen::Vector3d _position)
    {
      uwbmeas = dist;
      weight = _weight;
      position = _position;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
      Eigen::Vector3d anchor_pos(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Vector3d tmp = position-anchor_pos;
      double predist = tmp.norm();
      residuals[0] = weight*(uwbmeas - predist );
      if(jacobians)
      {
            jacobians[0][0] = -weight*(parameters[0][0]-position[0])/predist;
            jacobians[0][1] = -weight*(parameters[0][1]-position[1])/predist;
            jacobians[0][2] = -weight*(parameters[0][2]-position[2])/predist;
            //jacobians[0][2] = 0;//-weight*parameters[0][2]/predist;

      }
      //printf(" Meas of uwb is : %f, residual: %f \n", uwbmeas, residuals[0]);
      return true;
    }
};

