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

class UWBFactor : public ceres::SizedCostFunction<1,3,3>
{
  public:
    UWBFactor() = delete;
    float uwbmeas;
    UWBFactor(float dist)
    {
      uwbmeas = dist;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
      Eigen::Vector3d pos(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Vector3d anchor_pos(parameters[1][0], parameters[1][1], parameters[1][2]);
      Eigen::Vector3d tmp(pos-anchor_pos);
      float predist = tmp.norm();
      float weight = 1;
      residuals[0] = weight*(uwbmeas - predist );
      if(jacobians)
      {
            jacobians[0][0] = weight*(parameters[1][0]-parameters[0][0])/predist;
            jacobians[0][1] = weight*(parameters[1][1]-parameters[0][1])/predist;
            jacobians[0][2] = 0;//-weight*parameters[0][2]/predist;

            jacobians[1][0] = -weight*(parameters[1][0]-parameters[0][0])/predist;
            jacobians[1][1] = -weight*(parameters[1][1]-parameters[0][1])/predist;
            jacobians[1][2] = 0;//-weight*parameters[0][2]/predist;
      }
      //printf(" Meas of uwb is : %f, residual: %f \n", uwbmeas, residuals[0]);
      return true;
    }
};

