
#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class intialAnchorFactor{
  public:
    double range;
    intialAnchorFactor(double uwbmeas){
      range = uwbmeas;
    }

    intialAnchorFactor() = delete;

    Eigen::Vector3d pos;
    void setPos(Eigen::Vector3d pos_i){
      pos = pos_i;
    }


    template <typename T>
    bool operator()(const T* const x , T* residuals) const {
      double weight = 1;
      residuals[0] = weight*(range - sqrt((x[0]-pos[0])*(x[0]-pos[0])+(x[1]-pos[1])*(x[1]-pos[1])+(x[2]-pos[2])*(x[2]-pos[2])));
      return true;
    }


  // Example:
  // oneFactor = new MyScalarCostFunctor(uwbmeas);
  // Eigen::Vector3d rsPos;
  // oneFactor.setPos(rsPos);
  // CostFunction* cost_function
  //   = new AutoDiffCostFunction<MyScalarCostFunctor, 1, 3>(oneFactor);
  // problem.AddResidualBlock(cost_function, NULL, anchor_pos)

};

