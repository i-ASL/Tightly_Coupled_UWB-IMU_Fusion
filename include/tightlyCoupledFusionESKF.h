#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

class TightlyCoupled {
public:
    TightlyCoupled();
    ~TightlyCoupled();
    void setImuVar(const double stdV, const double stdW);
    void setDt(const double delta_t);
    void setAnchorPositions(const Eigen::Matrix<double, 3, 8> &anchorPositions);
    Eigen::Matrix3d Exp(const Eigen::Vector3d &omega);
    Eigen::Quaterniond getQuaFromAA(Eigen::Matrix<double, 3, 1> vec);
    void prediction(const ImuData<double> &imu_data);
    void setZ(const UwbData<double> currUwbData);    
    void motionModel(const ImuData<double> &imu_data);
    void motionModelJacobian(const ImuData<double> &imu_data);
    Eigen::Matrix3d vectorToSkewSymmetric(const Eigen::Vector3d &vector);
    void measurementModel();
    void updateH();
    void measurementModelJacobian ();
    StateforESKF<double> correction();
    void setState(StateforESKF<double> &State);
    StateforESKF<double> getState();

private:
    Eigen::Matrix<double, 15, 15> covP, covQ, jacobianMatF;
    Eigen::Matrix<double, 8, 8> covR;
    Eigen::Matrix<double, 8, 1> vecZ, vecH;
    Eigen::Matrix<double, 3, 8> anchorPositions;
    Eigen::Matrix<double, 8, 15> jacobianMatH;
    Eigen::Matrix<double, 16, 15> XdX;
    double dt, TOL;
    Eigen::Matrix<double, 3, 1> _g;
    StateforESKF<double> STATE;

};