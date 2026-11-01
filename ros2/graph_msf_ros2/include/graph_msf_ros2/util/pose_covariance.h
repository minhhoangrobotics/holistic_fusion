#pragma once
#include <Eigen/Geometry>
namespace graph_msf {
// GTSAM Pose3 uses right/body [rotation,translation]. ROS odometry uses
// fixed-world [translation,rotation], about the published sensor origin.
inline Eigen::Matrix<double,6,6> fixedWorldPoseCovariance(
    const Eigen::Matrix<double,6,6>& body,const Eigen::Matrix3d& R) {
  Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Zero();
  J.topRightCorner<3,3>()=R;J.bottomLeftCorner<3,3>()=R;
  return J*body*J.transpose();
}
inline Eigen::Matrix<double,6,6> spatialWorldPoseCovarianceAtOrigin(
    const Eigen::Matrix<double,6,6>& spatial,const Eigen::Vector3d& worldPosition) {
  // GraphManager already uses Ad_T to express a left/spatial perturbation
  // about world origin. ROS position covariance belongs at the IMU origin:
  // dp = d_rho - [p]x d_theta. Do not rotate the world covariance again.
  Eigen::Matrix3d skew;const auto& p=worldPosition;
  skew<<0,-p.z(),p.y(),p.z(),0,-p.x(),-p.y(),p.x(),0;
  Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Zero();
  J.topLeftCorner<3,3>()=-skew;J.topRightCorner<3,3>().setIdentity();J.bottomLeftCorner<3,3>().setIdentity();
  return J*spatial*J.transpose();
}
inline Eigen::Matrix<double,6,6> movePoseCovarianceOrigin(
    const Eigen::Matrix<double,6,6>& covariance,const Eigen::Vector3d& worldLever) {
  Eigen::Matrix3d skew;skew<<0,-worldLever.z(),worldLever.y(),worldLever.z(),0,-worldLever.x(),-worldLever.y(),worldLever.x(),0;
  Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Identity();J.topRightCorner<3,3>()=-skew;
  return J*covariance*J.transpose();
}
}
