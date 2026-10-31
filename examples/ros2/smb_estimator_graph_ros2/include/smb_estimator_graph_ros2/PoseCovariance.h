#pragma once
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
namespace smb_se {
using PoseCovariance=Eigen::Matrix<double,6,6>;
// ROS: [world translation, fixed-world rotation]. GTSAM Pose3 residual:
// [right/body rotation, right/body translation], about the sensor origin.
inline PoseCovariance poseResidualCovariance(const PoseCovariance& ros,const Eigen::Matrix3d& R) {
  PoseCovariance J=PoseCovariance::Zero();
  J.topRightCorner<3,3>()=R.transpose();J.bottomLeftCorner<3,3>()=R.transpose();
  return J*ros*J.transpose();
}
inline bool validPoseCovariance(const PoseCovariance& covariance) {
  if(!covariance.allFinite() || !covariance.isApprox(covariance.transpose(),1e-8))return false;
  const Eigen::LDLT<PoseCovariance> ldlt(covariance);
  return ldlt.info()==Eigen::Success && ldlt.isPositive() && ldlt.vectorD().minCoeff()>0;
}
}
