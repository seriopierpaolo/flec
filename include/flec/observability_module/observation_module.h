#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <flec/utils.h>
#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>
#include <flec/ceresOptimization.h>



Eigen::MatrixXd
jacobian(Eigen::Matrix4d t1, Eigen::Matrix4d t2, Eigen::Vector3d t12_, Eigen::Quaterniond q12_);
