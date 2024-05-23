#ifndef UTILS_H
#define UTILS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <ceres/ceres.h>
#include <flec/datastructure.h>

static constexpr double EPSILON = 1.0e-4;

/**
* @brief
* Compute the SO3 EXP operation.
* @param input_vector_
* Input vector.
* @return
* Result of the operation.
*/
Eigen::Matrix<double, 3, 3> SO3Exp(const Eigen::Matrix<double, 3, 1>& input_vector_);


template <typename T>
Eigen::Matrix<T, 3, 1> SO3Log(Eigen::Matrix<T, 3, 3>& input_matrix_);

/**
* @brief
* Compute a skew-symmetric matrix from a vector.
* @param input_vector_
* Input vector.
* @return
* Associated skew-symmetric matrix.
*/
Eigen::Matrix<double, 3, 3> skewSymmetric(const Eigen::Matrix<double, 3, 1>& input_vector_);

Eigen::MatrixXd CRSMatrixToEigen(const ceres::CRSMatrix& matrix);

#endif // UTILS