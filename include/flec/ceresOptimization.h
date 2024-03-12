#ifndef CERESOPTIMIZATION_H
#define CERESOPTIMIZATION_H

#include <flec/datastructure.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>

class ceresOptimization {
public:
    ceresOptimization(tfAccumulator* Buffer);
    ~ceresOptimization();


    //setOptMatrixes(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2);

    struct CostFunction {

        CostFunction(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2);

        template <typename T>
        bool operator()(const T* const q, const T* const t, T* residuals) const;
        //bool operator()( Eigen::Quaterniond q,  Eigen::Vector3d t, Eigen::Vector3d residuals) const;

    private:

    //tfAccumulator* buffer;
    const Eigen::Matrix4d Tl1_;
    const Eigen::Matrix4d Tl2_;

    Eigen::Quaternion<double> q1_;
    Eigen::Quaternion<double> q2_;
    Eigen::Matrix<double, 3, 1> t1_;
    Eigen::Matrix<double, 3, 1> t2_;


    };

    void solve();

    static Eigen::Matrix<double, 3, 3> SO3Exp(const Eigen::Matrix<double, 3, 1>& input_vector_);
 
    static Eigen::Matrix<double, 3, 1> SO3Log(const Eigen::Matrix<double, 3, 3>& input_matrix_);

    static Eigen::Matrix<double, 3, 3> skewSymmetric(const Eigen::Matrix<double, 3, 1>& input_vector_);

private:

    tfAccumulator buffer_;
    ceres::Problem problem_;
    const Eigen::Matrix4d Tl1_;
    const Eigen::Matrix4d Tl2_;
    Eigen::Vector4d q_;  // Quaternion parameters
    Eigen::Vector3d t_;  // Translation parameters
    static constexpr double EPSILON = 1.0e-4;
};



#endif // CERESOPTIMIZATION_H
