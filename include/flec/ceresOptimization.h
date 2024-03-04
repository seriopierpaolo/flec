#ifndef CERESOPTIMIZATION_H
#define CERESOPTIMIZATION_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

class ceresOptimization {
public:
    ceresOptimization(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2);
    ~ceresOptimization();


    //setOptMatrixes(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2);

    struct CostFunction {

        CostFunction(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2);

        template <typename T>
        bool operator()(const T* const q, const T* const t, T* residuals) const;
        //bool operator()( Eigen::Quaterniond q,  Eigen::Vector3d t, Eigen::Vector3d residuals) const;

    private:

    const Eigen::Matrix4d Tl1_;
    const Eigen::Matrix4d Tl2_;


    };

    void solve();

private:

    ceres::Problem problem_;
    const Eigen::Matrix4d Tl1_;
    const Eigen::Matrix4d Tl2_;
    Eigen::Vector4d q_;  // Quaternion parameters
    Eigen::Vector3d t_;  // Translation parameters

};



#endif // CERESOPTIMIZATION_H
