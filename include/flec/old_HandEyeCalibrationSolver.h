#ifndef HAND_EYE_CALIBRATION_SOLVER_H
#define HAND_EYE_CALIBRATION_SOLVER_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

class HandEyeCalibrationSolver {
public:
    HandEyeCalibrationSolver();

    void SetHandEyeTransformations(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2);

    void Solve();

private:
    Eigen::Matrix4d Tg1_;
    Eigen::Matrix4d Tg2_;
    double q_[4];  // Quaternion parameters
    double t_[3];  // Translation parameters

    struct HandEyeCostFunctor {
        HandEyeCostFunctor(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2);

        template <typename T>
        bool operator()(const T* const q, const T* const t, T* residuals) const;

        static ceres::CostFunction* Create(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2);

    private:
        const Eigen::Matrix4d Tg1_;
        const Eigen::Matrix4d Tg2_;
    };
};

#endif // HAND_EYE_CALIBRATION_SOLVER_H
