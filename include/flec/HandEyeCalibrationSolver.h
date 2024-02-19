#include <ceres/ceres.h>
#include <iostream>

class HandEyeCalibrationSolver {
public:
    HandEyeCalibrationSolver() : Tg1_(Eigen::Matrix4d::Identity()), Tg2_(Eigen::Matrix4d::Identity()) {}

    void SetHandEyeTransformations(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2) {
    }

    void Solve() {
    }

private:
    Eigen::Matrix4d Tg1_;
    Eigen::Matrix4d Tg2_;
    double q_[4];  // Initial guess: identity quaternion
    double t_[3];       // Initial guess: zero translation

    struct HandEyeCostFunctor {
    };
};

