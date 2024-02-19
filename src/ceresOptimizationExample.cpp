#include <ceres/ceres.h>
#include <iostream>

class HandEyeCalibrationSolver {
public:
    HandEyeCalibrationSolver() : Tg1_(Eigen::Matrix4d::Identity()), Tg2_(Eigen::Matrix4d::Identity()) {}

    void SetHandEyeTransformations(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2) {
        Tg1_ = Tg1;
        Tg2_ = Tg2;
    }

    void Solve() {
        ceres::Problem problem;

        // Add cost function to the problem
        problem.AddResidualBlock(HandEyeCostFunctor::Create(Tg1_, Tg2_), nullptr, q_, t_);

        // Set Ceres Solver options
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        // Solve the problem
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Display the results
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Optimized quaternion: " << q_[0] << ", " << q_[1] << ", " << q_[2] << ", " << q_[3] << "\n";
        std::cout << "Optimized translation: " << t_[0] << ", " << t_[1] << ", " << t_[2] << "\n";
    }

private:
    Eigen::Matrix4d Tg1_;
    Eigen::Matrix4d Tg2_;
    double q_[4] = {1.0, 0.0, 0.0, 0.0};  // Initial guess: identity quaternion
    double t_[3] = {0.0, 0.0, 0.0};       // Initial guess: zero translation

    struct HandEyeCostFunctor {
        HandEyeCostFunctor(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2)
            : Tg1_(Tg1), Tg2_(Tg2) {}

        template <typename T>
        bool operator()(const T* const q, const T* const t, T* residuals) const {
            // Same as before...
        }

        static ceres::CostFunction* Create(const Eigen::Matrix4d& Tg1, const Eigen::Matrix4d& Tg2) {
            return new ceres::AutoDiffCostFunction<HandEyeCostFunctor, 1, 4, 3>(
                new HandEyeCostFunctor(Tg1, Tg2));
        }

    private:
        const Eigen::Matrix4d Tg1_;
        const Eigen::Matrix4d Tg2_;
    };
};

int main() {
    HandEyeCalibrationSolver solver;

    // Input hand-eye transformations
    Eigen::Matrix4d Tg1, Tg2;
    // Initialize Tg1 and Tg2 with your data

    solver.SetHandEyeTransformations(Tg1, Tg2);

    // Solve the hand-eye calibration problem
    solver.Solve();

    return 0;
}
