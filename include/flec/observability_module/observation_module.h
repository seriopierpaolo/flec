#ifndef OBSERVABILITY_ENFORCER_H
#define OBSERVABILITY_ENFORCER_H


#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>
#include <flec/ceresOptimization.h>


// Custom callback to enforce observability constraints using TSVD
class ObservabilityEnforcer : public ceres::IterationCallback {
public:
    ObservabilityEnforcer(double epsilon, ceres::Problem& problem);

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override;

private:
    void UpdateParameters(const Eigen::VectorXd& updates);

    double epsilon_;
    ceres::Problem& problem_;
};

#endif // OBSERVABILITY_ENFORCER_H