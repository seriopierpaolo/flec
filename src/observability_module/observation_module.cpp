#include <flec/observability_module/observation_module.h>

// Implementation of the ObservabilityEnforcer class
ObservabilityEnforcer::ObservabilityEnforcer(double epsilon, ceres::Problem& problem) : epsilon_(epsilon), problem_(problem) {}

ceres::CallbackReturnType ObservabilityEnforcer::operator()(const ceres::IterationSummary& summary) {
    // Extract the Jacobian and residuals
    ceres::CRSMatrix jacobian;
    problem_.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobian);

    // Convert CRSMatrix to Eigen dense matrix
    Eigen::MatrixXd J(jacobian.num_rows, jacobian.num_cols);
    J.setZero();
    for (int i = 0; i < jacobian.num_rows; ++i) {
        for (int j = jacobian.rows[i]; j < jacobian.rows[i + 1]; ++j) {
            J(i, jacobian.cols[j]) = jacobian.values[j];
        }
    }

    // Perform SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singular_values = svd.singularValues();

    // Truncate small singular values
    Eigen::VectorXd truncated_singular_values = singular_values;
    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values[i] < epsilon_) {
            truncated_singular_values[i] = 0.0;
        }
    }

    // Recompute the pseudo-inverse of the Jacobian
    Eigen::MatrixXd S_pseudo_inverse = truncated_singular_values.asDiagonal().inverse();
    Eigen::MatrixXd J_pseudo_inverse = svd.matrixV() * S_pseudo_inverse * svd.matrixU().transpose();

    // Update the parameters based on the pseudo-inverse Jacobian
    Eigen::VectorXd parameter_updates = J_pseudo_inverse * singular_values;
    UpdateParameters(parameter_updates);

    return ceres::SOLVER_CONTINUE;
}

void ObservabilityEnforcer::UpdateParameters(const Eigen::VectorXd& updates) {
    int index = 0;
    // Get the parameter blocks in the problem
    std::vector<double*> parameter_blocks;
    problem_.GetParameterBlocks(&parameter_blocks);

    // Iterate over each parameter block
    for (double* parameter_block : parameter_blocks) {
        int parameter_block_size = problem_.ParameterBlockSize(parameter_block);
        // Iterate over each parameter in the block
        for (int j = 0; j < parameter_block_size; ++j) {
            parameter_block[j] -= updates(index++);
        }
    }
}
