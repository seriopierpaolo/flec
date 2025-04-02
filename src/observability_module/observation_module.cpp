#include <flec/observability_module/observation_module.h>

// Implementation of the ObservabilityEnforcer class
ObservabilityEnforcer::ObservabilityEnforcer(double epsilon, ceres::Problem& problem) 
    : epsilon_(epsilon), problem_(problem) {}

ceres::CallbackReturnType ObservabilityEnforcer::operator()(const ceres::IterationSummary& summary) {
    // Extract the Jacobian and residuals
    ceres::CRSMatrix jacobian;
    std::vector<double> residuals;
    problem_.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, &residuals, nullptr, &jacobian);
    
    // Convert CRSMatrix to Eigen Matrix
    Eigen::MatrixXd J = CRSMatrixToEigen(jacobian);
    
    // Update parameters based on the Jacobian and residuals
    UpdateParameters(J, residuals);

    return ceres::SOLVER_CONTINUE;
}

void ObservabilityEnforcer::UpdateParameters(Eigen::MatrixXd J, std::vector<double> residuals) {
    // Retrieve parameter blocks from the problem
    std::vector<double*> parameter_blocks;
    problem_.GetParameterBlocks(&parameter_blocks);

    // Perform SVD on J^T * J
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.transpose() * J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singular_values = svd.singularValues();

    // Truncate small singular values
    Eigen::VectorXd truncated_singular_values = singular_values;
    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values[i] < epsilon_) {
            truncated_singular_values[i] = 0.0;
        }
    }

    // Compute the pseudo-inverse of the truncated singular values
    Eigen::VectorXd inv_truncated_singular_values = truncated_singular_values;
    for (int i = 0; i < truncated_singular_values.size(); ++i) {
        if (truncated_singular_values[i] > epsilon_) {
            inv_truncated_singular_values[i] = 1.0 / truncated_singular_values[i];
        } else {
            inv_truncated_singular_values[i] = 0.0;
        }
    }

    // Compute the update step
    Eigen::VectorXd b = -J.transpose() * Eigen::Map<Eigen::VectorXd>(residuals.data(), residuals.size());
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd Sigma_inv = inv_truncated_singular_values.asDiagonal();
    Eigen::VectorXd update = U.transpose() * Sigma_inv * U * b;

    // Update the parameters based on the computed update vector
    int index = 0;
    for (double* parameter_block : parameter_blocks) {
        int parameter_block_size = problem_.ParameterBlockSize(parameter_block);
        for (int j = 0; j < parameter_block_size; ++j) {
            if (index < update.size()) {
                parameter_block[j] += update[index++];
            }
            std::cout << "Updating parameter: " << parameter_block[j] << std::endl;
        }
    }
}

