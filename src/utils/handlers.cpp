
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <flec/utils.h>

    /**
    * @brief
    * Compute the SO3 EXP operation.
    * @param input_vector_
    * Input vector.
    * @return
    * Result of the operation.
    */
    Eigen::Matrix<double, 3, 3> SO3Exp(const Eigen::Matrix<double, 3, 1>& input_vector_)
    {
        Eigen::Matrix<double, 3, 3> exp_result = Eigen::MatrixXd::Identity(3, 3);
        
        double input_vector_norm = input_vector_.norm();
        if(input_vector_norm > EPSILON)
        {
            Eigen::Matrix<double, 3, 3> input_skew_symmetric = skewSymmetric(input_vector_/input_vector_norm);
 
            // Rodrigues Transformation
            exp_result += sin(input_vector_norm)*input_skew_symmetric
                            + (1.0 - cos(input_vector_norm))*input_skew_symmetric*input_skew_symmetric;
        }
 
        return exp_result;
    }


     /**
    * @brief
    * Compute the SO3 LOG operation.
    * @param input_matrix_
    * Input matrix.
    * @return
    * Result of the operation.
    */
    template <typename T>
    Eigen::Matrix<T, 3, 1> SO3Log(Eigen::Matrix<T, 3, 3>& input_matrix_)
    {
        Eigen::Matrix3d in_mat = input_matrix_.template cast<T>();
        double input_matrix_trace = in_mat.trace();
        
        double scalar_constant = (input_matrix_trace > 3.0 - EPSILON) ? 0.0 : acos(0.5*(input_matrix_trace - 1.0));
 
        Eigen::Matrix<T, 3, 1> output_vector(input_matrix_(2, 1) - input_matrix_(1, 2),
                                                  input_matrix_(0, 2) - input_matrix_(2, 0),
                                                  input_matrix_(1, 0) - input_matrix_(0, 1));
 
        Eigen::Matrix<T, 3, 1> output = (fabs(scalar_constant) < EPSILON) ? (0.5*output_vector) : ((0.5*scalar_constant/sin(scalar_constant))*output_vector);

         return output.template cast<T>();
    }

    /**
    * @brief
    * Compute a skew-symmetric matrix from a vector.
    * @param input_vector_
    * Input vector.
    * @return
    * Associated skew-symmetric matrix.
    */
    Eigen::Matrix<double, 3, 3> skewSymmetric(const Eigen::Matrix<double, 3, 1>& input_vector_)
    {
        Eigen::Matrix<double, 3, 3> output_matrix = Eigen::Matrix<double, 3, 3>::Zero();
 
        output_matrix << 0.0,              -input_vector_(2),  input_vector_(1),
                         input_vector_(2),  0.0,              -input_vector_(0),
                        -input_vector_(1),  input_vector_(0),  0.0;
 
        return output_matrix;
    }



// Function to convert CRS matrix to Eigen::MatrixXd
Eigen::MatrixXd CRSMatrixToEigen(const ceres::CRSMatrix& matrix) {
    int numRows = matrix.num_rows;
    int numCols = matrix.num_cols;

    Eigen::MatrixXd result(numRows, numCols);
    result.setZero();

    for (int i = 0; i < numRows; ++i) {
        for (int k = matrix.rows[i]; k < matrix.rows[i + 1]; ++k) {
            int j = matrix.cols[k];
            double value = matrix.values[k];
            result(i, j) = value;
        }
    }


    return result;
}