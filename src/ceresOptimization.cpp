#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/internal/eigen.h>
#include <iostream>
#include <flec/ceresOptimization.h>
#include <flec/datastructure.h>

#include <ceres/loss_function.h>

ceresOptimization::ceresOptimization(tfAccumulator &b ) 
: q_{1.0, 0.0, 0.0, 0.0}, t_{0.0, 0.0, 0.0} 
{
    //convert from Affine3D to Matrix4d
    buffer_ = b;
    
    Tl1_ = b.accumulatedTraj.back().transformation_F.matrix();
    Tl2_ = b.accumulatedTraj.back().transformation_S.matrix();
    

}


//Destructor
ceresOptimization::~ceresOptimization() {
}

//CostFunction class that implements all the methods needed for a ceres cost function
ceresOptimization::CostFunction::CostFunction(Eigen::Matrix4d input_t1, Eigen::Matrix4d input_t2)
{
    Tl1_cf = input_t1;
    Tl2_cf = input_t2;



    q1_ = Eigen::Quaternion<double>(Tl1_cf.block<3, 3>(0,0));
    q2_ = Eigen::Quaternion<double>(Tl2_cf.block<3, 3>(0,0));

    t1_ = Tl1_cf.block<3, 1>(0, 3);
    t2_ = Tl2_cf.block<3, 1>(0, 3);
}


//7 RESIDUALS!
/****************************************************************************************
Simple Cost Function for hand-eye calibration problem (AX = XB)

****************************************************************************************/

template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals_ptr) const 
{
    Eigen::Map<const Eigen::Quaternion<T>> q12_(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t12_(t);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals_ptr);

    Eigen::Quaternion<T> q1 = q1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t1 = t1_.template cast<T>();
    
    Eigen::Quaternion<T> q2 = q2_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t2 = t2_.template cast<T>();

    // From Versatile Multi-LiDAR Accurate Self-Calibration System Based on Pose Graph Optimization
    // By Inversion of Equations (3) and (4) 
    Eigen::Quaternion<T> res_quat = (q12_*q2).inverse()*q1*q12_;
    Eigen::Matrix<T, 3, 1> res_transl = (q1*t12_ + t1) - (q12_*t2 + t12_);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);

    res.template block<3, 1>(0, 0) = res_transl;
    res.template block<3, 1>(3, 0) = T(2.0) * res_quat.vec();

    return true;
}






    /**
    * @brief
    * Compute the SO3 EXP operation.
    * @param input_vector_
    * Input vector.
    * @return
    * Result of the operation.
    */
    Eigen::Matrix<double, 3, 3> ceresOptimization::SO3Exp(const Eigen::Matrix<double, 3, 1>& input_vector_)
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
    Eigen::Matrix<T, 3, 1> ceresOptimization::SO3Log(Eigen::Matrix<T, 3, 3>& input_matrix_)
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
    Eigen::Matrix<double, 3, 3> ceresOptimization::skewSymmetric(const Eigen::Matrix<double, 3, 1>& input_vector_)
    {
        Eigen::Matrix<double, 3, 3> output_matrix = Eigen::Matrix<double, 3, 3>::Zero();
 
        output_matrix << 0.0,              -input_vector_(2),  input_vector_(1),
                         input_vector_(2),  0.0,              -input_vector_(0),
                        -input_vector_(1),  input_vector_(0),  0.0;
 
        return output_matrix;
    }



    Optimization_Result ceresOptimization::solve() {

    Optimization_Result result;
    std::unique_ptr<ceres::Problem> problem(new ceres::Problem);

    ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
    

    int i = 0;
    int sample_size = buffer_.accumulatedTraj.size();

    if (sample_size > 2)
    {

    Eigen::Matrix4d m1;
    Eigen::Matrix4d m2;

    ceres::LossFunction* loss_function = new ceres::HuberLoss(30);
    //ceres::LossFunction* loss_function = nullptr;

    // Add cost function to the problem
    std::cout << "The sample size is " << sample_size << std::endl;
    for (i = 1; i < sample_size; i++)
    {
        m1 = buffer_.accumulatedTraj[i].transformation_F.matrix();
        m2 = buffer_.accumulatedTraj[i].transformation_S.matrix();


        problem->AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunction, 6, 4, 3>
                                                        (new::ceresOptimization::CostFunction(m1,m2)),
                                loss_function, 
                                q_.coeffs().data(), 
                                t_.data());
    }

    // problem->SetParameterLowerBound(t_.data(), 2, -1);
    // problem->SetParameterUpperBound(t_.data(), 2, 1);

    problem->SetManifold(q_.coeffs().data(), quaternion_manifold);


    // Set Ceres Solver options
    

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);

    

    // Display the results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Optimized quaternion: " << q_.w() << ", " << q_.x() << ", "
                                          << q_.y() << ", " << q_.z() << "\n";
                                          
    std::cout << "Optimized translation: " << t_[0] << ", " << t_[1] << ", " << t_[2] << "\n";

    }

    result.q = q_;
    result.t = t_;

    return result;


}