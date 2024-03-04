#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/internal/eigen.h>
#include <iostream>
#include <flec/ceresOptimization.h>


ceresOptimization::ceresOptimization(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2) : Tl1_(Tl1), Tl2_(Tl2) {
    // Initialize static arrays
    q_[0] = 1.0;
    q_[1] = 0.0;
    q_[2] = 0.0;
    q_[3] = 0.0;

    t_[0] = 0.0;
    t_[1] = 0.0;
    t_[2] = 0.0;
}


ceresOptimization::CostFunction::CostFunction(const Eigen::Matrix4d& Tl1, const Eigen::Matrix4d& Tl2)
    : Tl1_(Tl1), Tl2_(Tl2) {}



template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{
    Eigen::Map<const Eigen::Quaternion<T>> q_l12(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>>  t_l12(t);

    //Eigen::Quaternion<T> ql1 (Tl1_.template topLeftCorner<3, 3>());
    //Eigen::Quaternion<T> ql2 (Tl2_.template topLeftCorner<3, 3>());
    
    Eigen::Matrix<T, 3, 3> rotation_matrix1 = (Tl1_.template cast<T>()).template topLeftCorner<3, 3>();
    Eigen::Matrix<T, 3, 3> rotation_matrix2 = (Tl2_.template cast<T>()).template topLeftCorner<3, 3>();

    
    Eigen::Quaternion<T> ql1(rotation_matrix1);
    Eigen::Quaternion<T> ql2(rotation_matrix2);



    Eigen::Matrix<T, 3, 1> tl1 = (Tl1_.block<3,1>(0,3)).template cast<T>();
    Eigen::Matrix<T, 3, 1> tl2 = (Tl2_.block<3,1>(0,3)).template cast<T>();




    /*******************************************************/   
    /*ROTATIONAL RESIDUAL*/

    //R_L12*RL1
    Eigen::Quaternion<T> q_l12_1 = q_l12 * ql1;
    //R_L12*RL2
    Eigen::Quaternion<T> q_l12_2 = q_l12 * ql2;


    Eigen::Quaternion<T> q_l12_1_inverse = q_l12_1.conjugate();
    //Residual for Rotational Part
    Eigen::Quaternion<T> q_l12_estimated = q_l12_1_inverse * q_l12_2;

    

    /*******************************************************/
    /*TRANSLATIONAL RESIDUAL*/
    Eigen::Matrix<T, 3, 1> p1 = (ql1.toRotationMatrix() * t_l12) + tl1;
    Eigen::Matrix<T, 3, 1> p2 = (q_l12.toRotationMatrix() * tl2) + t_l12;


    Eigen::Matrix<T,3,1> t_l12_estimated = p2 - p1;

    /*******************************************************/
    /*RESIDUAL VECTOR*/
    
    Eigen::Matrix<T, 3, 1> residuals_translation(residuals);
    Eigen::Matrix<T, 3, 1> residuals_rotation(residuals + 3);

    residuals_translation = p2 - p1;

    // Convert quaternion to Eigen vector before assigning
    Eigen::Quaternion<T> q_l12_estimated_cast = q_l12_estimated.template cast<T>();
    residuals_rotation << q_l12_estimated_cast.x(), q_l12_estimated_cast.y(), q_l12_estimated_cast.z(), q_l12_estimated_cast.w();

    /*******************************************************/

    std::cout<<"Residual Translation\n"<<residuals_translation<<std::endl;
    
    std::cout<<"Residual "<<residuals<<std::endl;
    return true;

}


    void ceresOptimization::solve() {
    
    ceres::Problem problem;
    //ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
    CostFunction cost_function(Tl1_, Tl2_);
    

    // Add cost function to the problem
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunction, 1, 4, 3>(&cost_function),
                            nullptr, 
                            q_, 
                            t_);

    //problem.SetManifold(q_,quaternion_manifold);

    // Set Ceres Solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    

    // Display the results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Optimized quaternion: " << q_[0] << ", " << q_[1] << ", " << q_[2] << ", " << q_[3] << "\n";
    std::cout << "Optimized translation: " << t_[0] << ", " << t_[1] << ", " << t_[2] << "\n";
}







