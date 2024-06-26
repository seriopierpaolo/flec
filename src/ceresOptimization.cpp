#include <cstdlib> 

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/internal/eigen.h>
#include <iostream>

#include <flec/observability_module/observation_module.h>
#include <flec/ceresOptimization.h>
#include <flec/datastructure.h>
#include <flec/utils.h>

#include <ceres/loss_function.h>

ceresOptimization::ceresOptimization(TfBatch &b, bool useCallbackFlag) 
: q_{1.0, 0.0, 0.0, 0.0}, t_{0.0, 0.0, 0.0}
{
    //convert from Affine3D to Matrix4d
    

    buffer_ = b;
    
    Tl1_ = b.currentBatch.back().transformation_F.matrix();
    Tl2_ = b.currentBatch.back().transformation_S.matrix();
    useCallback = useCallbackFlag;
    

}


//Destructor
ceresOptimization::~ceresOptimization() {
}

/*
class JIterationCallback : public ceres::IterationCallback {
public:
  ceres::CRSMatrix jacobian;
  std::vector<double> residuals; 
  std::vector<double> param; 
  ceres::Problem* problem;  // Pointer to the problem instance

  JIterationCallback(ceres::Problem* prob) : problem(prob) {}

  ~JIterationCallback() override = default;

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    ceres::Problem::EvaluateOptions eval_options;
    std::vector<double*> *params;
    problem->GetParameterBlocks(params);
  
    
    problem->Evaluate(eval_options, nullptr, nullptr, nullptr, &jacobian);
    Eigen::MatrixXd jC2E = CRSMatrixToEigen(jacobian);
    Eigen::MatrixXd jCP = jC2E.transpose()*jC2E;
    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(jCP);
    //std::cout << "The singular values of the Ceres Jacobian are:\n" << svd.singularValues() << std::endl;
    //Eigen::VectorXd arraySvd = svd.singularValues();
    bool hasZeroSingularValue = (svd.singularValues().minCoeff() > 0.01);
    if (hasZeroSingularValue){
    //Eigen::MatrixXd jCP = jC2E.transpose()*jC2E;
    
    // Print Jacobian
    //std::cout << "\n" << "\n" << "Jacobian computed from CERES (J_t * J): \n"<< jCP << std::endl;

    return ceres::SOLVER_CONTINUE;
    //return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    }
    else 

    std::cout << "The singular values of the Ceres Jacobian are:\n" << svd.singularValues() << std::endl;

    
    return ceres::SOLVER_CONTINUE;
  }
};
   */
/*
  class JIterationCallback : public ceres::IterationCallback {
  public:
    
    ceres::CRSMatrix jacobian;

    JIterationCallback() = default;

    ~JIterationCallback() override = default;

      ceres::CallbackReturnType operator ()(ceres::Problem problem, std::vector<double*> parameterBlocks) {
      problem.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobian);
      return ceres::SOLVER_CONTINUE;
    }
  };
*/


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


class MyIterationCallback : public ceres::IterationCallback {
 public:
  MyIterationCallback(const double* m, const double* c) {}
  ~MyIterationCallback() override = default;
  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) final {

    return ceres::SOLVER_CONTINUE;
  }
};



//6 RESIDUALS!
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

    //ceres::CRSMatrix jacobianCeres;
    
    

    // From Versatile Multi-LiDAR Accurate Self-Calibration System Based on Pose Graph Optimization
    // By Inversion of Equations (3) and (4) 
    Eigen::Quaternion<T> res_quat = (q12_*q2).inverse()*q1*q12_;
    Eigen::Matrix<T, 3, 1> res_transl = (q1*t12_ + t1) - (q12_*t2 + t12_);

    //ceresOptimization::CostFunction::Evaluate(ceres::Problem::EvaluateOptions(), q, t, residuals_ptr, nullptr, &jacobianCeres);
    
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);

    res.template block<3, 1>(0, 0) = res_transl;
    res.template block<3, 1>(3, 0) = T(2.0) * res_quat.vec();

    
    return true;
}




    Optimization_Result ceresOptimization::solve() {
    
    //bool jacobianSVDCheck_ = 0;
    
    Optimization_Result result;
    std::unique_ptr<ceres::Problem> problem(new ceres::Problem);

    ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
    

    //int i = 0;
    
    int sample_size = buffer_.currentBatch.size();
    int i = sample_size;
    //if (sample_size > 2)
    //{

    Eigen::Matrix4d m1;
    Eigen::Matrix4d m2;

    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1);
    ceres::LossFunction* loss_function = nullptr;

    // Add cost function to the problem
    std::cout << "The sample size is " << sample_size << std::endl;
    
    for (i = 1; i <= sample_size - 1; i++)
    //WHY SAMPLE SIZE-1 ???????
    {
        m1 = buffer_.currentBatch[i].transformation_F.matrix();
        m2 = buffer_.currentBatch[i].transformation_S.matrix();
        
        
        problem->AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunction, 6, 4, 3>
                                                        (new::ceresOptimization::CostFunction(m1,m2)),
                                loss_function, 
                                q_.coeffs().data(), 
                                t_.data());
        
        
        //JACOBIAN FOR EACH ITERATION
        // Access Jacobian
        
      //ceres::CRSMatrix jacobianCeres;
      //problem->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobianCeres);
      //Eigen::MatrixXd jC2E = CRSMatrixToEigen(jacobianCeres);
      //Eigen::MatrixXd jCP = jC2E.transpose()*jC2E;
      //Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(jCP);
      
      
      //std::cout << "The singular values of the Ceres Jacobian are:\n" << svd.singularValues() << std::endl;
      //std::cout << "A good initial guess could be: \n"<< params << std::endl;


     
      //newSingularValues.minCoeff() << std::
    
   /*
    if (newSingularValues.minCoeff() > 0.02){
      std::cout << "END at " << i << "\n" << std::endl;
      //std::cout << "Initial Guess is " <<  << "\n" << std::endl;
      jacobianSVDCheck_ = 1;
       break;}
    
    
    }
    */
     //problem->SetParameterLowerBound(t_.data(), 2, -0.01);
     //problem->SetParameterUpperBound(t_.data(), 2, 0.01);

    problem->SetManifold(q_.coeffs().data(), quaternion_manifold);
      //std::cout << i << std::endl;
      }
    //}
    // Set Ceres Solver options

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    options.update_state_every_iteration = true;
    //options.check_gradients = true;

   
    double epsilon = 1e-6;

    if (useCallback){
    //Reaching Jacobian while the optimization is running
// Add the observability enforcer callback
    
    ObservabilityEnforcer observability_enforcer(epsilon, *problem); // 3 parameters (A, B, C)
    options.callbacks.push_back(&observability_enforcer);
    }
    // Solve the problem
    
        
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);

    
    // Display the results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Optimized quaternion: " << q_.w() << ", " << q_.x() << ", "
                                          << q_.y() << ", " << q_.z() << "\n";
                                          
    std::cout << "Optimized translation: " << t_[0] << ", " << t_[1] << ", " << t_[2] << "\n";

    
    /*
    //Jacobians
    Eigen::MatrixXd jF = jacobian(Tl1_, Tl2_, t_, q_);

    std::cout << "\n \n \n" << "Jacobian computed with FORMULAS:\n"
            << jF << "\n \n \n" <<std::endl;


    
    //COMPUTED INSIDE THE COST FUNCTION
    // Access Jacobian
    ceres::CRSMatrix jacobianCeres;
    problem->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobianCeres);
    Eigen::MatrixXd jC2E = CRSMatrixToEigen(jacobianCeres);
    Eigen::MatrixXd jCP = jC2E.transpose()*jC2E;
    
    // Print Jacobian
    std::cout << "\n" << "\n" << "Jacobian computed from CERES (J_t * J): \n"<< jCP << std::endl;
    }
    */
   ceres::CRSMatrix jacobianCeres;
      problem->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobianCeres);
      Eigen::MatrixXd jC2E = CRSMatrixToEigen(jacobianCeres);
      Eigen::MatrixXd jCP = jC2E.transpose()*jC2E;
      Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(jCP);

       Eigen::VectorXd singularValues = svd.singularValues();
      Eigen::VectorXd newSingularValues = singularValues.head(singularValues.size() - 1);
      std::cout << "The first five singular values of the Ceres Jacobian are:\n" << newSingularValues << std::endl;
    
    Eigen::VectorXd svds = newSingularValues;
    result.svd = svds;
    result.q = q_;
    result.t = t_;
    //result.jacobianSVDCheck = jacobianSVDCheck_;
    return result;
    

}