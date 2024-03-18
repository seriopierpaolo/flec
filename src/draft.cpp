//DRAFT SCRIPT FOR COST FUNCTIONS





/****************************************************************************************
Cost Function from formulas 3
Versatile Multi-LiDAR Accurate Self-Calibration System Based on Pose Graph Optimization
****************************************************************************************/
template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{
    Eigen::Map<const Eigen::Quaternion<T>> q_l12(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_l12(t);

    Eigen::Matrix<T, 3, 3> rotation_matrix1 = (Tl1_.template cast<T>()).template topLeftCorner<3, 3>();
    Eigen::Matrix<T, 3, 3> rotation_matrix2 = (Tl2_.template cast<T>()).template topLeftCorner<3, 3>();

    Eigen::Quaternion<T> ql1(rotation_matrix1);
    Eigen::Quaternion<T> ql2(rotation_matrix2);

    Eigen::Matrix<T, 3, 1> tl1 = (Tl1_.block<3, 1>(0, 3)).template cast<T>();
    Eigen::Matrix<T, 3, 1> tl2 = (Tl2_.block<3, 1>(0, 3)).template cast<T>();


    /*******************************************************/   
    /*ROTATIONAL RESIDUAL*/

    //R_L12*RL1
    Eigen::Quaternion<T> q_l12_1 = ql1 * q_l12;
    //R_L12*RL2
    Eigen::Quaternion<T> q_l12_2 = q_l12 * ql2;


    Eigen::Quaternion<T> q_l12_1_inverse = q_l12_1.conjugate();
    //Residual for Rotational Part
    Eigen::Quaternion<T> q_l12_estimated = q_l12_1_inverse * q_l12_2;

    

    /*******************************************************/
    /*TRANSLATIONAL RESIDUAL*/
    Eigen::Matrix<T,3,1> p1 = (ql1.toRotationMatrix() * t_l12) + tl1;
    Eigen::Matrix<T,3,1> p2 = (q_l12.toRotationMatrix() * tl2) + t_l12;


    Eigen::Matrix<T,3,1> t_l12_estimated = p2 - p1;

    /*******************************************************/
    /*RESIDUAL VECTOR*/
    
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_translation(residuals);
    Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals_rotation(residuals + 3);

    residuals_translation = p2 - p1;

    // Convert quaternion to Eigen vector before assigning
    Eigen::Quaternion<T> q_l12_estimated_cast = q_l12_estimated.template cast<T>();
    residuals_rotation << q_l12_estimated_cast.x(), q_l12_estimated_cast.y(), q_l12_estimated_cast.z(), q_l12_estimated_cast.w();

    /*******************************************************/

    std::cout<<"Residual Translation\n"<<residuals_translation<<std::endl;
    
    std::cout<<"Residual "<<residuals<<std::endl;
    return true;

}









/****************************************************************************************
Cost Function from formula used in PoseGraph3D problem listed in ceres example
Versatile Multi-LiDAR Accurate Self-Calibration System Based on Pose Graph Optimization
****************************************************************************************/

template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{

    Eigen::Map<const Eigen::Quaternion<T>> q_l12(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_l12(t);


    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residuals);

    Eigen::Quaternion<T> q1 = q1_.template cast<T>();
    Eigen::Quaternion<T> q2 = q2_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t1 = t1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t2 = t2_.template cast<T>();

    Eigen::Quaternion<T> rotation_error = q2.conjugate()*q_l12.conjugate()*q1*q2;

    Eigen::Matrix<T,3,1> p1 = (q1 * t_l12) + t1;
    Eigen::Matrix<T,3,1> p2 = (q_l12 * t2) + t_l12;
    Eigen::Matrix<T, 3, 1> traslation_error = p2 - p1;

    res.template block<3,1>(0,0) = 2.0*rotation_error.vec();
    res.template block<3,1>(3,0) = traslation_error;    

    //std::cout<<"Residual "<<residuals<<std::endl;

    return true;
}








/****************************************************************************************
Cost Function for hand-eye calibration problem as defined in 
Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods
****************************************************************************************/

template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{

    Eigen::Map<const Eigen::Quaternion<T>> q_l12(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_l12(t);

    Eigen::Map<Eigen::Matrix<T, 7, 1>> res(residuals);

    Eigen::Quaternion<T> q1 = q1_.template cast<T>();
    Eigen::Quaternion<T> q2 = q2_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t1 = t1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t2 = t2_.template cast<T>();

    //ROTATIONAL PART
    /****************************************************/
    Eigen::Matrix<T, 3, 3> r1 = q1.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> r2 = q2.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> r12 = q_l12.toRotationMatrix();

    Eigen::Quaternion<T> rot_res (r1*r12 - r12*r2);
    /****************************************************/

    //TRANSLATIONAL PART
    /****************************************************/
    Eigen::Matrix<T, 3, 1> transl_res = r1*t_l12 + t1 - r12*t2 - t_l12;

    /****************************************************/


    /*

    Eigen::Quaternion<T> rotation_error = q2.conjugate()*q_l12.conjugate()*q1*q2;

    Eigen::Matrix<T,3,1> p1 = (q1 * t_l12) + t1;
    Eigen::Matrix<T,3,1> p2 = (q_l12 * t2) + t_l12;
    Eigen::Matrix<T, 3, 1> traslation_error = p2 - p1;

    res.template block<3,1>(0,0) = 2.0*rotation_error.vec();
    res.template block<3,1>(3,0) = traslation_error;   
    */ 
    res.template block<4,1>(0,0) << rot_res.w(), rot_res.x(), rot_res.y(), rot_res.z();
    res.template block<3,1>(4,0) = transl_res.template cast<T>();

    //std::cout<<"Residual "<<residuals<<std::endl;

    return true;
}