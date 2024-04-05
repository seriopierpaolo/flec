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

    Eigen::Quaternion<T> ql1 = q1_.template cast<T>();
    Eigen::Quaternion<T> ql2 = q2_.template cast<T>();

    Eigen::Matrix<T, 3, 1> tl1 = t1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> tl2 = t2_.template cast<T>();


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













//3 RESIDUALS!
/****************************************************************************************
Cost Function for hand-eye calibration problem as defined in 
Versatile Multi-Lidar Calibration - Second Formula
TBM SO3Log 
****************************************************************************************/

template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{
    Eigen::Map<const Eigen::Quaternion<T>> q_l12(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_l12(t);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);

    Eigen::Quaternion<T> q1 = q1_.template cast<T>();
    Eigen::Quaternion<T> q2 = q2_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t1 = t1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t2 = t2_.template cast<T>();

    //MATRIX DEFINITION
    /****************************************************/
    
    //Tl12
    Eigen::Matrix<T, 4, 4> T12 = Eigen::Matrix4f::Identity();
    T12.template block<3,3>(0,0) = q_l12.toRotationMatrix();
    T12.template block<3,1>(0,3) = t_l12;

    //T1
    Eigen::Matrix<T, 4, 4> T1 = Eigen::Matrix4f::Identity();
    T1.template block<3,3>(0,0) = q1.toRotationMatrix();
    T1.template block<3,1>(0,3) = t1;

    //T2
    Eigen::Matrix<T, 4, 4> T2 = Eigen::Matrix4f::Identity();
    T2.template block<3,3>(0,0) = q2.toRotationMatrix();
    T2.template block<3,1>(0,3) = t2;

    //RESIDUAL MATRIX
    Eigen::Matrix<T, 4, 4> res_mat = T2.inverse() * T12.inverse() * T1 * T12;

    Eigen::Matrix<T, 3, 3> res_mat3 = res_mat.template block<3,3>(0,0);
    Eigen::Matrix<T, 3, 1> res_vec = SO3Log(res_mat3);
    
    //HO TOLTO UN PEZZO DI QUATERNIONE
    res = res_vec;
    //res.template block<3,1>(0,0) <<  rot_res.x(), rot_res.y(), rot_res.z();
    //res.template block<3,1>(3,0) = transl_res.template cast<T>();

    //std::cout<<"Residual "<<residuals<<std::endl;

    return true;
}




//7 RESIDUALS!
/****************************************************************************************
Simple Cost Function for hand-eye calibration problem (AX = XB)

****************************************************************************************/

template <typename T>
bool ceresOptimization::CostFunction::operator()(const T* const q, const T* const t, T* residuals) const 
{
    Eigen::Map<const Eigen::Quaternion<T>> q12_(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t12_(t);

    //Eigen::Map<Eigen::Matrix<T, 7, 1>> res(residuals);

    Eigen::Quaternion<T> q1 = q1_.template cast<T>();
    Eigen::Quaternion<T> q2 = q2_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t1 = t1_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t2 = t2_.template cast<T>();

    //MATRIX DEFINITION
    /****************************************************/
    
    //Tl12
    Eigen::Matrix<T, 4, 4> T12;
    T12.template block<3,3>(0,0) = q12_.toRotationMatrix();
    T12.template block<3,1>(0,3) = t12_;
    
    //T1
    Eigen::Matrix<T, 4, 4> T1;
    T1.template block<3,3>(0,0) = q1.toRotationMatrix();
    T1.template block<3,1>(0,3) = t1;

    //T2
    Eigen::Matrix<T, 4, 4> T2;
    T2.template block<3,3>(0,0) = q2.toRotationMatrix();
    T2.template block<3,1>(0,3) = t2;

    //RESIDUAL MATRIX
    Eigen::Matrix<T, 4, 4> res_mat = (T1 * T12) + (T12 * T2);

    Eigen::Quaternion<T> res_quat(res_mat.template block<3,3>(0,0));
    Eigen::Matrix<T, 3, 1> res_transl = res_mat.template block<3,1>(0,3);


    Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals_rotation(residuals);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_translation(residuals + 4);
    

    residuals_translation = res_transl;
    residuals_rotation  <<  res_quat.w(), res_quat.x(), res_quat.y(), res_quat.z();
   

    return true;
}
