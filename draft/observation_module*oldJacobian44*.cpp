#include <flec/observability_module/observation_module.h>




Eigen::MatrixXd
jacobian(Eigen::Matrix4d t1, Eigen::Matrix4d t2, Eigen::Vector3d t12_, Eigen::Quaterniond q12_)
{
    //Extracting rotation and translation components
    //--------------------------------------------------------------
    Eigen::Quaternion<double> q1 (t1.block<3, 3>(0,0));
    Eigen::Quaternion<double> q2 (t2.block<3, 3>(0,0));
    

    Eigen::Vector3d t1_t = t1.block<3, 1>(0, 3);
    Eigen::Vector3d t2_t = t2.block<3, 1>(0, 3);
    //--------------------------------------------------------------


    double q11 = q1.w(); 
    double q12 = q1.x(); 
    double q13 = q1.y(); 
    double q14 = q1.z(); 

    double q21 = q2.w(); 
    double q22 = q2.x(); 
    double q23 = q2.y(); 
    double q24 = q2.z(); 

    double q121 = q12_.w(); 
    double q122 = q12_.x(); 
    double q123 = q12_.y(); 
    double q124 = q12_.z(); 


    //Translation Part Jacobian

    Eigen::Matrix3d Jt_dt = q1.toRotationMatrix() - Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Jt_dq = -skewSymmetric(q12_.toRotationMatrix()*t2_t);


    //Rotation Part Jacobian
    //----------------------------------------------------------------

    double J11 = q121*(q11*q21 + q12*q22 + q13*q23 + q14*q24) + q122*(q11*q22 - q12*q21 - q13*q24 + q14*q23)
         + q123*(q11*q23 - q13*q21 + q12*q24 - q14*q22) + q124*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
         - q11*(q22*q122 - q21*q121 + q23*q123 + q24*q124) + q12*(q21*q122 + q22*q121 + q23*q124 - q24*q123) 
         + q13*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q14*(q21*q124 + q22*q123 - q23*q122 + q24*q121);

    double J12 = q122*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q123*(q11*q24 - q12*q23 + q13*q22 - q14*q21) - q124*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        + q11*(q21*q122 + q22*q121 + q23*q124 - q24*q123) + q12*(q22*q122 - q21*q121 + q23*q123 + q24*q124) 
        - q13*(q21*q124 + q22*q123 - q23*q122 + q24*q121) + q14*(q21*q123 + q23*q121 - q22*q124 + q24*q122);

    double J13 = q123*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q122*(q11*q24 - q12*q23 + q13*q22 - q14*q21) + q124*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q11*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q12*(q21*q124 + q22*q123 - q23*q122 + q24*q121) 
        + q13*(q22*q122 - q21*q121 + q23*q123 + q24*q124) - q14*(q21*q122 + q22*q121 + q23*q124 - q24*q123);
    
    double J14 = q122*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q121*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        + q124*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q123*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q11*(q21*q124 + q22*q123 - q23*q122 + q24*q121) - q12*(q21*q123 + q23*q121 - q22*q124 + q24*q122) 
        + q13*(q21*q122 + q22*q121 + q23*q124 - q24*q123) + q14*(q22*q122 - q21*q121 + q23*q123 + q24*q124);

    double J21 = q122*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q123*(q11*q24 - q12*q23 + q13*q22 - q14*q21) - q124*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q11*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q12*(q22*q122 - q21*q121 + q23*q123 + q24*q124) 
        + q13*(q21*q124 + q22*q123 - q23*q122 + q24*q121) - q14*(q21*q123 + q23*q121 - q22*q124 + q24*q122);

    double J22 = q12*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q122*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q123*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q124*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        - q11*(q22*q122 - q21*q121 + q23*q123 + q24*q124) - q121*(q11*q21 + q12*q22 + q13*q23 + q14*q24) 
        + q13*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q14*(q21*q124 + q22*q123 - q23*q122 + q24*q121);

    double J23 = q121*(q11*q24 - q12*q23 + q13*q22 - q14*q21) - q122*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q124*(q11*q21 + q12*q22 + q13*q23 + q14*q24) + q123*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q11*(q21*q124 + q22*q123 - q23*q122 + q24*q121) - q12*(q21*q123 + q23*q121 - q22*q124 + q24*q122) 
        + q13*(q21*q122 + q22*q121 + q23*q124 - q24*q123) + q14*(q22*q122 - q21*q121 + q23*q123 + q24*q124);

    double J24 = q123*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q122*(q11*q24 - q12*q23 + q13*q22 - q14*q21) + q124*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q11*(q21*q123 + q23*q121 - q22*q124 + q24*q122) - q12*(q21*q124 + q22*q123 - q23*q122 + q24*q121) 
        - q13*(q22*q122 - q21*q121 + q23*q123 + q24*q124) + q14*(q21*q122 + q22*q121 + q23*q124 - q24*q123);

    double J31 = q123*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q122*(q11*q24 - q12*q23 + q13*q22 - q14*q21) + q124*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q11*(q21*q123 + q23*q121 - q22*q124 + q24*q122) - q12*(q21*q124 + q22*q123 - q23*q122 + q24*q121) 
        - q13*(q22*q122 - q21*q121 + q23*q123 + q24*q124) + q14*(q21*q122 + q22*q121 + q23*q124 - q24*q123);

    double J32 = q122*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q121*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        + q124*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q123*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q11*(q21*q124 + q22*q123 - q23*q122 + q24*q121) + q12*(q21*q123 + q23*q121 - q22*q124 + q24*q122) 
        - q13*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q14*(q22*q122 - q21*q121 + q23*q123 + q24*q124);

    double J33 = q12*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q122*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q123*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q124*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        - q11*(q22*q122 - q21*q121 + q23*q123 + q24*q124) - q121*(q11*q21 + q12*q22 + q13*q23 + q14*q24) 
        + q13*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q14*(q21*q124 + q22*q123 - q23*q122 + q24*q121);

    double J34 = q121*(q11*q22 - q12*q21 - q13*q24 + q14*q23) - q122*(q11*q21 + q12*q22 + q13*q23 + q14*q24) 
        - q123*(q11*q24 - q12*q23 + q13*q22 - q14*q21) + q124*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        + q11*(q21*q122 + q22*q121 + q23*q124 - q24*q123) + q12*(q22*q122 - q21*q121 + q23*q123 + q24*q124) 
        - q13*(q21*q124 + q22*q123 - q23*q122 + q24*q121) + q14*(q21*q123 + q23*q121 - q22*q124 + q24*q122);

    double J41 = q122*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q121*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        + q124*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q123*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q11*(q21*q124 + q22*q123 - q23*q122 + q24*q121) + q12*(q21*q123 + q23*q121 - q22*q124 + q24*q122) 
        - q13*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q14*(q22*q122 - q21*q121 + q23*q123 + q24*q124);

    double J42 = q121*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q123*(q11*q21 + q12*q22 + q13*q23 + q14*q24) 
        + q122*(q11*q24 - q12*q23 + q13*q22 - q14*q21) - q124*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q11*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q12*(q21*q124 + q22*q123 - q23*q122 + q24*q121) 
        + q13*(q22*q122 - q21*q121 + q23*q123 + q24*q124) - q14*(q21*q122 + q22*q121 + q23*q124 - q24*q123);

    double J43 = q122*(q11*q21 + q12*q22 + q13*q23 + q14*q24) - q121*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        + q123*(q11*q24 - q12*q23 + q13*q22 - q14*q21) - q124*(q11*q23 - q13*q21 + q12*q24 - q14*q22) 
        - q11*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q12*(q22*q122 - q21*q121 + q23*q123 + q24*q124) 
        + q13*(q21*q124 + q22*q123 - q23*q122 + q24*q121) - q14*(q21*q123 + q23*q121 - q22*q124 + q24*q122);

    double J44 = q12*(q21*q122 + q22*q121 + q23*q124 - q24*q123) - q122*(q11*q22 - q12*q21 - q13*q24 + q14*q23) 
        - q123*(q11*q23 - q13*q21 + q12*q24 - q14*q22) - q124*(q11*q24 - q12*q23 + q13*q22 - q14*q21) 
        - q11*(q22*q122 - q21*q121 + q23*q123 + q24*q124) - q121*(q11*q21 + q12*q22 + q13*q23 + q14*q24) 
        + q13*(q21*q123 + q23*q121 - q22*q124 + q24*q122) + q14*(q21*q124 + q22*q123 - q23*q122 + q24*q121);



    
     

    
    //Eigen::Matrix4d Jr_dr = (Istar * q2.conjugate() * q1 * q12).toRotationMatrix() + (q12.conjugate() * q2.conjugate() * q1).toRotationMatrix();
        
Eigen::Matrix4d Jq_dq { {J11, J12, J13, J14},
                        {J21, J22, J23, J24},
                        {J31, J32, J33, J34},
                        {J41, J42, J43, J44}
};


Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svdt(Jt_dt);
std::cout << "\n \n \n" << std::endl;
std::cout << "Singular values of the translation part:\n" << svdt.singularValues() << "\n" <<std::endl;

Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svdq(Jq_dq);
std::cout << "Singular values of the rotational part:\n" << svdq.singularValues() << std::endl;

//Eigen::Matrix3d mat = ;
return Jq_dq;
}
