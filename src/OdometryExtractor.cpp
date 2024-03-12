#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>

#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>

#include <flec/ceresOptimization.h>


    OdometryExtractor::OdometryExtractor()  
    {
    };

    TfBundle tb;
    tfAccumulator tfBuffer;

    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    OdometryExtractor::extractTransformation (nav_msgs::OdometryConstPtr msg) 
    {
        // Extract rotation and translation components
        Eigen::Quaterniond rotation(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);

        Eigen::Vector3d translation(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);

        return {rotation, translation};

    }

    void OdometryExtractor::odometryCallbackLeft(const nav_msgs::OdometryConstPtr &msg)
    {
        
        Eigen::Quaterniond qL;
        Eigen::Vector3d transL; 
        std::tie(qL, transL) = extractTransformation(msg);
        Eigen::Matrix3d rotL = qL.toRotationMatrix();

        this->tb.header_Left = msg->header;
        this->tb.transformation_Left.translation() = transL;
        this->tb.transformation_Left.linear()=rotL; 

        //ROS_INFO("Left time is %d", this->transformation.header_Left.stamp.nsec);
        this->performOptimization();
        
    }

    void OdometryExtractor::odometryCallbackRight(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Quaterniond qR;
        Eigen::Vector3d transR; 
        std::tie(qR, transR) = extractTransformation(msg);
        Eigen::Matrix3d rotR = qR.toRotationMatrix();

        this->tb.header_Right = msg->header;
        this->tb.transformation_Right.translation() = transR;
        this->tb.transformation_Right.linear()=rotR; 

        //ROS_INFO("Right time is %d", this->transformation.header_Right.stamp.nsec);
        this->performOptimization();
    }



    void OdometryExtractor::performOptimization() {


        Eigen::Matrix4d Tg1 = this->tb.transformation_Left.matrix();
        Eigen::Matrix4d Tg2 = this->tb.transformation_Right.matrix();

        tfBuffer.addElement(tb);

        ceresOptimization solver(*tfBuffer);

        // Update the solver with the new transformations
        //solver.updateTransformations(Tg1, Tg2);

        // Solve the hand-eye calibration problem
        solver.solve();

        // Optional: Output or use the calibrated transformations
        //Eigen::Matrix4d calibrated_Tg1 = solver.getTransform1();
        //Eigen::Matrix4d calibrated_Tg2 = solver.getTransform2();
    }
