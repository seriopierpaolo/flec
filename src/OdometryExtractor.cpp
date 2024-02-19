#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>


    OdometryExtractor::OdometryExtractor()  
    {
    };

    TfBundle transformation;
    

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

        this->transformation.header_Left = msg->header;
        this->transformation.transformation_Left.translation() = transL;
        this->transformation.transformation_Left.linear()=rotL; 

        //ROS_INFO("Left time is %d", this->transformation.header_Left.stamp.nsec);
        
    }

    void OdometryExtractor::odometryCallbackRight(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Quaterniond qR;
        Eigen::Vector3d transR; 
        std::tie(qR, transR) = extractTransformation(msg);
        Eigen::Matrix3d rotR = qR.toRotationMatrix();

        this->transformation.header_Right = msg->header;
        this->transformation.transformation_Right.translation() = transR;
        this->transformation.transformation_Right.linear()=rotR; 

        //ROS_INFO("Right time is %d", this->transformation.header_Right.stamp.nsec);

    }



