#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>

#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>

#include <flec/ceresOptimization.h>

/*
    void 
    publish
    (const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation, ros::Publisher pub_) 
    {
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "Lidar1";
        msg.child_frame_id = "Lidar2";
        msg.transform.translation.x = translation.x();
        msg.transform.translation.y = translation.y();
        msg.transform.translation.z = translation.z();
        msg.transform.rotation.x = rotation.x();
        msg.transform.rotation.y = rotation.y();
        msg.transform.rotation.z = rotation.z();
        msg.transform.rotation.w = rotation.w();
        pub_.publish(msg);
    }
*/


    OdometryExtractor::OdometryExtractor()  
    {
    };

    TfBundle tb;
    tfAccumulator tfBuffer;

    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    OdometryExtractor::extractTransformation (nav_msgs::OdometryConstPtr msg) 
    {
        // Extract rotation and translation components from a nav_msgs::Odometry message
        Eigen::Quaterniond rotation(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);

        Eigen::Vector3d translation(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);

        return {rotation, translation};

    }

        void OdometryExtractor::odometryCallbackUnique(const nav_msgs::OdometryConstPtr &msg1, const nav_msgs::OdometryConstPtr &msg2)
    {
        //F = First Lidar - S = Second Lidar

        //First Lidar        
        Eigen::Quaterniond qF;
        Eigen::Vector3d transF; 
        std::tie(qF, transF) = extractTransformation(msg1);
        Eigen::Matrix3d rotF = qF.toRotationMatrix();
        this->tb.header_F = msg1->header;
        this->tb.transformation_F.translation() = transF;
        this->tb.transformation_F.linear()=rotF; 

        //Second Lidar
        Eigen::Quaterniond qS;
        Eigen::Vector3d transS; 
        std::tie(qS, transS) = extractTransformation(msg2);
        Eigen::Matrix3d rotS = qS.toRotationMatrix();
        this->tb.header_S = msg2->header;
        this->tb.transformation_S.translation() = transS;
        this->tb.transformation_S.linear()=rotS; 

        //ROS_INFO("Left time is %d", this->transformation.header_Left.stamp.nsec);
        this->performOptimization();
        
    }

    void OdometryExtractor::performOptimization() {

        

        tfBuffer.addElement(tb);
        ceresOptimization solver(tfBuffer);


        // Solve the hand-eye calibration problem
        this->optResult = solver.solve();

        publisher.publish(this->optResult.t, this->optResult.q);

    }


