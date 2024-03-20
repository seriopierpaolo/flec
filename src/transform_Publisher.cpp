#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <flec/datastructure.h>


    Results_Publisher::Results_Publisher(){}

    void 
    Results_Publisher::publish
    (const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) 
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


