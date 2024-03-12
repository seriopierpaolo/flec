#ifndef ODOMETRY_EXTRACTOR_H
#define ODOMETRY_EXTRACTOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <flec/datastructure.h>


class OdometryExtractor
{
    public:
    
    OdometryExtractor();

    TfBundle tb;

    
    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    extractTransformation (nav_msgs::OdometryConstPtr msg);

    void odometryCallbackLeft(const nav_msgs::OdometryConstPtr &msg);

    void odometryCallbackRight(const nav_msgs::OdometryConstPtr &msg);


    void performOptimization();


};

#endif // ODOMETRY_EXTRACTOR_H

