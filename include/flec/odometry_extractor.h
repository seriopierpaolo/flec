#ifndef ODOMETRY_EXTRACTOR_H
#define ODOMETRY_EXTRACTOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <flec/datastructure.h>


/*
The OdometryExtractor class manages the DLO odomtery results.
It stores all the methods for the pipeline that goes from 
the double nav_msgs/Odometry LiDAR messages to the ceres optimization.
*/

class OdometryExtractor
{
    public:
    
    //Constructor
    OdometryExtractor();

    //Container for two different transformations simultaneously
    TfBundle tb;

    //extractTransformation extracts the pose from a nav_msgs/Odometry and it is used in odometryCallbackUnique  
    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    extractTransformation (nav_msgs::OdometryConstPtr msg);

    //callback that just mixes data from two topics
    void odometryCallbackUnique(const nav_msgs::OdometryConstPtr &msg1, const nav_msgs::OdometryConstPtr &msg2);

    //instantiate a ceresOptimization class with all the relevant information
    void performOptimization();


};

#endif // ODOMETRY_EXTRACTOR_H

