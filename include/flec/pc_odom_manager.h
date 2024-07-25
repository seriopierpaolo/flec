#ifndef ODOMETRY_EXTRACTOR_H
#define ODOMETRY_EXTRACTOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <flec/datastructure.h>


#include <std_msgs/Header.h>
#include <flec/pcodom.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <ceres/ceres.h>

#include <flec/ceresOptimization.h>

/*
The OdometryExtractor class manages the DLO odomtery results.
It stores all the methods for the pipeline that goes from 
the double nav_msgs/Odometry LiDAR messages to the ceres optimization.
*/

class PointCloud_Odometry_Manager
{
    public:
    
    //Constructor
    PointCloud_Odometry_Manager(TfAccumulator* buffer);

    //Pointer to the segment buffer and its subsets
    TfAccumulator* b;
    //TfPair* tp = &b->pair;
    //TfBatch* tb = &b->batch;

    //Container for optimization results
    Optimization_Result optResult;

    //Class to publish all the needed results
    //Results_Publisher publisher;

    //extractTransformation extracts the pose from a nav_msgs/Odometry and it is used in odometryCallbackUnique  
    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    extractTransformation (flec::pcodomConstPtr msg);

    //callback that just mixes data from two topics
    void pair_Callback(const flec::pcodomConstPtr &msg1, const flec::pcodomConstPtr &msg2);

    //instantiate a ceresOptimization class with all the relevant information
    //void performOptimization();

    
    

    private:
    
    


};

#endif // ODOMETRY_EXTRACTOR_H

