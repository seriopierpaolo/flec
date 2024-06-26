//Header file that contains all the custom data structures for the project

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>


struct Optimization_Result{
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::VectorXd svd;
    //bool jacobianSVDCheck;

    //Optimal_Transformation_Publisher pub;
};

struct TfPair{

    std_msgs::Header header_F;
    Eigen::Affine3d transformation_F;

    std_msgs::Header header_S;
    Eigen::Affine3d transformation_S;

};


class TfBatch {
private:
    

public:

    std::vector<TfPair> currentBatch;

    int nSample;
    double batch_start_time;
    
    TfBatch() {this->nSample = 0;}

};





class TfAccumulator {
    private:
    
    

public:
    TfPair pair;
    TfBatch batch;
    std::vector<TfBatch> segmentBuffer;
    
    void updateBatch();

    void updateBuffer();
};



#endif // DATASTRUCTURE_H