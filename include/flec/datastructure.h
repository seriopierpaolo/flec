//Header file that contains all the custom data structures for the project

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>



struct Optimization_Result{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;

};

struct TfBundle{

    std_msgs::Header header_F;
    Eigen::Affine3d transformation_F;

    std_msgs::Header header_S;
    Eigen::Affine3d transformation_S;

    

};


class tfAccumulator {
private:
    
    

public:
    std::vector<TfBundle> accumulatedTraj;
    int nSample;
    
    tfAccumulator() {this->nSample = 0;}
    
    //Callback
    void addElement(TfBundle newBundle)
    {
        accumulatedTraj.push_back(newBundle);
    }

};


#endif // DATASTRUCTURE_H