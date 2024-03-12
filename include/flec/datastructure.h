//Header file that contains all the datastructures for the project

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <flec/odometry_extractor.h>



struct TfBundle{

    std_msgs::Header header_Left;
    Eigen::Affine3d transformation_Left;

    std_msgs::Header header_Right;
    Eigen::Affine3d transformation_Right;

    struct TfBundle *prev;

};

class tfAccumulator {
private:
    std::vector<TfBundle> accumulatedTraj;

    tfAccumulator() {this.nSample = 0;}

public:
    //Callback
    void addElement(TfBundle newBundle)
    {
        accumulatedTraj.push_back(newBundle);
    }



}


#endif // DATASTRUCTURE_H