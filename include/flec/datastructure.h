//Header file that contains all the custom data structures for the project

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>


class Results_Publisher {
public:
    Results_Publisher();

    ros::Publisher pub_;

    void publish(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);


private:

    
    //const std::string parent_frame_id_;
    //const std::string child_frame_id_;
};



struct Optimization_Result{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;

    //Optimal_Transformation_Publisher pub;
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