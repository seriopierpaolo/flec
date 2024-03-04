#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>

#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>

#include <flec/ceresOptimization.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_extractor_node");

    ros::NodeHandle nh;
    ros::Subscriber odometry_sub_Left;
    ros::Subscriber odometry_sub_Right;

    std::string odometry_topic_left;
    std::string odometry_topic_right;

    OdometryExtractor odometry_extractor;

    // Parameters
    nh.param<std::string>("odometry_topic/left", odometry_topic_left, "/robot/dlo/odom_left");
    nh.param<std::string>("odometry_topic/right", odometry_topic_right, "/robot/dlo/odom_right");

    // Subscriber
    odometry_sub_Left = nh.subscribe(odometry_topic_left, 1, &OdometryExtractor::odometryCallbackLeft,&odometry_extractor);
    odometry_sub_Right = nh.subscribe(odometry_topic_right, 1, &OdometryExtractor::odometryCallbackRight,&odometry_extractor);

    // Input hand-eye transformations
    Eigen::Matrix4d Tg1 = odometry_extractor.tb.transformation_Left.matrix();
    Eigen::Matrix4d Tg2 = odometry_extractor.tb.transformation_Right.matrix();


    //ceresOptimization solver(Tg1, Tg2);
    //odometry_extractor.performOptimization();
    // Solve the hand-eye calibration problem

    //solver.solve();
    std::cout << "SONO QUI\n";
    //delete &solver;


    ros::spin();

    return 0;
}

