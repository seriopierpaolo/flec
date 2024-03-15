#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <flec/odometry_extractor.h>
#include <flec/datastructure.h>

#include <flec/ceresOptimization.h>

using namespace message_filters;

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
    message_filters::Subscriber<nav_msgs::Odometry> odom1_sub(nh, odometry_topic_left, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom2_sub(nh, odometry_topic_right, 1);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom1_sub, odom2_sub);
    sync.registerCallback(boost::bind(&OdometryExtractor::odometryCallbackUnique, &odometry_extractor, _1, _2));




    ros::spin();

    return 0;
}

