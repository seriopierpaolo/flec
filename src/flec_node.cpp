#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>

#include <flec/pcodom.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <flec/pc_odom_manager.h>
#include <flec/datastructure.h>


#include <flec/ceresOptimization.h>
//#include <flec/observability_module/svdPublisher.h>
//#include <flec/pc_PostProcessing.h>

using namespace message_filters;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flec_node");

    ros::NodeHandle nh;
    //ros::Subscriber odometry_sub_Left;
    //ros::Subscriber pc_sub;

    ros::Publisher optimization;

    std::string odometry_topic_left;
    std::string odometry_topic_right;
    
    TfAccumulator buffer;
    
    PointCloud_Odometry_Manager pco_manager(&buffer);
    
    //Results_Publisher publisher;
    //svdPublisher svd_publisher(nh);
    //Optimal_Transformation_Publisher pub(nh);
    
    //PC_PostProcessing post_proc(nh);

    // Parameters
    nh.param<std::string>("odometry_topic/left", odometry_topic_left, "/robot/dlo/pcodom_left");
    nh.param<std::string>("odometry_topic/right", odometry_topic_right, "/robot/dlo/pcodom_right");
    
    //pc_sub = nh.subscribe("/odom_trajectory_right",10,)

    //ptimization = nh.advertise<geometry_msgs::TransformStamped>("l2_to_l1_transform", 10);
    //odometry_extractor.publisher.pub_ = optimization;



    //Message_filters used here to couple two reading that have been acquired almost at the same time

    // Subscriber
    message_filters::Subscriber<flec::pcodom> odom1_sub(nh, odometry_topic_left, 1);
    message_filters::Subscriber<flec::pcodom> odom2_sub(nh, odometry_topic_right, 1);

    typedef sync_policies::ApproximateTime<flec::pcodom, flec::pcodom> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom1_sub, odom2_sub);
    sync.registerCallback(boost::bind(&PointCloud_Odometry_Manager::pair_Callback, &pco_manager, _1, _2));

    //pub.publish(odometry_extractor.optResult.t,odometry_extractor.optResult.q);
    //post_proc.result = odometry_extractor.optResult;



    ros::spin();

    return 0;
}

