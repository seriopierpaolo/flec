#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <flec/datastructure.h>

class PC_PostProcessing
{
//Bunch of elements fos post optimization phase

    public:

    Optimization_Result result;
    //pcl::PointCloud<pcl::PointXYZ> *read_pc;

    PC_PostProcessing(ros::NodeHandle nh) 
    {
        ros::Subscriber sub;
        nh.param<std::string>("read_pc_topic", read_pc_topic, "/odom_trajectory_left");
        sub = nh.subscribe(read_pc_topic, 1, &PC_PostProcessing::rotate_cloud, this);


        
        nh.param<std::string>("output_pcd_topic", output_pcd_topic, "/rotated_pc");

        // Publisher
        output_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(output_pcd_topic, 1);


    }

    void rotate_cloud(const sensor_msgs::PointCloud2ConstPtr &msg)
    {   
        sensor_msgs::PointCloud2 *msgtbp;
        Eigen::Affine3f transform;
        pcl::fromROSMsg(*msg,*rotated_cloud);
        //Eigen::Matrix3f rot = this->result.q.toRotationMatrix();
        transform.translation() << this->result.t;
        transform.rotate(this->result.q);

        pcl::transformPointCloud (*rotated_cloud, *rotated_cloud, transform);

        msgtbp->header = msg->header;
        pcl::toROSMsg(*rotated_cloud,*msgtbp);

        output_pcd_pub.publish(*msgtbp);

    }

    private:
    
    pcl::PointCloud<pcl::PointXYZ> *rotated_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    ros::Publisher output_pcd_pub;
    std::string output_pcd_topic;
    std::string read_pc_topic;

};