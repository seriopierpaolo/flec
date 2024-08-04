#include <flec/pc_odom_manager.h>


    PointCloud_Odometry_Manager::PointCloud_Odometry_Manager(TfAccumulator* buffer)  
    {
        b = buffer;
        
    };

    
    

    std::tuple <Eigen::Quaterniond, Eigen::Vector3d>
    PointCloud_Odometry_Manager::extractTransformation (flec::pcodomConstPtr msg) 
    {
        // Extract rotation and translation components from a nav_msgs::Odometry message
        Eigen::Quaterniond rotation(msg->Odometry.pose.orientation.w,
                                    msg->Odometry.pose.orientation.x,
                                    msg->Odometry.pose.orientation.y,
                                    msg->Odometry.pose.orientation.z);

        Eigen::Vector3d translation(msg->Odometry.pose.position.x,
                                    msg->Odometry.pose.position.y,
                                    msg->Odometry.pose.position.z);

        return {rotation, translation};

        std::cout << "So qua" << std::endl;

    }

        void PointCloud_Odometry_Manager::pair_Callback(const flec::pcodomConstPtr &msg1, const flec::pcodomConstPtr &msg2)
    {
        //F = First Lidar - S = Second Lidar

        //First Lidar        
        Eigen::Quaterniond qF;
        Eigen::Vector3d transF; 
        std::tie(qF, transF) = extractTransformation(msg1);
        Eigen::Matrix3d rotF = qF.toRotationMatrix();
        //std::cout << this->b->pair.header_F  << std::endl;
        //std::cout << msg1->header  << std::endl;
        this->b->pair.header_F = msg1->header;

        this->b->pair.transformation_F.translation() = transF;
        this->b->pair.transformation_F.linear()=rotF; 

        //Second Lidar
        Eigen::Quaterniond qS;
        Eigen::Vector3d transS; 
        std::tie(qS, transS) = extractTransformation(msg2);
        Eigen::Matrix3d rotS = qS.toRotationMatrix();
        this->b->pair.header_S = msg2->header;
        this->b->pair.transformation_S.translation() = transS;
        this->b->pair.transformation_S.linear()=rotS; 

        //ROS_INFO("Left time is %d", this->transformation.header_Left.stamp.nsec);
        //this->performOptimization();
        
        //Update the buffer with the current pair (and eventually move to another batch)
        
        this->b->updateBatch();
        //this->b->updateBuffer(*tb);

        
    }




