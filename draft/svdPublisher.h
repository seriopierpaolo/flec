#include "ros/ros.h"
//#include "flec/svd.h" // Include the message header file

class svdPublisher {
public:
    svdPublisher(ros::NodeHandle nh);
        
    void updateValues(double i1, double i2, double i3);

    void publishData();

    void run();
    
private:
    ros::NodeHandle nh_;
    ros::Publisher svd_pub_;
    flec::svd svd_msg_;
    ros::Rate rate_;
};

