#include <flec/observability_module/svdPublisher.h>


    svdPublisher::svdPublisher(ros::NodeHandle nh)
        :  nh_("~") {
        svd_pub_ = nh_.advertise<flec::svd>("svd", 10);
        svd_msg_.double1 = 0.0;
        svd_msg_.double2 = 0.0;
        svd_msg_.double3 = 0.0;
        rate_ = ros::Rate(10); // 10Hz publishing rate
    }

    void svdPublisher::updateValues(double i1, double i2, double i3) {
        // Update the values in svd_msg_
        // Here you can implement your logic to update the values
        svd_msg_.double1 = i1;
        svd_msg_.double2 = i2;
        svd_msg_.double3 = i3;
    }

    void svdPublisher::publishData() {
        // Publish the svd_msg_
        svd_pub_.publish(svd_msg_);
    }

    void svdPublisher::run() {
        while (ros::ok()) {
            updateValues();
            publishData();
            rate_.sleep();
        }
    }

