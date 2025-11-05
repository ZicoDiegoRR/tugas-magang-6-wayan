#include "ros/ros.h"
#include "opencv_ros/detection.h"
#include <string>
#include <iostream>

bool describe(
    opencv_ros::detection::Request &req, 
    opencv_ros::detection::Response &res) {
    
    if(!req.shape.empty()) ROS_INFO("Detected %s.", req.shape.c_str());
    if(req.orange_amount > 0) ROS_INFO(
        "Detected %i areas with orange color.", req.orange_amount
    );
    if(req.white_amount > 0) ROS_INFO(
        "Detected %i areas with white color.", req.white_amount
    );

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_describer");
    ros::NodeHandle nh;
    ros::ServiceServer srv = nh.advertiseService("process_image", describe);
    ros::spin();

    return 0;
}