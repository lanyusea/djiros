#include "dji_sdk_read_cam/dji_sdk_read_cam_node.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dji_sdk_read_cam_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //new an object of adapter

    DJISDKReadCamNode* dji_sdk_read_cam_node = new DJISDKReadCamNode(nh, nh_private);

    ros::spin();

    //clear
    delete dji_sdk_read_cam_node;

    return 0;
}
