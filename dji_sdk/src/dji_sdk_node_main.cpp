#include "dji_sdk/dji_sdk_node.h"

#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dji_sdk_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //new an object of adapter

    dji_sdk::DJISDKNode* dji_sdk_node = new dji_sdk::DJISDKNode(nh, nh_private);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //clear
    delete dji_sdk_node;

    return 0;
}
