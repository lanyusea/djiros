#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dji_sdk/dji_sdk_node.h>
#include <nodelet/nodelet.h>

namespace dji_sdk {

class DJISDKNodelet : public nodelet::Nodelet
{
public:
  DJISDKNodelet() {}

  ~DJISDKNodelet() {
    if (node) delete node;
  }

private:
  void onInit() {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();
    node = new DJISDKNode(nh, nh_private);
  }

  DJISDKNode* node;
};

}

PLUGINLIB_EXPORT_CLASS(dji_sdk::DJISDKNodelet, nodelet::Nodelet)
