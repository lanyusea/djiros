#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dji_sdk_read_cam/dji_sdk_read_cam_node.h>
#include <nodelet/nodelet.h>

namespace dji_sdk {

class DJISDKReadCamNodelet : public nodelet::Nodelet
{
public:
  DJISDKReadCamNodelet() {}

  ~DJISDKReadCamNodelet() {
    if (node) delete node;
  }

private:
  void onInit() {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();
    node = new DJISDKReadCamNode(nh, nh_private);
  }

  DJISDKReadCamNode* node;
};

}

PLUGINLIB_EXPORT_CLASS(dji_sdk::DJISDKReadCamNodelet, nodelet::Nodelet)
