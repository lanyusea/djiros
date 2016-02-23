#include <dji_sdk/dji_sdk_node.h>
#include <functional>
#include <cstdio>
#include <algorithm>

#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

//
// Process actions
// 

bool DJISDKNode::process_waypoint(dji_sdk::Waypoint new_waypoint) 
{
    double dst_latitude = new_waypoint.latitude;
    double dst_longitude = new_waypoint.longitude;
    float dst_altitude = new_waypoint.altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = global_position.altitude;

    double dis_x, dis_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    double det_x,det_y;
    float det_z;


    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.z = dst_altitude;
    flight_ctrl_data.yaw = new_waypoint.heading;


    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress <100) {
        if(waypoint_navigation_action_server->isPreemptRequested()) {
            return false;
        }

        double d_lon = dst_longitude - global_position.longitude;
        double d_lat = dst_latitude - global_position.latitude;

        flight_ctrl_data.x = ((d_lat) *C_PI/180) * C_EARTH;
        flight_ctrl_data.y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);
        rosAdapter->flight->setFlight(&flight_ctrl_data);

        det_x = (100 * (dst_latitude - global_position.latitude))/dis_x;
        det_y = (100 * (dst_longitude - global_position.longitude))/dis_y;
        det_z = (100 * (dst_altitude - global_position.altitude))/dis_z;

        latitude_progress = 100 - std::abs((int) det_x);
        longitude_progress = 100 - std::abs((int) det_y);
        altitude_progress = 100 - std::abs((int) det_z);

        //lazy evaluation
        //need to find a better way
        if (std::abs(dst_latitude - global_position.latitude) < 0.00001) latitude_progress = 100;
        if (std::abs(dst_longitude - global_position.longitude) < 0.00001) longitude_progress = 100;
        if (std::abs(dst_altitude - global_position.altitude) < 0.12) altitude_progress = 100;

        waypoint_navigation_feedback.latitude_progress = latitude_progress;
        waypoint_navigation_feedback.longitude_progress = longitude_progress;
        waypoint_navigation_feedback.altitude_progress = altitude_progress;
        waypoint_navigation_action_server->publishFeedback(waypoint_navigation_feedback);

        usleep(20000);

    }
    ros::Duration(new_waypoint.staytime).sleep();
    return true;
}


bool DJISDKNode::drone_task_action_callback(const dji_sdk::DroneTaskGoalConstPtr& goal)
{
  uint8_t request_action = goal->task;

  if (request_action == 1)
  {
    //takeoff
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF);
  }
  else if (request_action == 2)
  {
    //landing
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING);
  }
  else if (request_action == 3)
  {
    //gohome
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_GOHOME);
  }

  drone_task_feedback.progress = 1;
  drone_task_action_server->publishFeedback(drone_task_feedback);
  drone_task_action_server->setSucceeded();
  
  return true;
}


bool DJISDKNode::local_position_navigation_action_callback(const dji_sdk::LocalPositionNavigationGoalConstPtr& goal)
{
  /*IMPORTANT*/
  /*
     There has been declared a pointer `local_navigation_action` as the function parameter,
     However, it is the `local_navigation_action_server` that we should use.
     If `local_navigation_action` is used instead, there will be a runtime sengmentation fault.

     so interesting
  */

  float dst_x = goal->x;
  float dst_y = goal->y;
  float dst_z = goal->z;

  float org_x = local_position.x;
  float org_y = local_position.y;
  float org_z = local_position.z;

  float dis_x = dst_x - org_x;
  float dis_y = dst_y - org_y;
  float dis_z = dst_z - org_z; 

  float det_x, det_y, det_z;

  DJI::onboardSDK::FlightData flight_ctrl_data;
  flight_ctrl_data.flag = 0x90;
  flight_ctrl_data.z = dst_z;
  flight_ctrl_data.yaw = 0;

  int x_progress = 0; 
  int y_progress = 0; 
  int z_progress = 0; 
  while (x_progress < 100 || y_progress < 100 || z_progress <100) {

     flight_ctrl_data.x = dst_x - local_position.x;
     flight_ctrl_data.y = dst_y - local_position.y;
     rosAdapter->flight->setFlight(&flight_ctrl_data);

     det_x = (100 * (dst_x - local_position.x)) / dis_x;
     det_y = (100 * (dst_y - local_position.y)) / dis_y;
     det_z = (100 * (dst_z - local_position.z)) / dis_z;

     x_progress = 100 - (int)det_x;
     y_progress = 100 - (int)det_y;
     z_progress = 100 - (int)det_z;

     //lazy evaluation
     if (std::abs(dst_x - local_position.x) < 0.1) x_progress = 100;
     if (std::abs(dst_y - local_position.y) < 0.1) y_progress = 100;
     if (std::abs(dst_z - local_position.z) < 0.1) z_progress = 100;

     local_position_navigation_feedback.x_progress = x_progress;
     local_position_navigation_feedback.y_progress = y_progress;
     local_position_navigation_feedback.z_progress = z_progress;
     local_position_navigation_action_server->publishFeedback(local_position_navigation_feedback);

     usleep(20000);
  }

  local_position_navigation_result.result = true;
  local_position_navigation_action_server->setSucceeded(local_position_navigation_result);

  return true;
}


bool DJISDKNode::global_position_navigation_action_callback(const dji_sdk::GlobalPositionNavigationGoalConstPtr& goal)
{
    double dst_latitude = goal->latitude;
    double dst_longitude = goal->longitude;
    float dst_altitude = goal->altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = global_position.altitude;

    double dis_x, dis_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    double det_x, det_y;
    float det_z;

    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.z = dst_altitude;
    flight_ctrl_data.yaw = 0;


    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress < 100) {

        double d_lon = dst_longitude - global_position.longitude;
        double d_lat = dst_latitude - global_position.latitude;

        flight_ctrl_data.x = ((d_lat) *C_PI/180) * C_EARTH;
        flight_ctrl_data.y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);
        rosAdapter->flight->setFlight(&flight_ctrl_data);

        det_x = (100* (dst_latitude - global_position.latitude))/dis_x;
        det_y = (100* (dst_longitude - global_position.longitude))/dis_y;
        det_z = (100* (dst_altitude - global_position.altitude))/dis_z;


        latitude_progress = 100 - (int)det_x;
        longitude_progress = 100 - (int)det_y;
        altitude_progress = 100 - (int)det_z;

        //lazy evaluation
        if (std::abs(dst_latitude - global_position.latitude) < 0.00001) latitude_progress = 100;
        if (std::abs(dst_longitude - global_position.longitude) < 0.00001) longitude_progress = 100;
        if (std::abs(dst_altitude - global_position.altitude) < 0.12) altitude_progress = 100;


        global_position_navigation_feedback.latitude_progress = latitude_progress;
        global_position_navigation_feedback.longitude_progress = longitude_progress;
        global_position_navigation_feedback.altitude_progress = altitude_progress;
        global_position_navigation_action_server->publishFeedback(global_position_navigation_feedback);

        usleep(20000);

    }

    global_position_navigation_result.result = true;
    global_position_navigation_action_server->setSucceeded(global_position_navigation_result);

    return true;
}


bool DJISDKNode::waypoint_navigation_action_callback(const dji_sdk::WaypointNavigationGoalConstPtr& goal)
{
    dji_sdk::WaypointList new_waypoint_list;
    new_waypoint_list = goal->waypoint_list;

    bool isSucceeded;
    for (int i = 0; i < new_waypoint_list.waypoint_list.size(); i++) {
        const dji_sdk::Waypoint new_waypoint = new_waypoint_list.waypoint_list[i];
        waypoint_navigation_feedback.index_progress = i;
        isSucceeded = process_waypoint(new_waypoint);
        if(!isSucceeded) {
            waypoint_navigation_result.result = false;
            waypoint_navigation_action_server->setPreempted(waypoint_navigation_result);
            return false;
        }
    }

    waypoint_navigation_result.result = true;
    waypoint_navigation_action_server->setSucceeded(waypoint_navigation_result);

    return true;
}

//
// Process services
// 

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = request.flag;
    flight_ctrl_data.x = request.x;
    flight_ctrl_data.y = request.y;
    flight_ctrl_data.z = request.z;
    flight_ctrl_data.yaw = request.yaw;

    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}


bool DJISDKNode::camera_action_control_callback(dji_sdk::CameraActionControl::Request& request, dji_sdk::CameraActionControl::Response& response)
{
    if (request.camera_action == 0) {
        rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_SHOT);
        response.result = true;
    }
    else if (request.camera_action == 1) {
        rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_START);
        response.result = true;
    }
    else if (request.camera_action == 2) {
        rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_STOP);
        response.result = true;
    }
    else {
        response.result = false;
    }
    return true;
}


bool DJISDKNode::drone_task_control_callback(dji_sdk::DroneTaskControl::Request& request, dji_sdk::DroneTaskControl::Response& response)
{
    if (request.task== 4) {
        //takeoff
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF);
        response.result = true;
    }
    else if (request.task == 6) {
        //landing
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING);
        response.result = true;
    }
    else if (request.task == 1) {
        //gohome
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_GOHOME);
        response.result = true;
    }
    else
        response.result = false;
    return true;
}


bool DJISDKNode::gimbal_angle_control_callback(dji_sdk::GimbalAngleControl::Request& request, dji_sdk::GimbalAngleControl::Response& response) 
{
    DJI::onboardSDK::GimbalAngleData gimbal_angle;
    gimbal_angle.yaw = request.yaw;
    gimbal_angle.roll = request.roll;
    gimbal_angle.pitch = request.pitch;
    gimbal_angle.duration = request.duration;
    gimbal_angle.mode = 0xF0;
    gimbal_angle.mode &= request.absolute_or_incremental ? 0xFF : 0x7F;
    gimbal_angle.mode &= request.yaw_cmd_ignore ? 0xFF : 0xBF;
    gimbal_angle.mode &= request.roll_cmd_ignore ? 0xFF : 0xDF;
    gimbal_angle.mode &= request.pitch_cmd_ignore ? 0xFF : 0xEF;

    rosAdapter->camera->setGimbalAngle(&gimbal_angle);

    response.result = true;
    return true;
}


bool DJISDKNode::gimbal_speed_control_callback(dji_sdk::GimbalSpeedControl::Request& request, dji_sdk::GimbalSpeedControl::Response& response)
{
    DJI::onboardSDK::GimbalSpeedData gimbal_speed;
    gimbal_speed.yaw = request.yaw_rate;
    gimbal_speed.roll = request.roll_rate;
    gimbal_speed.pitch = request.pitch_rate;
    gimbal_speed.reserved = 0x80; //little endian. enable

    rosAdapter->camera->setGimbalSpeed(&gimbal_speed);

    response.result = true;
    return true;
}


bool DJISDKNode::global_position_control_callback(dji_sdk::GlobalPositionControl::Request& request, dji_sdk::GlobalPositionControl::Response& response)
{
    float dst_x;
    float dst_y;
    float dst_z = request.altitude;

    if(global_position_ref_seted == 0)
    {
        printf("Cannot run global position navigation because home position has not been set yet!");
        response.result = false;
        return false;
    }

    gps_convert_ned(dst_x, 
            dst_y,
            request.longitude, request.latitude,
            global_position.longitude, global_position.latitude);

    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.x = dst_x - local_position.x;
    flight_ctrl_data.y = dst_y - local_position.y;
    flight_ctrl_data.z = dst_z;
    flight_ctrl_data.yaw = request.yaw;

    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}


bool DJISDKNode::local_position_control_callback(dji_sdk::LocalPositionControl::Request& request, dji_sdk::LocalPositionControl::Response& response)
{
    float dst_x = request.x;
    float dst_y = request.y;
    float dst_z = request.z;

    if(global_position_ref_seted == 0)
    {
        printf("Cannot run local position navigation because home position has not been set yet!");
        response.result = false;
        return false;
    }

    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.x = dst_x - local_position.x;
    flight_ctrl_data.y = dst_y - local_position.y;
    flight_ctrl_data.z = dst_z;
    flight_ctrl_data.yaw = request.yaw;

    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}


bool DJISDKNode::sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response)
{
    if (request.control_enable == 1) {
        printf("Request Control");
        rosAdapter->coreAPI->setControl(1);
        response.result = true;
    }
    else if (request.control_enable == 0) {
        printf("Release Control");
        rosAdapter->coreAPI->setControl(0);
        response.result = true;
    }
    else
        response.result = false;

    return true;
}


bool DJISDKNode::velocity_control_callback(dji_sdk::VelocityControl::Request& request, dji_sdk::VelocityControl::Response& response)
{
    DJI::onboardSDK::FlightData flight_ctrl_data;
    if (request.frame)
        //world frame 
        flight_ctrl_data.flag = 0x40;
    else
        //body frame
        flight_ctrl_data.flag = 0x42;

    flight_ctrl_data.x = request.vx;
    flight_ctrl_data.y = request.vy;
    flight_ctrl_data.z = request.vz;
    flight_ctrl_data.yaw = request.yawAngle;

    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}


bool DJISDKNode::send_data_to_remote_device_callback(dji_sdk::SendDataToRemoteDevice::Request& request, dji_sdk::SendDataToRemoteDevice::Response& response)
{
    memcpy(transparent_transmission_data, &request.data[0], request.data.size());
    rosAdapter->sendToMobile(transparent_transmission_data, request.data.size());
    response.result = true;

    return true;
}


//
// Process state updates
//

void DJISDKNode::transparent_transmission_callback(uint8_t *buf, size_t len)
{
    dji_sdk::TransparentTransmissionData transparent_transmission_data;
    transparent_transmission_data.data.resize(len);
    memcpy(&transparent_transmission_data.data[0], buf, len);
    data_received_from_remote_device_publisher.publish(transparent_transmission_data);
}

void DJISDKNode::broadcast_callback()
{
    DJI::onboardSDK::BroadcastData bc_data = rosAdapter->getBroadcastData();
    unsigned short msg_flags = bc_data.dataFlag;

    auto current_time = ros::Time::now();

    //update attitude msg
    if ( (msg_flags & HAS_Q) && (msg_flags & HAS_W) ) {
        attitude_quaternion.header.frame_id = "/world";
        attitude_quaternion.header.stamp = current_time;
        attitude_quaternion.q0 = bc_data.q.q0;
        attitude_quaternion.q1 = bc_data.q.q1;
        attitude_quaternion.q2 = bc_data.q.q2;
        attitude_quaternion.q3 = bc_data.q.q3;
        attitude_quaternion.wx = bc_data.w.x;
        attitude_quaternion.wy = bc_data.w.y;
        attitude_quaternion.wz = bc_data.w.z;
        attitude_quaternion.ts = bc_data.timeStamp;
        attitude_quaternion_publisher.publish(attitude_quaternion);
    }

    //update global_position msg
    if (msg_flags & HAS_POS) {
        global_position.header.frame_id = "/world";
        global_position.header.stamp = current_time;
        global_position.ts = bc_data.timeStamp;
        global_position.latitude = bc_data.pos.latitude * 180.0 / C_PI;
        global_position.longitude = bc_data.pos.longitude * 180.0 / C_PI;
        global_position.height = bc_data.pos.height;
        global_position.altitude = bc_data.pos.altitude;
        global_position.health = bc_data.pos.health;
        global_position_publisher.publish(global_position);

        //TODO:
        // FIX BUG about flying at lat = 0
        if (global_position.ts != 0 && global_position_ref_seted == 0 && global_position.latitude != 0 && global_position.health > 3) {
            global_position_ref = global_position;
            global_position_ref_seted = 1;
        }

        //update local_position msg
        local_position.header.frame_id = "/world";
        local_position.header.stamp = current_time;
        gps_convert_ned(
                local_position.x,
                local_position.y,
                global_position.longitude,
                global_position.latitude,
                global_position_ref.longitude,
                global_position_ref.latitude
                );
        local_position.z = global_position.height;
        local_position.ts = global_position.ts;
        local_position_ref = local_position;
        local_position_publisher.publish(local_position);
    }


    //update velocity msg
    if (msg_flags & HAS_V) {
        velocity.header.frame_id = "/world";
        velocity.header.stamp = current_time;
        velocity.ts = bc_data.timeStamp;
        velocity.vx = bc_data.v.x;
        velocity.vy = bc_data.v.y;
        velocity.vz = bc_data.v.z;
        velocity_publisher.publish(velocity);
    }

    //update acceleration msg
    if (msg_flags & HAS_A) {
        acceleration.header.frame_id = "/world";
        acceleration.header.stamp = current_time;
        acceleration.ts = bc_data.timeStamp;
        acceleration.ax = bc_data.a.x;
        acceleration.ay = bc_data.a.y;
        acceleration.az = bc_data.a.z;
        acceleration_publisher.publish(acceleration);
    }

    //update gimbal msg
    if (msg_flags & HAS_GIMBAL) {
        gimbal.header.frame_id = tf::resolve(tf_prefix, "base_link");
        gimbal.header.stamp = current_time;
        gimbal.ts = bc_data.timeStamp;
        gimbal.roll = bc_data.gimbal.roll;
        gimbal.pitch = bc_data.gimbal.pitch;
        gimbal.yaw = bc_data.gimbal.yaw;
        gimbal_publisher.publish(gimbal);

        tf::Quaternion quaternion;
        quaternion.setRPY(gimbal.roll, gimbal.pitch, gimbal.yaw);

        br.sendTransform(tf::StampedTransform(tf::Transform(
            quaternion, 
            tf::Vector3(0, 0, 0)), 
            current_time, tf::resolve(tf_prefix, "base_link"), tf::resolve(tf_prefix, "gimbal")));
    }

    //update odom msg
    if ( (msg_flags & HAS_POS) && (msg_flags & HAS_Q) && (msg_flags & HAS_W) && (msg_flags & HAS_V) ) {
        odometry.header.frame_id = "/world";
        odometry.header.stamp = current_time;
        odometry.pose.pose.position.x = local_position.x;
        odometry.pose.pose.position.y = local_position.y;
        odometry.pose.pose.position.z = local_position.z;
        odometry.pose.pose.orientation.w = attitude_quaternion.q0;
        odometry.pose.pose.orientation.x = attitude_quaternion.q1;
        odometry.pose.pose.orientation.y = attitude_quaternion.q2;
        odometry.pose.pose.orientation.z = attitude_quaternion.q3;
        odometry.twist.twist.angular.x = attitude_quaternion.wx;
        odometry.twist.twist.angular.y = attitude_quaternion.wy;
        odometry.twist.twist.angular.z = attitude_quaternion.wz;
        odometry.twist.twist.linear.x = velocity.vx;
        odometry.twist.twist.linear.y = velocity.vy;
        odometry.twist.twist.linear.z = velocity.vz;
        odometry_publisher.publish(odometry);

        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(odometry.pose.pose.orientation, quaternion);

        br.sendTransform(tf::StampedTransform(tf::Transform(
            quaternion, 
            tf::Vector3(local_position.x, local_position.y, local_position.z)), 
            current_time, "/world", tf::resolve(tf_prefix, "base_link")));
    }

    //update rc_channel msg
    if (msg_flags & HAS_RC) {
        rc_channels.header.frame_id = "/rc";
        rc_channels.header.stamp = current_time;
        rc_channels.ts = bc_data.timeStamp;
        rc_channels.pitch = bc_data.rc.pitch;
        rc_channels.roll = bc_data.rc.roll;
        rc_channels.mode = bc_data.rc.mode;
        rc_channels.gear = bc_data.rc.gear;
        rc_channels.throttle = bc_data.rc.throttle;
        rc_channels.yaw = bc_data.rc.yaw;
        rc_channels_publisher.publish(rc_channels);
    }

    //update compass msg
    if (msg_flags & HAS_MAG) {
        compass.header.frame_id = "/world";
        compass.header.stamp = current_time;
        compass.ts = bc_data.timeStamp;
        compass.x = bc_data.mag.x;
        compass.y = bc_data.mag.y;
        compass.z = bc_data.mag.z;
        compass_publisher.publish(compass);
    }

    //update flight_status 
    if (msg_flags & HAS_STATUS) {
        std_msgs::UInt8 msg;
        flight_status = bc_data.status;
        msg.data = flight_status;
        flight_status_publisher.publish(msg);
    }

    //update battery msg
    if (msg_flags & HAS_BATTERY) {
        power_status.percentage = bc_data.battery;
        power_status_publisher.publish(power_status);
    }

    //update flight control info
    if (msg_flags & HAS_DEVICE) {
        flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.device;
        flight_control_info.serial_req_status = bc_data.ctrlInfo.signature;
        flight_control_info_publisher.publish(flight_control_info);
    }

    std_msgs::UInt8 msg;

    //update obtaincontrol msg
    sdk_permission_opened = bc_data.controlStatus;
    msg.data = bc_data.controlStatus;
    sdk_permission_publisher.publish(msg);

    //update activation msg
    activated = bc_data.activation;
    msg.data = bc_data.activation;
    activation_publisher.publish(msg);
}

//
// Init
//
void DJISDKNode::init_parameters_and_activate(ros::NodeHandle& nh_private)
{
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_version;
    std::string app_bundle_id;
    std::string enc_key;

    tf_prefix = tf::getPrefixParam(nh_private);
    nh_private.param("serial_name", serial_name, std::string("/dev/cu.usbserial-A603T4HK"));
    nh_private.param("baud_rate", baud_rate, 230400);
    nh_private.param("app_id", app_id, 1022384);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
    nh_private.param("enc_key", enc_key,
            std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));

    // activation
    user_act_data.ID = app_id;
    user_act_data.version = DJI::onboardSDK::SDK_VERSION;
    strcpy((char*) user_act_data.iosID, app_bundle_id.c_str());
    user_act_data.encKey = app_key;
    strcpy(user_act_data.encKey, enc_key.c_str());

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.ID);
    printf("app version: 0x0%X\n", user_act_data.version);
    printf("app key: %s\n", user_act_data.encKey);
    printf("=================================================\n");

    rosAdapter = new DJI::onboardSDK::ROSAdapter;
    rosAdapter->init(serial_name, baud_rate);
    rosAdapter->activate(&user_act_data, NULL);
    rosAdapter->setBroadcastCallback(&DJISDKNode::broadcast_callback, this);
    rosAdapter->setFromMobileCallback(&DJISDKNode::transparent_transmission_callback, this);
}


DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
    init_publishers(nh);
    init_services(nh);
    init_actions(nh);
    init_parameters_and_activate(nh_private);
}


DJISDKNode::~DJISDKNode()
{
    delete rosAdapter;
}

//
// Utility
//
inline void DJISDKNode::gps_convert_ned(float &ned_x, float &ned_y,
            double gps_t_lon, double gps_t_lat,
            double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

dji_sdk::LocalPosition DJISDKNode::gps_convert_ned(dji_sdk::GlobalPosition loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
        loc.longitude, loc.latitude,
        global_position_ref.longitude, global_position_ref.latitude
    );
    local.z = loc.height;
    return local;
}

