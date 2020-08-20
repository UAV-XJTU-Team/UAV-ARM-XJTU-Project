#include "dji_sdk/dji_sdk.h"
#include <math.h>
#include <my_flight_demo/Visual_msg.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>
#include "my_flight_demo/common.h"


geometry_msgs::Vector3 current_velocity;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 current_position;
my_flight_demo::Visual_msg detect;

ros::Publisher ctrlVelYawRatePub;
ros::ServiceClient drone_task_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient set_local_pos_reference;

// Pid_control pid_x;
// Pid_control pid_y;
// Pid_control pid_z;

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(authority);

  if (!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
  current_atti = msg->quaternion;
}

void visual_callback(const my_flight_demo::Visual_msg::ConstPtr &msg)
{
  int id = msg->id;
  geometry_msgs::Vector3 rvec = msg->rvec;
  geometry_msgs::Vector3 tvec = msg->tvec;
  float a = rvec.x;
  float b = rvec.y;
  float c = rvec.z;
  Eigen::Matrix3f R3 = Rodrigues(a, b, c); // marker frame to camera frame
  float x = current_atti.x;
  float y = current_atti.y;
  float z = current_atti.z;
  float w = current_atti.w;
  Eigen::Matrix3f R1 = QuaternionTomatrix(w, x, y, z); //body frame to ground frame
  Eigen::Matrix3f R2;                                  //camera frame to body frame
  R2 = eulerTomatrix(0, 0, 1.1415926 / 2);
  Eigen::Matrix3f R = R1 * R2 * R3;
  float tx = tvec.x;
  float ty = tvec.y;
  float tz = tvec.z;
  Eigen::Vector3f marker_frame_v = position_input(tx, ty, tz);
  Eigen::Vector3f V = R * marker_frame_v;
  float vx = V[0];
  float vy = V[1];
  float vz = V[2];
  std::cout << "vx: " << vx << " "
            << "vy: " << vy << " "
            << "vz: " << vz << std::endl;
}

void visualbody_callback(const my_flight_demo::Visual_msg::ConstPtr &msg)
{
  int id = msg->id;
  geometry_msgs::Vector3 rvec = msg->rvec;
  geometry_msgs::Vector3 tvec = msg->tvec;
  if (id == 100)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_BODY |
                    DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);
    std::cout<<"0"<<std::endl;
    // ctrlVelYawRatePub.publish(controlVelYawRate);
  }
  else
  {
    current_position=tvec;
    float tx = tvec.x;
    float ty = tvec.y;
    float tz = tvec.z;
    Eigen::Vector3f V = get_vel(tx, ty, tz);

    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_BODY |
                    DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(1);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);
    std::cout<<V[0]<<" "<<V[1]<<" "<<V[2]<<std::endl;
    // ctrlVelYawRatePub.publish(controlVelYawRate);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  bool obtain_control_result = obtain_control();
  if (takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    ROS_INFO("start takeoff");
  }
  else
  {
    ROS_INFO("takeoff failed");
  }
  ros::Duration(0.01).sleep();
  ros::spinOnce();
  // pid_x.PID_init(0.1,0.05,0.1,0,current_position.x);
  // pid_y.PID_init(0.1,0.05,0.1,0,current_position.y);
  // pid_z.PID_init(0.1,0.05,0.1,50,current_position.z);
  ros::Subscriber visualSub = nh.subscribe("visual", 10, &visualbody_callback);
  ros::spin();
  return 0;
}
