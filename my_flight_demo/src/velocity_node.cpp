#include "my_flight_demo/common.h"
#include "my_flight_demo/pid.h"
#include <std_srvs/SetBool.h>

geometry_msgs::Vector3 current_velocity;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 current_position;
my_flight_demo::Visual_msg detect;

ros::Publisher ctrlVelYawRatePub;
ros::ServiceClient drone_task_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient set_local_pos_reference;

bool do_grasp=false;

Pid_control pid_x;
Pid_control pid_y;
Pid_control pid_z;


std::vector<float> get_pid_vel(float x, float y, float z)
{
    if(isnan(x)|isnan(y)|isnan(z)){
        std::vector<float> vel;
        vel.push_back(0);
        vel.push_back(0);
        vel.push_back(0);
        return vel;
    }
    float v_x=pid_x.PID_realize(0,x);
    float v_y=pid_y.PID_realize(0,y);
    float v_z=pid_z.PID_realize(0.5,z);
    float v_body_x=-v_z;
    float v_body_y=-v_y;
    float v_body_z=-v_x;

    std::vector<float> vel;
    vel.push_back(v_body_x);
    vel.push_back(v_body_y);
    vel.push_back(v_body_z);
    return vel;
}

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

// void visual_callback(const my_flight_demo::Visual_msg::ConstPtr &msg)
// {
//   int id = msg->id;
//   geometry_msgs::Vector3 rvec = msg->rvec;
//   geometry_msgs::Vector3 tvec = msg->tvec;
//   float a = rvec.x;
//   float b = rvec.y;
//   float c = rvec.z;
//   Eigen::Matrix3f R3 = Rodrigues(a, b, c); // marker frame to camera frame
//   float x = current_atti.x;
//   float y = current_atti.y;
//   float z = current_atti.z;
//   float w = current_atti.w;
//   Eigen::Matrix3f R1 = QuaternionTomatrix(w, x, y, z); //body frame to ground frame
//   Eigen::Matrix3f R2;                                  //camera frame to body frame
//   R2 = eulerTomatrix(0, 0, 1.1415926 / 2);
//   Eigen::Matrix3f R = R1 * R2 * R3;
//   float tx = tvec.x;
//   float ty = tvec.y;
//   float tz = tvec.z;
//   Eigen::Vector3f marker_frame_v = position_input(tx, ty, tz);
//   Eigen::Vector3f V = R * marker_frame_v;
//   float vx = V[0];
//   float vy = V[1];
//   float vz = V[2];
//   std::cout << "vx: " << vx << " "
//             << "vy: " << vy << " "
//             << "vz: " << vz << std::endl;
// }

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
    std::cout<<"wait to detect"<<std::endl;
    ctrlVelYawRatePub.publish(controlVelYawRate);
  }
  else
  {
    float tx = tvec.x;
    float ty = tvec.y;
    float tz = tvec.z;

    float delta_y=abs(ty);
    float delta_z=abs(tz-1);
    if(delta_y<0.1&&delta_z<0.1)
    {
      do_grasp=true;
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
      std::cout<<"hover! wait to grab"<<std::endl;
      ctrlVelYawRatePub.publish(controlVelYawRate);
    }
    else
    {
      std::vector<float> V= get_pid_vel(tx, ty, tz);

      sensor_msgs::Joy controlVelYawRate;
      uint8_t flag = (DJISDK::VERTICAL_POSITION |
                      DJISDK::HORIZONTAL_VELOCITY |
                      DJISDK::YAW_RATE |
                      DJISDK::HORIZONTAL_BODY |
                      DJISDK::STABLE_ENABLE);
      controlVelYawRate.axes.push_back(V[0]);
      controlVelYawRate.axes.push_back(V[1]);
      controlVelYawRate.axes.push_back(2);
      controlVelYawRate.axes.push_back(0);
      controlVelYawRate.axes.push_back(flag);
      std::cout<<"approach setpoint"<<std::endl;
      std::cout<<V[0]<<" "<<V[1]<<std::endl;
      ctrlVelYawRatePub.publish(controlVelYawRate);
    }
  }
}

bool do_move(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res)
{
  if(req.data==true&&do_grasp==true)
  {
      res.success=true;
      ROS_INFO("start grab");
      return true;
  }
  else
  {
      return false;
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
  ros::Duration(5).sleep();
  ros::spinOnce();
  pid_x.PID_init(0.1,0.05,0.1,0,0);
  pid_y.PID_init(0.1,0.05,0.1,0,0);
  pid_z.PID_init(0.1,0.05,0.1,0.5,0);
  ros::Subscriber visualSub = nh.subscribe("visual", 10, &visualbody_callback);
  ros::ServiceServer grasp_service = nh.advertiseService("move", do_move);
  ros::spin();
  return 0;
}
