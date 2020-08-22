#include "my_flight_demo/common.h"
#include "my_flight_demo/pid.h"
#include <std_srvs/SetBool.h>


geometry_msgs::Vector3 current_velocity;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 current_position;
my_flight_demo::Visual_msg detect;

ros::Publisher offsetPub;
ros::ServiceClient drone_task_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient set_local_pos_reference;

Pid_control pid_x;
Pid_control pid_y;
Pid_control pid_z;
bool detected=false;
bool do_grasp=false;

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
    float v_z=pid_z.PID_realize(50,z);
    std::vector<float> vel;
    vel.push_back(v_x);
    vel.push_back(v_y);
    vel.push_back(v_z);
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

void visual_callback(const my_flight_demo::Visual_msg::ConstPtr &msg)
{
  int id = msg->id;
  geometry_msgs::Vector3 rvec = msg->rvec;
  geometry_msgs::Vector3 tvec = msg->tvec;
  if (id == 100)
  {
    sensor_msgs::Joy controloffset;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_BODY |
                    DJISDK::STABLE_ENABLE);
    controloffset.axes.push_back(0);
    controloffset.axes.push_back(0);
    controloffset.axes.push_back(0);
    controloffset.axes.push_back(0);
    controloffset.axes.push_back(flag);
    std::cout<<"wait to detect"<<std::endl;
    offsetPub.publish(controloffset);
  }
  else
  {
    float tx = tvec.x;
    float ty = tvec.y;
    float tz = tvec.z;
    float delta_x=tz-0.5;
    float delta_y=ty-0;
    if(abs(delta_x)<0.1&&abs(delta_y)<0.1)
    {
        do_grasp=true;
        sensor_msgs::Joy hover;

        uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                        DJISDK::HORIZONTAL_VELOCITY |
                        DJISDK::YAW_RATE |
                        DJISDK::HORIZONTAL_BODY |
                        DJISDK::STABLE_ENABLE);
        hover.axes.push_back(0);
        hover.axes.push_back(0);
        hover.axes.push_back(0);
        hover.axes.push_back(0);
        hover.axes.push_back(flag);
        std::cout<<"hover! prepare to grab"<<std::endl;
        offsetPub.publish(hover);

    }
    // std::vector<float> V= get_pid_vel(tx, ty, tz);
    else
    {
      sensor_msgs::Joy controloffset;
      uint8_t flag = (DJISDK::VERTICAL_POSITION |   //fixed height
                      DJISDK::HORIZONTAL_VELOCITY | //control horizontal vel
                      DJISDK::YAW_RATE |
                      DJISDK::HORIZONTAL_BODY |
                      DJISDK::STABLE_ENABLE);
      controloffset.axes.push_back(10);
      controloffset.axes.push_back(10);
      controloffset.axes.push_back(6);
      controloffset.axes.push_back(0);
      controloffset.axes.push_back(flag);
      std::cout<<delta_x<<" "<<delta_y<<std::endl;
      offsetPub.publish(controloffset);
    }
    
    // if(!detected)
    // {
    //     offsetPub.publish(controloffset);
    //     detected=true;
    // }
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
  ros::init(argc, argv, "demo_offset_node");
  ros::NodeHandle nh;
  ros::ServiceServer grasp_service = nh.advertiseService("move", do_move);
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  offsetPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
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
  pid_x.PID_init(0.1,0.05,0.1,0,0);
  pid_y.PID_init(0.1,0.05,0.1,0,0);
  pid_z.PID_init(0.1,0.05,0.1,50,0);
  ros::Subscriber visualSub = nh.subscribe("visual", 10, &visual_callback);
  ros::spin();
  return 0;
}
