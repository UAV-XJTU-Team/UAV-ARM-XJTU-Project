#include "dji_sdk/dji_sdk.h"
#include <my_flight_demo/Visual_msg.h>
#include "my_flight_demo/common.h"
#include <std_srvs/SetBool.h>
#include <my_flight_demo/pid.h>

const float distance = 1; //decide the distance between camera and marker along camera's z axis(the face),unit is meter


geometry_msgs::Vector3 current_velocity;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 current_position;

ros::Publisher ctrlVelYawRatePub;
ros::ServiceClient drone_task_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient set_local_pos_reference;

ros::Publisher graspPub;

Pid_control pid_x;
Pid_control pid_y;
Pid_control pid_z;

bool do_grasp=false;
bool has_grasped=false;

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


std::vector<float> get_pid_vel(float x, float y, float z)
{
    if(isnan(x)|isnan(y)|isnan(z)){
        std::vector<float> vel;
        vel.push_back(0);
        vel.push_back(0);
        return vel;
    }
    bool x_ok=pid_x.PID_realize(0,x,0.1);  // return true if drone has approach the setpoint by erro=+-0.1m
    bool y_ok=pid_y.PID_realize(0,y,0.1);
    bool z_ok=pid_z.PID_realize(1,z,0.1);
    if (y_ok&&z_ok){ // only flight in y-z plane of marker frame;
        do_grasp=true;
    }
    float v_x=pid_x.pid.OutPutVel;
    float v_y=pid_y.pid.OutPutVel;
    float v_z=pid_z.pid.OutPutVel;
    boundle(v_x,0.2); //[-0.2,0.2]
    boundle(v_y,0.2);
    boundle(v_z,0.2);
    float v_body_forwrad=-v_z;
    float v_body_left=-v_y;
    std::vector<float> vel;
    vel.push_back(v_body_forwrad);
    vel.push_back(v_body_left);
    return vel;
}

void visualbody_callback(const my_flight_demo::Visual_msg::ConstPtr &msg)
{
    if(has_grasped)
    {
        takeoff_land(6);
    }
    else
    {
        int id = msg->id;
        geometry_msgs::Vector3 rvec = msg->rvec;
        geometry_msgs::Vector3 tvec = msg->tvec;
        if (id == 100)
        {
            // float vx = 0;
            // float vy = 0;
            // float vz = 0;
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
            // std::cout << "vx: " << vx << " "
            //           << "vy: " << vy << " "
            //           << "vz: " << vz << std::endl;
        }
        else
        {
            float tx = tvec.x;
            float ty = tvec.y;
            float tz = tvec.z;
            std::cout<<"current position:"<<std::endl;
            std::cout << "tx: " << tx << " "
                    << "ty: " << ty << " "
                    << "tz: " << tz << std::endl;
            std::vector<float> V = get_pid_vel(tx, ty, tz); //this function can change do_grasp 's value
            float vx = V[0];
            float vy = V[1];
            if(do_grasp)
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
                std::cout<<"hover! wait to grab"<<std::endl;
                ctrlVelYawRatePub.publish(controlVelYawRate);
                // std::cout<<"hover"<<std::endl;
            }
            else
            {
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
                // std::cout<<"current velocity:"<<std::endl;
                // std::cout << "vx: " << vx << " "
                //           << "vy: " << vy << " "<< std::endl;
            }
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
  else if(req.data==false)
  {
    has_grasped=true;
    return true;
  }
  else
  {
      res.success=false;
      return true;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_flight_control_node");
    ros::NodeHandle nh;
    // start dji connection
    
    drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
    bool obtain_control_result = obtain_control();
    // start takeoff
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
    pid_z.PID_init(0.1,0.05,0.1,0.7,0);
    ros::ServiceServer grasp_service = nh.advertiseService("move", do_move);
    ros::Subscriber visualSub = nh.subscribe("visual", 10, &visualbody_callback);
    ros::spin();
    return 0;
}
