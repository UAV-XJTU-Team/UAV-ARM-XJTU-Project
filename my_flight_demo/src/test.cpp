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
    float v_z=pid_z.PID_realize(50,z);
    std::vector<float> vel;
    vel.push_back(v_x);
    vel.push_back(v_y);
    vel.push_back(v_z);
    return vel;
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
        float vx = 0;
        float vy = 0;
        float vz = 0;
        std::cout << "vx: " << vx << " "
                  << "vy: " << vy << " "
                  << "vz: " << vz << std::endl;
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
        std::vector<float> V = get_pid_vel(tx, ty, tz);
        float vx = V[0];
        float vy = V[1];
        float vz = V[2];
        std::cout<<"current velocity:"<<std::endl;
        std::cout << "vx: " << vx << " "
                  << "vy: " << vy << " "
                  << "vz: " << vz << std::endl;
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_flight_control_node");
    ros::NodeHandle nh;
    pid_x.PID_init(0.1,0.05,0.1,0,0);
    pid_y.PID_init(0.1,0.05,0.1,0,0);
    pid_z.PID_init(0.1,0.05,0.1,50,0);
    ros::Subscriber visualSub = nh.subscribe("visual", 10, &visualbody_callback);
    ros::spin();
    return 0;
}
