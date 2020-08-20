#include <math.h>
#include <ros/ros.h>
#include "dji_sdk/dji_sdk.h"
#include <my_flight_demo/Visual_msg.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

// Pid_control pid_x;
// Pid_control pid_y;
// Pid_control pid_z;

Eigen::Matrix3f QuaternionTomatrix(float w, float x, float y, float z)
{
    Eigen::Matrix3f R;
    R << 1 - 2 * y * y - 2 * z * z, 2 * x * y + 2 * w * z, 2 * x * z - 2 * w * y,
        2 * x * y - 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z + 2 * w * x,
        2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x, 1 - 2 * x * x - 2 * y * y;

    return R;
}

Eigen::Matrix3f eulerTomatrix(float a, float b, float c)
{
    Eigen::Matrix3f R;
    R << cos(b) * cos(a), sin(c) * sin(b) * cos(a) - cos(c) * sin(b), cos(c) * sin(b) * cos(a) + sin(c) * sin(a),
        cos(b) * sin(a), sin(c) * sin(b) * sin(a) + cos(c) * cos(a), cos(c) * sin(b) * sin(a) - sin(c) * cos(b),
        -sin(b), sin(c) * cos(b), cos(c) * cos(b);

    return R;
}

Eigen::Matrix3f Rodrigues(float a, float b, float c)
{
    Eigen::Vector3f rvec;
    rvec << a, b, c;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3, 3);
    float theta = rvec.squaredNorm();
    rvec = rvec / theta;
    Eigen::Matrix3f rom;
    rom << 0, -rvec[2], rvec[1],
        rvec[2], 0, -rvec[0],
        -rvec[1], rvec[0], 0;
    Eigen::Matrix3f R;
    R = cos(theta) * I + (1 - cos(theta)) * rvec * rvec.transpose() + sin(theta) * rom;
    return R;
}

Eigen::Vector3f position_input(float x, float y, float z)
{
    float vx = 0;
    float vy = 0;
    float vz = 0;
    if (-2 < y < 2 && -21 < x < -10 && 10 < z < 21)
    {
        float vx = 0;
        float vy = 0;
        float vz = 0;
    }
    else
    {
        if (-2 < y || y > 2)
        {
            vy = -y / 10;
        }
        if (x > -10 || x < -21)
        {
            vx = -(20 + x) / 20;
        }
        if (z > 21 || z < 10)
        {
            vz = -(z - 20) / 20;
        }
    }
    Eigen::Vector3f V;
    V << vx, vy, vz;
    return V;
    // Eigen::Matrix<float,3,3> R3=eulerTomatrix(r1,r2,r3);
    // float qx=current_atti.x;
    // float qy=current_atti.y;
    // float qz=current_atti.z;
    // float qw=current_atti.w;
    // Eigen::Matrix<float,3,3> R2=QuaternionTomatrix(qw,qx,qy,qz);
    // Eigen::Matrix<float,3,3> R=R2*R3;
    // Eigen::Matrix<float,1,3> v;
    // v[0]=vx;
    // v[1]=vy;
    // v[2]=vz;
    // // transition v to ground frame
    // Eigen::Matrix<float,1,3> v_ground = v * R;

    // float v_ground_x=v_ground[0];
    // float v_ground_y=v_ground[1];
    // float v_ground_z=v_ground[2];
    // sensor_msgs::Joy controlVelYawRate;
    // controlVelYawRate.axes.push_back(v_ground_x);
    // controlVelYawRate.axes.push_back(v_ground_y);
    // controlVelYawRate.axes.push_back(v_ground_z);
    // controlVelYawRate.axes.push_back(0);
    // posctrlVYawRatePub.publish(controlVelYawRate);
}

void boundle(float &v)
{
    if(v>0.5)
        v=0.5;
    if(v<-0.5)
        v=-0.5;
}


Eigen::Vector3f get_vel(float x, float y, float z)
{
    float vx = 0;
    float vy = 0;
    float vz = 0;

    if (-2.5 > y || y < 2.5)
    {
        vy = -y;
        if(vy>1)
        {
            vy=1;
        }
        else if(vy<-1)
        {
            vy=-1;
        }
    }
    else
    {
        vy = 0;
    }
    if (x > -5 || x < -10)
    {
        vx = -(7.5 + x);
        if(vx>1)
        {
            vx=1;
        }
        else if(vx<-1)
        {
            vx=-1;
        }
    }
    else
    {
        vx = 0;
    }
    if (z > 20 || z < 10)
    {
        vz = -(z - 15);
        if(vz>1)
        {
            vz=1;
        }
        else if(vz<-1)
        {
            vz=-1;
        }
    }
    else
    {
        vz = 0;
    }
    Eigen::Vector3f V;
    V << -vz, -vy, -vx;
    return V;
}

// std::vector<float> get_pid_vel(float x, float y, float z)
// {
//     float v_x=pid_x.PID_realize(0,x);
//     float v_y=pid_y.PID_realize(0,y);
//     float v_z=pid_z.PID_realize(50,z);
//     std::vector<float> vel;
//     vel.push_back(v_x);
//     vel.push_back(v_y);
//     vel.push_back(v_z);
//     return vel;
// }