#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 
#include<ros/ros.h>
#include<std_srvs/SetBool.h>
bool catch_handle_fuc(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response&res){
    if(req.data==true){
        res.success=true;
        return true;
    }else{
        return false;
    }
}
int main(int argc,char** argv){
    ros::init(argc,argv,"move_server");
    ros::NodeHandle nd;
    ros::ServiceServer service=nd.advertiseService("move",catch_handle_fuc);
    ros::spin();
    return 0;
}