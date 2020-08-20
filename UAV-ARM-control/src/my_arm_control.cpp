
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 
#include<ros/ros.h>
#include<std_srvs/SetBool.h>

int main(int argc,char** argv){
  ros::init(argc,argv,"client");
  ros::NodeHandle nh;
   ros::AsyncSpinner spinner(1);
  ros::ServiceClient client=nh.serviceClient<std_srvs::SetBool>("move");
  bool flag=true;
  while(flag){
      std_srvs::SetBool srv;
      srv.request.data=true;
      if(client.call(srv)){
        ROS_INFO("request success");
        flag=false;
      }else{
        ROS_INFO("no response");
      }
  }
  //后续可以写成循环的形式，调用多个服务来实现复杂的机能
  spinner.start();
    moveit::planning_interface::MoveGroupInterface group_arm("arm");
    moveit::planning_interface::MoveGroupInterface group_hand("hand");
        // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan hand_plan;

      group_arm.setNamedTarget("catch_forward");
      group_hand.setNamedTarget("close");
      bool success = group_arm.plan(arm_plan).SUCCESS && group_hand.plan(hand_plan).SUCCESS;
      ROS_INFO("Visualizing plan  (pose goal)");   
        
        //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
        if(success){
            group_arm.execute(arm_plan);
            sleep(0.5);
            group_hand.execute(hand_plan);
           sleep(0.5);
            success=false;
        }
        
        
         group_arm.setNamedTarget("reset");
         group_hand.setNamedTarget("open");
         success = group_arm.plan(arm_plan).SUCCESS && group_hand.plan(hand_plan).SUCCESS;
         if(success){
            group_arm.execute(arm_plan);
             sleep(0.5);
            group_hand.execute(hand_plan);
            sleep(0.5);
        }
      
        ros::shutdown();
       return 0;
}
