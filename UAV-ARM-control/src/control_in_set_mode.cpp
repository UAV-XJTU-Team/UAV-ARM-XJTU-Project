
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 
#include<ros/ros.h>
#include<std_srvs/SetBool.h>

int main(int argc,char** argv){
  ros::init(argc,argv,"set_mode_arm");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  //后续可以写成循环的形式，调用多个服务来实现复杂的机能
  spinner.start();
  moveit::planning_interface::MoveGroupInterface group_arm("arm");
  moveit::planning_interface::MoveGroupInterface group_hand("hand");
      // 进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
  moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
  group_arm.allowReplanning(true);
  group_arm.setGoalPositionTolerance(0.2);
  group_arm.setGoalOrientationTolerance(0.5);
  group_arm.setPlanningTime(10);
  group_hand.setGoalPositionTolerance(0.5);
  group_hand.setGoalOrientationTolerance(0.5);
  geometry_msgs::Pose target_pose;
  // target_pose.orientation.x=-0.280225;
  // target_pose.orientation.y=0.63647;
  // target_pose.orientation.z=0.65051;
  target_pose.orientation.w=-0.409517;
  target_pose.position.x=-0.0030771;
  target_pose.position.y=0.213873;
  target_pose.position.z=0.177508;
// 4.63401540182531e-05, 6.57061110250652e-05, 5.383514254353942e-05, -3.4986781422048792e-06, -4.165919008664787e-05, 3.5677548730745905e-05
  // group_arm.setNamedTarget("reset");
  // group_hand.setNamedTarget("open");
  // bool success = group_arm.plan(arm_plan).SUCCESS && group_hand.plan(hand_plan).SUCCESS;
  //   //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  // if(success){
  //     group_arm.execute(arm_plan);
  //     sleep(1);
  //     group_hand.execute(hand_plan);
  //     sleep(1);
  // }
  group_arm.setPoseTarget(target_pose);
  group_hand.setNamedTarget("close");
  bool success = group_arm.plan(arm_plan).SUCCESS && group_hand.plan(hand_plan).SUCCESS;
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  if(success){
      group_arm.execute(arm_plan);
      sleep(2);
      group_hand.execute(hand_plan);
      sleep(2);
      success=false;
  }
  group_arm.setNamedTarget("catch_backward");
  group_hand.setNamedTarget("open");
  success = group_arm.plan(arm_plan).SUCCESS && group_hand.plan(hand_plan).SUCCESS;
    //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
  if(success){
      group_arm.execute(arm_plan);
      sleep(2);
      group_hand.execute(hand_plan);
      sleep(2);
  }
  ros::shutdown();
  return 0;
}
