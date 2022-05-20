#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>






int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //
   

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    //
    

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("Spraying", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("start"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //


    // 2. Place the TCP (Tool Center Point, the tip of the robot) infront of bush
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = current_pose.pose.orientation;
    //target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.8;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //


    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. close the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
    ros::Duration(0.5).sleep();

                                

  //   
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1; 
                            
    target_pose2.position.x -=  0.6;
    waypoints.push_back(target_pose2);  // right

    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // down
   
    target_pose2.position.x +=  0.6;
    waypoints.push_back(target_pose2);  // left
   
    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // down
 
    target_pose2.position.x -= 0.6;
    waypoints.push_back(target_pose2);  // right

    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // down
 
    target_pose2.position.x += 0.6;
    waypoints.push_back(target_pose2);  // left

      target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // down
 
    target_pose2.position.x -= 0.6;
    waypoints.push_back(target_pose2);  // right

    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // down
 
    target_pose2.position.x += 0.6;
    waypoints.push_back(target_pose2);  // left


    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group_interface_arm.execute(trajectory);
 
    ros::Duration(0.5).sleep();
  
    // 7. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
    ros::Duration(0.5).sleep();

    //


     // 8. Move to finnish position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("finnish"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //


  ros::shutdown();
  return 0;
}
