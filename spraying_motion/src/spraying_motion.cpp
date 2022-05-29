#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "find_target/target_position.h"




int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  //ROS maa spinne foer program, dette gjor asyncspinner
  // .
  ros::AsyncSpinner spinner(1);
  spinner.start();

   
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
    // bruker :planning_interface:`MoveGroupInterface` klassen 
    // 
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //
   

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    //
    

    // liste av alle grupene i roboten
    ROS_INFO_NAMED("Spraying", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //
    
    // 1. setter roboten i "start" posisjon
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("start"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan start position (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //
    // Get the box and the target position from the opencv node
    ros::ServiceClient target_position_srv_client = n.serviceClient<find_target::target_position>("target_position");

    find_target::target_position srv;


    if(target_position_srv_client.call(srv)) {
      ROS_INFO_STREAM("3d target position camera frame: x " << srv.response.target_position.x << " y " << srv.response.target_position.y << " z " << srv.response.target_position.z);
    } else {
      ROS_INFO_STREAM("Failed to call target position service");
    }

    ros::Duration(0.5).sleep();

    // 2. plasserer end-effektoren i en viss x, y, z pos
    ROS_INFO_NAMED("Spraying", " Placing the TCP (Tool Center Point, the tip of the robot) infornt of bush");

    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = srv.response.target_position.x;
    target_pose1.position.y = srv.response.target_position.y - 0.35;
    target_pose1.position.z = srv.response.target_position.z;

    move_group_interface_arm.setPlanningTime(10); //the kdl planner har problems executing the motion

    move_group_interface_arm.setPoseTarget(target_pose1);
    move_group_interface_arm.setPlanningTime(10); //the kdl planner har problems executing the motion

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Goal position x:%f y:%f z:%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
    ROS_INFO_NAMED("Spraying", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.setPlanningTime(10); //the kdl planner har problems executing the motion

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //


    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. tar igjen gripper for aa spraye
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 3 (pose goal) begynner spraying %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
   

                                

  //   
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1; 
                            
    target_pose2.position.x -=  0.3;
    waypoints.push_back(target_pose2);  // hoyre

    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // ned
   
    target_pose2.position.x +=  0.6;
    waypoints.push_back(target_pose2);  // venstre
   
    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // ned
 
    target_pose2.position.x -= 0.6;
    waypoints.push_back(target_pose2);  // hoyre

    target_pose2.position.z -= 0.2;
    waypoints.push_back(target_pose2);  // ned
 
    target_pose2.position.x += 0.6;
    waypoints.push_back(target_pose2);  // venstre



    //utforer den "cartesian" vei
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
   
    move_group_interface_arm.execute(trajectory);
 
  
  
    // 7. aapner gripperen(tar av sprayen)
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
    ros::Duration(0.5).sleep();

    //


     // 8. beveger til "finnish" posisjon
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("finnish"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Spraying", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    //


  ros::shutdown();
  return 0;
}
