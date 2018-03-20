#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PickupActionResult.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickObject
{
public:
  moveit::planning_interface::MoveGroupInterface arm;
  moveit::planning_interface::MoveGroupInterface gripper;
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::PlanningScene planning_scene;
  PickObject() : arm("arm"), gripper("gripper")
  {
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
  }

  bool executePick()
  {
    arm.setPlanningTime(20.0);
    return arm.planGraspsAndPick() == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
};

octomapCallback(octomap_msgs::Octomap msg)
{
 planning_scene.world.octomap.header.frame_id = "odom_combined";
 planning_scene.world.octomap.header.stamp = ros::Time::now();
 planning_scene.world.octomap.octomap = msg;
 octomap_pub.publish(planning_scene);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_object_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle n;
  PickObject po;
  ros::Subscriber sub = n.subscribe("/octomap_full", 1000, octomapCallback);
  bool success = false;
  ros::Publisher octomap_pub = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

  ROS_INFO("Picking Object");
  while(ros::ok())
  {
    if(success)
    {
      po.arm.setNamedTarget("extended");
      while(!po.arm.move())
        ROS_ERROR("moving to extended pose failed.");
      po.gripper.setNamedTarget("open");
      while(!po.gripper.move())
        ROS_ERROR("opening gripper failed.");
      po.arm.setNamedTarget("pour_default");
      while(!po.arm.move())
        ROS_ERROR("moving home failed.");
      ros::Duration(5).sleep();
      success = false;
    }
	
    success = po.executePick();
  }

  return 0;
}
