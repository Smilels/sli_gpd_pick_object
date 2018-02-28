/* Author: Shuang Li */
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace {
bool isIKSolutionValid(const planning_scene::PlanningSceneConstPtr planning_scene,
                       robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                       const double* ik_solution)
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();
  return !planning_scene->isStateColliding(*state, jmg->getName());
}
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"ur5move");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  tf::TransformListener listener;
  sleep(20);
  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  planning_scene_monitor::PlanningSceneMonitorPtr psm_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  psm_->startStateMonitor();
  psm_->startSceneMonitor("move_group/monitored_planning_scene");
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_visualization", 10);
  // move to "extended" pose

  // group.setNamedTarget("pour_default_2");
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // bool success = group.move() == moveit_msgs::MoveItErrorCodes::SUCCESS ;
  // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);

  // setJointValueTarget
  //group.setStartStateToCurrentState ();
  geometry_msgs::PoseStamped target2;
  //target2.header.frame_id="table_top";??can't work when set the pose with respenct to "table_top"
  target2.header.frame_id="/world";
  target2.header.stamp=ros::Time::now();
  target2.pose.orientation.w=1;
  target2.pose.position.x=0.657917615558;// 0.371849018129
  target2.pose.position.y=0.866543526334;// 0.298981806364
  target2.pose.position.z= 1.23391489751;//1.80035830512
  group.setJointValueTarget(target2);

  bool success = group.move() == moveit_msgs::MoveItErrorCodes::SUCCESS ;
  ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
  sleep(4.0);

  planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm_);
  const planning_scene::PlanningSceneConstPtr scene= static_cast<const planning_scene::PlanningSceneConstPtr>(locked_scene);
  auto valid_fn= boost::bind(&isIKSolutionValid, scene, _1, _2, _3);
  moveit::core::RobotState state(scene->getCurrentState());
  const moveit::core::JointModelGroup* jmg= state.getJointModelGroup("arm");

  Eigen::Affine3d grasp_pose, observation_pose,trans;
  tf::poseMsgToEigen(target2.pose, grasp_pose);
  grasp_pose= scene->getFrameTransform(target2.header.frame_id) * grasp_pose;
  observation_pose= grasp_pose * Eigen::Translation<double,3>(-0.50,0,0);

  tf::StampedTransform transform;
  try{
      listener.waitForTransform("/r200_camera_link", "/s_model_tool0", ros::Time::now(), ros::Duration(5.0));
      listener.lookupTransform ("/r200_camera_link", "/s_model_tool0",ros::Time(0), transform);
    }
  catch(std::runtime_error &e){
    std::cout<<"tf error"<<"/n";
    return 0;
    }

tf::transformTFToEigen (transform, trans);
while(ros::ok())
{
//const Eigen::Affine3d cons_transform=state.getFrameTransform ("/r200_camera_link");
observation_pose= grasp_pose * trans;
geometry_msgs::Pose obs;
tf::poseEigenToMsg(observation_pose,obs);
geometry_msgs::PoseStamped obs2;
  //target2.header.frame_id="table_top";??can't work when set the pose with respenct to "table_top"
  obs2.header.frame_id="/world";
  obs2.header.stamp=ros::Time::now();
  obs2.pose=obs;
pub.publish(obs2);

// geometry_msgs::Pose obs;
// tf::poseEigenToMsg(observation_pose,obs);
//
// tf::Transform transform1;
// transform1.setOrigin(tf::Vector3(obs.position.x,obs.position.y,obs.position.z));
// tf::Quaternion ori;
// tf::quaternionMsgToTF (obs.orientation,ori);
// transform1.setRotation(ori);
// transform1=transform1*transform;
//
// geometry_msgs::Pose obs1;
// obs1.position.x=transform1.getOrigin().getX ();
// obs1.position.y=transform1.getOrigin().getY ();
// obs1.position.z=transform1.getOrigin().getZ ();
// tf::quaternionTFToMsg(transform1.getRotation(),obs1.orientation);
//
// tf::poseMsgToEigen(obs1,observation_pose);
if(state.setFromIK(jmg, grasp_pose, 1, 0.1, valid_fn))
{
  ROS_INFO("grasp_pos IK sueccess");
  if ( state.setFromIK(jmg, observation_pose, 1, 0.1, valid_fn))
  {
    group.setJointValueTarget(state);
    success = group.move() == moveit_msgs::MoveItErrorCodes::SUCCESS ;
    ROS_INFO("Visualizing plan 2++ (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(4.0);
  }
  else
  {
    ROS_INFO("IK FAILED");
  }
}

}
  return 0;
}
