#include <examine_grasps.h>

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

ExamineGPDGrasps::ExamineGPDGrasps()
  {
    pnh_.param("offset", offset_, 0.40);
    pnh_.param("group", group_name_, std::string("arm"));
    repredict_quality_=false;
    group_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);

    psm_= std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    psm_->startStateMonitor();
    psm_->startSceneMonitor("move_group/monitored_planning_scene");

    get_grasps_= nh_.serviceClient<moveit_msgs::GraspPlanning>("plan_grasps");
    dexnet_client = nh_.serviceClient<sli_gpd_pick_object::depth_GQCNNPredict>("depth_gqcnn_predict");
    observation_poses_pub_= nh_.advertise<geometry_msgs::PoseArray>("valid_observation_poses", 1);
  }

bool ExamineGPDGrasps::run(){
    while(ros::ok()){
      ROS_INFO("move out of the way");
      group_->setNamedTarget("home");
      while(!group_->move()) ROS_ERROR_THROTTLE(1.0,"move to home failed");
      ros::Duration(5.0).sleep();
      moveit_msgs::GraspPlanning srv;
      get_grasps_.call(srv);

      ROS_INFO("computing IK");
      std::vector<ObservableGrasp> grasps= compute_ik(srv.response.grasps);

      {
        geometry_msgs::PoseArray poses;
	      poses.header.frame_id= group_->getPlanningFrame();
        for(auto& g : grasps){
          Eigen::Affine3d p= g.observation_state.getGlobalLinkTransform("s_model_tool0");
          poses.poses.emplace_back();
          tf::poseEigenToMsg(p, poses.poses.back());
        }
	       observation_poses_pub_.publish(poses);
      }

      ROS_INFO("try to observe grasp position");

      for(ObservableGrasp& g : grasps){
        group_->setJointValueTarget(g.observation_state);
        if(!group_->move())
          ROS_ERROR("Failed to move to observation pose");
        else
        {
          sli_gpd_pick_object::depth_GQCNNPredict predict_srv;
          predict_srv.request.gripper_position=g.grasp.grasp_pose.pose.position;
          dexnet_client.call(predict_srv);
          repredict_quality_=predict_srv.response.repredict_quality;
          if (repredict_quality_)
              break;
          else
              continue;
        }
      }
      // TODO: wait for image
      ROS_INFO("Waiting for image from observation state");
      ros::Duration(5).sleep();
      return repredict_quality_;
    }
  }

  /* returns vector of pairs of feasible grasps and their respective observation poses */
std::vector<ExamineGPDGrasps::ObservableGrasp> ExamineGPDGrasps::compute_ik(std::vector<moveit_msgs::Grasp>& grasps){
    std::vector<ExamineGPDGrasps::ObservableGrasp> valid_grasps;
    valid_grasps.reserve(grasps.size());

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm_);
    const planning_scene::PlanningSceneConstPtr scene= static_cast<const planning_scene::PlanningSceneConstPtr>(locked_scene);
    auto valid_fn= boost::bind(&isIKSolutionValid, scene, _1, _2, _3);
    moveit::core::RobotState state(scene->getCurrentState());

    const moveit::core::JointModelGroup* jmg= state.getJointModelGroup(group_name_);

    for(auto& g : grasps){
      Eigen::Affine3d grasp_pose, observation_pose;
      tf::poseMsgToEigen(g.grasp_pose.pose, grasp_pose);

      grasp_pose= scene->getFrameTransform(g.grasp_pose.header.frame_id) * grasp_pose;

      observation_pose= grasp_pose * Eigen::Translation<double,3>(-offset_,0,0);

      if(state.setFromIK(jmg, grasp_pose, 1, 0.1, valid_fn) && state.setFromIK(jmg, observation_pose, 1, 0.1, valid_fn)){
        valid_grasps.push_back( ObservableGrasp(g, state) );
      }
    }

    return valid_grasps;
  }
