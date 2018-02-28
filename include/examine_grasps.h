#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/CollisionObject.h>
#include <sli_gpd/GraspConfigList.h>
#include <sli_gpd/GraspConfig.h>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sli_gpd_pick_object/depth_GQCNNPredict.h>

namespace {
bool isIKSolutionValid(const planning_scene::PlanningSceneConstPtr planning_scene,
                       robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                       const double* ik_solution);}

class ExamineGPDGrasps
{
  struct ObservableGrasp {
      ObservableGrasp(moveit_msgs::Grasp g, moveit::core::RobotState rs) :
        grasp(g),
        observation_state(rs)
      {}
      moveit_msgs::Grasp grasp;
      moveit::core::RobotState observation_state;
    };

  public:
    ExamineGPDGrasps();

    bool run();
    /* returns vector of pairs of feasible grasps and their respective observation poses */
    std::vector<ObservableGrasp> compute_ik(std::vector<moveit_msgs::Grasp>& grasps);

  private:
    // offset for the evaluation pose
    double offset_;
    bool repredict_quality_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string group_name_;
    moveit::planning_interface::MoveGroupInterfacePtr group_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    ros::ServiceClient get_grasps_;
    ros::ServiceClient dexnet_client;
    ros::Publisher observation_poses_pub_;
  };
