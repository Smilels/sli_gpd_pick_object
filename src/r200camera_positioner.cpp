#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_position_node");
  // TF communication channels
  tf::TransformListener listener;
  tf::TransformBroadcaster br;

  // constant transforms
  tf::StampedTransform optical_transform;
  tf::StampedTransform world_camera_transform;


  // name of the frames of the camera
  std::string camera_link;
  std::string camera_rgb_optical_frame;  
  ros::NodeHandle private_node("~");
  private_node.param<std::string>("camera_link", camera_link, "/camera_link");

  while(ros::ok())
  {
      while(true){
         try {
            listener.waitForTransform("/world", "/r200_camera_link", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/world", "/r200_camera_link", ros::Time(0), world_camera_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for world->r200_camera_link");
      }
  br.sendTransform(tf::StampedTransform(world_camera_transform, ros::Time::now(), "/world", camera_link));
 }
  return 0;
};
