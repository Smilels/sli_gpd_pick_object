#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

class Bounding_Box_Preprocess{
  private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;
    ros::Publisher pointcloud_pub;
    tf::TransformListener *tf_listener;
  public:
    Bounding_Box_Preprocess(){
      tf_listener = new tf::TransformListener;
      pointcloud_sub=nh.subscribe("/kinect2/qhd/points",1,&Bounding_Box_Preprocess::cloud_callback,this);
      pointcloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/table_top_points",1);
    }
    void cloud_callback(const sensor_msgs::PointCloud2& msg)
    {
      PointCloud::Ptr cloud_in (new PointCloud);
      PointCloud::Ptr cloud_tf (new PointCloud);
      PointCloud::Ptr cloud_filtered (new PointCloud);
      pcl::fromROSMsg(msg,*cloud_in);
      tf::StampedTransform transform;
      try{
          tf_listener->waitForTransform("/table_top", msg.header.frame_id, msg.header.stamp, ros::Duration(5.0));
          tf_listener->lookupTransform ("/table_top", msg.header.frame_id, msg.header.stamp, transform);
        }
         catch(std::runtime_error &e){
           return;
         }
      pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
      cloud_tf->header.frame_id = "/table_top";

      pcl::CropBox<Point> box;
      box.setInputCloud(cloud_tf);
      //this is our region of interest
      box.setMin(Eigen::Vector4f(-0.35,-0.3,0.002,1.0));
      box.setMax(Eigen::Vector4f(0.4,0.3,0.25,1.0));
      box.filter (*cloud_filtered);

      try{
          tf_listener->waitForTransform(msg.header.frame_id, "/table_top", msg.header.stamp, ros::Duration(5.0));
          tf_listener->lookupTransform (msg.header.frame_id, "/table_top",msg.header.stamp, transform);
        }
         catch(std::runtime_error &e){
           return;
         }
      pcl_ros::transformPointCloud (*cloud_filtered, *cloud_filtered, transform);
      cloud_filtered->header.frame_id =msg.header.frame_id;

      sensor_msgs::PointCloud2 table_top_points;
      pcl::toROSMsg(*cloud_filtered,table_top_points);
      pointcloud_pub.publish(table_top_points);
    }

    ~Bounding_Box_Preprocess(){}
};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"bounding_box_preprocess");
  Bounding_Box_Preprocess bbp;
  ros::spin();
  return 0;
}
