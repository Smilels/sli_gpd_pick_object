#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PointStamped> ("object_cloud", 1);
	tf::TransformListener *tf_listener(new tf::TransformListener);
  tf::TransformBroadcaster *tf_pub(new tf::TransformBroadcaster);
  tf::StampedTransform transform;
  try{
    tf_listener->waitForTransform("/r200_camera_link","/table_top", ros::Time::now(),ros::Duration(5.0));
    tf_listener->lookupTransform ("/r200_camera_link","/table_top", ros::Time(0), transform);
  }
  catch(std::runtime_error &e){
    std::cout<<"tf listener error"<<std::endl;
    return 1;
  }

  Eigen::Affine3d grasp_pose, observation_pose,trans;

  geometry_msgs::PoseStamped target2;
  //target2.header.frame_id="table_top";??can't work when set the pose with respenct to "table_top"
  target2.header.frame_id="/world";
  target2.header.stamp=ros::Time::now();
  target2.pose.orientation.w=1;
  target2.pose.position.x=0.957917615558;// 0.371849018129
  target2.pose.position.y=0.566543526334;// 0.298981806364
  target2.pose.position.z= 1.93391489751;//1.80035830512
  tf::poseMsgToEigen(target2.pose, grasp_pose);
  observation_pose= grasp_pose* Eigen::Translation<double,3>(-0.50,0,0);
//	tf::Vector3 cam_=tf::Vector3 (0,0,1);
//	tf::Matrix3x3 uptf;
//	uptf.setRotation(transform.inverse().getRotation());
  //  tf::Vector3 cam_z=uptf*tf::Vector3 (0,0,1);

	//vector under different frames,but vector never change
  //  std::cout<<"ang angle is "<< cam_z.getX()<<cam_z.getY()<<cam_z.getZ()<<std::endl;
geometry_msgs::PointStamped point;

Eigen::Matrix3d frame_rot;
frame_rot <<  -0.075051,  -0.995969,  0.0491332,
 -0.993939,  0.0707469, -0.0841457,
 0.0803305, -0.0551506,  -0.995241;
while(ros::ok()){
 tf::Matrix3x3 uptf;
 uptf.setRotation(transform.inverse().getRotation());
 Eigen::Matrix3d trans;
 tf::matrixTFToEigen(uptf,trans);
 // for(int a=0; a<3; a++)
 //         for(int b=0; b<3; b++)
 //           trans(a,b) = uptf[a][b];
 Eigen::Matrix3d val_frame=trans*frame_rot;


 Eigen::Quaternion<double> rot_quaternion1 = Eigen::Quaternion<double>(val_frame);

    geometry_msgs::Quaternion tag_pose1;
     tag_pose1.x = rot_quaternion1.x();
     tag_pose1.y = rot_quaternion1.y();
     tag_pose1.z = rot_quaternion1.z();
     tag_pose1.w = rot_quaternion1.w();
     tf::Quaternion ee1;
   tf::quaternionMsgToTF(tag_pose1,ee1);
   tf::StampedTransform transform2;
  transform2.setOrigin(tf::Vector3(0,0,0));
 transform2.setRotation( ee1 );
 tf_pub->sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "r200_camera_link", "/object1"));

frame_rot=trans.inverse()*val_frame;
Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(frame_rot);
 geometry_msgs::Quaternion tag_pose;
  tag_pose.x = rot_quaternion.x();
  tag_pose.y = rot_quaternion.y();
  tag_pose.z = rot_quaternion.z();
  tag_pose.w = rot_quaternion.w();
  tf::Quaternion ee;
 tf::quaternionMsgToTF(tag_pose,ee);
 tf::StampedTransform transform1;
 transform1.setOrigin(tf::Vector3(0,0,0));
 transform1.setRotation( ee );
 tf_pub->sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "/object"));

 //calculate the angle between upright direction and approach direction
 tf::Vector3 cam_approch;
 tf::vectorEigenToTF(frame_rot.col(0),cam_approch);
 tf::Vector3 cam_z=tf::Vector3 (0,0,1);
 tfScalar up_angle=cam_approch.angle (cam_z);
 std::cout<< "The angle between approach direction and upright direction is " << up_angle*180/M_PI << std::endl;
 if (up_angle*180/M_PI<90)
 {
   Eigen::Matrix3d frame_mat;
   frame_mat=frame_rot;
   frame_mat.col(0)<<frame_rot.col(0)[0],frame_rot.col(0)[1],0;
   std::cout<<"val_frame is "<< frame_mat.col(0)<<std::endl;
   frame_mat.col(2)=frame_mat.col(0).cross(frame_mat.col(1));
   //frame transfer back

   Eigen::Quaternion<double> rot_quaternion2 = Eigen::Quaternion<double>(frame_mat);
   geometry_msgs::Quaternion tag_pose2;
   tag_pose2.x = rot_quaternion2.x();
   tag_pose2.y = rot_quaternion2.y();
   tag_pose2.z = rot_quaternion2.z();
   tag_pose2.w = rot_quaternion2.w();
   tf::Quaternion ee2;
  tf::quaternionMsgToTF(tag_pose2,ee2);
  tf::StampedTransform transform3;
  transform3.setOrigin(tf::Vector3(0,0,0));
  transform3.setRotation( ee2 );
  tf_pub->sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "/object2"));
}
	// Eigen::Vector3d yyy;
	// yyy<<  -0.075051,-0.993939,0.0803305;
	// Eigen::Matrix3d m;
	// Eigen::Vector3d yy;
	// yy<<0,0,1;

  Eigen::Vector3d yyy;
  yyy=val_frame.col(0);
  Eigen::Vector3d yy;
  yy<<0,0,1;

  double dot_rot=yyy.dot(yy);
  double lenSq1 = yyy[0]*yyy[0] + yyy[1]*yyy[1] + yyy[2]*yyy[2];
  double lenSq2 =yy[0]*yy[0] +yy[1]*yy[1] + yy[2]*yy[2];
  double dot_angle=std::acos(dot_rot/sqrt(lenSq1*lenSq2));




// 	yy<<yyy[0],yyy[1],0;
//     Eigen::Vector3d rot_nor=yy.cross(yyy);
// 	std::cout<< "rot_nor is " << rot_nor<< std::endl;
// 	std::cout<< "rot_nor is " <<frame_rot.col(1)<< std::endl;
// 	double dot_rot1=yyy.dot(yy);
// 	double len1 = yyy[0]*yyy[0] + yyy[1]*yyy[1] + yyy[2]*yyy[2];
//  	double len2 =yy[0]*yy[0] +yy[1]*yy[1] + yy[2]*yy[2];
//  	double par_angle=std::acos(dot_rot1/sqrt(len1*len2));
// 	std::cout<< "The y angle is " << par_angle*180/M_PI << std::endl;
//
//    //Eigen::Vector3d rot_norm=rot_nor.normalized();
//    Eigen::AngleAxisd rotation_vector(par_angle,rot_nor);
//    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
//    rotation_matrix = rotation_vector.toRotationMatrix();
//    frame_rot.noalias() =  rotation_matrix*frame_rot;
//    std::cout<< "frame_rot is " << frame_rot << std::endl;
//
//     yyy=frame_rot.col(0);
//     dot_rot=yyy.dot(yy);
//     lenSq1 = yyy[0]*yyy[0] + yyy[1]*yyy[1] + yyy[2]*yyy[2];
//     lenSq2 =yy[0]*yy[0] +yy[1]*yy[1] + yy[2]*yy[2];
//     dot_angle=std::acos(dot_rot/sqrt(lenSq1*lenSq2));
//     std::cout<< "The angle between approach direction and upright direction is " << dot_angle*180/M_PI << std::endl;
//
// Eigen::Vector3d rot_normal=yy.cross(yyy);
// m = Eigen::AngleAxisd(M_PI/6, yy);
// //std::cout<<m<<std::endl;
// m = Eigen::AngleAxisd(M_PI/6, yy)*frame_rot;
//std::cout<<m<<std::endl;
    //while (1){
    //tf::vectorEigenToTF(m.col(1),cam_z);
	//tf::Vector3 cam_=tf::Vector3 (0,1,0);
	//tfScalar up_angle=cam_z.angle (cam_);
  //  std::cout<<"ang angle is "<< up_angle*180/M_PI<<" ang rad is "<< up_angle<<std::endl;

	//point.point.x=cam_z[0];
	//point.point.y=cam_z[1];
	//point.point.z=cam_z[2];
	//point.header.frame_id= "r200_camera_link";
	//pub.publish(point);

 //tfScalar approach_ang=M_PI-up_angle;
  // tf::Vector3 cam_y=uptf*tf::Vector3 (0,1,0);

//Eigen::vector3d rot_normal=m.col(1).cross(yyy);
//Eigen::Matrix3d m;
//m = AngleAxisf(approach_ang, rot_normal)*m;

  //}
}
    return 0;
}
