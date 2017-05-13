#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <apriltags/AprilTagDetections.h>
#include <boost/foreach.hpp>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <tf/transform_listener.h>
using namespace std;

std::string turtle_name;


//This is going to broadcast 2 transforms using br.sendTransform. 1st is from pose data "tag-*"(child_frameid)
//wrt to "pg_15291221"(tag_transform.frame_id_) and second is from lookuptransform "local2posetag"(tf of tag directly
//wrt local_origin) wrt to "local_origin". Mavros publishes "fcu" data wrt "local_origin". With current
//way of lookuptransform when you put fixed frame as "local_origin" then "local2posetag" and "tag-*" appear to be
//same with no lag

// updated comments: local2posetag doesn't make sense when we are able to detect at high frame rate but apr callback is 
// neccesary as we the personal robotics apriltag do not support tf publishing of aprs wrt camera. 

void apr_Callback(const apriltags::AprilTagDetections& msg){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform locpose2tag;
  apriltags::AprilTagDetections apriltag_detections = msg;
  apriltags::AprilTagDetection detectio1;
  geometry_msgs::PoseStamped tag_pose;
  static tf::TransformListener listener;
  tf::StampedTransform local_pose2tag;

  char child_frameid[20];


  if(!apriltag_detections.detections.empty()){
	  //!!check the size of detections
	  //ROS_INFO("Posex: %f", apriltag_detections.detections[0].pose.position.x);
	  for(unsigned int i = 0; i < apriltag_detections.detections.size(); ++i){
		  detectio1 = apriltag_detections.detections[i];
		  tag_pose.pose = detectio1.pose;
		  tag_pose.header = detectio1.header;
		  tf::Stamped<tf::Transform> tag_transform;
		  tf::poseStampedMsgToTF(tag_pose, tag_transform);
		  sprintf(child_frameid, "tag-%d", detectio1.id);
		  br.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, child_frameid));
/*
		try{
		  listener.waitForTransform("local_origin", tag_transform.stamp_, child_frameid, tag_transform.stamp_,
				  "local_origin", ros::Duration(3.0));
		  listener.lookupTransform("local_origin", tag_transform.stamp_, child_frameid, tag_transform.stamp_,
				  "local_origin",local_pose2tag);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		//  ros::Duration(1.0).sleep();
		//  continue;
		}
		local_pose2tag.child_frame_id_ = "local2posetag";


		ROS_INFO("tf_locpose2tag staamp %d . %d", local_pose2tag.stamp_.sec, local_pose2tag.stamp_.nsec);
		br.sendTransform(local_pose2tag);
*/
	  }

  }

}


//Here we convert pose from mavros to another tf.  "local_origin"(odom_transform.stamp_) to "fcu"(odom_mavros.child_frame_id)
void odom_Callback(const nav_msgs::Odometry& msg){

	static tf::TransformBroadcaster br;
	//tf::Transform transform;
	nav_msgs::Odometry odom_mavros = msg;
	geometry_msgs::PoseStamped mav_pose;
	char child_frameid[20];

	//ROS_INFO("inside odom");

	mav_pose.pose = odom_mavros.pose.pose;
	mav_pose.header = odom_mavros.header;
	tf::Stamped<tf::Transform> odom_transform;
	tf::poseStampedMsgToTF(mav_pose, odom_transform);
	br.sendTransform(tf::StampedTransform(odom_transform, odom_transform.stamp_, odom_transform.frame_id_,
			odom_mavros.child_frame_id));
}

//here we transmit the fixed frame "fcu" to "pg_15291221" please check the Kalibr file camchain-imucam-cam_imu5_back3.yaml for
//the reprojection data
void cam2imu_publish(const ros::TimerEvent& event)
{
	//ROS_INFO("Inside cam2imu");
	Eigen::Matrix4f Tm;
	static tf::TransformBroadcaster br;
	Tm <<   -0.00791611419753699, 0.999967939542828, 0.001206243128064114, -0.008567966125148742,
			0.9999677896055346, 0.007917699528254411, -0.00131521313972195, 0.1428987636922613,
			-0.0013247216440334443, 0.0011957928970889968, -0.9999984075946895, -0.06986153003210434,
	            0,          0,          0,       1;


	tf::Vector3 origin;
	origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

	//ROS_INFO("origin x:%f y:%f z:%f", Tm(0,3), Tm(1,3), Tm(2,3));

	tf::Matrix3x3 tf3d;
	tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
		static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
		static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	tf::Transform transform;
	transform.setOrigin(origin);
	transform.setRotation(tfqt);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "pg_15291221"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Subscriber apr_sub = node.subscribe("/apriltags/detections", 1, &apr_Callback); //pose of tag wrt cam
 // ros::Subscriber odom_sub = node.subscribe("/mavros/local_position/odom", 1, &odom_Callback); // pose of fcu wrt local_origin
  ros::Timer timer = node.createTimer(ros::Duration(0.01), &cam2imu_publish);
  bool use_sim_time = false;

  // ************** lines between these are only used to test --clock bag option and /use_sim_time is working ***************//
 /*
  ros::param::set("/use_sim_time", true);
  if (!node.getParam("/use_sim_time", use_sim_time))
    ROS_ERROR("did not get use_sim_time");

  if(use_sim_time){
	  ROS_INFO("use_sim_time");
  }

  if(ros::Time::isSimTime()){
	  ROS_INFO("inside is_sim_time");

  }
  if(ros::param::get("/use_sim_time", use_sim_time)){
	  ROS_INFO("inside ros.getparam");
  }
  ROS_INFO("use_sim_time is True");
  */
  // ************** lines between these are only used to test --clock bag option and /use_sim_time is working ***************//


  ros::spin();


  return 0;
};
