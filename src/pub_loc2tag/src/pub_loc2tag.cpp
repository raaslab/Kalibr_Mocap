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
#include <string>


std::string turtle_name;

void apr_Callback(const apriltags::AprilTagDetections& msg){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform locpose2tag;
  apriltags::AprilTagDetections apriltag_detections = msg;
  apriltags::AprilTagDetection detectio1;
//  geometry_msgs::PoseStamped tag_pose;
  static tf::TransformListener listener;
  tf::StampedTransform local_pose2tag;

  char child_frameid[20];


  if(!apriltag_detections.detections.empty()){
	  //!!check the size of detections
	  //ROS_INFO("Posex: %f", apriltag_detections.detections[0].pose.position.x);
	  for(unsigned int i = 0; i < apriltag_detections.detections.size(); ++i){
		  detectio1 = apriltag_detections.detections[i];
		  sprintf(child_frameid, "tag-%d", detectio1.id);

	//	  tag_pose.pose = detectio1.pose;
	//	  tag_pose.header = detectio1.header;
	//	  tf::Stamped<tf::Transform> tag_transform;
	//	  tf::poseStampedMsgToTF(tag_pose, tag_transform);
	//	  sprintf(child_frameid, "tag-%d", detectio1.id);
	//	  br.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, child_frameid));

		std::cout << "now time " << ros::Time::now()<< "stamp time " << detectio1.header.stamp << std::endl;
		try{
		  listener.waitForTransform("local_origin", detectio1.header.stamp, child_frameid, detectio1.header.stamp,
				  "local_origin", ros::Duration(3.0));
		  listener.lookupTransform("local_origin", detectio1.header.stamp, child_frameid, detectio1.header.stamp,
				  "local_origin",local_pose2tag);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		//  ros::Duration(1.0).sleep();
		//  continue;
		}
	    std::string id_str = std::to_string(detectio1.id);
		char *loc2tag_frame_id_ = (char *)malloc(sizeof(char)*(std::strlen("loc2tag-") + std::strlen(id_str.c_str())));
		sprintf(loc2tag_frame_id_, "loc2tag-%d", detectio1.id);
		std::string str_loc2tag_frame_id_(loc2tag_frame_id_);
		local_pose2tag.child_frame_id_ = str_loc2tag_frame_id_;


		//ROS_INFO("tf_local_pose2tag %d . %d", local_pose2tag.stamp_.sec, local_pose2tag.stamp_.nsec);
		br.sendTransform(local_pose2tag);

	  }

  }

}




int main(int argc, char** argv){
  ros::init(argc, argv, "pub_loc2tag_main");

  ros::NodeHandle node;
  
  ros::Subscriber apr_sub = node.subscribe("/apriltags/detections", 1, &apr_Callback); //pose of tag wrt cam

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
