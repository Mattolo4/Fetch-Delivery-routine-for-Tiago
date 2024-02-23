#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "assignment_2_group_12/getTagsPose.h"

using namespace std;
using namespace ros;

//function to tilt tiago's head in the desired inclination
void moveHead(float tilt, NodeHandle n){
    Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1000);
    while (pub.getNumSubscribers() < 1) {
      //ROS_INFO("Waiting for subscribers...");
    }
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = Time(0);
    msg.header.frame_id = "";

    //Set joint names
    msg.joint_names.push_back("head_1_joint");
    msg.joint_names.push_back("head_2_joint");

    //create trajectory points
    trajectory_msgs::JointTrajectoryPoint points;
    points.positions.push_back(0.0);  //head_1_joint
    points.positions.push_back(tilt); //head_2_joint
    points.time_from_start = Duration(1.0); 

    msg.points.push_back(points);

    pub.publish(msg);

    return;
}

apriltag_ros::AprilTagDetectionArray convertDetectTags(apriltag_ros::AprilTagDetectionArray::ConstPtr tags){
  //saves the apriltag detections into an array
  vector<apriltag_ros::AprilTagDetection> detections = tags->detections;

  //create new detection array for the new frame of reference
  apriltag_ros::AprilTagDetectionArray detectedTags;
  detectedTags.header.frame_id = "map";


  //initialize the TF2 Buffer and Listener
  tf2_ros::Buffer  tfBuffer(Duration(10.0)); 
  tf2_ros::TransformListener tfListener(tfBuffer);
  Duration(1.0).sleep();
      
  if(tags){
    for (const auto& detection : detections){
      //create the new detected tag with different frame
      apriltag_ros::AprilTagDetection newDetectedTag;
      newDetectedTag.id=detection.id;
      newDetectedTag.size=detection.size;

      const geometry_msgs::PoseWithCovarianceStamped& tag_pose = detection.pose;
      //create a PoseStamped message for the new transformed pose
      geometry_msgs::PoseWithCovarianceStamped base_pose;
      base_pose.header.frame_id = "map";
      base_pose.pose.pose.orientation.w = 1.0; 

      try{
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "xtion_rgb_optical_frame", Time(0));
        tf2::doTransform(tag_pose.pose.pose, base_pose.pose.pose, transformStamped);

        //Prints out the old and new poses
        ROS_INFO("Tag ID: %d", detection.id[0]);
        ROS_INFO("Original Pose in %s frame: x=%f, y=%f, z=%f", "xtion_rgb_optical_frame", tag_pose.pose.pose.position.x, tag_pose.pose.pose.position.y, tag_pose.pose.pose.position.z);
        ROS_INFO("Transformed Pose in %s frame: x=%f, y=%f, z=%f", "map", base_pose.pose.pose.position.x, base_pose.pose.pose.position.y, base_pose.pose.pose.position.z);

        //save the new PoseWithCovarianceStamperd with new frame
        newDetectedTag.pose = base_pose;
      }
      catch (tf2::TransformException& ex){
        ROS_WARN("Failed to transform pose to %s frame: %s", "map", ex.what());
        //ROS_WARN("%s",ex.what());
        Duration(0.1).sleep();
        continue;
      }

      //push back the detected tag with new reference frame
      detectedTags.detections.push_back(newDetectedTag);
    }
  }

  return detectedTags;
}

// callback function that obtains the apriltag detections and convertes them into the desired reference frame map
bool getTagsPoseCallback(assignment_2_group_12::getTagsPose::Request& req, assignment_2_group_12::getTagsPose::Response& res) {
    
    //obtain detected tags array
    apriltag_ros::AprilTagDetectionArray::ConstPtr tags = topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
    //convert the detected tag array from camera frame t map frame;
    apriltag_ros::AprilTagDetectionArray baseFrame_tags = convertDetectTags(tags);
    res.tagsPose = baseFrame_tags;

    return true;
}


int main(int argc, char **argv){

    init(argc, argv, "node_b");
    NodeHandle n;
    //tilt head once it has arrived in pick location
    moveHead(-0.6,n);

    ServiceServer service = n.advertiseService("get_tags_pose", getTagsPoseCallback);

    ROS_INFO("Service for apriltag detection ready");
    
    //tilt head back
    moveHead(0.6,n);
    spin();
    return 0;
}
