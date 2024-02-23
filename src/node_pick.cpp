#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment_2_group_12/pickAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "assignment_2_group_12/getTagsPose.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "assignment_2_group_12/getBarrelsPos.h"
#include <moveit/planning_interface/planning_interface.h>
#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <ros/service_client.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;
using namespace ros;
using namespace actionlib;


/////////////////////////////
typedef SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;
typedef SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;

//TODO
Publisher gripper_pub;
Publisher joint_pub;

tf2_ros::Buffer tfBuffer;

//Values for the gripper
vector<double> open_gripper_joint_values {0.04, 0.04};
vector<double> closed_gripper_joint_values {0.00, 0.00};

//Waypoint values for the joints in the pick phase
vector<float> waypoint_joint_green =  {1.30, 1.02, -0.14, 0.41, -1.57, 1.37, 0.00};
vector<float> waypoint_joint_blue = {2.44, 1.02, -0.04, 0.41, -1.57, 1.37, 0.00};
vector<float> waypoint_joint_red = {0.70, 1.02, -0.04, 0.85, -1.57, 1.37, 0.00};

//Safe Tiago joints orientations	
vector<float> safe_joint_angles{0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00};
////////////////////////////


void moveHead(float tilt, NodeHandle n){
    Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1000);
    while (pub.getNumSubscribers() < 1) {
      //ROS_INFO("Waiting for subscribers...");
    }
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = Time(0);
    msg.header.frame_id = "";

    // Set joint names
    msg.joint_names.push_back("head_1_joint");
    msg.joint_names.push_back("head_2_joint");

    //create trajectory points
    trajectory_msgs::JointTrajectoryPoint points;
    points.positions.push_back(0.0);  // Replace with the desired position for head_1_joint
    points.positions.push_back(tilt); // Replace with the desired position for head_2_joint
    points.time_from_start = Duration(1.0); 

    msg.points.push_back(points);

    pub.publish(msg);

    Duration(2.0).sleep();
    spinOnce();

    return;
}

moveit_msgs::CollisionObject getTable(){
    moveit_msgs::CollisionObject table;
    table.id = "pick_table";
    table.header.frame_id = "map";
    //convert from world to map by adding these coords
    float world_X = 6.58580978;
    float world_Y = -1.369998;

    //create a pose for the collision object
    geometry_msgs::Pose pose;
    pose.position.x = 7.7; //7.82663669;  
    pose.position.y = -2.96;//-2.75017069;  
    pose.position.z = 0.755;  
    pose.orientation.x = 0.0;  
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    //create a collision solid
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    
    //////////////////////////////////////////////////////////
    float padding_x,padding_y, padding_z;
    /*
    padding_x = 0.08;
    padding_y = 0.08;
    padding_z = 0.09;
    */
    
    padding_x = 0.05;
    padding_y = 0.05;
    padding_z = 0.06;
    
    box.dimensions.resize(3);
    box.dimensions[0] = 0.95 + padding_x;  
    box.dimensions[1] = 0.95 + padding_y;
    box.dimensions[2] = 0.04 + padding_z; //0.04
//////////////////////////////////////////////////////////////

    
    table.primitives.push_back(box);
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::CollisionObject::ADD;
    return table;
}

vector<moveit_msgs::CollisionObject> createCollisionObjects(vector<apriltag_ros::AprilTagDetection> detections, const int pickID){
  vector<moveit_msgs::CollisionObject> collision_objects;

  //----------------------table------------------------------
    
  collision_objects.push_back(getTable());

  //----------------------objects------------------------------
  shape_msgs::SolidPrimitive goldCylinder;
  goldCylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  goldCylinder.dimensions.resize(2);
  
  float extra_h = 0.09;
  float extra_r = 0.04;
  goldCylinder.dimensions[0] = 0.15 + extra_h;  //height
  goldCylinder.dimensions[1] = 0.05 + extra_r; //radius
  
for(const auto& detection : detections){
    if(detection.id[0] != pickID){ //not the object to pick
        moveit_msgs::CollisionObject object;
        string objectID = "object " + to_string(detection.id[0]);
        object.id = objectID;
        object.header.frame_id = "map";

        object.primitives.push_back(goldCylinder);

        geometry_msgs::Pose shiftedObjPose;
        shiftedObjPose = detection.pose.pose.pose;
        shiftedObjPose.position.z =  shiftedObjPose.position.z - (goldCylinder.dimensions[0]/2.0) +0.01;

        object.primitive_poses.push_back(shiftedObjPose);
        object.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.push_back(object);

        ROS_INFO("Added collision object: %s", objectID.c_str());
    }
}


  //-----------------------------------------------------------
  return collision_objects;
  
}

moveit_msgs::CollisionObject createPlaceObj(vector<apriltag_ros::AprilTagDetection> detections,const int pickID){
    moveit_msgs::CollisionObject pickOBJ;
    pickOBJ.id = "pickOBJ";
    pickOBJ.header.frame_id = "map";
    int detection_idx;
    int index = 0;

    for(const auto& detection : detections){
    	 //changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if(detection.id[0] == pickID){
            cout << "\n detection id : " << detection.id[0] << "\n";
            detection_idx = index;
        }
        index ++;
    }
    cout << "\n detection index : " << detection_idx << "\n";
    shape_msgs::SolidPrimitive obj;


    if(pickID==2 || pickID == 3){
        obj.type = shape_msgs::SolidPrimitive::BOX;
        obj.dimensions.resize(3);
        obj.dimensions[0] = 0.05;  
        obj.dimensions[1] = 0.05;
        obj.dimensions[2] = 0.05;
    }

    else if(pickID==1){
        obj.type = shape_msgs::SolidPrimitive::CYLINDER;
        obj.dimensions.resize(2);
        obj.dimensions[0] = 0.1;  //height
        obj.dimensions[1] = 0.04;
    }

    //create a pose for the collision object
    geometry_msgs::Pose pose;
    pose = detections[detection_idx].pose.pose.pose;
    pose.position.z -= 0.01; 

    
    pickOBJ.primitives.push_back(obj);
    pickOBJ.primitive_poses.push_back(pose);
    pickOBJ.operation = moveit_msgs::CollisionObject::ADD;
    return pickOBJ;
}


////////////////////////////////

	//Set a default configuration to the joints of the arm
	void configureJoints(const vector<float> joints)
	{
        moveit::planning_interface::MoveGroupInterface group_arm("arm");

        group_arm.setPlannerId("SBLkConfigDefault");

		//Set the value to each joint
        map<string, float> target_position;
        target_position["arm_1_joint"] = joints[0];
        target_position["arm_2_joint"] = joints[1];
        target_position["arm_3_joint"] = joints[2];
        target_position["arm_4_joint"] = joints[3];
        target_position["arm_5_joint"] = joints[4];
        target_position["arm_6_joint"] = joints[5];
        target_position["arm_7_joint"] = joints[6];

        vector<string> arm_joint_names;
        arm_joint_names = group_arm.getJoints();

        group_arm.setStartStateToCurrentState();
        group_arm.setMaxVelocityScalingFactor(0.2);
        for (int i = 0; i < arm_joint_names.size(); ++i)
            group_arm.setJointValueTarget(arm_joint_names[i], target_position[arm_joint_names[i]]);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        group_arm.setPlanningTime(1000.0);
		//Set the plan
        bool success = static_cast<bool>(group_arm.plan(my_plan));
        if (!success)           
            ROS_INFO("No plan found for joints");
            
        group_arm.move();	
	}

	//Set a specific configuration for the gripper, it's used to close the finger in PICK and open the finger in PLACE, now in node_place, using action client
	void setGripper(vector<double>& joint_values)
	{
        gripper_control_client gripper_client("/gripper_controller/follow_joint_trajectory", true);
        
        while(!gripper_client.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the gripper action server to come up");

        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

        
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = joint_values;
        point.time_from_start = ros::Duration(1.0);

        goal.trajectory.points.push_back(point);
        goal.goal_tolerance.resize(2);
        for (int i = 0; i < 2; ++i)
        {
            goal.goal_tolerance[i].name = goal.trajectory.joint_names[i];
            goal.goal_tolerance[i].position = 0.0;
            goal.goal_tolerance[i].velocity = 0.1;
            goal.goal_tolerance[i].acceleration = 0.1;
        }

        gripper_client.sendGoal(goal);
 
        bool finished_before_timeout = gripper_client.waitForResult(ros::Duration(5.0));
        if (finished_before_timeout)
            actionlib::SimpleClientGoalState state = gripper_client.getState();
        else
            ROS_INFO("The action for the gripper is not over before the timeout..");
    }

	//Set the gripper as OPEN 0.04 for each finger, it's used in PICK
	void openGripper(trajectory_msgs::JointTrajectory& grip)
	{
        grip.joint_names.resize(2);
        grip.joint_names[0] = "gripper_left_finger_joint";
        grip.joint_names[1] = "gripper_right_finger_joint";
        grip.points.resize(1);
        grip.points[0].positions.resize(2);
        grip.points[0].positions[0] = 0.04;
        grip.points[0].positions[1] = 0.04;
        grip.points[0].time_from_start = ros::Duration(0.5);
    }
	
	//Pick for green and red object
	void pick(const assignment_2_group_12::pickGoalConstPtr &goal, const geometry_msgs::PoseStamped& obj) 
	{
        //Create a move group for Tiago and set the end effector's frame
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setEndEffectorLink("gripper_grasping_frame");

        //Set the grasp pose in according to the object_pose
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        
        geometry_msgs::PoseStamped object_pose = obj;
        
        
        tf2::Quaternion orientation(object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w);
        float tau = 2 * M_PI;
        grasps[0].grasp_pose.header.frame_id = "map";

        tf2::Matrix3x3 mat(orientation);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        grasps[0].grasp_pose.pose.position.x = object_pose.pose.position.x;
        grasps[0].grasp_pose.pose.position.y = object_pose.pose.position.y;
        grasps[0].grasp_pose.pose.position.z = object_pose.pose.position.z;


		//By removing the comment and correcting the pick_phase function in the if-else accordingly, the pick of the blue object is faster 
		//and with better positioning of the end-effector.
		//However, it was commented out because even though it is defined it does not always account for collision objects
		/*
        //Blue lateral pick
        if(goal->obj_id == 1)
        {
            grasps[0].grasp_pose.pose.position.y += 0.2;
            orientation.setRPY(roll -tau/4, pitch, -tau/4);
        }
        */
        
        
        if(goal->obj_id==2)
        {
			grasps[0].grasp_pose.pose.position.z += 0.2;
			orientation.setRPY(roll, pitch + tau / 4, yaw - (45 * M_PI / 180));
		}
		else{
        
        grasps[0].grasp_pose.pose.position.z += 0.2;
        orientation.setRPY(roll, pitch + tau/4, yaw);
        }

        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

        //Set the approach
        grasps[0].pre_grasp_approach.direction.header.frame_id = "map";

        grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.1;
        grasps[0].pre_grasp_approach.desired_distance = 0.15;

        //Set the gripper to open
        openGripper(grasps[0].pre_grasp_posture);

        //Set support surface as table.
        group_arm_torso.setSupportSurfaceName("pick_table");
        
        //Call "native" pick for grasp object
        string obj_name = "obj_id";
        obj_name += std::to_string(goal->obj_id);
        group_arm_torso.pick(obj_name, grasps);

    }
	
	//Pick for blue object
	void appro_pick_blue(const assignment_2_group_12::pickGoalConstPtr &goal, const geometry_msgs::PoseStamped& object_pose)
	{
		moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
		group_arm_torso.setPoseReferenceFrame("map");
        group_arm_torso.setEndEffectorLink("gripper_grasping_frame");
        
        geometry_msgs::PoseStamped obj = object_pose;
        
        //Set the orientation
        double tau = 2 * M_PI;
		tf2::Quaternion orientation(object_pose.pose.orientation.x,object_pose.pose.orientation.y,object_pose.pose.orientation.z,object_pose.pose.orientation.w);
		tf2::Matrix3x3 mat(orientation);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);
		
        obj.pose.position.z += 0.10;
        orientation.setRPY(roll, pitch + tau/4, yaw);
        obj.pose.orientation = tf2::toMsg(orientation);
        
        geometry_msgs::PoseStamped approach_pose = obj;
		approach_pose.pose.position.z += 0.10;
		
		group_arm_torso.setPoseTarget(approach_pose);

		//Set the plan 
		moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
		group_arm_torso.setPlanningTime(150.0);
		bool success_appro = static_cast<bool>(group_arm_torso.plan(approach_plan));
        if (!success_appro)           
            ROS_INFO("No plan found for joints");
            
        group_arm_torso.move();	
        
        ///// PICK PART  /////
		
		geometry_msgs::PoseStamped pick_pose = obj;
		group_arm_torso.setPoseTarget(pick_pose);
		
		//Set the plan
		moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
		group_arm_torso.setPlanningTime(150.0);
		bool success_pick = static_cast<bool>(group_arm_torso.plan(pick_plan));
        if (!success_pick)           
            ROS_INFO("No plan found for joints");
            
        group_arm_torso.move();	
	}
	
	//Set the height of the torso using the action client
	void torso(float height)
	{
        torso_control_client torso_client("/torso_controller/follow_joint_trajectory", true);

        while(!torso_client.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the torso action server to come up");

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();

        goal.trajectory.joint_names.push_back("torso_lift_joint");

        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions.push_back(height);
        goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

        torso_client.sendGoal(goal);

        bool finished_before_timeout = torso_client.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout)
            actionlib::SimpleClientGoalState state = torso_client.getState();
        else
            ROS_INFO("The action for the torso is not over before the timeout.");
    }

	//Attach the object to the gripper
	void attach(const assignment_2_group_12::pickGoalConstPtr &goal, NodeHandle n)
	{
        //Service client for the attach
        ServiceClient attach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

        //Request
        gazebo_ros_link_attacher::AttachRequest req;
        if (goal->obj_id == 1){
            req.model_name_1 = "Hexagon";
            req.link_name_1 = "Hexagon_link";
        }
        else if (goal->obj_id == 2){
            req.model_name_1 = "Triangle";
            req.link_name_1 = "Triangle_link";
        }
        else if (goal->obj_id == 3){
            req.model_name_1 = "cube";
            req.link_name_1 = "cube_link";
        }
        req.model_name_2 = "tiago";
        req.link_name_2 = "arm_7_link";

        gazebo_ros_link_attacher::AttachResponse resp;
        if (attach_srv.call(req, resp))
            ROS_INFO("Attachment successful");
        else
            ROS_ERROR("Failed to attach");

	}

	//Start the approach sequence: configureJoints, pick
	void approach_phase(const assignment_2_group_12::pickGoalConstPtr &goal, const geometry_msgs::PoseStamped& object_pose)
	{
		int id = goal->obj_id;
		
		ROS_INFO("Start approach phase for object with ID:%d",id);
		
		//Set the configuration for the specific object given the ID
        switch (id) 
        {
			case 1:
				configureJoints(waypoint_joint_blue);
				break;
			case 2:
				configureJoints(waypoint_joint_green);
				break;
			case 3:
				configureJoints(waypoint_joint_red);
				break;
		}
		ROS_INFO("Approach phase for the object with ID:%d completed",id);
		
		
		//ROS_INFO("INIZIO DELLA PICK");
		//Pick the object
		//pick(goal, object_pose);
		/*
		if(id == 2 || id ==3)
			pick(goal, object_pose);
		else
			appro_blue(goal, object_pose);
		*/
	}
	
	//Start the pick sequence: setGripper, attach, configureJoints, configureJoint, torso
	void pick_phase(const assignment_2_group_12::pickGoalConstPtr &goal, geometry_msgs::PoseStamped& object_pose, NodeHandle n)
	{
		int id = goal->obj_id;
		
		ROS_INFO("Start pick phase for object with ID:%d",id);
		
		if(id == 2 || id ==3)
			pick(goal, object_pose);
		else
		{
			appro_pick_blue(goal, object_pose);
		}
		
		//Attach the obj to the gripper
		attach(goal,n);
		
		//Close the gripper
		setGripper(closed_gripper_joint_values);

		//Set the configuration for the specific object given the ID
        switch (id) 
        {
			case 1:
				configureJoints(waypoint_joint_blue);
				break;
			case 2:
				configureJoints(waypoint_joint_green);
				break;
			case 3:
				configureJoints(waypoint_joint_red);
				break;
		}

		//Set the configuration for the specific object given the ID, safe configuration 
		configureJoints(safe_joint_angles);

		//Set torso, std value
        torso(0.15);
        
        ROS_INFO("pick phase for the object with ID:%d completed",id);
	}
	
	//Do all the pick sequence, not used
	void doPick(const assignment_2_group_12::pickGoalConstPtr &goal, const geometry_msgs::PoseStamped& object_pose, NodeHandle n)
	{
		int id = goal->obj_id;
		
		ROS_INFO("Start pick phase for object with ID:%d",id);
		
		//Set the configuration for the specific object given the ID
        switch (id) 
        {
			case 1:
				configureJoints(waypoint_joint_blue);
				break;
			case 2:
				configureJoints(waypoint_joint_green);
				break;
			case 3:
				configureJoints(waypoint_joint_red);
				break;
		}
		
		//Pick the object
		pick(goal, object_pose);
		
		if(id == 2 || id ==3)
			pick(goal, object_pose);
		else
			appro_pick_blue(goal, object_pose);
		
		//Close the gripper
		setGripper(closed_gripper_joint_values);

		//Attach the obj to the gripper
		attach(goal,n);

		//Set the configuration for the specific object given the ID
        switch (id) 
        {
			case 1:
				configureJoints(waypoint_joint_blue);
				break;
			case 2:
				configureJoints(waypoint_joint_green);
				break;
			case 3:
				configureJoints(waypoint_joint_red);
				break;
		}

		//Set the configuration for the specific object given the ID, safe configuration 
		configureJoints(safe_joint_angles);

		//Set torso, std value
        torso(0.15);
        
        ROS_INFO("pick phase for the object with ID:%d completed",id);
		
	}

/////////////////////////////////



class pickAction{

    // Declaring Action Server variables
    protected:
        NodeHandle n;
        SimpleActionServer<assignment_2_group_12::pickAction> as_;
        string action_name;
        assignment_2_group_12::pickFeedback feedback_;
        assignment_2_group_12::pickResult result_;

    public:
        pickAction(string name): as_(n, name, boost::bind(&pickAction::execute, this, _1), false), action_name(name){
            as_.start();
        }

    ~pickAction(void){}


	void execute(const assignment_2_group_12::pickGoalConstPtr &goal){
        NodeHandle n;
        //tilt head once it has arrived in pick location
        cout << "Moving head" << endl;
        moveHead(-0.6,n);
        geometry_msgs::PoseStamped object_pose;

        ServiceClient client = n.serviceClient<assignment_2_group_12::getTagsPose>("get_tags_pose");

        assignment_2_group_12::getTagsPose srv;

        //create planning interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        sleep(1.0);
        // Call the service to compute coordinates
        if (client.call(srv)) {
            // Process the received AprilTagDetectionArray
            vector<apriltag_ros::AprilTagDetection> detections = srv.response.tagsPose.detections;
            for (const auto& detection : detections){
                ROS_INFO("Tag ID: %d", detection.id[0]);
                ROS_INFO("Transformed Pose in %s frame: x=%f, y=%f, z=%f", "map", detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
				if(detection.id[0] == goal->obj_id)
					object_pose.pose = detection.pose.pose.pose;
            }

            int pickID = goal->obj_id; //ID of object to be picked!

            vector<string> object_ids;
            object_ids = planning_scene_interface.getKnownObjectNames(false);
            if(object_ids.size()>0) {
                planning_scene_interface.removeCollisionObjects(object_ids);
            }


            // Add the collision object to the MoveIt! planning scene

            moveit_msgs::CollisionObject pickOBJ = createPlaceObj(detections, pickID);
            vector<moveit_msgs::CollisionObject> collisions = createCollisionObjects(detections, pickID);
            collisions.push_back(pickOBJ);


            planning_scene_interface.applyCollisionObjects(collisions);
            sleep(5.0);

            //----- start the appro phase -----
            object_pose.header.frame_id = "map";
			approach_phase(goal,object_pose);
            //----- end of appro phase, remove the obj to be picked-----
            
            vector<string> removeObj;
            removeObj.push_back("pickOBJ");
            planning_scene_interface.removeCollisionObjects(removeObj);
            sleep(0.5);

            //----- pick the obj -----
			pick_phase(goal,object_pose,n);
            
            ROS_INFO("Received AprilTag detections");
        } else {
            ROS_ERROR("Failed to call service");
        }
        //tilt head back
        moveHead(-0.2,n);

        vector<string> object_ids;
        object_ids = planning_scene_interface.getKnownObjectNames(false);
        if(object_ids.size()>0) {
            planning_scene_interface.removeCollisionObjects(object_ids);
        }
        sleep(0.5);


        result_.picked=true;
        as_.setSucceeded(result_);
            
    }
};




int main(int argc, char** argv){

    init(argc, argv, "node_pick");
    cout << "Starting the server.." << endl;

    pickAction pick("pick");

    spin();
    return 0;
}
