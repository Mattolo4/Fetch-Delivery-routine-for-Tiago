#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment_2_group_12/placeAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include "assignment_2_group_12/getTagsPose.h"
#include <moveit/move_group_interface/move_group_interface.h>
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
vector<double> open_gripper_joint_values {0.06, 0.06};

//Safe Tiago joints orientations	
vector<float> safe_joint_angles{0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00};

//waypoint value for the joints before the place
vector<float> waypoint_joint_angles{1.30, 1.02, -0.14, 0.41, -1.57, 1.37, 0.00};

//Coordinates for the place position and deltas
float x_place = 0.8;
float y_place = 0.0;
float z_place = 1.0;
float delta2=0.2;
float delta1=0.1;
float delta13=0.13;
float delta05=0.05;

//Values for the place position
vector<float> blue_place = {x_place - delta2, y_place + delta1, z_place};
vector<float> green_place = {x_place - delta13, y_place + delta1, z_place};
vector<float> red_place = {x_place, y_place, z_place};

////////////////////////////

std::vector<moveit_msgs::CollisionObject> placeBarrelCollision(vector<float> placePos){

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    moveit_msgs::CollisionObject barrel;
    barrel.id = "barrel";
    barrel.header.frame_id = "map";

    //barrel pose
    geometry_msgs::Pose barrel0;
    barrel0.position.x = placePos[0];   
    barrel0.position.y = placePos[1];  
    barrel0.position.z = 0.345;  
    barrel0.orientation.x = 0.0;  
    barrel0.orientation.y = 0.0;
    barrel0.orientation.z = 0.0;
    barrel0.orientation.w = 1.0;

    //create a collision solid
    
    float extra_r = 0.07;
    float extra_h = 0.01;
    
    shape_msgs::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = 0.69 + extra_h;  //height
    cylinder.dimensions[1] = 0.22 + extra_r; //radius = 21

    barrel.primitives.push_back(cylinder);
    barrel.primitive_poses.push_back(barrel0);
    barrel.operation = moveit_msgs::CollisionObject::ADD;
    collision_objects.push_back(barrel);
    
    return collision_objects;
}


/////////////////////////////

	//Set an orientation roll,pitch,yaw
	geometry_msgs::Quaternion setOrientation(float roll, float pitch, float yaw)
    {
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(roll, pitch, yaw);
        myQuaternion = myQuaternion.normalize();
        return tf2::toMsg(myQuaternion);
    }

	//Set the 0, 2PI, 0 orientation 
	geometry_msgs::Quaternion downOrientation()
    {
        return setOrientation(0, M_PI_2, 0);
    }

	//Place function
	void place_actuator(geometry_msgs::PoseStamped position, geometry_msgs::Quaternion orientation)
    {
        moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
        geometry_msgs::PoseStamped goal_pose;

        //For the place we consider a new reference frame: base_footprint. we consider the distance from the robot to the table
        goal_pose.header.frame_id = "base_footprint";

		//Set the goal value 
        goal_pose.pose.position = position.pose.position;
        goal_pose.pose.orientation = orientation;

        move_group.setPoseTarget(goal_pose);
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setPlanningTime(5.0);

        //Set and act the plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
           move_group.move();
        } else 
        {
            ROS_INFO("Impossible to get a plan");
        }
    }

	//Set the values for the place function
	void place(float x, float y, float z, geometry_msgs::Quaternion orientation)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        place_actuator(pose, orientation);
    }

	//Detach the object from the gripper
	void detach(const assignment_2_group_12::placeGoalConstPtr &goal, NodeHandle n)
	{    
        //Service client for the detach
        ServiceClient detach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

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
        else if (goal->obj_id == 3) {
            req.model_name_1 = "cube";
            req.link_name_1 = "cube_link";
        }
        req.model_name_2 = "tiago";
        req.link_name_2 = "arm_7_link";

        gazebo_ros_link_attacher::AttachResponse resp;
        if (detach_srv.call(req, resp))
            ROS_INFO("Detachment successful");
        else
            ROS_ERROR("Failed to detach");

	}

	//Set a specific configuration for the gripper, it's used to close the finger in PICK and open the finger in PLACE, using action client
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

	//Set a default configuration to the joints of the arm
	void configureJoints(vector<float> joints)
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
		//Compute and act the plan
        bool success = static_cast<bool>(group_arm.plan(my_plan));
        if (!success)           
            ROS_INFO("No plan found for joints");
            
        group_arm.move();	
	}

	//Start the place sequence: configureJoints, place, setGripper, detach, configureJoints, configureJoints
	void place_phase(const assignment_2_group_12::placeGoalConstPtr &goal, vector<float>place_pose, NodeHandle n)
	{
		int id = goal->obj_id;
		
		ROS_INFO("Start place phase for object with ID:%d",id);
		
		//Set the configuration for the arm: initial configuration
		configureJoints(waypoint_joint_angles);
		
		//Place the object
		place(place_pose[0], place_pose[1], place_pose[2], downOrientation());
		
		//Open the gripper
		setGripper(open_gripper_joint_values);
		
		//Detach the object from the gripper
		detach(goal,n);
		
		//Return to initial configuration
		configureJoints(waypoint_joint_angles);
		
		//Set the safe configuration 
		configureJoints(safe_joint_angles);
		
		ROS_INFO("Place phase for the object with ID:%d completed",id);
	}

////////////////////////


class placeAction{

    // Declaring Action Server variables
    protected:
        NodeHandle nh_place;
        SimpleActionServer<assignment_2_group_12::placeAction> as_place;
        string action_name_place;
        assignment_2_group_12::placeFeedback feedback_place;
        assignment_2_group_12::placeResult result_place;

    public:
        placeAction(string name_place): as_place(nh_place, name_place, boost::bind(&placeAction::execute, this, _1), false), action_name_place(name_place){
            as_place.start();
        }

    ~placeAction(void){}


    void execute(const assignment_2_group_12::placeGoalConstPtr &goal){

        //-----creates a new planning_scene_interface for the place phase
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        sleep(0.5);
        vector<string> object_ids;
        object_ids = planning_scene_interface.getKnownObjectNames(false);
        ROS_INFO("gotten object names");
        if(object_ids.size()>0) {
            planning_scene_interface.removeCollisionObjects(object_ids);
            ROS_INFO("removed collision");
        }

        //-----obtain barrel table location to place object
        vector<float> placePos = goal->placePos;

        //-----places collision obj for the barrel
        std::vector<moveit_msgs::CollisionObject> barrels = placeBarrelCollision(placePos);
        planning_scene_interface.applyCollisionObjects(barrels);
        sleep(1.0);


        
        //Takes the obj_id passed as a goal
        int obj_id = goal->obj_id;

        /*
		float x = barrels[0].primitive_poses[0].position.x;
		float y = barrels[0].primitive_poses[0].position.y;
		float z = 0.8;//0.69;
		
		geometry_msgs::PoseStamped object_pose;
		object_pose.header.frame_id = "map";
		object_pose.pose.position.x = x;
		object_pose.pose.position.y = y;
		object_pose.pose.position.z = z;
		*/
		
		//The phase changes slightly in the x and y coordinates in each table
		switch(obj_id)
		{
				case 1:
				place_phase(goal,blue_place,nh_place);
				break;
				
				case 2:
				place_phase(goal,green_place,nh_place);
				break;
				
				case 3:
				place_phase(goal,red_place,nh_place);
				break;
		}
		//------------------------------------------------------
		
		//Start the place phase sequence
		//place_phase(goal,object_pose,nh_place);

        //---- remove barrel collision objs ----
        object_ids = planning_scene_interface.getKnownObjectNames(false);
        if(object_ids.size()>0) {
            planning_scene_interface.removeCollisionObjects(object_ids);
        }

        result_place.placed=true;
        as_place.setSucceeded(result_place);

    }
};



int main(int argc, char** argv){

    init(argc, argv, "node_place");
    cout << "Starting the server.." << endl;

    placeAction place("place_object");

    spin();
    return 0;
}