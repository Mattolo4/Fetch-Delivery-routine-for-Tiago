#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <assignment_2_group_12/TaskAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace ros;
using namespace actionlib;


class TaskAction{

    // Declaring Action Server variables
    protected:
        NodeHandle nh_;
        SimpleActionServer<assignment_2_group_12::TaskAction> as_;
        string action_name;
        assignment_2_group_12::TaskFeedback feedback_;
        assignment_2_group_12::TaskResult result_;

    public:
        TaskAction(string name): as_(nh_, name, boost::bind(&TaskAction::execute, this, _1), false), action_name(name){
            as_.start();
        }

    ~TaskAction(void){}


    void execute(const assignment_2_group_12::TaskGoalConstPtr &goal){
        Rate r(1);   //1 sec
        bool success = true;


        feedback_.current_state = "Connecting to 'move_base' server..";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());


        SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        move_base_msgs::MoveBaseGoal MoveGoal;  // Creating the move_base goal msg

        feedback_.current_state = "Setting robot's target";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());


        //Unpacking goal
        float x_goal = goal->point[0];
        float y_goal = goal->point[1];
        float z_goal = goal->point[2];    // 0

        float x_orient_goal = goal->point[3]; // 0
        float y_orient_goal = goal->point[4]; // 0
        float z_orient_goal = goal->point[5];
        float w_orient_goal = goal->point[6];

        //Set the global map reference frame
        MoveGoal.target_pose.header.frame_id = "map";
        MoveGoal.target_pose.header.stamp    = Time::now();

        //Set the position according to the user's prompt
        MoveGoal.target_pose.pose.position.x = x_goal;
        MoveGoal.target_pose.pose.position.y = y_goal;
        MoveGoal.target_pose.pose.position.z = z_goal;

        MoveGoal.target_pose.pose.orientation.x = x_orient_goal;
        MoveGoal.target_pose.pose.orientation.y = y_orient_goal;
        MoveGoal.target_pose.pose.orientation.z = z_orient_goal;
        MoveGoal.target_pose.pose.orientation.w = w_orient_goal;

        ac.waitForServer();

        feedback_.current_state = "Sending goal";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
        ac.sendGoal(MoveGoal);
        feedback_.current_state = "The robot is moving";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

        // Wait for the result 
        bool finished_before_timeout = ac.waitForResult(Duration(100.0));

        if (finished_before_timeout || ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            
            feedback_.current_state = "The robot reached the Target!";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
        
        }else{
            success = false;
            feedback_.current_state = "Some errors occured";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            return;
        }

        if(success){
            feedback_.current_state = "Robot arrived in place";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            r.sleep();  // to let all the feedbacks to be printed by the client
            as_.setSucceeded(result_);
        }
    }
};



int main(int argc, char** argv){

    init(argc, argv, "server");
    cout << "Starting the server.." << endl;

    TaskAction task("task");

    spin();
    return 0;
}