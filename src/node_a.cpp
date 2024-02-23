#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"  
#include "tiago_iaslab_simulation/Objs.h"
#include <assignment_2_group_12/TaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <assignment_2_group_12/pickAction.h>
#include <assignment_2_group_12/placeAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "assignment_2_group_12/getTableLocation.h"
#include "assignment_2_group_12/getBarrelsPos.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace ros;
using namespace std;
using namespace actionlib;
using namespace cv;

vector<float> getPlacePos(int obj_id){
    //----- obtain barrels position ------ 
    NodeHandle m;
    ServiceClient barrelClient = m.serviceClient<assignment_2_group_12::getBarrelsPos>("get_barrels_pose");
    ROS_INFO("called service...");

    assignment_2_group_12::getBarrelsPos srv;
    vector<geometry_msgs::Point> centers;
    if (barrelClient.call(srv)) {
        
        centers.push_back(srv.response.center0);
        centers.push_back(srv.response.center1);
        centers.push_back(srv.response.center2);
    }
    ROS_INFO("got barrels centers");

    // 2. calls the getTableLocation service to know which of the 3 tables is the correct one
    ServiceClient tableClient = m.serviceClient<assignment_2_group_12::getTableLocation>("/get_table_location"); 
    assignment_2_group_12::getTableLocation tableLocationSrv;
    tableLocationSrv.request.obj_idx = obj_id;
    ROS_INFO("got desired barrel location");

        // 0 = dx
        // 1 = center
        // 2 = sx
    int barrel_id; 
    if(tableClient.call(tableLocationSrv)){
        barrel_id = tableLocationSrv.response.location_idx;
        ROS_INFO("The service output is: [%d]", barrel_id);
    }else{
        ROS_ERROR("Something wrong happened!");
    }
    
    geometry_msgs::Point placeBarrel = centers[barrel_id];

    vector<float> placePos;
    placePos.push_back(placeBarrel.x);
    placePos.push_back(placeBarrel.y);
    placePos.push_back(0.);
    placePos.push_back(0.);
    placePos.push_back(0.);
    placePos.push_back(0.713);
    placePos.push_back(0.7);

    return placePos;
}


void feedbackCallback(const assignment_2_group_12::TaskActionFeedbackConstPtr& feedback){
    // Process and print feedback
    ROS_INFO("[Feedback]: %s", feedback->feedback.current_state.c_str());
}

// Move the robot to a given point
void moveTo(vector<float> point){

    float x_goal = point[0];
    float y_goal = point[1];
    float z_goal = point[2];    // 0

    float x_orient_goal = point[3]; // 0
    float y_orient_goal = point[4]; // 0
    float z_orient_goal = point[5];
    float w_orient_goal = point[6];
    
    // Setup a feedback callback
    NodeHandle feedback_nh;
    Subscriber feedback_sub = feedback_nh.subscribe("task/feedback", 10, feedbackCallback);

    // Create an Action client
    SimpleActionClient<assignment_2_group_12::TaskAction> ac("task", true);
    ROS_INFO("Waiting the 'task Server' to start..");

    ac.waitForServer(); //will wait for infinite time

    // Setting the Goal according to user's Input
    ROS_INFO("'task Server' started, sending goal.");
    assignment_2_group_12::TaskGoal goal;

    goal.point.push_back(x_goal);
    goal.point.push_back(y_goal);
    goal.point.push_back(z_goal);

    goal.point.push_back(x_orient_goal);
    goal.point.push_back(y_orient_goal);
    goal.point.push_back(z_orient_goal);
    goal.point.push_back(w_orient_goal);


    ac.sendGoal(goal);
    
    // Set this while to allow the feedback_nh to subscribe the topic and print the feedback
    Rate r(10);
    while(ok() && !ac.waitForResult(Duration(0.1))){
        spinOnce();
        r.sleep();
    }

    assignment_2_group_12::TaskResultConstPtr result;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || ac.waitForResult()){
        ROS_INFO("Robot arrived in place");
    }else{
        ROS_INFO("Action did have some problems.");
        ac.cancelGoal();
    }
}




int main(int argc, char **argv){
    init(argc, argv, "node_a");
    cout << "Node A online" << endl;
    
    // Map cube-position in front of the table (with associate orientation)
    vector<float> after_obstacle   = {8.124, 0.394, 0, 0, 0, -0.706, 0.708};
    vector<float> blue  = {8.524, -2.268, 0, 0, 0, -0.806, 0.308};      // id = 1
    vector<float> green = {7.647, -4.090, 0, 0, 0,  0.695, 0.719};      // id = 2
    vector<float> red   = {7.525, -2.068, 0, 0, 0, -0.706, 0.708};      // id = 3
    vector<float> green_cp_up   = {8.881, -4.341, 0, 0, 0, 1.0, 0.019};   // To don't crash into the table
    vector<float> green_cp_down = {8.881, -4.341, 0, 0, 0, 0.707, 0.707}; // To don't crash into the table
    vector<float> table_room    = {11.093,-3.148, 0, 0, 0, 0.760, 0.752};    
    vector<float> leftside_table    = {8.910, -2.042, 0, 0, 0, -0.713, 0.7};
    vector<float> leftside_table_up    = {8.910, -2.042, 0, 0, 0, 0.713, 0.7};
    vector<float> upside_table    = {8.910, -3.921, 0, 0, 0, -0.713, 0.7};  
    vector<float> table_room_up    = {11.093,-3.148, 0, 0, 0, -0.760, 0.752};  
    vector<float> toward_table = {9.134, -4.077, 0.000, 0.000, 0.000, -0.998, 0.058};
    vector<float> toward_table_room = {8.126, -4.382, 0.000, 0.000, 0.000, 0.023, 1.000};
    vector<float> green_left = {7.647, -4.090, 0, 0, 0,  0.000, 0.023, 1.000};      // id = 2

 
    // Tables    
    // Commented due to implemented Extra-points task
    /*
    vector<float> table_red    = {10.532, 0.65, 0, 0, 0, -0.713, 0.7};
    vector<float> table_green  = {11.560, 0.65, 0, 0, 0, -0.713, 0.7};
    vector<float> table_blue   = {12.428, 0.65, 0, 0, 0, -0.713, 0.7};
    */


    //------- contact the Human node srv to obtain the list of id's------------
    int obj_id;
    NodeHandle n;
    ServiceClient client = n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv"); //Name of the service, don't change

    tiago_iaslab_simulation::Objs srv;
    srv.request.ready   = true;   
    srv.request.all_objs = true;    // If .all_objs is 'false' the service return only one value 

    if(client.call(srv)){   // Check the service output
        ROS_INFO("The Human Node output is: ");
        for(int i=0; i<srv.response.ids.size(); i++){
            ROS_INFO("[%d] ", srv.response.ids[i]);
        }
    }else{
        ROS_ERROR("Something wrong happened!");
        return -1;
    }


    //--------------pick action server init----------
    SimpleActionClient<assignment_2_group_12::pickAction> pick("pick", true);
    //ROS_INFO("Waiting the 'pick Server' to start..");
    pick.waitForServer(); //will wait for infinite time
    assignment_2_group_12::pickGoal pickGoal;
    assignment_2_group_12::pickResultConstPtr result;

    //-------move to after obstacle-------
    moveTo(after_obstacle);

    //-----starts for cycle------------- 

    for(int i=0; i<srv.response.ids.size(); i++){

        obj_id =  srv.response.ids[i]; //curren id
        ROS_INFO("obtained id: %d" , obj_id);
        pickGoal.obj_id = obj_id;
        
	    if(i==0 && obj_id == 2){//we pick first the green object
        moveTo(leftside_table);
        }
	
        
        if(obj_id == 1){//if we have to pick the blue object
            cout << "Object ID 1: Blue" << endl;
            if(i==0){
            moveTo(leftside_table);
            }
            else{
            moveTo(leftside_table_up);
            }
            moveTo(blue);
            
            //------- start pick phase with node_pick --------------
            // Setting the Goal according to user's Input
            //ROS_INFO("'pick Server' started, sending goal.");
            pick.sendGoal(pickGoal);
            //ROS_INFO("Goal sent, obj_id: %d", obj_id);
            cout << "finished pick phase of obj 1" << endl;

            
            if (pick.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || pick.waitForResult()){
                ROS_INFO("Robot pickd correctly");
            }else{
                ROS_INFO("Action did have some problems.");
                pick.cancelGoal();
            }


            //----------------------------------------------
            ROS_INFO("picked the object");
            sleep(0.5);
            moveTo(leftside_table);
            moveTo(upside_table);

        }

        else if(obj_id == 2){//if we have to pick the green object
            std::cout << "Object ID 2: Green" << std::endl;
            //moveTo(leftside_table);
            moveTo(green_cp_up);
            moveTo(green);
            
            //------- start pick phase with node_pick --------------
            // Setting the Goal according to user's Input
            //ROS_INFO("'pick Server' started, sending goal.");
            pick.sendGoal(pickGoal);
            //ROS_INFO("Goal sent, obj_id: %d", obj_id);
            if (pick.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || pick.waitForResult()){
                ROS_INFO("Robot pickd correctly");
            }else{
                ROS_INFO("Action did have some problems.");
                pick.cancelGoal();
            }
            //-------------------------------
            ROS_INFO("picked the object");
            sleep(0.5);
            //moveTo(green_cp_up);
            moveTo(green_left);    
        }
        
        else if(obj_id == 3){//if we have to pick the red object
            std::cout << "Object ID 3: Red" << std::endl;
            if(i==0){
            moveTo(leftside_table);
            }
            else{
            moveTo(leftside_table_up);
            }
            moveTo(red);
          
            //------- start pick phase with node_pick --------------
            // Setting the Goal according to user's Input
            //ROS_INFO("'pick Server' started, sending goal.");
            pick.sendGoal(pickGoal);
            //ROS_INFO("Goal sent, obj_id: %d", obj_id);
            if (pick.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || pick.waitForResult()){
                ROS_INFO("Robot picked correctly");
            }else{
                ROS_INFO("Action did have some problems.");
                pick.cancelGoal();
            }
            //---------------------------------------------
            ROS_INFO("picked the object");
            sleep(0.5);
            moveTo(leftside_table);
            moveTo(upside_table);

        }
        else{
            ROS_ERROR("SOMETHING'S WRONG");
        }



        //---------- move to table room -----------
        cout << "starting place phase" << endl;
        moveTo(table_room);

        //------make detection-----
        vector<float> placePos = getPlacePos(obj_id); // center of barrel to place obj

        //------ move robot to place location-------
        vector<float> placePosRobot = placePos;
        placePosRobot[1] -= 0.7;  //shift the location back for the robot to stand
        moveTo(placePosRobot);

        //----- complete tha place phase --------

        SimpleActionClient<assignment_2_group_12::placeAction> place("place_object", true);
        //ROS_INFO("Waiting the 'place Server' to start..");

        place.waitForServer(); //will wait for infinite time

        // Setting the Goal according to user's Input
        //ROS_INFO("'place Server' started, sending goal.");
        assignment_2_group_12::placeGoal placeGoal;
        placeGoal.obj_id = obj_id;
        placeGoal.placePos = placePos;


        place.sendGoal(placeGoal);
        //ROS_INFO("Goal sent, obj_id: %d", obj_id);

        assignment_2_group_12::placeResultConstPtr result;
        if (place.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || place.waitForResult()){
            ROS_INFO("Robot placed correctly");
        }else{
            ROS_INFO("Action did have some problems.");
            place.cancelGoal();
        }

        //------- goes back to beginning of table room to pick another object --------------

        cout << "finished place phase" << endl;
        moveTo(table_room_up);


    }//end for


    //moveTo(table_room);

    
    cout << "\n PICK AND PLACE TASK COMPLETED! \n";
    spinOnce();

    return 0;
}