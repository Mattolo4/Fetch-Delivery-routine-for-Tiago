#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <vector>
#include "assignment_2_group_12/getBarrelsPos.h"


using namespace std;
using namespace ros;

struct Point {
    float x;
    float y;
};

const float THRESHOLD = 0.6; //distance of objects from walls
const float FILL_TRESH = 0.4;// delta of distances to be considered still the object

float angle_increment, angle_min;

const int edge = 200; //bins not to consider on the sides of the laser scan



//function that takes the vector of detected right-edges "cleanDistances" and fills the indexes corresponding
//to the object (within a distance thershold) with their original distance value.
//this approach lead to a vector with all zeros except for when points correspond to objects,
//essentially a vector with distance data of only objects and no walls
vector<float> fill(const vector<float>& cleanDistances,const vector<float>& distances){
    vector<float> filled = cleanDistances;
    float beginningValue;
    for (int i = 0; i < cleanDistances.size(); ++i) {
        if(cleanDistances[i]>0){//there is an edge
            beginningValue = cleanDistances[i];
            while(abs(distances[i+1]-beginningValue)<FILL_TRESH){//cycle until the next point doesn't have distance close enough to be same object
                filled[i+1] = distances[i+1];//save the original distance in the new vector
                i++;
            }
        }  
    }

    return filled;
}

//function that takes the raw distance laser data vector and does sort of a right-edge-detection algorithm
//by taking the difference of the distance of a point with the distance of the previous point
//this results in a vector with only non-zero indexes corresponding to the first (right-edge) point of the object
//it then uses this vector "cleanDistances" in the fill function to obtain the vector with only object distance data
vector<float> computeObjVec(const vector<float>& distances){
    vector<float> derivative = distances;
    vector<float> cleanDistances = distances;
    derivative[0]=0;
    cleanDistances[0]=0;
    int count = 0;
    for (int i = 1; i < distances.size(); ++i) {
        derivative[i] = distances[i-1]-distances[i]; //edge-detection/first order discrete derivative
        cleanDistances[i] = distances[i];
        
        if(distances[i]<0.25 || i < edge ||i > 666 - edge){//points are too close (robot feet) and too on the side
            derivative[i]=0;
            cleanDistances[i]=0;
        }
        else if(derivative[i]<THRESHOLD){//difference is too little
            derivative[i]=0;
            cleanDistances[i]=0;
            }
        else{//right-edge found
            count++;
            } 
    }
    vector<float> filled =  fill(cleanDistances,distances);
    return filled;
}

//function that calculates the X,Y coordinates of the center point of a circle obtained interpolating three points
geometry_msgs::Point calculateCircleCenter(const Point& p1, const Point& p2, const Point& p3) {
    float x1 = p1.x, x2 = p2.x, x3 = p3.x;//X coords
    float y1 = p1.y, y2 = p2.y, y3 = p3.y;//Y coords
    float ma = (y2 - y1) / (x2 - x1); //Slope
    float mb = (y3 - y2) / (x3 - x2); //Slope
    // Calculate center coordinates
    geometry_msgs::Point center;
    //Point center;
    center.y = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
    center.x = -1 * (center.y - (x1 + x2) / 2) / ma + (y1 + y2) / 2;
    return center;
}

//function to transform polar coordinates into cartsian
Point transformXY(float dist, int ind){
    float theta = angle_min+ind*angle_increment;//w.r.t. x-axis (forward) (negative=clockwise)
    Point point;
    point.x = dist*sin(theta);
    point.y = dist*cos(theta);
    return point;
}

//function that takes all the deteced objects point-clusters and calculates the center point of the cicle for each of them
vector<geometry_msgs::Point> computeCenterPoints(const vector<float>& distances) {
    //compute the derivative
    vector<float> objectsDist = computeObjVec(distances);
    int firstInd, halfInd, lastInd;

    vector<geometry_msgs::Point> centers;

    Point first,half,last;
    int numObj =0;
    int count;
    for(int i =0; i< objectsDist.size(); i++){
        if(objectsDist[i]!=0){//objects begin
            count=1;
            firstInd=i;//index of beginning of objects
            while(objectsDist[i+1]!=0){//empties the vector until the objects is finshed
                count++; //counting the number of points of this "object" to discard ones too small
                i++;
            }
            if(count>9 && count <150){
                lastInd = i;//last index of the object
                halfInd = firstInd + round((lastInd-firstInd)/2);//index of a middle point

                numObj++;//number of points of this object

                //compute the X,Y coords for three points
                first = transformXY(objectsDist[firstInd],firstInd);
                half = transformXY(objectsDist[halfInd],halfInd);
                last = transformXY(objectsDist[lastInd],lastInd);

                //interpolate the circle and obtain the X,Y coordinate of the center
                geometry_msgs::Point center = calculateCircleCenter(first,half,last);
                //push found centers in a vector
                centers.push_back(center);
                
                //-------------------OUTPUTS IN TERMINAL FOUND CENTER OF OBJECTS-------------------
                ROS_INFO("Center of object n.%d of coordinates [%f, %f]",numObj,center.x,center.y);
                //---------------------------------------------------------------------------------
            }
        }
    }
    return centers;
}

//Callback function that makes the detection and transformation of the barrel where to place the object into the map frame
bool getBarrelsPosCallback(assignment_2_group_12::getBarrelsPos::Request& req, assignment_2_group_12::getBarrelsPos::Response& res){
    // ------------ Obstacle detection ------------
    NodeHandle m;
    sensor_msgs::LaserScan::ConstPtr laserMsg = topic::waitForMessage<sensor_msgs::LaserScan>("/scan",m);
    vector<geometry_msgs::Point> centers;
    if(laserMsg){
        vector<float> distances = laserMsg->ranges;
        angle_min = laserMsg->angle_min;
        angle_increment = laserMsg->angle_increment;
        centers = computeCenterPoints(distances);
    }

    // ------------ Initialize the TF2 Buffer and Listener ------------
    tf2_ros::Buffer  tfBuffer(Duration(10.0)); 
    tf2_ros::TransformListener tfListener(tfBuffer);
    Duration(1.0).sleep();
    vector<geometry_msgs::PointStamped> correctCenters;

    for(int i =0; i<3;i++){
        geometry_msgs::PointStamped baseLinkCenters;
        geometry_msgs::PointStamped laserLinkCenters;
        laserLinkCenters.point.x = centers[i].x;
        laserLinkCenters.point.y = centers[i].y;

        try{
            geometry_msgs::TransformStamped  transformStamped= tfBuffer.lookupTransform("map", "base_laser_link", Time(0));
            tf2::doTransform(laserLinkCenters, baseLinkCenters, transformStamped);
            correctCenters.push_back(baseLinkCenters);
        }
        catch (tf2::TransformException& ex){
            ROS_WARN("Failed to transform pose to %s frame: %s", "base_link", ex.what());
            Duration(0.1).sleep();
            continue;
        }


    }
    //saves the center points of the three detected barrels
    res.center0 = correctCenters[0].point;
    res.center1 = correctCenters[1].point;
    res.center2 = correctCenters[2].point;

    return true;
}


int main(int argc, char** argv){

    init(argc, argv, "node_barrelScan");
    NodeHandle n;
    
    ServiceServer service = n.advertiseService("get_barrels_pose", getBarrelsPosCallback);

    ROS_INFO("Service for obtaining center points of barrels ready");
    

    spin();
    return 0;
}