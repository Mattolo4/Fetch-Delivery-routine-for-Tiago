#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "assignment_2_group_12/getTableLocation.h"

using namespace std;
using namespace ros;
using namespace cv;

//callback function that takes the image from the tiago camera and detects the position of the dfferent colored barrels
bool getLocation(assignment_2_group_12::getTableLocation::Request &req, assignment_2_group_12::getTableLocation::Response &res){

    ROS_INFO("OK");
    cout << "all good"<< endl;
    
    NodeHandle imgGetter;
    cout << "waiting for img" << endl;
    sensor_msgs::Image::ConstPtr img = topic::waitForMessage<sensor_msgs::Image>("xtion/rgb/image_color", imgGetter);
        
    if(img){
        ROS_INFO("RGB Image obtained");
        
        try{
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            Mat pov = cvImg->image;

            // saving the img 
            /*
            if(cv::imwrite("robotPov.jpg", pov)){

                cout << "Image saved" << endl;
            }*/
            
            int COLOR_WANTED_T    = 180;
            int COLOR_DISCARDED_T = 100;

            // Defines the position of the desired table wrt to the robot
            // 0 = dx
            // 1 = center
            // 2 = sx
            int POSITION;

            // Defines the id of the object to be placed
            // 1 = BLUE
            // 2 = GREEN
            // 3 = RED
            int ID_OBJ = req.obj_idx;

            //("image", pov);
            Mat segmentedPov = pov; 

            // Segment based on the wanted color
            for(int i=0; i<segmentedPov.cols; i++){
                for(int j=0; j<segmentedPov.rows; j++){
                    Vec3b pixel = segmentedPov.at<Vec3b>(j, i);

                    // Extract individual color channels (BGR)
                    int b = pixel[0];
                    int g = pixel[1];
                    int r = pixel[2];

                    switch(ID_OBJ){
                        case 1:
                            //blue
                            if(!(b > COLOR_WANTED_T && g < COLOR_DISCARDED_T && r < COLOR_DISCARDED_T)){
                                segmentedPov.at<Vec3b>(j, i)[0] = 0;
                                segmentedPov.at<Vec3b>(j, i)[1] = 0;
                                segmentedPov.at<Vec3b>(j, i)[2] = 0; 
                            }
                            break; 
                        case 2:
                            //green
                            if(!(b < COLOR_DISCARDED_T && g > COLOR_WANTED_T && r < COLOR_DISCARDED_T)){
                                segmentedPov.at<Vec3b>(j, i)[0] = 0;
                                segmentedPov.at<Vec3b>(j, i)[1] = 0;
                                segmentedPov.at<Vec3b>(j, i)[2] = 0; 
                            }
                            break; 
                        case 3:
                            // red
                            if(!(b < COLOR_DISCARDED_T && g < COLOR_DISCARDED_T && r > COLOR_WANTED_T)){
                                segmentedPov.at<Vec3b>(j, i)[0] = 0;
                                segmentedPov.at<Vec3b>(j, i)[1] = 0;
                                segmentedPov.at<Vec3b>(j, i)[2] = 0; 
                            }
                            break; 
                        default:
                        cout << "Invalid choice." << endl;
                        break;
                    }
                }
            }

            // Col values for the separation lines
            int x1 = segmentedPov.cols/3; 
            int x2 = segmentedPov.cols/3*2;  

            // Check the segmented part in which side is
            bool getOut = false;
            for(int i=0; i<segmentedPov.cols && !getOut; i++){
                for(int j=0; j<segmentedPov.rows; j++){

                    Vec3b pixel = segmentedPov.at<Vec3b>(j, i);
                    int b = pixel[0];
                    int g = pixel[1];
                    int r = pixel[2];
                    
                    if(b!=0 || g!=0 || r!=0){

                        circle(segmentedPov, Point(i,j), 5, Scalar(180, 105, 255), FILLED);
                        if(i < x1) POSITION = 2; 
                        else if(i > x1 && i < x2) POSITION = 1; 
                        else if(i > x2) POSITION = 0;
                        getOut = true;
                        break;
                    }
                }
            }
            // Draw the border lines
            line(segmentedPov, Point(x1, 0), Point(x1, segmentedPov.rows - 1), Scalar(255, 255, 255), 1);
            line(segmentedPov, Point(x2, 0), Point(x2, segmentedPov.rows - 1), Scalar(255, 255, 255), 1);

            if(POSITION==0){
                cout << "DX" << endl;
            }else if(POSITION==1){
                cout << "CENTER" << endl;
            }else if(POSITION==2){
                cout << "SX" << endl;
            }

            res.location_idx = POSITION;
            cout << "DONE" <<endl;

        }catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
        }
    }

    return true;
}

int main(int argc, char **argv){
    init(argc, argv, "node_barrelDetect");
    NodeHandle n;
    ServiceServer service = n.advertiseService("get_table_location", getLocation);
    
    ROS_INFO("Service to detect desired place barrel ready");
    spin();
    return 0;
}