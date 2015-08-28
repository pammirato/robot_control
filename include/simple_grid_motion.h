#ifndef __SIMPLE_GRID_MOTION_H__
#define __SIMPLE_GRID_MOTION_H__

#include "ros/ros.h"
#include "std_srvs/Empty.h"


#include "aria_robot_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>

#include <string>
#include <sstream>


#define METERS_TO_MILLIMETERS 1000
#define DEFAULT_KINECT_BASE_NAME "kinect2"

class SimpleGridMotion
{

  protected:

    AriaRobotControl robot;
    //Parameters for path planning

    double grid_width;
    double grid_height;
    double grid_res;//resolution, distance between cells
    double turn_res;//how much to turn before taking the next pic     
    double trans_res;

    double turn_offset;//make up for not quite inplace turning


    double max_rotation;//how much to roatate at each point(usually 360)

    std::string kinect_base_name_1;
    std::string kinect_base_name_2;
    std::string kinect_base_name_3;

    
    ros::NodeHandle nh;
    ros::ServiceClient save_client_1;    
    ros::ServiceClient save_client_2; 
    ros::ServiceClient save_client_3; 

    std_srvs::Empty save_images_srv;   

    //functions
    void rotate(double total_degrees, double turn_res, bool ccw, bool save_images, double wait_time=0);

   public:
    SimpleGridMotion();
    int run();
};








#endif
