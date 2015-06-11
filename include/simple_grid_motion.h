#ifndef __SIMPLE_GRID_MOTION_H__
#define __SIMPLE_GRID_MOTION_H__

#include "ros/ros.h"


#include "aria_robot_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>

#include <string>
#include <sstream>


#define METERS_TO_MILLIMETERS 1000


class SimpleGridMotion
{

  private:

    AriaRobotControl robot;
    //Parameters for path planning

    double grid_width;
    double grid_height;
    double grid_res;//resolution, distance between cells
    double turn_res;//how much to turn before taking the next pic     
    double turn_offset;//make up for not quite inplace turning

    
    ros::NodeHandle nh;
    ros::ServiceClient save_client;    

    //functions
    bool initialize();


   public:
    SimpleGridMotion();
    int run();
};








#endif
