#pragma once
#ifndef __SIMPLE_GRID_MOTION_H__
#define __SIMPLE_GRID_MOTION_H__

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "std_srvs/Empty.h"
#include "rosaria/get_state.h"
#include "rosaria/float_message.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>

#include <string>
#include <sstream>


class SimpleGridMotion
{

  public:
    ros::NodeHandle nh;//this ros node

    ros::ServiceClient get_state_client;
    ros::ServiceClient move_client;
    ros::ServiceClient heading_client;
    ros::ServiceClient enable_motors_client;

    rosaria::get_state get_state_srv; 
    rosaria::float_message float_srv; 
    std_srvs::Empty enable_motors_srv;



    ros::Duration trans_wait;//how long to wait after a trans move
    ros::Duration turn_wait;//how long to wati after a turn

     
    //Parameters for path planning

    double grid_width;
    double grid_height;
    double grid_res;//resolution, distance between cells
    double turn_res;//how much to turn before taking the next pic     
    int num_cols_hit;//grid_width/grid_res
    bool left;//whether to go left or right




    //functions
    bool initialize();
    bool wait_until_stopped(ros::Duration wait, int max_iterations);
    bool do_rotation(double total_degrees, double step_size, bool cw);
    bool turn(double degrees);
    bool do_translation(double millimeters);

    int run();





   public:
    SimpleGridMotion();

};








#endif
