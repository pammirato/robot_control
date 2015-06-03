#ifndef __SIMPLE_GRID_MOTION_H_INLCUDED__
#ifndef __SIMPLE_GRID_MOTION_H_INLCUDED__

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "std_srvs/Empty.h"
#include "rosaria/get_state.h"
#include "rosaria/float_message.h"

#include <stdio.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>

#include <string>
#include <sstream>


class SimpleMotionControl
{

  private:
    ros::NodeHandle nh;//this ros node

    ros::ServiceClient get_state_client;
    ros::ServiceClient move_client;
    ros::ServiceClient heading_client;

    ros::Duration trans_wait;//how long to wait after a trans move
    ros::Duration turn_wait;//how long to wati after a turn

     
    //Parameters for path planning

    double grid_width;
    double grid_height;
    double grid_res;//resolution, distance between cells
    double turn_angle_res;//how much to turn before taking the next pic     
    int num_cols_hit;//grid_width/grid_res
    bool left;//whether to go left or right




    //functions
    void initialize();
    bool wait_until_stopped(ros::Duration wait, int max_iterations=100);
    int run();


};








#endif
