#ifndef __ARIA_ROBOT_CONTROL_H__
#define __ARIA_ROBOT_CONTROL_H__

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

#define WAIT_TO_MOVE_TIME .5
#define WAIT_TO_TURN_TIME .5
#define MOVE_DISTANCE 100
#define ROTATION_ANGLE 45
#define CHANGE_ROW_OFFSET -50

class AriaRobotControl
{

  private:
    ros::NodeHandle nh;//this ros node

    ros::ServiceClient get_state_client;
    ros::ServiceClient move_client;
    ros::ServiceClient heading_client;
    ros::ServiceClient enable_motors_client;
    ros::ServiceClient save_images_client;
    ros::ServiceClient max_speeds_client;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_message;

    rosaria::get_state get_state_srv; 
    rosaria::float_message float_srv; 
    rosaria::float_message max_speeds_srv; 
    std_srvs::Empty enable_motors_srv;
    std_srvs::Empty save_images_srv;


    ros::Duration trans_wait;//how long to wait after a trans move
    ros::Duration turn_wait;//how long to wati after a turn

     
    //Parameters for path planning



  public:

    //functions
    bool wait_until_stopped(ros::Duration wait, int max_iterations=100);
    bool wait_until_stopped(double seconds, int max_iterations=100);
    bool do_rotation(double total_degrees, double step_size, bool cw, bool save_images=false);
    bool turn(double degrees); 
    bool turn(double degrees, double max_velocity);
    bool do_translation(double millimeters);

    bool set_max_speeds(double max_trans, double max_rot);
    int run();



    AriaRobotControl();

};








#endif
