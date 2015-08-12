#ifndef __SIMPLE_GRID_MOTION_LOCALIZATION_H__
#define __SIMPLE_GRID_MOTION_LOCALIZATION_H__

#include <simple_grid_motion.h>

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include <rtabmap_ros/GetOdom.h>

#include "aria_robot_control.h"
#include "point.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>
#include <pthread.h>

#include <string>
#include <sstream>
#include <fstream>

#define DISTANCE_TO_POINT_THRESHOLD_MM 300
#define ORIENTATION_THRESHOLD_RADIANS .03 



class SimpleGridMotionLocalization : SimpleGridMotion
{

  protected:
    bool debug;

    bool max_rotate_180;
    double slam_turn_res;
    std::string filename;
//    Point ** goal_points;
    std::vector<Point> goal_points;
    Point cur_point;//current position
    double cur_orientation;

    bool trans_forward;  
    bool turn_ccw;


    ros::ServiceClient slam_odom_client;
    rtabmap_ros::GetOdom slam_odom_srv;

    ros::ServiceClient slam_pause_client;
    ros::ServiceClient slam_resume_client;
    std_srvs::Empty empty_srv; 


    pthread_mutex_t position_lock;


    double get_desired_orientation(Point goal_point); 
    bool is_at_point(Point goal_point);
    void update_current_position();
    void change_orientation(double amount_to_change);
    void slam_move(double dist, bool forward = true);

  public:
    int run();
    SimpleGridMotionLocalization();
};








#endif
