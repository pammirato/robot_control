#ifndef __SIMPLE_GRID_MOTION_LOCALIZATION_H__
#define __SIMPLE_GRID_MOTION_LOCALIZATION_H__

#include <simple_grid_motion.h>

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"


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


#define DISTANCE_TO_POINT_THRESHOLD_MM 100
#define ORIENTATION_THRESHOLD_RADIANS .03 



class SimpleGridMotionLocalization : SimpleGridMotion
{

  protected:
    bool debug;

//    Point ** goal_points;
    std::vector<Point> goal_points;
    Point cur_point;//current position
    double cur_orientation;
  
    ros::Subscriber slam_odom_sub;

    pthread_mutex_t position_lock;


    double get_desired_orientation(Point goal_point); 
    bool is_at_point(Point goal_point);

  public:
    void slam_odom_cb(nav_msgs::Odometry::ConstPtr odom_msg);
    int run();
    SimpleGridMotionLocalization();
};








#endif
