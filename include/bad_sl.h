#ifndef __SIMPLE_GRID_MOTION_FEEDBACK_H__
#define __SIMPLE_GRID_MOTION_FEEDBACK_H__

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"

#include "aria_robot_control.h"
#include "point.h"


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>

#include <string>
#include <sstream>
#include <mutex>

#define METERS_TO_MILLIMETERS 1000


class SimpleGridMotionLocalization
{

  private:

    AriaRobotControl robot;
    //Parameters for path planning

    double grid_width;
    double grid_height;
    double grid_res;//resolution, distance between cells
    double turn_res;//how much to turn before taking the next pic     
    double turn_offset;//make up for not quite inplace turning

    //mutext for critical sections using position variables
    std::mutex mtx;

    Point ** goal_points;
    Point cur_point_mutex;//current position,a reminder to lock b4 using

    //ros things    
    ros::NodeHandle nh;
    ros::Subscriber slam_odom_sub;


    //functions
    bool initialize();
    void slam_odom_cb(std_msgs::String::ConstPtr& msg); 

   public:
    SimpleGridMotionLocalization();
//    ~SimpleGridMotionLocalization();
    int run();
};








#endif
