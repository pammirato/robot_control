#ifndef __SIMPLE_GRID_MOTION_LOCALIZATION_H__
#define __SIMPLE_GRID_MOTION_LOCALIZATION_H__

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

    //mutext for critical sections using position variable
//    static std::mutex mtx;

    Point ** goal_points;
    Point cur_point;//current position
   
    //ROS things 
    ros::NodeHandle nh;
    ros::Subscriber slam_odom_sub;

    //functions
    bool initialize();
    void slam_odom_cb(nav_msgs::Odometry &msg);


   public:
    SimpleGridMotionLocalization();
    ~SimpleGridMotionLocalization();
    int run();
};








#endif
