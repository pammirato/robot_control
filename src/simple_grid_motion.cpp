#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "rosaria/get_state.h"

#include <stdio.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>
/*#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
*/

#include <string>
#include <sstream>


#define WAIT_TO_MOVE_TIME .5
#define MOVE_DISTANCE 100
#define ROTATION_ANGLE 45






namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}










int main(int argc, char **argv){


  ros::init(argc, argv, "controller");
  ros::NodeHandle n = ros::NodeHandle("~");

  //parameters in meters
  double grid_width = 10;
  double grid_height = 10;
  double grid_resolution = .5;

  //set parameters from command line
  n.getParam("width", grid_width);
  n.getParam("height", grid_height);
  n.getParam("resolution", grid_resolution);
 
  ROS_INFO("PARAMS W:%f  H:%f R:%f", grid_width, grid_height, grid_resolution);

  int publish_buffer_size = 10;


 //register publishers
  ros::Publisher move_pub = n.advertise<std_msgs::String>("/RosAria/move",publish_buffer_size);

  ros::Publisher heading_pub = n.advertise<std_msgs::String>("/RosAria/heading",publish_buffer_size);

  //register client to get robot state service 
  ros::ServiceClient get_state_client = n.serviceClient<rosaria::get_state>("/RosAria/get_state");
  rosaria::get_state get_state_srv; //service object, has request/response

  //what frequecny the main loop will run at TODO do i neeid this
  ros::Rate loop_rate(1);//run at  x HZ

 //how long to wait for robot to stop moving
  ros::Duration wait = ros::Duration(WAIT_TO_MOVE_TIME);
  
  std_msgs::String msg;

  int counter = 0;
  while(ros::ok())
  {
/* 
    //get the state of the robot
    while(!get_state_client.call(get_state_srv));
     
    //wait until the robot has stopped moving to issue a move command
    while(!(get_state_srv.response.isStopped))
    {
      ROS_INFO("ROBOT STIL moving");
      wait.sleep();
      while(!get_state_client.call(get_state_srv)){
        ROS_INFO("failed service call!");
      }
    } 
      

*/
 
   
    if(counter%2==0)
    { 
      ROS_INFO("move command");
      msg.data = patch::to_string(MOVE_DISTANCE);
      move_pub.publish(msg);
    } 
    else
    {
      ROS_INFO("rot commnad");
      msg.data = patch::to_string(ROTATION_ANGLE);
      heading_pub.publish(msg);
    }




    ros::spinOnce();//so our callbacks get called
    loop_rate.sleep();
    counter++;
  }









  return 0;

}//end main
