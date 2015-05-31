#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

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



  ros::Rate loop_rate(1);//run at  x HZ

  std_msgs::String msg;
  std::stringstream ss;

  int counter = 0;
  while(ros::ok())
  {
   
    ss <<  45;
    msg.data = ss.str();
   
    if(counter%2==0)
    { 
      move_pub.publish(msg);
    } 
    else
    {
      heading_pub.publish(msg); 
    }

    ros::spinOnce();//so our callbacks get called
    loop_rate.sleep();
    counter++;
  }









  return 0;

}//end main
