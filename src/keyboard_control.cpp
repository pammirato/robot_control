#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


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
  ros::NodeHandle n;

  int publish_buffer_size = 10;



  ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",publish_buffer_size);


  //keep track of the robots current linear and angular speed
  double cur_linear = 0.0;
  double cur_angular = 0.0;

  //how much to change the speeds in one step
  double linear_step = .1;
  double angular_step = .1;

  //the message that will be published to the ROS topic
  geometry_msgs::Twist message = geometry_msgs::Twist();

  ros::Rate loop_rate(10);




  while(ros::ok())
  {
  
    //int c = getch();
    char c;
    scanf(" %c",&c); 
 
    switch(c)
    {
      case 'f': 
        cur_linear = 0.0;
        cur_angular = 0.0;
        std::cout << "STOP" << std::endl;
        break;
  
      case 'z':
        cur_linear = 0; 
        std::cout << "stop linear" << std::endl;
        break;
      case 'x':
        cur_angular = 0; 
        std::cout << "stop angular" << std::endl;
        break;
      case 'w':
        cur_linear += linear_step; 
        std::cout << "forward" << std::endl;
        break;
      case 's':
        cur_linear += -linear_step;
        std::cout << "backward" << std::endl;
        break;
      case 'a':
        cur_angular += angular_step;
        std::cout << "turning CCW" << std::endl;
        break;
      case 'd':
        cur_angular += -angular_step;
        std::cout << "turning CW" << std::endl;
        break;
      default:
        std::cout << "ERROR: " << patch::to_string(errno) << std::endl;
    


    }//switch(c)


    //set the linear speed, and angular speed.
    message.linear.x = cur_linear;
    message.angular.z = cur_angular;
    control_pub.publish(message);
 
    loop_rate.sleep();
  }//end while








  return 0;

}//end main
