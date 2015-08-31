#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"


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


#include "aria_robot_control.h"


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















void rotate(AriaRobotControl robot, ros::ServiceClient save_client_1,
         ros::ServiceClient save_client_2,ros::ServiceClient save_client_3,
         double total_degrees, double turn_res, bool ccw, bool save_images,
          double wait_time)
{

  std_srvs::Empty save_images_srv;

  int num_turns = total_degrees/turn_res;

  double leftover = total_degrees - (num_turns * turn_res);

  for(int i=0; i<num_turns; i++)
  {

    robot.wait_until_stopped(ros::Duration(.2));
    robot.do_rotation(turn_res, turn_res,ccw, save_images);
    robot.wait_until_stopped(ros::Duration(wait_time));
    //ros::Duration(wait_time).sleep();
    if(save_images)
    {
//      robot.wait_until_stopped(ros::Duration(wait_time));
      ros::Duration(.1).sleep();
      if(!save_client_1.call(save_images_srv))
      { ROS_ERROR("FAILED SAVE SERVICE CALL K1"); }
      if(!save_client_2.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K2");
      }
      if(!save_client_3.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K3");
      }
    }
    //ros::Duration(wait_time).sleep();
  }//for i 


  if(leftover > 1)
  {

    robot.wait_until_stopped(ros::Duration(.2));
    robot.do_rotation(leftover, leftover,ccw, save_images);
    robot.wait_until_stopped(ros::Duration(wait_time));
    //ros::Duration(wait_time).sleep();
    if(save_images)
    {
//      robot.wait_until_stopped(ros::Duration(wait_time));
      ros::Duration(.1).sleep();
      if(!save_client_1.call(save_images_srv))
      { ROS_ERROR("FAILED SAVE SERVICE CALL K1"); }
      if(!save_client_2.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K2");
      }
      if(!save_client_3.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K3");
      }
    }

  }


}//rotate













int main(int argc, char **argv){

  
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh = ros::NodeHandle("~");

  int publish_buffer_size = 10;

  std::string save_1 = "/K1/save_images";
  std::string save_2 = "/K2/save_images";
  std::string save_3 = "/K3/save_images";
  double turn_res = 15.0;

  nh.getParam("base_name_1", save_1);
  nh.getParam("base_name_2", save_2);
  nh.getParam("base_name_3", save_3);
  nh.getParam("turn_res",turn_res);
    

  //conotrols the robot
  AriaRobotControl robot = AriaRobotControl();


  ros::ServiceClient save_client_1 = nh.serviceClient<std_srvs::Empty>(save_1);
  ros::ServiceClient save_client_2 = nh.serviceClient<std_srvs::Empty>(save_2);
  ros::ServiceClient save_client_3 = nh.serviceClient<std_srvs::Empty>(save_3);
  std_srvs::Empty save_images_srv;


  bool saved  = false;
  ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",publish_buffer_size);


  //keep track of the robots current linear and angular speed
  double cur_linear = 0.0;
  double cur_angular = 0.0;

  //how much to change the speeds in one step
  double linear_step = .1;
  double angular_step = .1;

  //the message that will be published to the ROS topic
  geometry_msgs::Twist message = geometry_msgs::Twist();

  ros::Rate loop_rate(10);


  bool publish = true;
  bool quit = false;
  while(ros::ok() && !quit)
  {
    publish = true; 
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
      case 'r':
        rotate(robot,save_client_1,save_client_2,save_client_3,180,turn_res,true,true,1);
        rotate(robot,save_client_1,save_client_2,save_client_3,180,45,false,false,0);

        rotate(robot,save_client_1,save_client_2,save_client_3,195,45,false,false,0);
        rotate(robot,save_client_1,save_client_2,save_client_3,195,turn_res,true,true,1);

        publish = false;//dont publish a velocity command

        ROS_ERROR("DONE WITH CURRENT POINT!");
        break;
      case 't':
        rotate(robot,save_client_1,save_client_2,save_client_3,360,turn_res,true,true,1);

        rotate(robot,save_client_1,save_client_2,save_client_3,360,45,false,false,0);

        ROS_ERROR("DONE WITH CURRENT POINT!");
        publish = false;
        break;
      case 'e':
        std::cout << "MOVING 100mm" << std::endl;
        robot.do_translation(500);
        robot.wait_until_stopped(1.0);

        publish = false;
        break;
      case '1':
        saved = save_client_1.call(save_images_srv);
        std::cout << "saved " << saved << std::endl;
        break;
      case '2':
        saved = save_client_2.call(save_images_srv);
        std::cout << "saved " << saved << std::endl;
        break;
      case 'q':
        quit = true;
        publish = false; 
        break;
      default:
        std::cout << "ERROR: " << patch::to_string(errno) << std::endl;
    


    }//switch(c)

    //if we want to publish velocity(not for degree turns)
    if(publish)
    {
      //set the linear speed, and angular speed.
      message.linear.x = cur_linear;
      message.angular.z = cur_angular;
      control_pub.publish(message);
    }
 
    loop_rate.sleep();
  }//end while






  ros::shutdown();

  return 0;

}//end main
