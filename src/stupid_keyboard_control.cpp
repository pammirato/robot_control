#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include <rtabmap_ros/GetOdom.h>
#include <nav_msgs/Odometry.h>
#include "rosaria/float_message.h"

#include <stdio.h>
#include <errno.h>
#include <ncurses.h>
#include <time.h>
#include <mutex>
#include <iostream>
/*#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
*/


#include "aria_robot_control.h"
#include "point.h"


#include <string>
#include <sstream>


#define DISTANCE_TO_POINT_THRESHOLD 150




namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}



double cur_x, cur_y, cur_z;
std::mutex slam_odom_lock;
double cur_orientation = -1;


//conotrols the robot
//AriaRobotControl robot = AriaRobotControl();


void turn(AriaRobotControl robot, double degrees)
{
  std::cout << "kye turn" <<std::endl;
  robot.turn(degrees);

}//end turn 






void rotate(ros::ServiceClient save_client_1,
         ros::ServiceClient save_client_2,ros::ServiceClient save_client_3,
         double total_degrees, double turn_res, bool ccw, bool save_images,
          double wait_time, rosaria::float_message save_images_srv, int num_kinects,
          ros::Publisher control_pub, geometry_msgs::Twist message )
   
{

//  std_srvs::Empty save_images_srv;

/*  double step_size = turn_res;
  if(!ccw)
  {
    step_size = -step_size;
  }
  int num_turns = total_degrees/turn_res;

  double leftover = total_degrees - (num_turns * turn_res);

  for(int i=0; i<num_turns; i++)
  {
    clock_t begin, end;
    double time_spent;

    begin = clock(); 
   
    std::cout << "KEY  ROTATE STEP: " << step_size << std::endl; 
    robot.wait_until_stopped(ros::Duration(.2));
    //robot.do_rotation(turn_res, turn_res,ccw, save_images);
    robot.turn(step_size);    
    
    //robot.wait_until_stopped(ros::Duration(wait_time));
    message.linear.x = 0;
    message.angular.z = 0;
    control_pub.publish(message);
    //ros::Duration(wait_time).sleep();
    if(save_images)
    {
      end = clock();
      time_spent = ((double)(end) - (double)(begin)) / (double)CLOCKS_PER_SEC;
      //std::cerr << "time rotate: "<< patch::to_string(time_spent) << std::endl;

      std::cerr << "ROTATE time: " << std::endl;
      std::cerr << time_spent << std::endl;
      begin = clock(); 
//    robot.wait_until_stopped(ros::Duration(wait_time));
      ros::Duration(.1).sleep();
      if(!save_client_1.call(save_images_srv))
      { ROS_ERROR("FAILED SAVE SERVICE CALL K1"); }
      if(num_kinects > 1 && !save_client_2.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K2");
      }
      if(!num_kinects > 2 && save_client_3.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL K3");
      }
      end = clock();
      time_spent = ((double)(end) - (double)(begin)) / (double)CLOCKS_PER_SEC;
//      std::cerr << "time save: "<< patch::to_string(time_spent) << std::endl;

      std::cerr << "SAVE: " << std::endl;
      std::cerr << time_spent << std::endl;
    }
    //ros::Duration(wait_time).sleep();
  }//for i 


  if(leftover > 1)
  {

    robot.wait_until_stopped(ros::Duration(.2));
    //robot.do_rotation(leftover, leftover,ccw, save_images);
    if(!ccw)
    {
      leftover = -leftover;
    }
    robot.turn(leftover);
    //robot.wait_until_stopped(ros::Duration(wait_time));
    message.linear.x = 0;
    message.angular.z = 0;
    control_pub.publish(message);
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
*/

}//rotate




void slam_odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg)
{
  ROS_INFO("SLAM ODOM CALLBACK"); 
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  double z = odom_msg->pose.pose.position.z;

//  double qx = odom_msg->pose.pose.orientation.x;
//  double qy = odom_msg->pose.pose.orientation.y;
  double qz = odom_msg->pose.pose.orientation.z;
  double qw = odom_msg->pose.pose.orientation.w;

  double curo = acos(qw) * 180 / M_PI;


  if(qz < 0)
  {
    curo *= -1;
  }
  slam_odom_lock.lock();
  cur_x = x;
  cur_y = y;
  cur_z = z;
  cur_orientation = curo;
  slam_odom_lock.unlock();
}



bool is_at_goal_point(double goal_x, double goal_y, double goal_z)
{
   
  double ss = (cur_x - goal_x) * (cur_x - goal_x) +  (cur_y - goal_y) * (cur_y - goal_y); 
  double dist = sqrt(ss) * 1000;

  return dist < DISTANCE_TO_POINT_THRESHOLD;

}//is at goal point






double get_desired_orientation(double goal_x, double goal_y, double goal_z)
{
  //make current point 0,0
  double gx = goal_x - cur_x;
  double gy = goal_y - cur_y;

  double g_size = sqrt(gx*gx + gy*gy);
  
  double theta = acos(fabs(gx/g_size));//will be <90   
  if(gx < 0)
  {
    theta = M_PI - theta;//add 90 degrees(in radians)
  }  

  if(gy <0)
  {
    theta = -theta;
  }
  return theta;
}//get_desired orientastion






















int main(int argc, char **argv){

  
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh = ros::NodeHandle("~");

  int publish_buffer_size = 10;

  std::string save_1 = "/kinect2_saver/save_service";
  std::string save_2 = "/K2/save_data";
  std::string save_3 = "/K3_save_data/save_service";
  double turn_res = 30;
  double trans_res = .3;
  double one_way_turn_max = 90;

  nh.getParam("base_name_1", save_1);
  nh.getParam("base_name_2", save_2);
  nh.getParam("base_name_3", save_3);
  nh.getParam("turn_res",turn_res);
  nh.getParam("trans_res",trans_res);
    

  //conotrols the robot
//  AriaRobotControl robot = AriaRobotControl();
  AriaRobotControl robot = AriaRobotControl(1);

  //save kinect data
  ros::ServiceClient save_client_1 = nh.serviceClient<rosaria::float_message>(save_1);
  ros::ServiceClient save_client_2 = nh.serviceClient<rosaria::float_message>(save_2);
  ros::ServiceClient save_client_3 = nh.serviceClient<rosaria::float_message>(save_3);
  rosaria::float_message save_images_srv; 



  //SLAM communications
  

//ros::ServiceClient slam_get_odom_client=nh.serviceClient<rtabmap_ros::GetOdom>("/rtabmap/get_odom");
  //rtabmap_ros::GetOdom slam_get_odom_srv;

//  std::mutex slam_odom_lock;
  ros::Subscriber slam_odom_sub = nh.subscribe("/rtabmap/odom",1,slam_odom_callback);

  ros::ServiceClient slam_pause_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/pause");
  ros::ServiceClient slam_resume_client=nh.serviceClient<std_srvs::Empty>("/rtabmap/resume");
  ros::ServiceClient slam_pause_odom_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/pause_odom");
  ros::ServiceClient slam_resume_odom_client=nh.serviceClient<std_srvs::Empty>("/rtabmap/resume_odom");
  std_srvs::Empty empty_srv; 


//  double cur_x, cur_y, cur_z;
  double goal_x, goal_y, goal_z;

//  double cur_orientation = -1;



  bool saved  = false;
  ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",publish_buffer_size);


  //keep track of the robots current linear and angular speed
  double cur_linear = 0.0;
  double cur_angular = 0.0;

  //how much to change the speeds in one step
  double linear_step = .1;
  double angular_step = .1;

  double cluster_id = 0;

  //the message that will be published to the ROS topic
  geometry_msgs::Twist message = geometry_msgs::Twist();

  ros::Rate loop_rate(10);


  bool publish = true;
  bool quit = false;
  while(ros::ok() && !quit)
  {
    ros::spinOnce();
    publish = true; 
    //int c = getch();
    char c;
    scanf(" %c",&c); 
 
    switch(c)
    {
      case 'c': //STOP EVERYTHING 
        cur_linear = 0.0;
        cur_angular = 0.0;
        std::cout << "STOPPED" << std::endl;
        break;
      case 'z': //STOP TRANSLATING
        cur_linear = 0; 
        break;
      case 'x': //STOP ROTATING
        cur_angular = 0; 
        break;
      case 'w': //TRANSLATE FORWARD
        cur_linear += linear_step; 
        break;
      case 's': //TRANSLATE BACKWARD
        cur_linear += -linear_step;
        break;
      case 'a': //ROTATE CCW
        cur_angular += angular_step;
        break;
      case 'd': //ROTATE CLOCKWISE
        cur_angular += -angular_step;
        break;
      case 'e': //ROTATE 2*one_way_turn_max DEGREES, one_way_turn_max CCW(saving), 90 CW, 90 CW(saving), 90 CCW 


        //robot point, save clients (1-3), total_degrees, turn_res, ccw,save,wait_time(per rot), save_images_srv, numkinects, controlpub, message

        //rotate one_way_turn_max and take pictures
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,turn_res,true,true,1, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);
        // rotate back to original position 
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,45,false,false,0, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);
        //rotate one_way_turn_max the other way
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,45,false,false,0, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);
        // rotate back to original position and pitcures
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,turn_res,true,true,1, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);

        publish = false;//dont publish a velocity command

        ROS_ERROR("DONE WITH CURRENT POINT!");
        break;
      case 'r'://ROTATE one_way_turn_max DEGREES, one_way_turn_max CCW(saving), one_way_turn_max CW


        turn(robot,30);

        //ros::Duration(2).sleep();
        //std::cout << "key jsut rturn" << std::endl;
        //robot.turn(30);
/*
        //rotate one_way_turn_max and take pictures
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,turn_res,true,true,1, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);

        ROS_INFO("NOW GO BACK");
        // rotate back to original position 
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,45,false,false,0, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);


        publish = false;//dont publish a velocity command
        ROS_ERROR("DONE WITH HALF POINT!");
  */      break;
      case 't'://ROTATE one_way_turn_max DEGREES, one_way_turn_max CW, one_way_turn_max CCW(saving)
        //rotate one_way_turn_max the other way
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,45,false,false,0, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);
        // rotate back to original position and pitcures
        rotate(save_client_1,save_client_2,save_client_3,one_way_turn_max,turn_res,true,true,1, save_images_srv, 1, control_pub,message);
        message.linear.x = 0;
        message.angular.z = 0;
        control_pub.publish(message);


        publish = false;//dont publish a velocity command

        ROS_ERROR("DONE WITH HALF  POINT!");
        break;
      case 'g': //TRANSLATE trans_res METERS
        robot.do_translation(trans_res * 1000);
        robot.wait_until_stopped(.1);

        publish = false;
        break;
      case 'f': //TRANSLATE BACKWARD trans_res METERS
        robot.do_translation(-trans_res * 1000);
        robot.wait_until_stopped(.1);

        publish = false;
        break;
      case 'v': //ROTATE CCW turn_res DEGREES
        ROS_ERROR("CHOICE V");
        robot.turn(-turn_res);
        robot.wait_until_stopped(.1);

        publish = false;
        break;
      case 'b': //ROTATE CCW turn_res DEGREES
        ROS_ERROR("CHOICE B");
        robot.turn(turn_res);
        robot.wait_until_stopped(.1);

        publish = false;
        break;
















      //SLAM stuff
      case 'p':
        saved = slam_pause_client.call(empty_srv);
        std::cout << "slam pause:  " << saved << std::endl;
        saved = slam_pause_odom_client.call(empty_srv);
        std::cout << "slam pause odom:  " << saved << std::endl;
        break;
      case 'o':
        saved = slam_resume_client.call(empty_srv);
        std::cout << "slam resumed:  " << saved << std::endl;
        saved = slam_resume_odom_client.call(empty_srv);
        std::cout << "slam resumed odom:  " << saved << std::endl;
        break;
      case 'i':
        slam_odom_lock.lock();
        std::cout << "Slam Odom:" << std::endl;
        std::cout << "X: " << cur_x <<  std::endl;
        std::cout << "Y: " << cur_y <<  std::endl;
        std::cout << "Z: " << cur_z <<  std::endl;
        std::cout << "Angle: " << cur_orientation <<  std::endl;
        slam_odom_lock.unlock();
        break;
      
      case 'y':
        cluster_id = cluster_id + 1;
        save_images_srv.request.input = cluster_id;
        std::cout << "saved " << saved << std::endl;
        break;
      //save kinect data
      case '1':
        saved = save_client_1.call(save_images_srv);
        std::cout << "saved " << saved << std::endl;
        break;
      case '2':
        saved = save_client_2.call(save_images_srv);
        std::cout << "saved " << saved << std::endl;
        break;
      case '3':
        saved = save_client_3.call(save_images_srv);
        std::cout << "saved " << saved << std::endl;
        break;
      case '4':
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
