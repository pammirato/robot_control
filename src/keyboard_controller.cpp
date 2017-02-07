
/*
*
* EDITED BY PHIL AMMIRATO   ammirato@cs.unc.edu
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <errno.h>
//#include <ncurses.h>
#include <time.h>

#include <fstream>
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
//#include <rtabmap_ros/GetOdom.h>
#include "rosaria/float_message.h"


#include "aria_robot_control.h"
#include "point.h"

#define METERS_TO_MILLIMETERS 1000

class Controller 
{
public:

private:


  ros::NodeHandle nh;

  //save kinect data
  std::string save_1;
 // std::string save_2;
  //std::string save_3;
  ros::ServiceClient save_client_1;
//  ros::ServiceClient save_client_2;
//  ros::ServiceClient save_client_3;
  rosaria::float_message save_images_srv; 


  //SLAM communications
  ros::Subscriber slam_odom_sub;

  //ros::ServiceClient slam_pause_client;
  //ros::ServiceClient slam_resume_client;
  //ros::ServiceClient slam_pause_odom_client;
  //ros::ServiceClient slam_resume_odom_client;
  std_srvs::Empty empty_srv; 


  int publish_buffer_size;
  ros::Publisher control_pub; 
  //the message that will be published to the ROS topic
  geometry_msgs::Twist control_message;
  
  //conotrols the robot
  AriaRobotControl robot;






  double cur_x, cur_y, cur_z,cur_orientation;
//  double goal_x, goal_y, goal_z;
  std::mutex slam_odom_lock;


  //how much to turn,tranlste, rotate at once  
  double turn_res, trans_res, one_way_turn_max;


  //keep track of the robots current linear and angular speed
  double cur_linear;
  double cur_angular;

  //how much to change the speeds in one step
  double linear_step;
  double angular_step;


  int cluster_id;



public:
  Controller(double turn_res, double trans_res, double one_way_turn_max, int start_cluster_id)
    : turn_res(turn_res), trans_res(trans_res), one_way_turn_max(one_way_turn_max),
     nh("~"), cur_x(-1), cur_y(-1), cur_z(-1), cur_orientation(-1), cluster_id(start_cluster_id),
     cur_linear(0), cur_angular(0), linear_step(.1), angular_step(.1) 
  {

  save_1 = "/kinect2_saver/save_service";
 // std::string save_2;
  //std::string save_3;
  save_client_1 = nh.serviceClient<rosaria::float_message>(save_1);
//  ros::ServiceClient save_client_2;
//  ros::ServiceClient save_client_3;


  //SLAM communications

//  slam_odom_sub = nh.subscribe("/rtabmap/odom",1,slam_odom_callback);
//  slam_odom_sub = nh.subscribe("/rtabmap/odom",1,boost::bind(&Controller::slam_odom_callback, this, _1));
  //slam_odom_sub = nh.subscribe("/rtabmap/odom",1,&Controller::slam_odom_callback, this);
  slam_odom_sub = nh.subscribe("/RosAria/pose",1,&Controller::rosaria_odom_callback, this);
//  slam_pause_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/pause");
//  slam_resume_client=nh.serviceClient<std_srvs::Empty>("/rtabmap/resume");
//  slam_pause_odom_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/pause_odom");
//  slam_resume_odom_client=nh.serviceClient<std_srvs::Empty>("/rtabmap/resume_odom");

 
  publish_buffer_size = 10;
  control_pub =  nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",publish_buffer_size);
 
  //the message that will be published to the ROS topic
  control_message  = geometry_msgs::Twist();
  
  //conotrols the robot
  robot = AriaRobotControl();

 


  }

  ~Controller()
  {
  }

  void run()
  {
    start();
    stop();
  }

private:
  void start()
  {
    std::cout << "start! " << std::endl;

    char c;
    bool quit = false;
    int num_loops = 0;
    
    ros::Rate loop_rate(10);

    while(ros::ok() && !quit)
    {
      ros::spinOnce();
      scanf(" %c",&c); 
   
      switch(c)
      {



      //VELOCITY COMMANDS


        case 'c': //STOP EVERYTHING 
          stop_all();
          std::cout << "STOPPED" << std::endl;
          break;
        case 'z': //STOP TRANSLATING
          cur_linear = 0; 
          publish_velocities();
          break;
        case 'x': //STOP ROTATING
          cur_angular = 0; 
          publish_velocities();
          break;
        case 'w': //TRANSLATE FORWARD
          cur_linear += linear_step; 
          publish_velocities();
          break;
        case 's': //TRANSLATE BACKWARD
          cur_linear += -linear_step;
          publish_velocities();
          break;
        case 'a': //ROTATE CCW
          cur_angular += angular_step;
          publish_velocities();
          break;
        case 'd': //ROTATE CLOCKWISE
          cur_angular += -angular_step;
          publish_velocities();
          break;




      //SINGLE CLUSTER ROTATE COMMANDS 

      //rotate(total_degress, ccw, save)
        
        case 'e': //ROTATE COUNTER CLOCKWISE ONE_TURN MAX(half cluster)
        
          rotate(one_way_turn_max,true,true);//rotate and save
          //rotate(one_way_turn_max,false,false);//go back and dont save
          break;
        case 'r': //ROTATE CLOCKWISE ONE_TURN MAX(half cluster)
          rotate(one_way_turn_max,false,true);//rotate and save
          //rotate(one_way_turn_max,true,false);//go back and dont save
          break;
        case 't': //ROTATE CLOCKWISE ONE_TURN MAX(half cluster)
          rotate_full_cluster();
          break;
        case 'v': //ROTATE COUNTER CLOCKWISE TURN RES (one point) 
          rotate(turn_res,true,true);//rotate and save
          break;
        case 'b': //ROTATE CLOCKWISE 2*ONE_TURN MAX(one point)
          rotate(turn_res,false,true);//rotate and save
          break;
        case 'V': //ROTATE COUNTER CLOCKWISE TURN RES (one point) 
          rotate(turn_res,true,false);//rotate and dont save
          break;
        case 'B': //ROTATE CLOCKWISE 2*ONE_TURN MAX(one point)
          rotate(turn_res,false,false);//rotate and dont save
          break;
        case 'h': //ROTATE CLOCKWISE 2*ONE_TURN MAX(one point)
          turn_to_zero();
          break;


        //SINGLE MOVE TRANSLATE COMMANDS

        case 'f': //TRANSLATE BACKWARD TRANS_RES
          //break;
          //increment_cluster_id(1);
          slow_translate(-trans_res);
          break;
        case 'g': //TRANSLATE FORWARD TRANS_RES
          break;
          //increment_cluster_id(1);
          slow_translate(trans_res);
          //break;





        //COMPOUND COMMANDS

        case 'j': //TRANSLATE FORWARD AND ROTATE FULL CLUSTER 
          //move to new spot 
          increment_cluster_id(1);
          slow_translate(trans_res);
          
          //for extra safety
          stop_all(); 
          robot.wait_until_stopped(.5,5);
   
          rotate_full_cluster();
          break;
        case 'k': //TRANSLATE FORWARD AND ROTATE FULL CLUSTER, REPEAT
          //get number of times to repeat 
          scanf(" %i",&num_loops); 
          
          //for safety just stop  
          stop_all(); 
          robot.wait_until_stopped(.1,5);

          for(int jj = 0; jj<num_loops; jj++)
          { 
            increment_cluster_id(1);
            rotate_full_cluster();
            
            robot.wait_until_stopped(.1,5);
            
            //move to new spot 
            slow_translate(trans_res);
            
            robot.wait_until_stopped(.5,5);
            stop_all(); 
          }//for jj
            increment_cluster_id(1);
            rotate_full_cluster();
            
          break;
        case 'l': //TRANSLATE FORWARD AND ROTATE FULL CLUSTER, REPEAT
          //get number of times to repeat 
          scanf(" %i",&num_loops); 
          
            //move to new spot 
            slow_translate(trans_res);
         
           //for safety just stop  
          stop_all(); 
          robot.wait_until_stopped(.1,5);

          for(int jj = 0; jj<num_loops; jj++)
          { 
            increment_cluster_id(1);
            rotate_full_cluster();
            
            robot.wait_until_stopped(.1,5);
            
            //move to new spot 
            slow_translate(trans_res);
            
            robot.wait_until_stopped(.1,5);
            stop_all(); 
          }//for jj
            increment_cluster_id(1);
            rotate_full_cluster();
            
          break;






        //DENSITY COMMANDS (PATH COMMANDS)
        case 'p'://go forward every 2m, stopping every 2cm to take an image, never turn
          increment_cluster_id(1);
          save_images();
          for(int jj = 0; jj<100; jj++) 
          {
            increment_cluster_id(1);
            slow_translate(.02);
            robot.wait_until_stopped(2.0);
            save_images();
          }
          std::cout << "DONE K" << std::endl;
          break;
        case 'o'://go forward every 1m, stopping every 2cm to take an image, never turn
          increment_cluster_id(1);
          save_images();
          for(int jj = 0; jj<50; jj++) 
          {
            increment_cluster_id(1);
            slow_translate(.02);
            robot.wait_until_stopped(2.0);
            save_images();
          }
          std::cout << "DONE K" << std::endl;
          break;




        //Odometry / SLAM CONTROLS

        /*case 'p':
          success = slam_pause_client.call(empty_srv);
          std::cout << "slam pause:  " << success << std::endl;
          success = slam_pause_odom_client.call(empty_srv);
          std::cout << "slam pause odom:  " << success << std::endl;
          break;
        case 'o':
          success = slam_resume_client.call(empty_srv);
          std::cout << "slam resumed:  " << success << std::endl;
          success = slam_resume_odom_client.call(empty_srv);
          std::cout << "slam resumed odom:  " << success << std::endl;
          break; */
        case 'i':
          slam_odom_lock.lock();
          std::cout << "Odom:" << std::endl;
          std::cout << "X: " << cur_x <<  std::endl;
          std::cout << "Y: " << cur_y <<  std::endl;
          std::cout << "Z: " << cur_z <<  std::endl;
          std::cout << "Angle: " << cur_orientation <<  std::endl;
          slam_odom_lock.unlock();
          break;
        



        //META DATA CONTROLS

        case 'y'://INCREMENT CLUSTERID
          increment_cluster_id(1);
          break;
        case 'u'://DECREMENT CLUSTERID
          increment_cluster_id(-1);
          break;



        case 'P':
          std::cout << "Begin video capture" << std::endl;
          for (int jj=1; jj<9000; jj++)
          {
            save_images();
            ros::Duration(.05).sleep();
          }
          std::cout << "DONE VIDEO CAPTURE!!!!!" << std::endl;
        case 'q': //quit
          quit = true;
          break;
       }//switch c

      ros::spinOnce();
      loop_rate.sleep();
    }//while
  }//start

  void stop()
  {
    std::cout << "stop! " << std::endl;
/*    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }*/
  }//stop



  void increment_cluster_id(int x)
  {
    cluster_id = cluster_id + x;
    save_images_srv.request.input = cluster_id;
    std::cout << "cluster id:  " << cluster_id << std::endl;
  }

  void publish_velocities()
  {
    std::cout << "publish: " << std::to_string(cur_linear) << std::endl;
    control_message.linear.x = cur_linear;
    control_message.angular.z = cur_angular;
    control_pub.publish(control_message);

  }//publish velocities 



  void save_images()
  {

      if(!save_client_1.call(save_images_srv))
      { ROS_ERROR("FAILED SAVE SERVICE CALL K1"); }
      //if(num_kinects > 1 && !save_client_2.call(save_images_srv))
     // {
     //   ROS_ERROR("FAILED SAVE SERVICE CALL K2");
     // }
     // if(!num_kinects > 2 && save_client_3.call(save_images_srv))
     // {
     //   ROS_ERROR("FAILED SAVE SERVICE CALL K3");
     // }
  }//save_images


  void stop_all()
  {
    cur_linear = 0;
    cur_angular = 0;
    publish_velocities();
  }//stop all

 

  void slow_translate(double meter_distance)
  {
    //robot.set_max_speeds(30,10);
    robot.do_translation(meter_distance*1000);
    std::cout << "here" << std::endl;
    robot.wait_until_stopped(ros::Duration(.2), 10);
    //robot.set_max_speeds(1000,10);
  }//slow_move

 
  void rotate(double total_degrees,bool ccw, bool save)
  {


    double wait_time = .5;
    //how much to turn at once
    double step_size = turn_res;
    if(!ccw)
    {
      step_size = -step_size;
    }

    //if we are saving, slow down the turns for clear pictures
    if(save)
    {
      //robot.set_max_speeds(0,15); 
    }
    else
    {
      step_size = 1.5*step_size;
      wait_time = .1;
    }
    


    //total num turns needed
    int num_turns = total_degrees/abs(step_size);

    //aything thats left over after integer division
    double leftover = total_degrees - (num_turns * abs(step_size));


    //do num_turns turns
    for(int i=0; i<num_turns; i++)
    {
     
      robot.wait_until_stopped(ros::Duration(wait_time));
      robot.turn(step_size);    
      robot.wait_until_stopped(ros::Duration(wait_time));
      //just make sure we are stopped
      stop_all();
      robot.wait_until_stopped(ros::Duration(wait_time));
      
      if(save)
      {
        save_images();
      } 
 
    }//for i 

    //do the leftover turn
    if(leftover > 10)
    {

      robot.wait_until_stopped(ros::Duration(wait_time));
      if(!ccw)
      {
        leftover = -leftover;
      }
     
      robot.wait_until_stopped(ros::Duration(wait_time));
      robot.turn(leftover);
      robot.wait_until_stopped(ros::Duration(wait_time));
      //just make sure we are stopped
      stop_all();
      robot.wait_until_stopped(ros::Duration(wait_time));
      
     // if(save)
      //{  
       // save_images();
      //} 

    }

    //reset the max speed to something high
   // robot.set_max_speeds(0,150); 
  }//rotate


  void rotate_full_cluster()
  {
    //make sure odom callback is called
    ros::spinOnce();
    //get current orientation, to make sure we end up there later

    //rotate and capture 
    rotate(12*turn_res,true,true);//rotate and save
//    rotate(one_way_turn_max,true,true);//rotate and save
//    rotate(one_way_turn_max,false,false);//go back and dont save
//    rotate(one_way_turn_max,false,false);//rotate to new position 
//    rotate(turn_res,false,false);//rotate to new position, extra to make sure we get coverage
 //   rotate(one_way_turn_max,true,true);//go back and save
 //   rotate(2*turn_res,true,true);//rotate to new position 


    //rotate(15,true,true);

    //make sure we return to the original orientation
//    turn_to_angle(org_orientation);



  }//roatate full cluster



/*  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }*/

  /*void slam_odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg)
  {
    //ROS_INFO("SLAM ODOM CALLBACK"); 
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    double z = odom_msg->pose.pose.position.z;

  //  double qx = odom_msg->pose.pose.orientation.x;
  //  double qy = odom_msg->pose.pose.orientation.y;
    double qz = odom_msg->pose.pose.orientation.z;
    double qw = odom_msg->pose.pose.orientation.w;

    double curo = 2*acos(qw) * 180 / M_PI;


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
  }*/


  void rosaria_odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg)
  {
    ROS_INFO("ARIA ODOM CALLBACK"); 
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    double z = odom_msg->pose.pose.position.z;

  //  double qx = odom_msg->pose.pose.orientation.x;
  //  double qy = odom_msg->pose.pose.orientation.y;
    double qz = odom_msg->pose.pose.orientation.z;
    double qw = odom_msg->pose.pose.orientation.w;

    double curo = 2*acos(qw) * 180 / M_PI;


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
  }//rosaria slam cb
 

  void turn_to_zero()
  {
    ros::spinOnce();//make sure odom callback gets called

    robot.turn(-cur_orientation); 

  }//turn to zero


  void turn_to_angle(double angle)
  {
    ros::spinOnce();//make sure odom callback gets called



    robot.turn(angle - cur_orientation); 

  }//turn to zero
};





int main(int argc, char **argv)
{



  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);
//  ros::init(argc, argv, "controller");

  if(!ros::ok())
  {
    return 0;
  }

  int start_cluster_id = 0;

  if(argc > 1)
  {
    start_cluster_id = atoi(argv[1]);
  }
    



  double turn_res = 31.2;
//  turn_res = 31.4;
// turn_res = 33.0;
  double trans_res = .30;
  double one_way_turn_max = 360;

  Controller controller(turn_res,trans_res,one_way_turn_max, start_cluster_id);

  ROS_INFO("starting controller...");
  controller.run();

  ros::shutdown();
  return 0;
}

