#include <aria_robot_control.h>



AriaRobotControl::AriaRobotControl() 
{
   //register client
   get_state_client = nh.serviceClient<rosaria::get_state>("/RosAria/get_state");
  move_client = nh.serviceClient<rosaria::float_message>("/RosAria/move");
  heading_client = nh.serviceClient<rosaria::float_message>("/RosAria/heading");
  max_speeds_client = nh.serviceClient<rosaria::float_message>("/RosAria/max_speeds");
  
  enable_motors_client  = nh.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
  enable_motors_client.call(enable_motors_srv); //enable the motors

  save_images_client = nh.serviceClient<std_srvs::Empty>("/save_images");



  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
  cmd_vel_message = geometry_msgs::Twist();

  //how long to wait for robot to stop moving
  trans_wait = ros::Duration(WAIT_TO_MOVE_TIME);
  turn_wait = ros::Duration(WAIT_TO_TURN_TIME);
}//end contructor





bool AriaRobotControl::wait_until_stopped(ros::Duration wait, int  max_iterations)
{
  wait.sleep();
  int count = 0;
  //get the state of the robot
  while(!get_state_client.call(get_state_srv));
  
  //std::cout << "IS STOPPED: " <<  get_state_srv.response.isStopped << std::endl; 
  //wait until the robot has stopped moving to issue a move command
  while((!(get_state_srv.response.isStopped) || !(get_state_srv.response.isHeadingDone) \
        || !(get_state_srv.response.isMoveDone)) && count < max_iterations)
  {
    ROS_INFO("ROBOT STILL moving");
    wait.sleep();
    while(!get_state_client.call(get_state_srv)){
      ROS_INFO("failed service call!");
    }
    count++;
  } 
  if(get_state_srv.response.isStopped)
  {
    return true;
  } 
  else
  {
    return false;
  }
}//end is robot stopped



bool AriaRobotControl::wait_until_stopped(double seconds, int  max_iterations)
{
  return wait_until_stopped(ros::Duration(seconds), max_iterations);
}





bool AriaRobotControl::do_rotation(double total_degrees, double step_size,bool ccw, bool save_images)

{
  step_size = abs(step_size);
  int num_turns = total_degrees/step_size;//total turns to do 
  int count = 0;//current num of turns executed
  double max_speed = .15;

 
  if(!ccw)
  {
   step_size = -step_size;
    max_speed = -max_speed;
  } 

  while(count < num_turns)
  {
    if(!wait_until_stopped(turn_wait))
    { 
      return false;
    }
    else
    {
      //ros::Duration(.5).sleep();//just for extra saftey
     // if(!save_images)
     // {
     //   turn(step_size);
     // }
     // else
     // {
     //   turn(step_size,.15);  
     // }
     std::cout << "aria control do_rotate turn step size: " <<  step_size<< std::endl;
   //  turn(step_size,max_speed);  
     turn(step_size);  
      count++;
    }
  }

  return true;
}//end do_rotation

//GIVES TURN COMMNAD IMMEADITELY
bool AriaRobotControl::turn(double degrees)
{
  std::cout << "aRAI contril TURN FUNCION degrees: " << degrees << std::endl;
  float_srv.request.input = degrees;
  if(!heading_client.call(float_srv))
  { 
    ROS_ERROR("FAILED SERVICE CALL: TURN");
    return false;
  }
  return true;
}//end turn




//turn at a certain speed
bool AriaRobotControl::turn(double degrees, double max_velocity)
{
  double seconds = abs( ((degrees * M_PI) / 180)/max_velocity);
  std::cout << "ROBOT TURN FUNCION2 " << degrees << std::endl;
  
  cmd_vel_message.angular.z = max_velocity;
  cmd_vel_pub.publish(cmd_vel_message);
  
  ros::Duration(seconds).sleep();

  cmd_vel_message.angular.z = 0;
  cmd_vel_pub.publish(cmd_vel_message);

  return true;
}//end turn





bool AriaRobotControl::do_translation(double millimeters)
{

  if(!wait_until_stopped(trans_wait))
  { 
    return false;
  }
  float_srv.request.input = millimeters;
  if(!move_client.call(float_srv))
  { 
   return false;
  }

  return true;
}//end do translation


bool AriaRobotControl::set_max_speeds(double max_trans, double max_rot)
{
  max_speeds_srv.request.input = max_trans;
  max_speeds_srv.request.input2 = max_rot;

  if(!max_speeds_client.call(max_speeds_srv))
  { 
    return false;
  } 

  return true;
}//set_max_speeds
