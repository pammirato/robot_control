#include <aria_robot_control.h>





AriaRobotControl::AriaRobotControl() 
{
   //register client
   get_state_client = nh.serviceClient<rosaria::get_state>("/RosAria/get_state");
  move_client = nh.serviceClient<rosaria::float_message>("/RosAria/move");
  heading_client = nh.serviceClient<rosaria::float_message>("/RosAria/heading");
  
  enable_motors_client  = nh.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
  enable_motors_client.call(enable_motors_srv); //enable the motors

  //how long to wait for robot to stop moving
  trans_wait = ros::Duration(WAIT_TO_MOVE_TIME);
  turn_wait = ros::Duration(WAIT_TO_TURN_TIME);
}//end contructor





bool AriaRobotControl::wait_until_stopped(ros::Duration wait, int  max_iterations=100)
{
  wait.sleep();
  int count = 0;
  //get the state of the robot
  while(!get_state_client.call(get_state_srv));
  
  ROS_INFO("IS STOPPED: %d", get_state_srv.response.isStopped); 
  //wait until the robot has stopped moving to issue a move command
  while(!(get_state_srv.response.isStopped) && count < max_iterations)
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


bool AriaRobotControl::do_rotation(double total_degrees, double step_size,
                                    bool ccw)
{
  step_size = abs(step_size);
  int num_turns = total_degrees/step_size;//total turns to do 
   
  int count = 0;//current num of turns executed
 
  if(!ccw)
  {
    step_size = -step_size;
  } 

  while(count < num_turns)
  {
    turn(step_size);
    count++;
  }

  return true;
}//end do_rotation


bool AriaRobotControl::turn(double degrees)
{
  if(!wait_until_stopped(turn_wait))
  { 
    return false;
  }
  float_srv.request.input = degrees;
  if(!heading_client.call(float_srv))
  { 
    ROS_ERROR("FAILED SERVICE CALL: TURN");
    return false;
  }
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





