#include <simple_grid_motion.h>


#define WAIT_TO_MOVE_TIME .5
#define WAIT_TO_TURN_TIME .2
#define MOVE_DISTANCE 100
#define ROTATION_ANGLE 45
#define CHANGE_ROW_OFFSET -50



SimpleGridMotion::SimpleGridMotion() {}


bool SimpleGridMotion::initialize()
{

  nh = ros::NodeHandle("~");

  //set parameters from command line
  nh.getParam("width", grid_width);
  nh.getParam("height", grid_height);
  nh.getParam("resolution", grid_res);
  nh.getParam("rotation_angle", turn_res);
 
  ROS_INFO("PARAMS W:%f  H:%f GR:%f TR:%f", grid_width, grid_height, grid_res, turn_res);
  
   //register client
   get_state_client = nh.serviceClient<rosaria::get_state>("/RosAria/get_state");
  move_client = nh.serviceClient<rosaria::float_message>("/RosAria/move");
  heading_client = nh.serviceClient<rosaria::float_message>("/RosAria/heading");
  
  enable_motors_client  = nh.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
  enable_motors_client.call(enable_motors_srv); //enable the motors

  //how long to wait for robot to stop moving
  trans_wait = ros::Duration(WAIT_TO_MOVE_TIME);
  turn_wait = ros::Duration(WAIT_TO_TURN_TIME);
  

  return true;
}//end initialize



bool SimpleGridMotion::wait_until_stopped(ros::Duration wait, int  max_iterations=100)
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

}//end is robot stopped


bool SimpleGridMotion::do_rotation(double total_degrees, double step_size,
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


bool SimpleGridMotion::turn(double degrees)
{
  if(!wait_until_stopped(turn_wait))
  { 
    return false;
  }
  float_srv.request.input = degrees;
  if(!heading_client.call(float_srv))
  { 
   return false;
  }
  return true;
}//end turn



bool SimpleGridMotion::do_translation(double millimeters)
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
}





int SimpleGridMotion::run()
{
  if(!initialize())
  {
    return false;
  }

  ROS_INFO("HERE");
  do_rotation(90,45,true);
  do_translation(250);

  ROS_INFO("HERE2");

 
/*
  while(ros::ok())
  {

    if(!wait_until_stopped(trans_wait, 100))
    {
      return -1;
    }

    ros::spinOnce();//so our callbacks get called
  }
*/

}//end run









int main(int argc, char **argv){


  ros::init(argc, argv, "controller");

  SimpleGridMotion sgm = SimpleGridMotion();
  sgm.run();

  return 0;

}//end main
