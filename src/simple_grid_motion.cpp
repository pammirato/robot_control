#include <simple_grid_motion.h>




SimpleGridMotion::SimpleGridMotion()
{
  //conotrols the robot
  robot = AriaRobotControl();

  //defaults
  grid_width = .5; 
  grid_height = .5; 
  grid_res = .5;
  turn_res = 15;
  trans_res = 500;


  turn_offset = .1;

  kinect_base_name_1 = DEFAULT_KINECT_BASE_NAME;
  kinect_base_name_2 = "";
  kinect_base_name_3 = "";


  nh = ros::NodeHandle("~");




  //set parameters from command line
  nh.getParam("grid_width", grid_width);
  nh.getParam("grid_height", grid_height);
  nh.getParam("grid_res", grid_res);
  nh.getParam("turn_res", turn_res);
  nh.getParam("trans_res", trans_res);
  nh.getParam("turn_offset", turn_offset);
  nh.getParam("base_name_1",kinect_base_name_1);
  nh.getParam("base_name_2",kinect_base_name_2);
  nh.getParam("base_name_3",kinect_base_name_3);
   



  ROS_INFO("PARAMS W:%f  H:%f GR:%f TR:%f TO:%f", grid_width, grid_height, grid_res, turn_res, turn_offset);
 
  save_client_1 = nh.serviceClient<std_srvs::Empty>("/" + kinect_base_name_1 + "/save_images"); 
  save_client_2 = nh.serviceClient<std_srvs::Empty>("/" + kinect_base_name_2 + "/save_images"); 
  save_client_3 = nh.serviceClient<std_srvs::Empty>("/" + kinect_base_name_3 + "/save_images"); 

}//contructor



//main logic here

//GOAL: visit every grid point, turn 360 and back
int SimpleGridMotion::run()
{

  ROS_INFO("RUNNING BASIC");


  bool left = true;//whether to go left or right
  int num_cols = grid_width/grid_res;
  int num_rows = grid_height/grid_res;
  int num_rows_done = 0;
  double trans_dist = grid_res;
  double end_dist = grid_res - turn_offset;



  //while we have not traversed every row 
  while(num_rows_done < num_rows)
  {

    //whether we are going left or right
    if(!left)
    {
      trans_dist *= -1;
    }

    //360, then go to next grid point
    for(int i=0;i<num_cols;i++)
    {
      rotate(360, turn_res,true,true);
      rotate(360, turn_res,false,false);
      ROS_INFO("ROT 2 DONE");
      robot.do_translation(trans_dist*METERS_TO_MILLIMETERS);
    }//end for i 


    //360
    rotate(360, turn_res,true, true);
    rotate(360, turn_res,false,false);

    //go to next row in grid
    rotate(90,turn_res,true,false);
    robot.do_translation(end_dist*METERS_TO_MILLIMETERS);
    rotate(90,turn_res,false,false);

    //go in the other direction
    left = !left;

    num_rows_done++;
  }//end while num_rows_done 

 return 1; 

}//end run






void SimpleGridMotion::rotate(double total_degrees, double turn_res, bool ccw, bool save_images, double wait_time)
{
  int num_turns = total_degrees/turn_res;
  for(int i=0; i<num_turns; i++)
  {
    robot.do_rotation(turn_res, turn_res,ccw);
    robot.wait_until_stopped(ros::Duration(wait_time));
    ros::Duration(wait_time).sleep();
    if(save_images)
    {
      robot.wait_until_stopped(ros::Duration(wait_time));
      ros::Duration(wait_time).sleep();
      if(!save_client_1.call(save_images_srv))
      {
        ROS_ERROR("FAILED SAVE SERVICE CALL %s", kinect_base_name_1.c_str());
      } 
      save_client_2.call(save_images_srv); 
      save_client_3.call(save_images_srv); 
    }
    //ros::Duration(wait_time).sleep();
  }//for i 


}//rotate












/*int main(int argc, char **argv){


  ros::init(argc, argv, "controller");

  SimpleGridMotion sgm = SimpleGridMotion();
  sgm.run();
  ros::shutdown();
  return 0;

}//end main
*/
