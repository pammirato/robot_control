#include <simple_grid_motion.h>




SimpleGridMotion::SimpleGridMotion()
{
  initialize();
}



//main logic here

//GOAL: visit every grid point, turn 360 and back
int SimpleGridMotion::run()
{



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
      robot.do_rotation(361, turn_res,true,true);
      robot.do_rotation(361, 90,false);
      ROS_INFO("ROT 2 DONE");
      robot.do_translation(trans_dist*METERS_TO_MILLIMETERS);
    }//end for i 


    //360
    robot.do_rotation(361, turn_res,true, true);
    robot.do_rotation(361, 90,false);

    //go to next row in grid
    robot.do_rotation(90,90,true);
    robot.do_translation(end_dist*METERS_TO_MILLIMETERS);
    robot.do_rotation(90,90,false);



    num_rows_done++;
  }//end while num_rows_done 

 return 1; 

}//end run



//initialize the ros node and paramterrs
bool SimpleGridMotion::initialize()
{

  //conotrols the robot
  robot = AriaRobotControl();

  //defaults
  grid_width = 1;  
  grid_height = 1;
  grid_res = .5;
  turn_res = 45;
  turn_offset = .1;
  nh = ros::NodeHandle("~");

  //set parameters from command line
  nh.getParam("grid_width", grid_width);
  nh.getParam("grid_height", grid_height);
  nh.getParam("grid_res", grid_res);
  nh.getParam("turn_res", turn_res);
  nh.getParam("turn_offset", turn_offset);
  ROS_INFO("PARAMS W:%f  H:%f GR:%f TR:%f TO:%f", grid_width, grid_height, grid_res, turn_res, turn_offset);
 
  save_client = nh.serviceClient<std_srvs::Empty>("save_images"); 

  return true;
}//end initialize






int main(int argc, char **argv){


  ros::init(argc, argv, "controller");

  SimpleGridMotion sgm = SimpleGridMotion();
  sgm.run();

  return 0;

}//end main
