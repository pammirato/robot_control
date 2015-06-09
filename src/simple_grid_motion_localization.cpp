#include <simple_grid_motion_localization.h>

 
std::mutex mtx;

 
//initialize the ros node and paramterrs
bool SimpleGridMotionLocalization::initialize()
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

  //the robots current position
  cur_point = Point();
    
  //initialize the goal points
  int num_rows  = grid_height/grid_res;
  int num_cols = grid_width/grid_res;
  //allocate the memory 
  goal_points = new Point* [num_rows];
  for(int i=0; i<num_rows; i++)
  {
    goal_points[i] = new Point[num_cols];
  }  

  
  for(int i=0; i<num_rows; i++)
  {
    for(int j=0; j<num_cols; j++)
    {
      goal_points[i][j] = Point(i*grid_res,j*grid_res,0,UNEXPLORED);
    }//for j
  }//for i   
 

//  SimpleGridMotionLocalization::mtx;//iI = std::mutex();
  return true;
}//end initialize




void SimpleGridMotionLocalization::slam_odom_cb(nav_msgs::Odometry & msg)
{
  mtx.lock();

  cur_point.x = msg.pose.pose.position.x;
  cur_point.y = msg.pose.pose.position.y;

  mtx.unlock();  

}





SimpleGridMotionLocalization::SimpleGridMotionLocalization()
{
  initialize();
}


SimpleGridMotionLocalization::~SimpleGridMotionLocalization()
{
  for(int i=0; i<(grid_height/grid_res); i++)
  {
   delete[] goal_points[i];
  }  
  delete[] goal_points; 
}


//main logic here

//GOAL: visit every grid point, turn 360 and back
int SimpleGridMotionLocalization::run()
{



  bool left = true;//whether to go left or right
  int num_cols = grid_width/grid_res;
  int num_rows = grid_height/grid_res;
  int num_rows_done = 0;
  double trans_dist = grid_res;
  double end_dist = grid_res - turn_offset;



  //while we have no traversed every row 
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
      robot.do_rotation(361, turn_res,true);
      robot.do_rotation(361, 90,false);
      ROS_INFO("ROT 2 DONE");
      robot.do_translation(trans_dist*METERS_TO_MILLIMETERS);
    }//end for i 


    //360
    robot.do_rotation(361, turn_res,true);
    robot.do_rotation(361, turn_res,false);

    //go to next row in grid
    robot.turn(90);
    robot.do_translation(end_dist*METERS_TO_MILLIMETERS);
    robot.turn(-90);



    num_rows_done++;
  }//end while num_rows_done 

  
  return true;
}//end run









int main(int argc, char **argv){


  ros::init(argc, argv, "controller");

  SimpleGridMotionLocalization sgm = SimpleGridMotionLocalization();
  sgm.run();

  return 0;

}//end main
