#include <simple_grid_motion_localization.h>




SimpleGridMotionLocalization::SimpleGridMotionLocalization()
{
  debug = true;


  slam_odom_sub = nh.subscribe("/rtabmap/odom", 1, &SimpleGridMotionLocalization::slam_odom_cb, this);

  //the robots current position
  cur_point = Point();
    
  //initialize the goal points
  int num_rows  = grid_height/grid_res +1;
  int num_cols = grid_width/grid_res +1;


  goal_points = std::vector<Point>(num_rows*num_cols);
  
  bool forward = true;//whether to put in the row forwards or backwards
  //std::vector cur_row(num_cols);//holds the points for the current row
  
  //fill in the goal points row by row
  for (int i=0; i<num_rows; i++)
  {
    for (int j=0; j<num_cols; j++)
    {
      if(forward)
      {
        goal_points[i*num_cols + j] = Point(j,i,0,UNEXPLORED);
      }
      else
      {
        goal_points[i*num_cols + j] = Point(num_rows-1-j,i,0,UNEXPLORED);
      }
    } 

    forward = !forward;
    
  }//for i       



/*  for (unsigned int i=0; i<goal_points.size(); i++)
  {
    goal_points[i] = Point((i%num_cols)*grid_res,  
                 floor(i/num_cols)*grid_res, 0, UNEXPLORED);
  }//for i       
*/

/*  //allocate the memory 
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
*/
}



//main logic here

//GOAL: visit every grid point, turn 360 and back
int SimpleGridMotionLocalization::run()
{

  ROS_INFO("RUNNNING LOCALIZATION"); 
  bool planning_active = true;
  int i = 0;
 // for(unsigned int i=0;i<goal_points.size(); i++)
  while(ros::ok())
  {
    ros::spinOnce();  

    if(planning_active)
    {
      ROS_INFO("HWEERE");
      Point goal_point = goal_points[i];
      int counter = 0;
      while(!is_at_point(goal_point) && counter < 10)
      {
        ros::spinOnce();  
        ROS_ERROR("not at point");
        robot.wait_until_stopped(.5);


        //LOCK ACQQUIREDI
        pthread_mutex_lock(&position_lock); 
       
        ROS_INFO("LOCK GAINED");
        double desired_orientation = get_desired_orientation(goal_point); 
        double orientation_diff =desired_orientation - cur_orientation ;
        
        double dist = cur_point.distance_to_2d(goal_point);  



        pthread_mutex_unlock(&position_lock); 
        //LOCK RELEASSED


        ROS_INFO("LOCK RELEASED");
        if(debug)
        {
          ROS_INFO("DO: %f, OD: %f, DIST: %f", desired_orientation, orientation_diff, dist);
          ros::Duration(10).sleep();
        }


        if(!(orientation_diff < ORIENTATION_THRESHOLD_RADIANS))
        {
          ROS_INFO( "change orientation");
          double angle = orientation_diff * 180 / M_PI;
          bool ccw = true;
          if(angle < 0)
          {
            ccw = false;
          }

          //only want to turn by turn_res at most
          int num_rots = angle/turn_res;//num or roatations using turn_res
          int subtotal = turn_res*num_rots;//total angle turned
          int leftover = angle - subtotal;//amount left to turn < turn_res
 
          rotate(subtotal,turn_res, ccw, false);
          rotate(leftover, leftover, ccw, false);
        }//if oreintation 

        robot.do_translation(dist*METERS_TO_MILLIMETERS);
        robot.wait_until_stopped(.5);

      }//while  
      i++;  
      if(i >=goal_points.size())
      {
        ROS_INFO("done planning");
        planning_active = false;
      }//if i 
      ROS_INFO("end of loop");
    }//if planning_active 
  }//for i 


 return 1; 

}//end run






void SimpleGridMotionLocalization::slam_odom_cb(nav_msgs::Odometry::ConstPtr  odom_msg)
{
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  double z = odom_msg->pose.pose.position.z;

  double qx = odom_msg->pose.pose.orientation.x;
  double qy = odom_msg->pose.pose.orientation.y;
  double qz = odom_msg->pose.pose.orientation.z;
  double qw = odom_msg->pose.pose.orientation.w;


  ROS_INFO("Poisition(x,y,z): %f, %f, %f\n ", x,y,z);

  ROS_INFO("Quaternion: %f, %f, %f, %f\n",qx,qy,qz,qw);
  ROS_INFO("Approx Angle: %f\n\n\n",2*acos(qw));


  pthread_mutex_lock(&position_lock);
  cur_point.x = x;
  cur_point.y = y;
  cur_point.z = z;

  cur_orientation = 2*acos(qw);
  pthread_mutex_unlock(&position_lock);

}//slam_odomo_cb





double SimpleGridMotionLocalization::get_desired_orientation(Point goal_point)
{
  return acos(abs(cur_point.x - goal_point.x)/cur_point.distance_to_2d(goal_point));
}//get_desired orientastion


bool SimpleGridMotionLocalization::is_at_point(Point goal_point)
{

  ROS_INFO("cx: %f gx: %f", cur_point.x, goal_point.x);

  return cur_point.distance_to_2d(goal_point)*METERS_TO_MILLIMETERS <  DISTANCE_TO_POINT_THRESHOLD_MM;

}//is_at_point





int main(int argc, char **argv){


  ros::init(argc, argv, "controller");

//  ros::AsyncSpinner spinner(2); // Use 4 threads
//  spinner.start();
  SimpleGridMotionLocalization sgml = SimpleGridMotionLocalization();
  sgml.run();
//  while(ros::ok())
 //   ros::spinOnce();  
  ros::shutdown();
  return 0;

}//end main
