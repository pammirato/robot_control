#include <simple_grid_motion_localization.h>




SimpleGridMotionLocalization::SimpleGridMotionLocalization()
{
  debug = true;

  max_rotate_180 = true;
  slam_turn_res = 15;
  filename = "";

  nh.getParam("slam_turn_res", slam_turn_res);
  nh.getParam("filename", filename);
  nh.getParam("max_rotate_180", max_rotate_180);

  trans_forward = true;
  turn_ccw = true;
  
  slam_odom_client =nh.serviceClient<rtabmap_ros::GetOdom>("/rtabmap/get_odom");


  slam_pause_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/pause_odom");

  slam_resume_client = nh.serviceClient<std_srvs::Empty>("/rtabmap/resume_odom");


  //the robots current position
  cur_point = Point();







      
  //initialize the goal points
 
  if(filename != "") 
  {
    filename = "/home/ammirato/" + filename;
    if(debug)
      ROS_INFO("TRYING TO OPEN FILE");
    std::ifstream file (filename);
    if(file.is_open())
    {
      goal_points = std::vector<Point>();
      std::string xval,yval;
      while( std::getline(file, xval,',') )
      {
        std::getline(file,yval); 
       
        goal_points.push_back( Point(std::stod(xval), std::stod(yval),
                                  0,  UNEXPLORED)); 
        ROS_INFO(" HEY X:%s  Y:%s   POINT(%f,%f)", xval.c_str(),yval.c_str(),goal_points[goal_points.size()-1].x, goal_points[goal_points.size()-1].y);

                    

      }//while getline
    }//file isopen
    else
    {
      ROS_ERROR("COULD NOT OPEN FILE");
      filename = "";
    }//else file isopem    
  }//if ! filename

  //auto generate goal points
  if(filename == "")
  {
    ROS_INFO("AUTO GENERATING POINTS");
    int num_rows  = grid_height/grid_res +1;
    int num_cols = grid_width/grid_res +1;


    goal_points = std::vector<Point>(num_rows*num_cols);
    
    bool forward = true;//whether to put in the row forwards or forwards
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
  }//if filename

}//constructor



//main logic here

//GOAL: visit every grid point, turn 360 and back
int SimpleGridMotionLocalization::run()
{

  ROS_INFO("RUNNNING LOCALIZATION"); 
  bool planning_active = true;
  unsigned int i = 0;
 // for(unsigned int i=0;i<goal_points.size(); i++)
  while(ros::ok())
  {

    if(planning_active)
    {
      //just to be safe, make sure the robot is stopped 
      robot.wait_until_stopped(.5);
      ros::Duration(1.5).sleep();
     
      //get current position/orientation from SLAM node 
      update_current_position(); 


      Point goal_point = goal_points[i];
      int counter = 0;

      ///KEEP TRYING TO GET TO CURRENT GOAL POINT
      while(!is_at_point(goal_point) && counter < 1000)
      {

        ROS_ERROR("not at point: %f, %f", goal_point.x,goal_point.y);


        //##############################
        double desired_orientation = get_desired_orientation(goal_point);

        if(debug)
        {
          ROS_INFO("CurOrientation: %f, DesiredOrientation: %f",
                    cur_orientation, desired_orientation);
          ros::Duration(3).sleep();
        }


        double orientation_diff = desired_orientation - cur_orientation;


        //##############################
        //maybe move forwards/backwards
        trans_forward = true;
       
        //ASSUMES ROBOT IS AT 0 degrees orientation 
        if(fabs(orientation_diff) > (.5 * M_PI))
        {

          double new_diff = M_PI - fabs(orientation_diff) ;
         
          if(orientation_diff > 0)
          {
            new_diff = -new_diff;
          }
          orientation_diff = new_diff;
          trans_forward = false;
          
          if(debug)
          { 
            ROS_INFO("DIFF < 90, move backwards: %f", orientation_diff);
          }
        }//if diff > 90





        //##############################
        //decide if the angle is big enough to warrant turning to face goal
        if(!(fabs(orientation_diff) < ORIENTATION_THRESHOLD_RADIANS))
        {
          //turn in increments so slam can keep up
          change_orientation(orientation_diff);
        }//if oreintation 






        //find distance to goal point
        double dist = cur_point.distance_to_2d(goal_point);  
        dist = dist*METERS_TO_MILLIMETERS;
       

        //move, in increments so the slam can keep up 
        slam_move(dist, trans_forward);


        //give the slam a bit to catch up
        robot.wait_until_stopped(.5);
        ros::Duration(1).sleep();


        //get the new position from slam
        update_current_position();

        //reset orientation to 0
        change_orientation(-cur_orientation);
 
        

        //give the slam a bit to catch up
        robot.wait_until_stopped(.5);
        ros::Duration(1).sleep();
        //get the new position from slam
        update_current_position();

        counter++;
      }//while not at goal point 







     
      robot.wait_until_stopped(.5);
      //get the new position from slam
      update_current_position();




      //now we are at the desired point,take the pictures 
      slam_pause_client.call(empty_srv);

      if(max_rotate_180)
      {
        ROS_INFO("MAX ROTATE 180");
        rotate(180, turn_res,true,true,2);//take and save data
        rotate(180,45,false,false);//turn back around

        rotate(180,45,false,false);//turn back around
        rotate(180, turn_res,true,true,2);//take and save data
      }//if rotate 180 at a time
      else
      {
        rotate(360, turn_res,true,true,.5);//take and save data
        rotate(360,45,false,false);//turn back around
      }     


 
      robot.wait_until_stopped(.5);
      ros::Duration(4).sleep();
      slam_resume_client.call(empty_srv);




      i++;//next goal point  
    

      if(i >=goal_points.size())
      {
        ROS_INFO("done planning");
        planning_active = false;
      }//if i 

      if(debug){ROS_INFO("done with current point");}

    }//if planning_active 
  }//for i 


 return 1; 

}//end run











double SimpleGridMotionLocalization::get_desired_orientation(Point goal_point)
{
  //make current point 0,0
  double gx = goal_point.x - cur_point.x;
  double gy = goal_point.y - cur_point.y;

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







bool SimpleGridMotionLocalization::is_at_point(Point goal_point)
{

  ROS_INFO("cp: %f,%f   gp: %f,%f", cur_point.x, cur_point.y, goal_point.x, goal_point.y);

  return cur_point.distance_to_2d(goal_point)*METERS_TO_MILLIMETERS <  DISTANCE_TO_POINT_THRESHOLD_MM;

}//is_at_point








void SimpleGridMotionLocalization::update_current_position()
{

  slam_odom_client.call(slam_odom_srv);


  double x = slam_odom_srv.response.odom.pose.pose.position.x;
  double y = slam_odom_srv.response.odom.pose.pose.position.y;
  double z = slam_odom_srv.response.odom.pose.pose.position.z;

  double qx = slam_odom_srv.response.odom.pose.pose.orientation.x;
  double qy = slam_odom_srv.response.odom.pose.pose.orientation.y;
  double qz = slam_odom_srv.response.odom.pose.pose.orientation.z;
  double qw = slam_odom_srv.response.odom.pose.pose.orientation.w;

  cur_point.x = x;
  cur_point.y = y;
  cur_point.z = z;

  cur_orientation = 2*acos(qw);

  if(qz < 0)
  {
    cur_orientation *= -1;
  }   


  
  if(debug)
  {
    ROS_INFO("Poisition(x,y,z): %f, %f, %f\n ", x,y,z);

    ROS_INFO("Quaternion: %f, %f, %f, %f\n",qx,qy,qz,qw);
    ROS_INFO("Approx Angle: %f\n\n\n",2*acos(qw));
  }

}//update current position 






void SimpleGridMotionLocalization::change_orientation(double amount_to_change)
{

  ROS_INFO( "change orientation");

  bool ccw = true;
  if(amount_to_change<0)
  {
    ccw = false;
  }

  double angle = fabs(amount_to_change) * 180 / M_PI;


  //only want to turn by slam_turn_res at most
  int num_rots = angle/slam_turn_res;//num or roatations using slam_turn_res
  int subtotal = slam_turn_res*num_rots;//total angle turned
  int leftover = angle - subtotal;//amount left to turn < slam_turn_res

  ROS_INFO("ANGLE: %f, NUM_ROTS: %d, SUBTOTAL: %d, Leftover:%d", angle, num_rots,subtotal,leftover); 
  rotate(subtotal,slam_turn_res, ccw, false, 4);
  rotate(leftover,leftover,      ccw, false, 4);


}//change_orientation












void SimpleGridMotionLocalization::slam_move(double dist, bool forward)
{
  int num_trans = dist/trans_res;
  double subtotal = trans_res*num_trans;
  double leftover = dist - subtotal;

  //make sure distances are negative for forwards motion
  if(!forward)
  {
    trans_res = -trans_res;
    leftover = -leftover;
  }

  for (int j=0; j<num_trans; j++)
  {
    robot.do_translation(trans_res);
    ros::Duration(6).sleep();
    robot.wait_until_stopped(.5);
  }//for j

  robot.do_translation(leftover);
  ros::Duration(6).sleep();
  robot.wait_until_stopped(.5);

  if(!forward)
  {
    trans_res = -trans_res;//reset it 
  }

}//slam_move





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
