#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


#include <stdio.h>
#include <errno.h>
#include <ncurses.h>
#include <iostream>
/*#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
*/

#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}











void grid_map_callback( const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ROS_INFO("Grid Map res: %f\nwidth: %d, height: %d\n", msg->info.resolution, msg->info.width, msg->info.height);
}












int main(int argc, char **argv){


  ros::init(argc, argv, "grid_map_controller");
  ros::NodeHandle n;

  int buffer_size = 10;



  ros::Subscriber sub = n.subscribe("/rtabmap/grid_map", buffer_size, grid_map_callback);



  ros::spin();



  return 0;

}//end main
