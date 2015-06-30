#include <point.h>

Point::Point()
{
  Point(0,0,0,UNEXPLORED);
}

Point::Point(int ix=0, int iy=0, int iz=0, PointState is=UNEXPLORED)
{
  x = ix;
  y = iy;
  z = iz;
  state = is;
}

double Point::distance_to(Point p)
{
  return sqrt( pow(x-p.x,2) + pow(y-p.y,2) + pow(z-p.z,2)  );
}

double Point::distance_to_2d(Point p)
{
  return sqrt( pow(x-p.x,2) + pow(y-p.y,2)  );
}

