#ifndef __POINT_H__
#define __POINT_H__


#include <math.h>

enum PointState {
  UNEXPLORED = 1,
  UNREACHABLE,
  VISITED
};

class Point{

 // private:
  public:
    double x;
    double y;
    double z;

    PointState state;


  public:
 /*   void setX(int x);
    void setY(int y);  
    void setZ(int z);
    void setState(PointState s);

    
    double x();
    double y();
    double z();
    PointState state();
*/

    Point();
    Point(int ix, int iy, int iz, PointState is);

    double distanceTo(Point p);

};








#endif
