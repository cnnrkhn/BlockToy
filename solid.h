#include <vector>

#ifndef SOLID
#define SOLID

struct Point {
  double x;
  double y;
  double z;
};

struct Triangle {
  Point points[3];
};

class Solid {
  private:
    std::vector<Triangle> triangles;
  
  public:
    Solid(std::vector<Triangle>);
};

#endif
