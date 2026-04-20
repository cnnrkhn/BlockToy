#include <iostream>
#include "physengine.h"

int main() {
  Point points[4];
  double n = -6;
  for (int i = 0; i < 4; i++) {
    points[i].x = n++;
    points[i].y = n++;
    points[i].z = n++;
  }

  std::vector<Triangle> triangles;
  for (int i = 0; i < 4; i++) {
    Triangle triangle;
    int n = 0;
    for (int j = 0; j < 4; j++) {
      if (j != i) {
        triangle.points[n] = points[j];
        n++;
      }
    }  
    triangles.push_back(triangle);
  }

  PhysEngine ps = PhysEngine();
  ps.addSolid(Solid(triangles));
}
