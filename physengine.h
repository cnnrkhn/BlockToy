#include <vector>
#include "solid.h"

class PhysEngine {
  private:
    std::vector<Solid> solids;

  public:
    PhysEngine();
    void addSolid(Solid);
    std::vector<Solid> getSolids();
  
};

