#include "physengine.h"
#include "rigid_body.h"

PhysEngine::PhysEngine() {
  
}

void PhysEngine::addSolid(Solid sol) {
  this->solids.emplace_back(sol);
}

std::vector<Solid> PhysEngine::getSolids() {
  return this->solids;
} 
