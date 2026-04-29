#include <vector>
#include "force_gens.h"

#ifndef PHYSICS_SIMULATION
#define PHYSICS_SIMULATION

class PhysicsSim
{
public:
    typedef std::vector<RigidBody*> RigidBodies;

    PhysicsSim();

    void addBody(float inverseMass,
                 float linearDamping,
                 float angularDamping,
                 const glm::vec3& position,
                 const glm::quat& orientation,
                 const glm::vec3& velocity,
                 const glm::vec3& rotation,
                 const glm::mat3& inverseInertiaTensor);

    void startFrame();

    void runPhysics(float duration);

    std::vector<glm::vec3> getPositions();

    std::vector<glm::quat> getOrientations();

private:
    RigidBodies bodies; 

    Gravity g;
};

#endif