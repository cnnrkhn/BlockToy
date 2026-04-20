#include "physics_sim.h"

PhysicsSim::PhysicsSim() : g(glm::vec3(0,-1.0,0)) {}

void PhysicsSim::addBody(float inverseMass,
                         float linearDamping,
                         float angularDamping,
                         const glm::vec3& position,
                         const glm::quat& orientation,
                         const glm::vec3& velocity,
                         const glm::vec3& rotation,
                         const glm::mat3& inverseInertiaTensor)
{
    RigidBody* rb = new RigidBody(inverseMass,
                                  linearDamping,
                                  angularDamping,
                                  position,
                                  orientation,
                                  velocity,
                                  rotation,
                                  inverseInertiaTensor);
    bodies.push_back(rb);
}

void PhysicsSim::startFrame()
{
    for (RigidBodies::iterator b = bodies.begin();
         b != bodies.end(); 
         b++)
    {
        (*b)->clearAccumulators();
        (*b)->updateData();
    }
}

void PhysicsSim::runPhysics(float duration)
{
    for (RigidBodies::iterator b = bodies.begin();
         b != bodies.end(); 
         b++)
    {
        g.updateForce(*b,duration);
        (*b)->integrate(duration);
    }
}

std::vector<glm::vec3> PhysicsSim::getPositions()
{
    std::vector<glm::vec3> positions;

    for (RigidBodies::iterator b = bodies.begin();
         b != bodies.end(); 
         b++)
    {
        glm::vec3 pos = (*b)->getPosition();

        positions.push_back(pos);
    }

    return positions;
}

std::vector<glm::quat> PhysicsSim::getOrientations()
{
    std::vector<glm::quat> orientations;

    for (RigidBodies::iterator b = bodies.begin();
         b != bodies.end(); 
         b++)
    {
        glm::quat ori = (*b)->getOrientation();

        orientations.push_back(ori);
    }

    return orientations;
}
