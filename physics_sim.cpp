#include "physics_sim.h"

PhysicsSim::PhysicsSim()
{

}

void PhysicsSim::addBody(float inverseMass,
                         float linearDamping,
                         float angularDamping,
                         const glm::vec3& position,
                         const glm::quat& orientation,
                         const glm::vec3& velocity,
                         const glm::vec3& rotation,
                         const glm::mat3& inverseInertiaTensor,
                         const glm::vec3& halfWidths)
{
    RigidBody* rb = new RigidBody(inverseMass,
                                  linearDamping,
                                  angularDamping,
                                  position,
                                  orientation,
                                  velocity,
                                  rotation,
                                  inverseInertiaTensor);

    rb->clearAccumulators();
    rb->updateData();
    
    if (!hierarchy)
    {
        hierarchy = new BVHNode(NULL, BoundingBox(position, halfWidths), rb);
    }
    else
    {
        hierarchy->insert(rb, BoundingBox(position, halfWidths));
    }
}

void PhysicsSim::runPhysics(float duration)
{
    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        g.updateForce(b->body,duration);
        b->body->integrate(duration);
        b->calculateInternalData();

        uint32_t contactsNeeded = CollisionDetector::boxAndPlane((*b), floor, &data);
        if (!contactsNeeded)
        {
            data.reset(data.contacts.size() + data.contactsLeft);
        }
        
        cr.resolveContacts(data.contacts,
                           data.contacts.size(),
                           duration);
    }
}

std::vector<glm::vec3> PhysicsSim::getPositions()
{
    std::vector<glm::vec3> positions;

    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        glm::vec3 pos = b->body->getPosition();

        positions.push_back(pos);
    }

    return positions;
}

std::vector<glm::quat> PhysicsSim::getOrientations()
{
    std::vector<glm::quat> orientations;

    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        glm::quat ori = b->body->getOrientation();

        orientations.push_back(ori);
    }

    return orientations;
}
