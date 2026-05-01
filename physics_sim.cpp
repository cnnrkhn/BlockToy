#include "physics_sim.h"

PhysicsSim::PhysicsSim()
{

}

void PhysicsSim::addBox(float inverseMass,
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
    
    CollisionBox box(rb, glm::translate(glm::mat4(1.0f),position), halfWidths);
    box.body = rb;
    box.halfSizes = halfWidths;

    Contact con;
    con.setBodyData(rb, NULL, data.friction, data.restitution);
    data.addContact(con);

    boxes.push_back(box);
}

void PhysicsSim::startFrame()
{
    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        b->body->clearAccumulators();
        b->body->updateData();
        b->calculateInternalData();
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
        if (!CollisionDetector::boxAndPlane((*b), floor, &data))
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

std::vector<glm::vec3> PhysicsSim::getVelocities()
{
    std::vector<glm::vec3> velocities;

    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        glm::vec3 pos = b->body->getPosition();

        velocities.push_back(pos);
    }

    return velocities;
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

std::vector<glm::vec3> PhysicsSim::getRotations()
{
    std::vector<glm::vec3> rotations;

    for (CollisionBoxes::iterator b = boxes.begin();
         b != boxes.end(); 
         b++)
    {
        glm::vec3 pos = b->body->getPosition();

        rotations.push_back(pos);
    }

    return rotations;
}
