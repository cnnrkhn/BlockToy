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

    CollisionData d;

    Contact con;
    con.setBodyData(rb, NULL, d.friction, d.restitution);
    d.addContact(con);

    planeData.push_back(d);

    if (boxes.size() > 0)
    {
        for (CollisionBoxes::iterator b = boxes.begin();
             b != boxes.end(); 
             b++)
        {
            CollisionData d;

            Contact con;
            con.setBodyData(rb, b->body, d.friction, d.restitution);
            d.addContact(con);

            boxData.push_back(d);
        }
    }

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
    for(uint32_t j = 0; j < boxes.size(); j++)
    {
        g.updateForce(boxes[j].body,duration);
        (boxes[j].body)->integrate(duration);
        boxes[j].calculateInternalData();

        for(uint32_t i = 0; i < planeData.size(); i++)
        {
            if (!CollisionDetector::boxAndPlane((boxes[j]), floor, &planeData[i]))
            {
                planeData[i].reset(planeData[i].contacts.size() + planeData[i].contactsLeft);
            }
        
            cr.resolveContacts(planeData[i].contacts,
                               planeData[i].contacts.size(),
                               duration);
        }

        for(uint32_t i = 0; i < boxData.size(); i++)
        {
            if(boxData[i].contacts[0].body[0] == b)
            {
                if (!CollisionDetector::boxAndBox((*b), , &planeData[i]))
            {
                planeData[i].reset(planeData[i].contacts.size() + planeData[i].contactsLeft);
            }
        
            cr.resolveContacts(planeData[i].contacts,
                               planeData[i].contacts.size(),
                               duration);
            }
        }
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
