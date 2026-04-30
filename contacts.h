#include <glm/vec3.hpp>
#include "rigid_body.h"

#ifndef CONTACTS
#define CONTACTS

class Contact
{
public:
    RigidBody* body[2];

    float friction = 0;

    float restitution = 0;

    glm::vec3 contactPoint = glm::vec3();

    glm::vec3 contactNormal = glm::vec3();

    float penetration = 0;

    void setBodyData(RigidBody* body1, 
                     RigidBody* body2,
                     float friction, 
                     float restitution);
    

};

#endif