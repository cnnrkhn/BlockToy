#include "contacts.h"

void Contact::setBodyData(RigidBody* body1, 
                          RigidBody* body2,
                          float friction,
                          float restitution)
{
    this->body[0] = body1;
    this->body[1] = body2;
    this->friction = friction;
    this->restitution = restitution;
}
