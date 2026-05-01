#include "contacts.h"
#include "math_helper.h"

// Contact

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

void Contact::calculateContactBasis()
{
    glm::vec3 contactTangent1;
    glm::vec3 contactTangent2;

    // check if z-axis is closer to x-axis or y-axis
    if (abs(contactNormal.x) > abs(contactNormal.y))
    {
        const float scaling = 1.0f / sqrt(contactNormal.z * contactNormal.z +
                                          contactNormal.x * contactNormal.x);
        
        contactTangent1.x = contactNormal.z * scaling;
        contactTangent1.y = 0;
        contactTangent1.z = -contactNormal.x * scaling;

        contactTangent2.x = contactNormal.y * contactTangent1.x;
        contactTangent2.y = contactNormal.z * contactTangent1.x -
                              contactNormal.x * contactTangent1.z;
        contactTangent2.z = -contactNormal.y * contactTangent1.x;
    }
    else
    {
        const float scaling = 1.0f / sqrt(contactNormal.z * contactNormal.z +
                                          contactNormal.y * contactNormal.y);
        
        contactTangent1.x = 0;
        contactTangent1.y = -contactNormal.z * scaling;
        contactTangent1.z = contactNormal.y * scaling;

        contactTangent2.x = contactNormal.y * contactTangent1.z -
                              contactNormal.z * contactTangent1.y;
        contactTangent2.y = -contactNormal.x * contactTangent1.z;
        contactTangent2.z = contactNormal.x * contactTangent1.y;
    }

    contactToWorld = glm::mat3(contactNormal,
                               contactTangent1,
                               contactTangent2);
}

void Contact::applyVelocityChange(glm::vec3 velocityChange[2],
                                  glm::vec3 rotationChange[2])
{
    glm::mat3 inverseInertiaTensor[2];
    inverseInertiaTensor[0] = body[0]->getInertiaTensorWorld();
    if (body[1])
        inverseInertiaTensor[1] = body[1]->getInertiaTensorWorld();
    
    glm::vec3 impulseContact;

    if (friction == 0.0f)
    {
        impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
    }
    else
    {
        impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }

    glm::vec3 impulse = contactToWorld * impulseContact;

    glm::vec3 impulsiveTorque = glm::cross(relativeContactPosition[0], impulse);
    rotationChange[0] = inverseInertiaTensor[0] * impulsiveTorque;
    velocityChange[0] = glm::vec3(0.0f);
    velocityChange[0] += impulse * body[0]->getInverseMass();

    body[0]->addVelocity(velocityChange[0]);
    body[0]->addRotation(rotationChange[0]);

    if (body[1])
    {
        impulsiveTorque = glm::cross(impulse, relativeContactPosition[1]);
        rotationChange[1] = inverseInertiaTensor[1] * impulsiveTorque;
        velocityChange[1] = glm::vec3(0.0f);
        velocityChange[1] += impulse * -body[1]->getInverseMass();

        body[1]->addVelocity(velocityChange[1]);
        body[1]->addRotation(rotationChange[1]);
    }
}

void Contact::applyPositionChange(glm::vec3 linearChange[2],
                                  glm::vec3 angularChange[2],
                                  float penetration)
{
    const float angularLimit = 0.2f;
    
    float totalInertia = 0;
    float linearInertia[2];
    float angularIntertia[2];

    for (uint32_t i = 0; i < 2; i++) 
    {
        if (body[i])
        {
            glm::mat3 inverseInertiaTensor = body[i]->getInverseInertiaTensorWorld();

            glm::vec3 angularInertiaWorld = glm::cross(relativeContactPosition[i], contactNormal);
            angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
            angularInertiaWorld = glm::cross(angularInertiaWorld, relativeContactPosition[i]);
            angularIntertia[i] = glm::dot(angularInertiaWorld, contactNormal);

            linearInertia[i] = body[i]->getInverseMass();

            totalInertia += linearInertia[i] + angularIntertia[i];
        }
    }

    float linearMove[2];
    float angularMove[2];

    for (uint32_t i = 0; i < 2; i++) 
        if (body[i])
        {
            float sign = (i == 0) ? 1.0f : -1.0f;
            angularMove[i] = sign * penetration * 
                             (angularIntertia[i] / totalInertia);
            linearMove[i] = sign * penetration * 
                            (linearInertia[i] / totalInertia);

            glm::vec3 projection = relativeContactPosition[i];
            projection += contactNormal * 
                          glm::dot(-relativeContactPosition[i], contactNormal);
            
            float maxMagnitude = angularLimit * glm::length(projection);

            if (angularMove[i] < -maxMagnitude)
            {
                float totalMove = angularMove[i] + linearMove[i];
                angularMove[i] = -maxMagnitude;
                linearMove[i] = totalMove - angularMove[i];
            }
            else if (angularMove[i] > maxMagnitude)
            {
                float totalMove = angularMove[i] + linearMove[i];
                angularMove[i] = maxMagnitude;
                linearMove[i] = totalMove - angularMove[i];
            }

            if (angularMove[i] == 0)
            {
                angularChange[i] = glm::vec3(0.0f);
            }
            else
            {
                glm::vec3 targetAngularDirection = glm::cross(relativeContactPosition[i], contactNormal);
                glm::mat3 inverseInertiaTensor = body[i]->getInverseInertiaTensorWorld();
                angularChange[i] = (inverseInertiaTensor * targetAngularDirection) *
                                   (angularMove[i] / angularIntertia[i]);


            }

            linearChange[i] = contactNormal * linearMove[i];

            glm::vec3 pos = body[i]->getPosition();
            pos += contactNormal * linearMove[i];
            body[i]->setPosition(pos);

            glm::quat q = body[i]->getOrientation();
            q += glm::quat(0, angularChange[i]) * q * 0.5f;
            body[i]->setOrientation(q);

            //body[i]->updateData();
        }
}

glm::vec3 Contact::calculateFrictionlessImpulse(glm::mat3* inverseInertiaTensor)
{
    glm::vec3 deltaVelWorld = glm::cross(relativeContactPosition[0],contactNormal);
    deltaVelWorld = transformVec3byMat4(deltaVelWorld, inverseInertiaTensor[0]);
    deltaVelWorld = glm::cross(deltaVelWorld, relativeContactPosition[0]);

    float deltaVelocity = glm::dot(deltaVelWorld, contactNormal);

    deltaVelocity += body[0]->getInverseMass();

    if (body[1])
    {
        deltaVelWorld = glm::cross(relativeContactPosition[1],contactNormal);
        deltaVelWorld = transformVec3byMat4(deltaVelWorld, inverseInertiaTensor[1]);
        deltaVelWorld = glm::cross(deltaVelWorld, relativeContactPosition[1]);

        deltaVelocity = glm::dot(deltaVelWorld, contactNormal);

        deltaVelocity += body[1]->getInverseMass();
    }

    glm::vec3 impulseContact = glm::vec3(desiredDeltaVelocity / deltaVelocity,
                                         0,
                                         0);
    return impulseContact;
}
 
glm::vec3 Contact::calculateFrictionImpulse(glm::mat3* inverseInertiaTensor)
{
    glm::mat3 impulseToTorque = skewSymmetricVec3ToMat3(relativeContactPosition[0]);

    glm::mat3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= inverseInertiaTensor[0];
    deltaVelWorld *= impulseToTorque;
    deltaVelWorld *= -1.0f;

    float inverseMass = body[0]->getInverseMass();

    if (body[1])
    {
        impulseToTorque = skewSymmetricVec3ToMat3(relativeContactPosition[1]);

        glm::mat3 deltaVelWorld2 = impulseToTorque;
        deltaVelWorld2 *= inverseInertiaTensor[1];
        deltaVelWorld2 *= impulseToTorque;
        deltaVelWorld2 *= -1.0f; 

        deltaVelWorld += deltaVelWorld2;

        inverseMass += body[1]->getInverseMass();
    }

    glm::mat3 deltaVelocity = glm::transpose(contactToWorld);
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= contactToWorld;

    deltaVelocity[0][0] += inverseMass;
    deltaVelocity[1][1] += inverseMass;
    deltaVelocity[2][2] += inverseMass;

    glm::mat3 impulseMatrix = glm::inverse(deltaVelocity);

    glm::vec3 velKill = glm::vec3(desiredDeltaVelocity,
                                  -contactVelocity.y,
                                  -contactVelocity.z);

    glm::vec3 impulseContact = impulseMatrix * velKill;

    float planarImpulse = sqrt(impulseContact.y * impulseContact.y +
                               impulseContact.z * impulseContact.z);
    
    if (planarImpulse > impulseContact.x * friction)
    {
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity[0][0] +
                           deltaVelocity[0][1] * friction * impulseContact.y +
                           deltaVelocity[0][2] * friction * impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

void Contact::calculateInternalData(float duration)
{
    // if first body is NULL, swap it
    if (!body[0])
    {
        contactNormal *= -1;
        RigidBody* temp = body[0];
        body[0] = body[1];
        body[1] = temp;
    }
    assert(body[0]);

    calculateContactBasis();

    relativeContactPosition[0] = contactPoint - body[0]->getPosition();
    if (body[1])
    {
        relativeContactPosition[1] = contactPoint - body[1]->getPosition();
    }

    contactVelocity = calculateLocalVelocity(0, duration);
    if (body[1])
    {
        contactVelocity -= calculateLocalVelocity(1, duration);
    }

    calculateDesiredDeltaVelocity(duration);
}

glm::vec3 Contact::calculateLocalVelocity(uint32_t bodyIndex, float duration)
{
    RigidBody* localBody = body[bodyIndex];

    glm::vec3 velocity = glm::cross(localBody->getRotation(), relativeContactPosition[bodyIndex]);
    velocity += localBody->getVelocity();

    glm::vec3 contactVelocity = glm::transpose(contactToWorld) * velocity;

    glm::vec3 accVelocity = localBody->getLastFrameAcceleration() * duration;

    accVelocity.x = 0;

    contactVelocity += accVelocity;

    return contactVelocity;
}

void Contact::calculateDesiredDeltaVelocity(float duration)
{
    const static float velocityLimit = 0.25f;

    float velocityFromAcc = 0;

    velocityFromAcc += duration * 
                       glm::dot(body[0]->getLastFrameAcceleration(), 
                                contactNormal);
    
    if (body[1])
    {
        velocityFromAcc -= duration * 
                           glm::dot(body[1]->getLastFrameAcceleration(), 
                                    contactNormal);
    }

    float appliedRestitution = restitution;
    if (abs(contactVelocity.x) < velocityLimit)
    {
        appliedRestitution = 0.0f;
    }

    desiredDeltaVelocity = -contactVelocity.x -
                           appliedRestitution *
                           (contactVelocity.x - velocityFromAcc);
}

// ContactResolver

void ContactResolver::prepareContacts(std::vector<Contact> contacts, 
                                      float duration)
{
    for (std::vector<Contact>::iterator c = contacts.begin();
         c != contacts.end(); 
         c++)
    {
        c->calculateInternalData(duration);
    }
}

void ContactResolver::adjustPositions(std::vector<Contact> contacts, 
                                      uint32_t numContacts, 
                                      float duration)
{  
    for (uint32_t p = 0; p < positionIterations; p++)
    {
        float max = positionEpsilon;
        uint32_t index = numContacts;
        for (uint32_t i = 0; i < numContacts; i++)
        {
            if (contacts[i].penetration > max)
            {
                max = contacts[i].penetration;
                index = i;
            }
        }
        if (index == numContacts) return;

        glm::vec3 linearChange[2], angularChange[2];
        contacts[index].applyPositionChange(linearChange,
                                            angularChange,
                                            max);

        for (uint32_t i = 0; i < numContacts; i++)
        {
            for (uint32_t n = 0; n < 2; n++) 
            {
                if (contacts[i].body[n])
                {
                    for (uint32_t m = 0; m < 2; m++)
                    {
                        if (contacts[i].body[n] == contacts[index].body[m])
                        {
                            glm::vec3 deltaPosition = linearChange[m] + 
                                                      glm::cross(angularChange[m], 
                                                                 contacts[i].relativeContactPosition[n]);
                            
                            contacts[i].penetration += (n ? 1.0f : -1.0f) *
                                                       glm::dot(deltaPosition,
                                                                contacts[i].contactNormal);
                        }
                    }
                }
            }
        }
    }
}

void ContactResolver::adjustVelocities(std::vector<Contact> contacts, 
                                       uint32_t numContacts, 
                                       float duration)
{
    for (uint32_t v = 0; v < velocityIterations; v++)
    {
        float max = velocityEpsilon;
        uint32_t index = numContacts;
        for (uint32_t i = 0; i < numContacts; i++)
        {
            if (contacts[i].desiredDeltaVelocity > max)
            {
                max = contacts[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == numContacts) return;

        glm::vec3 velocityChange[2], rotationChange[2];
        contacts[index].applyVelocityChange(velocityChange,
                                            rotationChange);

        for (uint32_t i = 0; i < numContacts; i++)
        {
            for (uint32_t n = 0; n < 2; n++) 
            {
                if (contacts[i].body[n])
                {
                    for (uint32_t m = 0; m < 2; m++)
                    {
                        if (contacts[i].body[n] == contacts[index].body[m])
                        {
                            glm::vec3 deltaVelocity = velocityChange[m] + 
                                                      glm::cross(rotationChange[m], 
                                                                 contacts[i].relativeContactPosition[n]);
                            
                            contacts[i].contactVelocity += glm::transpose(contacts[i].contactToWorld) *
                                                           deltaVelocity *
                                                           (n ? 1.0f : -1.0f);
                            contacts[i].calculateDesiredDeltaVelocity(duration);
                        }
                    }
                }
            }
        }

    }
}

void ContactResolver::resolveContacts(std::vector<Contact> contacts,
                                      uint32_t numContacts,
                                      float duration)
{
    if (numContacts == 0) return;

    prepareContacts(contacts, duration);

    adjustPositions(contacts, numContacts, duration);

    adjustVelocities(contacts, numContacts, duration);
}