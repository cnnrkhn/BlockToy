#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include "rigid_body.h"
#include <vector>

#ifndef CONTACTS
#define CONTACTS

class ContactResolver;

class Contact
{
    friend class ContactResolver;

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
    
private:
    glm::mat3 contactToWorld;

    glm::vec3 contactVelocity;

    float desiredDeltaVelocity;

    glm::vec3 relativeContactPosition[2];

    void calculateContactBasis();

    void applyVelocityChange(glm::vec3 velocityChange[2],
                             glm::vec3 rotationChage[2]);

    void applyPositionChange(glm::vec3 linearChange[2],
                             glm::vec3 angularChange[2],
                             float penetration);

    glm::vec3 calculateFrictionlessImpulse(glm::mat3* inverseInertiaTensor);

    glm::vec3 calculateFrictionImpulse(glm::mat3* inverseInertiaTensor);

    void calculateInternalData(float duration);

    glm::vec3 calculateLocalVelocity(uint32_t bodyIndex, float duration);

    void calculateDesiredDeltaVelocity(float duration);
};

class ContactResolver
{
private:
    /*
        number of iterations to perform when
        resolving velocity
    */
    uint32_t velocityIterations = 3;

    /*
        number of iterations to perform when
        resolving position
    */
    uint32_t positionIterations = 3;

    /*
        velocities smaller than this value are considered to be zero
    */
    float velocityEpsilon = 0.01;

    /*
        positions smaller than this value are considered to be zero
    */
    float positionEpsilon = 0.01;

    void prepareContacts(std::vector<Contact> contacts,  
                         float duration);

    void adjustPositions(std::vector<Contact> contacts, 
                         uint32_t numContacts, 
                         float duration);

    void adjustVelocities(std::vector<Contact> contacts, 
                          uint32_t numContacts, 
                          float duration);

public:
    void resolveContacts(std::vector<Contact> contacts,
                         uint32_t numContacts,
                         float duration);
};

#endif