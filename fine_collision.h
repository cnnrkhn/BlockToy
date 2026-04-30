#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include "rigid_body.h"
#include "contacts.h"

#ifndef FINE_COLLISION
#define FINE_COLLISION

class CollisionPrimitive
{
public:
    RigidBody* body;
    glm::mat4 offset;

    void calculateInternals();

    glm::vec3 getAxis(uint32_t index) const
    {
        return glm::vec3(transform[index]);
    }

    const glm::mat4& getTransform() const
    {
        return transform;
    }

private:
    glm::mat4 transform;
};

class CollisionBox : public CollisionPrimitive
{
public:
    glm::vec3 halfSizes;
};

class CollisionPlane
{
public:
    /*
        the plane's normal
    */
    glm::vec3 direction;

    /*
        offset from the origin
    */
    float offset;
};

class IntersectionTests
{
public:
    static bool boxAndPlane(const CollisionBox& box,
                            const CollisionPlane& plane);
    
    static bool boxAndBox(const CollisionBox& box1,
                          const CollisionBox& box2);
};

struct CollisionData
{
    Contact* contacts;

    uint32_t contactsLeft;

    uint32_t contactCount;

    float friction;

    float restitution;

    float tolerance;

    bool hasMoreContacts()
    {
        return contactsLeft > 0;
    }

    void addContacts(uint32_t count)
    {
        contactsLeft -= count;
        contactCount += count;

        contacts += count;
    }
};

class CollisionDetector
{
public:
    static uint32_t boxAndPlane(
        const CollisionBox& box,
        const CollisionPlane& plane,
        CollisionData* data
    );

    static uint32_t boxAndBox(
        const CollisionBox& box1,
        const CollisionBox& box2,
        CollisionData* data
    );
};

#endif