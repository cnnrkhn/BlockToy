#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include "rigid_body.h"
#include "contacts.h"
#include <vector>

#ifndef FINE_COLLISION
#define FINE_COLLISION

class CollisionPrimitive
{
public:
    /*
        the rigid body represented by this primitive
    */
    RigidBody* body;

    /*
        the offset of this primitive from the given rigid body
    */
    glm::mat4 offset = glm::mat4(1.0f);

    /*
        calculate internal data for primitive
    */
    void calculateInternalData();

    glm::vec3 getAxis(uint32_t index) const
    {
        glm::vec4 axis = transform[index];
        return glm::vec3(axis);
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

    CollisionBox(RigidBody* b, glm::mat4 off, glm::vec3 hs);
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
    /*
        resizeable array of contacts
    */
    std::vector<Contact> contacts;

    /*
        how many more contacts the array can have
    */
    uint32_t contactsLeft = 20;

    /*
        the friction used for collisions
    */
    float friction = 0.0f;

    /*
        the restitution used for collisions
        where 0 is maximum stickiness and
        1 is maximum bounciness
    */
    float restitution = 0.0f;

    /*
        how far objects can be beforee generating a contact 
    */
    float tolerance = 0;

    bool hasMoreContacts()
    {
        return contactsLeft > 0;
    }

    void reset(uint32_t maxContacts)
    {
        contactsLeft = maxContacts;
        contacts = std::vector<Contact>();
    }

    void addContact(Contact& contact)
    {
        contactsLeft--;
        contacts.push_back(contact);
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