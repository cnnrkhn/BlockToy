#include <glm/exponential.hpp>
#include <limits>
#include "fine_collision.h"
#include "math_helper.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

// helper functions
static inline float transformToAxis(const CollisionBox& box,
                             const glm::vec3& axis)
{
    // return a float
    return box.halfSizes.x * abs(glm::dot(axis, box.getAxis(0))) +
           box.halfSizes.y * abs(glm::dot(axis, box.getAxis(1))) +
           box.halfSizes.z * abs(glm::dot(axis, box.getAxis(2)));
}

static inline bool overlapOnAxis(const CollisionBox& box1,
                                 const CollisionBox& box2,
                                 const glm::vec3& axis,
                                 const glm::vec3& toCenter)
{
    float projectBox1 = transformToAxis(box1, axis);
    float projectBox2 = transformToAxis(box2, axis);

    float distance = abs(glm::dot(toCenter, axis));

    return (distance < projectBox1 + projectBox2);
}

static inline float penetrationOnAxis(const CollisionBox& box1,
                                      const CollisionBox& box2,
                                      const glm::vec3& axis,
                                      const glm::vec3& toCenter)
{
    float projectBox1 = transformToAxis(box1, axis);
    float projectBox2 = transformToAxis(box2, axis);

    float distance = abs(glm::dot(toCenter, axis));

    return projectBox1 + projectBox2 - distance;
}

static inline bool tryAxis(const CollisionBox& box1,
                           const CollisionBox& box2,
                           glm::vec3 axis,
                           const glm::vec3& toCenter,
                           uint32_t index,
                           float& smallestPenetration,
                           uint32_t& smallestCase)
{
    if (glm::length2(axis) < 0.0001) return true;
    axis = glm::normalize(axis);

    float penetration = penetrationOnAxis(box1, box2, axis, toCenter);

    if (penetration < 0) return false;
    if (penetration < smallestPenetration) {
        smallestPenetration = penetration;
        smallestCase = index;
    }
    return true;
}

void fillPointFaceBoxBox(const CollisionBox& box1,
                         const CollisionBox& box2,
                         const glm::vec3& toCenter,
                         CollisionData* data,
                         uint32_t best,
                         float penetrate)
{
    Contact contact;

    glm::vec3 normal = box1.getAxis(best);
    if (glm::dot(box1.getAxis(best),toCenter) < 0)
    {
        normal = normal * -1.0f;
    }

    glm::vec3 vertex = box2.halfSizes;
    if (glm::dot(box2.getAxis(0),normal) < 0) vertex.x = -vertex.x;
    if (glm::dot(box2.getAxis(1),normal) < 0) vertex.y = -vertex.y;
    if (glm::dot(box2.getAxis(2),normal) < 0) vertex.z = -vertex.z;

    contact.contactNormal = normal;
    contact.penetration = penetrate;
    contact.contactPoint = transformVec3byMat4(vertex, box2.getTransform());
    contact.setBodyData(box1.body, 
                        box2.body,
                        data->friction,
                        data->restitution);
    
    data->addContact(contact);
}

static inline glm::vec3 contactPoint(const glm::vec3& ptOne,
                                     const glm::vec3& dirOne,
                                     float sizeOne,
                                     const glm::vec3& ptTwo,
                                     const glm::vec3& dirTwo,
                                     float sizeTwo,
                                     bool useOne)
{
    float len2One = glm::length2(dirOne);
    float len2Two = glm::length2(dirTwo);
    float dotOneTwo = glm::dot(dirOne, dirTwo);

    glm::vec3 toSta = ptOne - ptTwo;
    float dotStaOne = glm::dot(dirOne, toSta);
    float dotStaTwo = glm::dot(dirTwo, toSta);

    float denominator = len2One * len2Two - dotOneTwo * dotOneTwo;

    if (abs(denominator) < 0.0001f)
    {
        return useOne ? ptOne : ptTwo;
    }

    float muA = (dotOneTwo * dotStaTwo - len2Two * dotStaOne) / denominator;
    float muB = (len2One * dotStaTwo - dotOneTwo * dotStaOne) / denominator;

    if (muA > sizeOne ||
        muA < -sizeOne ||
        muB > sizeTwo ||
        muB < -sizeTwo)
    {
        return useOne ? ptOne : ptTwo;
    }
    else
    {
        glm::vec3 conPtOne = ptOne + dirOne * muA;
        glm::vec3 conPtTwo = ptTwo + dirTwo * muB;

        return conPtOne * 0.5f + conPtTwo * 0.5f;
    }
}

// CollisionPrimitive

void CollisionPrimitive::calculateInternalData()
{
    transform = body->getTransformMatrix() * offset;
}

// CollisionBox

CollisionBox::CollisionBox(RigidBody* b, glm::mat4 off, glm::vec3 hs)
{
    body = b;
    offset = off;
    halfSizes = hs;

    calculateInternalData();
}

// IntersectionTests
bool IntersectionTests::boxAndPlane(const CollisionBox& box,
                                    const CollisionPlane& plane)
{

    float projectedRadius = transformToAxis(box, plane.direction);

    float boxDistance = glm::dot(plane.direction, box.getAxis(3)) - projectedRadius;

    return boxDistance <= plane.offset;
}
    
bool IntersectionTests::boxAndBox(const CollisionBox& box1,
                                  const CollisionBox& box2)
{
    glm::vec3 toCenter = box2.getAxis(3) - box1.getAxis(3);

    glm::vec3 box1Axis0 = box1.getAxis(0);
    glm::vec3 box1Axis1 = box1.getAxis(1);
    glm::vec3 box1Axis2 = box1.getAxis(2);

    glm::vec3 box2Axis0 = box2.getAxis(0);
    glm::vec3 box2Axis1 = box2.getAxis(1);
    glm::vec3 box2Axis2 = box2.getAxis(2);

    // face axes
    bool overlap1 =
        overlapOnAxis(box1,box2,box1Axis0,toCenter) &&
        overlapOnAxis(box1,box2,box1Axis1,toCenter) &&
        overlapOnAxis(box1,box2,box1Axis2,toCenter);
    
    // face axes
    bool overlap2 =
        overlapOnAxis(box1,box2,box2Axis0,toCenter) &&
        overlapOnAxis(box1,box2,box2Axis1,toCenter) &&
        overlapOnAxis(box1,box2,box2Axis2,toCenter);

    // edge-edge axes
    bool overlapCross =
        overlapOnAxis(box1,box2,glm::cross(box1Axis0,box2Axis0),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis0,box2Axis1),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis0,box2Axis2),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis1,box2Axis0),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis1,box2Axis1),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis1,box2Axis2),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis2,box2Axis0),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis2,box2Axis1),toCenter) &&
        overlapOnAxis(box1,box2,glm::cross(box1Axis2,box2Axis2),toCenter);

    return (overlap1 && overlap2 && overlapCross);
}

// CollisionDetector

uint32_t CollisionDetector::boxAndPlane(const CollisionBox& box,
                                        const CollisionPlane& plane,
                                        CollisionData* data)
{
    if (data->contactsLeft <= 0) return 0;

    if (!IntersectionTests::boxAndPlane(box, plane))
    {
        return 0;
    }

    static float mults[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                               {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};
    
    uint32_t contactsUsed = 0;
    for (uint32_t i = 0; i < 8; i++)
    {
        Contact contact;
        glm::vec3 vertexPos = glm::vec3(mults[i][0], 
                                        mults[i][1], 
                                        mults[i][2]);
        vertexPos *= box.halfSizes;
        vertexPos = transformVec3byMat4(vertexPos, box.getTransform());

        float vertexDist = glm::dot(vertexPos, plane.direction);

        if (vertexDist <= plane.offset)
        {
            contact.contactPoint = plane.direction;
            contact.contactPoint *= (vertexDist - plane.offset);
            contact.contactPoint += vertexPos;
            contact.contactNormal = plane.direction;
            contact.penetration = plane.offset - vertexDist;

            contact.setBodyData(box.body, NULL,
                                 data->friction, data->restitution);
                
            data->addContact(contact);
            contactsUsed++;
            if (contactsUsed == data->contactsLeft) return contactsUsed;
        }
        
    }

    return contactsUsed;
}


#define CHECK_OVERLAP(axis,index) if (!tryAxis(box1,box2,(axis),toCenter,(index),penetrate,best)) return 0;

uint32_t CollisionDetector::boxAndBox(const CollisionBox& box1,
                                      const CollisionBox& box2,
                                      CollisionData* data)
{
    //if (!IntersectionTests::boxAndBox(box1,box2)) return 0;

    glm::vec3 toCenter = box2.getAxis(3) - box1.getAxis(3);

    float penetrate = std::numeric_limits<float>::max();
    uint32_t best = std::numeric_limits<uint32_t>::max();

    glm::vec3 box1Axis0 = box1.getAxis(0);
    glm::vec3 box1Axis1 = box1.getAxis(1);
    glm::vec3 box1Axis2 = box1.getAxis(2);

    glm::vec3 box2Axis0 = box2.getAxis(0);
    glm::vec3 box2Axis1 = box2.getAxis(1);
    glm::vec3 box2Axis2 = box2.getAxis(2);

    // face axes
    CHECK_OVERLAP(box1Axis0, 0);
    CHECK_OVERLAP(box1Axis1, 1);
    CHECK_OVERLAP(box1Axis2, 2);

    CHECK_OVERLAP(box2Axis0, 3);
    CHECK_OVERLAP(box2Axis1, 4);
    CHECK_OVERLAP(box2Axis2, 5);

    uint32_t bestSingleAxis = best;

    // edge-edge axes
    CHECK_OVERLAP(glm::cross(box1Axis0,box2Axis0), 6);
    CHECK_OVERLAP(glm::cross(box1Axis0,box2Axis1), 7);
    CHECK_OVERLAP(glm::cross(box1Axis0,box2Axis2), 8);
    CHECK_OVERLAP(glm::cross(box1Axis1,box2Axis0), 9);
    CHECK_OVERLAP(glm::cross(box1Axis1,box2Axis1), 10);
    CHECK_OVERLAP(glm::cross(box1Axis1,box2Axis2), 11);
    CHECK_OVERLAP(glm::cross(box1Axis2,box2Axis0), 12);
    CHECK_OVERLAP(glm::cross(box1Axis2,box2Axis1), 13);
    CHECK_OVERLAP(glm::cross(box1Axis2,box2Axis2), 14);

    // best should either have been updated, or we should have exited
    assert(best != std::numeric_limits<uint32_t>::max());

    if (best < 3)
    {
        fillPointFaceBoxBox(box1, box2, toCenter* - 1.0f, data, best, penetrate);
        return 1;
    }
    else if (best < 6)
    {
        fillPointFaceBoxBox(box2, box1, toCenter* - 1.0f, data, best-3, penetrate);
        return 1;
    }
    else
    {
        best -= 6;
        uint32_t box1AxisIndex = best / 3;
        uint32_t box2AxisIndex = best % 3;
        glm::vec3 box1Axis = box1.getAxis(box1AxisIndex);
        glm::vec3 box2Axis = box2.getAxis(box2AxisIndex);
        glm::vec3 axis = glm::cross(box1Axis, box2Axis);
        axis = glm::normalize(axis);

        if (glm::dot(axis, toCenter) > 0) axis = axis * -1.0f;

        glm::vec3 pointOnBox1Edge = box1.halfSizes;
        glm::vec3 pointOnBox2Edge = box2.halfSizes;
        for (uint32_t i = 0; i < 3; i++)
        {
            if (i == box1AxisIndex) pointOnBox1Edge[i] = 0;
            else if (glm::dot(box1.getAxis(i),axis) > 0) pointOnBox1Edge[i] = -pointOnBox1Edge[i];
        
            if (i == box2AxisIndex) pointOnBox2Edge[i] = 0;
            else if (glm::dot(box2.getAxis(i),axis) > 0) pointOnBox2Edge[i] = -pointOnBox2Edge[i];
        }

        pointOnBox1Edge = transformVec3byMat4(pointOnBox1Edge, box1.getTransform());
        pointOnBox2Edge = transformVec3byMat4(pointOnBox2Edge, box2.getTransform());
    
        glm::vec3 vertex = contactPoint(pointOnBox1Edge, 
                                        box1Axis,
                                        box1.halfSizes[box1AxisIndex],
                                        pointOnBox2Edge, 
                                        box2Axis,
                                        box2.halfSizes[box2AxisIndex],
                                        bestSingleAxis > 2);

        Contact contact;

        contact.penetration = penetrate;
        contact.contactNormal = axis;
        contact.contactPoint = vertex;
        contact.setBodyData(box1.body,
                             box2.body,
                             data->friction,
                             data->restitution);
        data->addContact(contact);
        return 1;
    }
    return 0;
}
#undef CHECK_OVERLAP