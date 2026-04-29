#include <glm/vec3.hpp>
#include "rigid_body.h"

/*
    axis-aligned bounding box (rectanglular prism)
*/
class BoundingBox
{
public:
    /*
        center point of the box
    */
    glm::vec3 center;

    /*
        half the width of the box along each axis
    */
    glm::vec3 halfWidths;

    /*
        create a new bounding box with the given center and dimensions
    */
    BoundingBox(const glm::vec3& center, const glm::vec3& halfWidths);

    /*
        create a new bounding box that contains the given 
        bounding boxes
    */
    BoundingBox(const BoundingBox& b1, const BoundingBox& b2);

    /*
        check if the bounding box overlaps with
        the given bounding box
    */
    bool overlaps(const BoundingBox* other) const;

    float getSize() const;

    float getGrowth(const BoundingBox& other) const;
};

/*
    two rigid bodies that potentially contact each other
*/
struct PotentialContact
{
    RigidBody* bodies[2];
};

/*
    node to a Bounding Volume Hierarchy tree
*/
class BVHNode
{
public:
    /*
        child nodes
    */
    BVHNode* children[2];

    /*
        parent node
    */
   BVHNode* parent;

    /*
        a single bounding volume (box) that encompasses
        all descendants of this node
    */
    BoundingBox volume;

    /*
        the rigid body of this node
        only leaf nodes have rigid bodies
    */
    RigidBody* body = NULL;

    /*
        create new node in hierarchy
    */
   BVHNode(BVHNode* parent, const BoundingBox& volume, RigidBody* body);

   /*
       delete this node from the hierarchy
   */
   ~BVHNode();

    /*
        checks if this node is a leaf
    */
    bool isLeaf() const;

    /*
        check if this node overlaps another node
    */
    bool overlaps(BVHNode* other) const;

    /*
        insert a new body into the tree
    */
    void insert(RigidBody* newBody, const BoundingBox& newVolume);

    /*
        checks the number of potential contacts from this node's descendants
    */
    uint64_t getPotentialContacts(PotentialContact* contacts, 
                                  uint64_t limit) const;

private:
    uint64_t getPotentialContactsWith(PotentialContact* contacts,
                                      BVHNode* other,
                                      uint64_t limit) const;

    void recalculateBoundingVolume();

};
