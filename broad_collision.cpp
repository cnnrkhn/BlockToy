#include "broad_collision.h"
#include <algorithm>

// BoundingBox

BoundingBox::BoundingBox(const glm::vec3& center, const glm::vec3& halfWidths)
{
    this->center = center;
    this->halfWidths = halfWidths;
}

BoundingBox::BoundingBox(const BoundingBox& b1, const BoundingBox& b2)
{
    glm::vec3 maxes1 = b1.center + b1.halfWidths;
    glm::vec3 mins1 = b1.center - b1.halfWidths;

    glm::vec3 maxes2 = b2.center + b2.halfWidths;
    glm::vec3 mins2 = b2.center - b2.halfWidths;

    float max_x = std::max(maxes1.x, maxes2.x);
    float max_y = std::max(maxes1.y, maxes2.y);
    float max_z = std::max(maxes1.z, maxes2.z);

    float min_x = std::min(mins1.x, mins2.x);
    float min_y = std::min(mins1.y, mins2.y);
    float min_z = std::min(mins1.z, mins2.z);

    float halfWidth_x = (max_x - min_x) / 2.0f;
    float halfWidth_y = (max_y - min_y) / 2.0f;
    float halfWidth_z = (max_z - min_z) / 2.0f;
    halfWidths = glm::vec3(halfWidth_x, 
                           halfWidth_y, 
                           halfWidth_z);

    center = glm::vec3(min_x,min_y,min_z) + halfWidths;
}


bool BoundingBox::overlaps(const BoundingBox* other) const
{
    if (other == NULL) return false;

    glm::vec3 maxes1 = center + halfWidths;
    glm::vec3 mins1 = center - halfWidths;

    glm::vec3 maxes2 = other->center + other->halfWidths;
    glm::vec3 mins2 = other->center - other->halfWidths;

    bool overlapping = maxes1.x >= mins2.x && maxes2.x >= mins1.x &&
                       maxes1.y >= mins2.y && maxes2.y >= mins1.y &&
                       maxes1.z >= mins2.z && maxes2.z >= mins1.z;
    
    return overlapping;
}

float BoundingBox::getSize() const
{
    return 8 * halfWidths.x * halfWidths.y * halfWidths.z;
}

float BoundingBox::getGrowth(const BoundingBox& other) const
{
    BoundingBox newBox(*this, other);

    return newBox.getSize() - getSize();
}

// BVHNode

BVHNode::BVHNode(BVHNode* parent, 
                 const BoundingBox& volume, 
                 RigidBody* body)
    : parent(parent), volume(volume), body(body)
{
    children[0] = children[1] = NULL;
}

BVHNode::~BVHNode()
{
    if(parent)
    {
        BVHNode* sibling;
        if (parent->children[0] == this) sibling = parent->children[1];
        else sibling = parent->children[0];

        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];

        sibling->parent = NULL;
        sibling->body = NULL;
        sibling->children[0] = NULL;
        sibling->children[1] = NULL;
        delete sibling;

        parent->recalculateBoundingVolume();
    }

    if (children[0])
    {
        children[0]->parent = NULL;
        delete children[0];
    }

    if (children[1])
    {
        children[1]->parent = NULL;
        delete children[1];
    }
}

bool BVHNode::isLeaf() const
{
    return (body != NULL);
}

bool BVHNode::overlaps(BVHNode* other) const
{
    return volume.overlaps(&(other->volume));
}

uint32_t BVHNode::getPotentialContacts(PotentialContact* contacts, 
                                       uint32_t limit) const
{
    if (isLeaf() || limit == 0) return 0;

    return children[0]->getPotentialContactsWith(contacts,children[1],limit);
}

void BVHNode::insert(RigidBody* newBody, const BoundingBox& newVolume)
{
    if (isLeaf())
    {
        children[0] = new BVHNode(this, volume, body);

        children[1] = new BVHNode(this, newVolume, newBody);

        this->body = NULL;

        recalculateBoundingVolume();
    }
    else
    {
        if (children[0]->volume.getGrowth(newVolume) <
            children[1]->volume.getGrowth(newVolume))
        {
            children[0]->insert(newBody,newVolume);
        }
        else
        {
            children[1]->insert(newBody,newVolume);
        }
    }
}

uint32_t BVHNode::getPotentialContactsWith(PotentialContact* contacts,
                                           BVHNode* other,
                                           uint32_t limit) const
{
    if (!overlaps(other) || limit == 0) return 0;

    if (isLeaf() && other->isLeaf())
    {
        contacts->bodies[0] = body;
        contacts->bodies[1] = other->body;
        return 1;
    }

    if (other->isLeaf() ||
        (!isLeaf() && volume.getSize() >= other->volume.getSize()))
    {
        uint32_t count = children[0]->getPotentialContactsWith(contacts,other,limit);

        if (limit > count) 
        {
            return count + children[1]->getPotentialContactsWith(contacts+count,other,limit+count);
        } 
        else 
        {
            return count;
        }
    }
    else 
    {
        unsigned count = getPotentialContactsWith(contacts,other->children[0],limit);

        if (limit > count) 
        {
            return count + getPotentialContactsWith(contacts+count,other->children[1],limit+count);
        }
        else
        {
            return count;
        }
    }
}

void BVHNode::recalculateBoundingVolume()
{
    if(isLeaf()) return;

    volume = BoundingBox(children[0]->volume,
                         children[1]->volume);
    
    if (parent) parent->recalculateBoundingVolume();
}

