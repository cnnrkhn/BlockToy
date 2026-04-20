#include "rigid_body.h"

class ForceGenerator
{
public:
    virtual void updateForce(RigidBody* body, float duration) = 0;
};

class Gravity : public ForceGenerator
{
private:
    glm::vec3 gravity;

public:
    Gravity(const glm::vec3& gravity);

    void updateForce(RigidBody* body, float duration) override;
};