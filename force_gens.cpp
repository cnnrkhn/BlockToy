#include "force_gens.h"

Gravity::Gravity(const glm::vec3& gravity) : gravity(gravity) {}

void Gravity::updateForce(RigidBody* body, float duration)
{
    if (!body->finiteMass()) return;

    body->addForce(gravity * body->getMass());
}