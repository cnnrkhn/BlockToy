#include <cmath>
#include "rigid_body.h"
#include <glm/vec4.hpp>
#include <glm/geometric.hpp>

// Private functions

void RigidBody::addForceAtPoint(const glm::vec3& force,
                                glm::vec3& point)
{
    point -= position;

    forceAccum += force;
    torqueAccum += glm::cross(point,force);
}

// Public functions

RigidBody::RigidBody(float inverseMass,
                     float linearDamping,
                     float angularDamping,
                     const glm::vec3& position,
                     const glm::quat& orientation,
                     const glm::vec3& velocity,
                     const glm::vec3& rotation,
                     const glm::mat3& inverseInertiaTensor)
    : inverseMass(inverseMass), linearDamping(linearDamping),
      angularDamping(angularDamping), position(position), 
      orientation(orientation), velocity(velocity), 
      rotation(rotation), inverseInertiaTensor(inverseInertiaTensor),
      forceAccum(glm::vec3(0.0f)), torqueAccum(glm::vec3(0.0f)),
      acceleration(glm::vec3(0.0f)), lastFrameAcceleration(glm::vec3(0.0f))
{
    this->updateData();
}

void RigidBody::integrate(float duration)
{
    lastFrameAcceleration = acceleration;
    lastFrameAcceleration += forceAccum * inverseMass;

    glm::vec3 angularAcceleration = inverseInertiaTensorWorld * torqueAccum;

    velocity += lastFrameAcceleration * duration;

    rotation += angularAcceleration * duration;

    velocity *= pow(linearDamping,duration);
    rotation *= pow(angularDamping,duration);

    position += velocity * duration;

    orientation += glm::quat(0,rotation) * orientation * duration * 0.5f;

    updateData();

    clearAccumulators();
}

void RigidBody::clearAccumulators()
{
    forceAccum = glm::vec3(0.0f);
    torqueAccum = glm::vec3(0.0f);
}

void RigidBody::updateData()
{
    orientation = glm::normalize(orientation);

    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f),position);
    glm::mat4 rotationMatrix = glm::mat4_cast(orientation);
    transformMatrix = translationMatrix * rotationMatrix;

    glm::mat3 R = glm::mat3_cast(orientation);
    inverseInertiaTensorWorld = R * inverseInertiaTensor * glm::transpose(R);
}

bool RigidBody::finiteMass()
{
    return inverseMass > 0;
}

void RigidBody::addForce(const glm::vec3& force)
{
    forceAccum += force;
}

float RigidBody::getMass() const
{
    return (1.0f/inverseMass);
}

glm::vec3 RigidBody::getPosition() const
{
    return position;
}

glm::quat RigidBody::getOrientation() const
{
    return orientation;
}

