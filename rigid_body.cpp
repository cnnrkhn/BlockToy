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

float RigidBody::getInverseMass() const
{
    return inverseMass;
}

glm::mat3 RigidBody::getInertiaTensor() const
{
    return glm::inverse(inverseInertiaTensor);
}

glm::mat3 RigidBody::getInverseInertiaTensor() const
{
    return glm::mat3(inverseInertiaTensor);
}

glm::mat3 RigidBody::getInertiaTensorWorld() const
{
    return glm::inverse(inverseInertiaTensorWorld);
}

glm::mat3 RigidBody::getInverseInertiaTensorWorld() const
{
    return glm::mat3(inverseInertiaTensorWorld);
}


glm::vec3 RigidBody::getPosition() const
{
    return glm::vec3(position);
}

void RigidBody::setPosition(glm::vec3& pos)
{
    position = glm::vec3(pos);
}

glm::quat RigidBody::getOrientation() const
{
    return glm::quat(orientation);
}

void RigidBody::setOrientation(glm::quat& orientate)
{
    orientation = orientate;
    orientation = glm::normalize(orientation);
}

glm::vec3 RigidBody::getVelocity() const
{
    return glm::vec3(velocity);
}

void RigidBody::addVelocity(glm::vec3& deltaVel)
{
    velocity += deltaVel;
}

glm::vec3 RigidBody::getRotation() const
{
    return glm::vec3(rotation);
}

void RigidBody::addRotation(glm::vec3& deltaRot)
{
    rotation += deltaRot;
}

glm::vec3 RigidBody::getLastFrameAcceleration() const
{
    return glm::vec3(lastFrameAcceleration);
}

glm::mat4 RigidBody::getTransformMatrix() const
{
    return glm::mat4(transformMatrix);
}