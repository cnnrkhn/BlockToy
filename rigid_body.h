#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>

#ifndef RIGID_BODY
#define RIGID_BODY

class RigidBody
{
private:
    /*
        inverse mass allows for bodies to interact with
        things of infinite mass, but not with things of
        zero mass
    */
    float inverseMass;

    /*
        amount of damping applied to linear motion
    */
    float linearDamping;

    /*
        amount of damping applied to angular motion
    */
    float angularDamping;

    /*
        position of rigid body (center of mass)
    */
    glm::vec3 position;

    /*
        angular orientation of rigid body
        represented as a quaternion
    */
    glm::quat orientation;

    /*
        linear velocity of rigid body
    */
    glm::vec3 velocity;

    /*
        angular velocity of rigid body
        represented as a vector that can be decomposed
        into a axis vector and angular speed scalar
    */
    glm::vec3 rotation;

    /*
        inverse moment of inertia tensor with respect
        to local coordinates
    */
    glm::mat3 inverseInertiaTensor;

    /*
        inverse moment of inertia tensor with respect
        to world coordinates
    */
    glm::mat3 inverseInertiaTensorWorld;

    /*
        net force on the body
    */
    glm::vec3 forceAccum;

    /*
        net torque on the body
    */
    glm::vec3 torqueAccum;

    /*
        linear acceleration of the body
    */
    glm::vec3 acceleration;

    /*
        linear acceleration of the body last frame
    */
    glm::vec3 lastFrameAcceleration;

    /*
        matrix used to transform between local and world coordinates
    */
    glm::mat4 transformMatrix;

    /*
        adds force to the point on the rigid body
        both are in world space
    */
    void addForceAtPoint(const glm::vec3& force,
                         glm::vec3& point);

public:
    /*
        Constructor
    */
    RigidBody(float inverseMass,
              float linearDamping,
              float angularDamping,
              const glm::vec3& position,
              const glm::quat& orientation,
              const glm::vec3& velocity,
              const glm::vec3& rotation,
              const glm::mat3& inverseInertiaTensor);

    void integrate(float duration);

    void clearAccumulators();

    void updateData();
    
    bool finiteMass();

    void addForce(const glm::vec3& force);

    float getMass() const;

    float getInverseMass() const;

    glm::mat3 getInertiaTensor() const;

    glm::mat3 getInverseInertiaTensor() const;

    glm::mat3 getInertiaTensorWorld() const;

    glm::mat3 getInverseInertiaTensorWorld() const;

    glm::vec3 getPosition() const;

    void setPosition(glm::vec3& pos);

    glm::quat getOrientation() const;

    void setOrientation(glm::quat& orientate);

    glm::vec3 getVelocity() const;

    void addVelocity(glm::vec3& deltaVel);

    glm::vec3 getRotation() const;

    void addRotation(glm::vec3& deltaRot);

    glm::vec3 getLastFrameAcceleration() const;
};

#endif