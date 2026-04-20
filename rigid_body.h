#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>

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
        inverse inertia tensor for moment of inertia
    */
    glm::mat3 inverseInertiaTensor;

    glm::mat3 inverseInertiaTensorWorld;

    glm::vec3 forceAccum;

    glm::vec3 torqueAccum;

    glm::vec3 acceleration;

    glm::vec3 lastFrameAcceleration;

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

    glm::vec3 getPosition() const;

    glm::quat getOrientation() const;

};