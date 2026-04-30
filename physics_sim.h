#include <vector>
#include "force_gens.h"
#include "fine_collision.h"
#include "contacts.h"


#ifndef PHYSICS_SIMULATION
#define PHYSICS_SIMULATION

class PhysicsSim
{
public:
    typedef std::vector<CollisionBox> CollisionBoxes;

    PhysicsSim();

    void addBox(float inverseMass,
                float linearDamping,
                float angularDamping,
                const glm::vec3& position,
                const glm::quat& orientation,
                const glm::vec3& velocity,
                const glm::vec3& rotation,
                const glm::mat3& inverseInertiaTensor);

    void startFrame();

    void runPhysics(float duration);

    std::vector<glm::vec3> getPositions();

    std::vector<glm::quat> getOrientations();

private:
    CollisionBoxes boxes; 

    CollisionPlane floor {glm::vec3(0.0f,0.5f,0.0f), 
                          -1.0f};
    
    CollisionData data;

    ContactResolver cr;

    Gravity g {glm::vec3(0.0f,-1.0f,0.0f)};
};

#endif