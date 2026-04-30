#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>

// helper functions

static inline glm::mat3 skewSymmetricVec3ToMat3(const glm::vec3 vector)
{
    return glm::mat3(0.0f, -vector.z, vector.y,
                     vector.z, 0.0f, -vector.x,
                     -vector.y, vector.x, 0.0f);
}

static inline glm::vec3 transformVec3byMat4(const glm::vec3 vector,
                                            const glm::mat4 matrix)
{
    return glm::vec3(matrix * glm::vec4(vector,1));
}