#include <glad/glad.h>
#include <glm/glm.hpp>

#include "vboindex.h"

#ifndef CREATE_BLOCKS
#define DREATE_BLOCKS

struct Block
{

};

class BlockMaker
{
public:
    Block createCube();

    Block createRectPrism(float length, float width, float height);
};

#endif