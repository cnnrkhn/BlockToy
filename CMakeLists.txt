cmake_minimum_required(VERSION 3.28)

project(ProjectBlockToy)

add_subdirectory(external/glad)

add_subdirectory(external/glfw-3.4)

add_subdirectory(external/glm-1.0.3)

# remove this
#add_executable(BlockToy)

#target_sources(BlockToy
#  PRIVATE
#    blocktoy.cpp
#    physengine.cpp
#    physengine.h
#    solid.cpp
#    solid.h
#    rigid_body.cpp
#    rigid_body.h
#    force_gens.cpp
#    force_gens.h
#)

#target_link_libraries(BlockToy
#    glm
#) 

add_executable(Visualizer
    visualizer.cpp
    shader.cpp
    shader.h
    rigid_body.cpp
    rigid_body.h
    force_gens.cpp
    force_gens.h
    physics_sim.cpp
    physics_sim.h
    broad_collision.cpp
    broad_collision.h
) 



target_link_libraries(Visualizer
    glad
    glfw
    glm
) 
