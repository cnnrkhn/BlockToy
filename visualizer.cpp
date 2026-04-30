#include <iostream>
#include <chrono>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include "shader.h"

#include "physics_sim.h"

using namespace std; 

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int main()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        cout << "Failed to initialize GLFW" << endl;
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    GLFWwindow* window;
    window = glfwCreateWindow(800, 600, "BlockToy", NULL, NULL);
    if (window == NULL)
    {
        cout << "Failed to open GLFW window" << endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Failed to initialize GLAD" << endl;
        return -1;
    }

    // Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

    // Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	GLuint vertexArrayID;
	glGenVertexArrays(1, &vertexArrayID);
	glBindVertexArray(vertexArrayID);

    Shader shaders("../shaders/vertex.vs", "../shaders/fragment.fs");

    // Get a handle for our "MVP" uniform
	GLuint matrixID = glGetUniformLocation(shaders.ID, "mvp");

	// Projection matrix
	glm::mat4 projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	glm::mat4 view = glm::lookAt(glm::vec3(10,2,0), glm::vec3(0,0,0), glm::vec3(0,1,0));
	// Model matrix
    glm::quat q = glm::quat(1,1,1,1);
    q = glm::normalize(q);
    glm::mat4 rotate = glm::mat4_cast(q);
	glm::vec3 position = glm::vec3(0.0f,2.0f,0.0f);
    glm::mat4 translate = glm::translate(glm::mat4(1.0f),position);
	glm::mat4 model = translate * rotate;
	// Our ModelViewProjection 
	glm::mat4 mvp = projection * view * model;

    static const GLfloat g_vertex_buffer_data[] = { 
		-1.0f,-1.0f,-1.0f,
		-1.0f,-1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		 1.0f, 1.0f,-1.0f,
		-1.0f,-1.0f,-1.0f,
		-1.0f, 1.0f,-1.0f,
		 1.0f,-1.0f, 1.0f,
		-1.0f,-1.0f,-1.0f,
		 1.0f,-1.0f,-1.0f,
		 1.0f, 1.0f,-1.0f,
		 1.0f,-1.0f,-1.0f,
		-1.0f,-1.0f,-1.0f,
		-1.0f,-1.0f,-1.0f,
		-1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f,-1.0f,
		 1.0f,-1.0f, 1.0f,
		-1.0f,-1.0f, 1.0f,
		-1.0f,-1.0f,-1.0f,
		-1.0f, 1.0f, 1.0f,
		-1.0f,-1.0f, 1.0f,
		 1.0f,-1.0f, 1.0f,
		 1.0f, 1.0f, 1.0f,
		 1.0f,-1.0f,-1.0f,
		 1.0f, 1.0f,-1.0f,
		 1.0f,-1.0f,-1.0f,
		 1.0f, 1.0f, 1.0f,
		 1.0f,-1.0f, 1.0f,
		 1.0f, 1.0f, 1.0f,
		 1.0f, 1.0f,-1.0f,
		-1.0f, 1.0f,-1.0f,
		 1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f,-1.0f,
		-1.0f, 1.0f, 1.0f,
		 1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		 1.0f,-1.0f, 1.0f
	};

	static const GLfloat g_color_buffer_data[] = { 
		0.583f,  0.771f,  0.014f,
		0.609f,  0.115f,  0.436f,
		0.327f,  0.483f,  0.844f,
		0.822f,  0.569f,  0.201f,
		0.435f,  0.602f,  0.223f,
		0.310f,  0.747f,  0.185f,
		0.597f,  0.770f,  0.761f,
		0.559f,  0.436f,  0.730f,
		0.359f,  0.583f,  0.152f,
		0.483f,  0.596f,  0.789f,
		0.559f,  0.861f,  0.639f,
		0.195f,  0.548f,  0.859f,
		0.014f,  0.184f,  0.576f,
		0.771f,  0.328f,  0.970f,
		0.406f,  0.615f,  0.116f,
		0.676f,  0.977f,  0.133f,
		0.971f,  0.572f,  0.833f,
		0.140f,  0.616f,  0.489f,
		0.997f,  0.513f,  0.064f,
		0.945f,  0.719f,  0.592f,
		0.543f,  0.021f,  0.978f,
		0.279f,  0.317f,  0.505f,
		0.167f,  0.620f,  0.077f,
		0.347f,  0.857f,  0.137f,
		0.055f,  0.953f,  0.042f,
		0.714f,  0.505f,  0.345f,
		0.783f,  0.290f,  0.734f,
		0.722f,  0.645f,  0.174f,
		0.302f,  0.455f,  0.848f,
		0.225f,  0.587f,  0.040f,
		0.517f,  0.713f,  0.338f,
		0.053f,  0.959f,  0.120f,
		0.393f,  0.621f,  0.362f,
		0.673f,  0.211f,  0.457f,
		0.820f,  0.883f,  0.371f,
		0.982f,  0.099f,  0.879f
	};

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	GLuint colorbuffer;
	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

    glViewport(0, 0, 800, 600);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // initialize physics simulation
    PhysicsSim ps;

    float invMass = 0.5f;

    // calculate moment of inertia for a box
    float hh = 2 * 2;
    float ww = 2 * 2;
    float dd = 2 * 2;
    float massDiv12 = (1.0f / 12.0f) * invMass;

    glm::mat3 momentInertia = glm::mat3(
        massDiv12 * (hh + dd), 0,                     0,
        0,                     massDiv12 * (ww + dd), 0,
        0,                     0,                     massDiv12 * (ww + hh)
    );

    // add rigid body
    ps.addBox(invMass,          // inverse mass
               0.99f,            // linear damping
               0.99f,            // angular damping
               position, // position
               q,                // orientation
               glm::vec3(0,0,0),  // velocity
               glm::vec3(0,0,0),  // rotation
               glm::inverse(momentInertia), // inverse moment of inertia
			   glm::vec3(1,1,1)); // halfWidths

    // measure ellapsed_time between frames
    float ellapsedTime = 0;
    chrono::steady_clock::time_point timePrev = chrono::steady_clock::now();
    chrono::steady_clock::time_point timeNow;

	// count number of frames
	int numFrames = 0;
	float secondCounter = 0;

	// initial frame
	ps.startFrame();

    // Check if window was closed
    while(!glfwWindowShouldClose(window))
    {
        // physics
        timeNow = chrono::steady_clock::now();

        ellapsedTime = chrono::duration_cast<chrono::nanoseconds>(timeNow - timePrev).count() / 1000000000.0f;
		//ellapsedTime = 0.0;

        ps.runPhysics(ellapsedTime);
        glm::vec3 position = ps.getPositions()[0];
        glm::quat orientation = ps.getOrientations()[0];

        rotate = glm::mat4_cast(orientation);
        translate = glm::translate(glm::mat4(1.0f),position);
	    model = translate * rotate;

		//view = glm::lookAt(glm::vec3(5,0,0), position * 0.5f, glm::vec3(0,1,0));

	    mvp = projection * view * model;

		// measure framerate
		numFrames++;
		secondCounter += ellapsedTime;
		if (secondCounter >= 1.0f)
		{
			//cout << "FPS:" << numFrames << endl;
			//cout << "Ellapsed Time:" << ellapsedTime << endl;
			numFrames = 0;
			secondCounter -= 1.0f;
		}

        
        timePrev = timeNow;

        // input
        processInput(window);

        // rendering
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shaders.use();

        glUniformMatrix4fv(matrixID, 1, GL_FALSE, &mvp[0][0]);

        glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

        glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glVertexAttribPointer(
			1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

        glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

        // check and call events
        glfwSwapBuffers(window);
        // swap buffers
        glfwPollEvents();
    }

    // Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &colorbuffer);
	glDeleteProgram(shaders.ID);
	glDeleteVertexArrays(1, &vertexArrayID);

    // CLose OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}
