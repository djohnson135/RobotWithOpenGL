#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>
#include "MatrixStack.h"
#include "Program.h"


#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

char* vertShaderPath = "../shaders/shader.vert";
char* fragShaderPath = "../shaders/shader.frag";

GLFWwindow *window;
double currentXpos, currentYpos;
glm::vec3 eye(0.0f, 0.0f, 8.0f);
glm::vec3 center(0.0f, 0.0f, 0.0f);
glm::vec3 up(0.0f, 1.0f, 0.0f);

Program program;
MatrixStack modelViewProjectionMatrix;

// Draw cube on screen
void DrawCube(glm::mat4& modelViewProjectionMatrix)
{
	program.SendUniformData(modelViewProjectionMatrix, "mvp");
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

class RobotElements
{
private:

public:
	RobotElements() {}
	~RobotElements() {}
};

void ConstructRobot()
{
}




void Display()
{	
	program.Bind();

	modelViewProjectionMatrix.loadIdentity();
	modelViewProjectionMatrix.pushMatrix();

	// Setting the view and Projection matrices
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	modelViewProjectionMatrix.Perspective(glm::radians(60.0f), float(width) / float(height), 0.1f, 100.0f);
	modelViewProjectionMatrix.LookAt(eye, center, up);
	
	// Model transformation for Cube 1
	modelViewProjectionMatrix.pushMatrix();
	modelViewProjectionMatrix.translate(-2.0f, 2.0f, 0.0f);
	modelViewProjectionMatrix.rotateY(glm::radians(45.0f));
	modelViewProjectionMatrix.scale(0.8);
	DrawCube(modelViewProjectionMatrix.topMatrix());
	modelViewProjectionMatrix.popMatrix();
	
	// Model transformation for Cube 2
	modelViewProjectionMatrix.pushMatrix();
	modelViewProjectionMatrix.translate(-2.0f, -2.0f, 0.0f);
	modelViewProjectionMatrix.rotateX(glm::radians(45.0f));
	modelViewProjectionMatrix.scale(0.8);
	DrawCube(modelViewProjectionMatrix.topMatrix());
	modelViewProjectionMatrix.popMatrix();
	
	// Model transformation for Cube 3
	modelViewProjectionMatrix.pushMatrix();
	modelViewProjectionMatrix.translate(2.0f, 0.0f, 0.0f);
	modelViewProjectionMatrix.rotateZ(glm::radians(45.0f));
	modelViewProjectionMatrix.scale(0.8);
	DrawCube(modelViewProjectionMatrix.topMatrix());
	modelViewProjectionMatrix.popMatrix();

	modelViewProjectionMatrix.popMatrix();

	program.Unbind();
	
}

// Mouse callback function
void MouseCallback(GLFWwindow* lWindow, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && GLFW_PRESS == action)
		std::cout << "Mouse left button is pressed." << std::endl;
}

// Mouse position callback function
void CursorPositionCallback(GLFWwindow* lWindow, double xpos, double ypos)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if (state == GLFW_PRESS)
		std::cout << "Mouse position is: x - " << xpos << ", y - " << ypos << std::endl;
}


// Keyboard character callback function
void CharacterCallback(GLFWwindow* lWindow, unsigned int key)
{
	std::cout << "Key " << (char)key << " is pressed." << std::endl;
}

void CreateCube()
{
	// x, y, z, r, g, b, ...
	float cubeVerts[] = {
		// Face x-
		-1.0f,	+1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		// Face x+
		+1.0f,	+1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		// Face y-
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		// Face y+
		+1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		// Face z-
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		// Face z+
		+1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f
	};

	GLuint vertBufferID;
	glGenBuffers(1, &vertBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, vertBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVerts), cubeVerts, GL_STATIC_DRAW);
	GLint posID = glGetAttribLocation(program.GetPID(), "position");
	glEnableVertexAttribArray(posID);
	glVertexAttribPointer(posID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
	GLint colID = glGetAttribLocation(program.GetPID(), "color");
	glEnableVertexAttribArray(colID);
	glVertexAttribPointer(colID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));

}

void FrameBufferSizeCallback(GLFWwindow* lWindow, int width, int height)
{
	glViewport(0, 0, width, height);
}

void Init()
{
	glfwInit();
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Assignment2 - <Your Name>", NULL, NULL);
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glewInit();
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	glfwSetMouseButtonCallback(window, MouseCallback);
	glfwSetCursorPosCallback(window, CursorPositionCallback);
	glfwSetCharCallback(window, CharacterCallback);
	glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	program.SetShadersFileName(vertShaderPath, fragShaderPath);
	program.Init();

	CreateCube();
}


int main()
{	
	Init();
	while ( glfwWindowShouldClose(window) == 0) 
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Display();
		glFlush();
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}