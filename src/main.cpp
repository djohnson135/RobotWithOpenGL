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
	glm::vec3 jointTranslationWrtParentJoint;
	glm::vec3 jointAngle;
	glm::vec3 translationWrtJoint;
	glm::vec3 scalingFactors;
	std::string identity;
	//glm::vec3 jointPos;
	//glm::vec3 attatchPoint;

public:
	RobotElements* parent;
	std::vector<RobotElements*> children;
	RobotElements() {}
	RobotElements(std::string _identity): identity() {}
	RobotElements(std::string _identity, glm::vec3 _jointTranslationWrtParentJoint, glm::vec3 _translationWrtJoint, glm::vec3 _scalingFactors, glm::vec3 _jointAngle) : 
		identity(_identity), jointTranslationWrtParentJoint(_jointTranslationWrtParentJoint), translationWrtJoint(_translationWrtJoint), scalingFactors(_scalingFactors), jointAngle(_jointAngle) {}
	~RobotElements() 
	{ 
		delete parent; 
		for (auto & robotElement : children) {
			delete robotElement;
		}
	}
	void set_identity(std::string identity) {
		this->identity = identity;
	}

	std::string get_identity() {
		return this->identity;
	}
	void Draw() 
	{
		//draw itself
		//for now just display the identity
		//std::cout << this->get_identity() << std::endl;

		
		//move componenent to center of coordinate
		modelViewProjectionMatrix.pushMatrix();
		//translate
		modelViewProjectionMatrix.translate(this->translationWrtJoint);

		modelViewProjectionMatrix.rotateX(glm::radians(this->jointAngle.x)); //need to calculate angle?
		modelViewProjectionMatrix.rotateY(glm::radians(this->jointAngle.y)); //need to calculate angle?
		modelViewProjectionMatrix.rotateZ(glm::radians(this->jointAngle.z)); //need to calculate angle?

		
		//scale
		modelViewProjectionMatrix.scale(this->scalingFactors);

		//translate to parent joint
		modelViewProjectionMatrix.translate(this->jointTranslationWrtParentJoint);
		
		// joint translation/ rotation/ component translation/ scaling

		modelViewProjectionMatrix.print("cube");
		DrawCube(modelViewProjectionMatrix.topMatrix());
		
		
		//pop
		modelViewProjectionMatrix.popMatrix();

		//draw children

		for (auto robotElement : this->children) {
			robotElement->Draw();
		}
	}


};

RobotElements* robot = nullptr;


RobotElements* ConstructRobot()
{

	//torso vectors
	glm::vec3 torsoJoint(0.0f, 0.0f, 0.0f);
	glm::vec3 torsoParentJoint(0.0f, 0.0f, 0.0f);
	glm::vec3 torsoScalingFactors(1.0f, 1.5f, 1.0f);
	glm::vec3 torsoJointAngle(0.0f, 0.0f, 0.0f);

	//rightarm vectors

	glm::vec3 rightArmJoint(0.0f, -0.3f, 0.0f);
	glm::vec3 rightArmParentJoint(3.0f, 0.0f, 0.0f);
	glm::vec3 rightArmScalingFactors(1.0f, 1.5f, 1.0f);
	glm::vec3 rightArmJointAngle(0.0f, 0.0f, 0.0f);


	//tree with torso as root
	RobotElements* torsoRoot = new RobotElements("torso", torsoParentJoint, torsoJoint, torsoScalingFactors, torsoJointAngle);
	torsoRoot->parent = nullptr; //root has no parent
	torsoRoot->children.push_back((RobotElements*) new RobotElements("left arm")); //leftarm
	torsoRoot->children.push_back((RobotElements*) new RobotElements("right arm", rightArmJoint, rightArmParentJoint, rightArmScalingFactors, rightArmJointAngle)); //rightarm
	torsoRoot->children.push_back((RobotElements*) new RobotElements("left leg")); //leftleg
	torsoRoot->children.push_back((RobotElements*) new RobotElements("right leg")); //rightleg
	torsoRoot->children.push_back((RobotElements*) new RobotElements("head")); //head


	for (int i = 0; i < torsoRoot->children.size() - 1; i++) {
		std::string identity = "lower " + torsoRoot->children[i]->get_identity();
		torsoRoot->children[i]->children.push_back((RobotElements*) new RobotElements(identity));
	}

	torsoRoot->Draw();

	return torsoRoot;

}

//transformations consists of 3 parts
//object positioning in world
//world to camera
//projection matrix

void Display()
{	
	program.Bind();

	modelViewProjectionMatrix.loadIdentity();
	modelViewProjectionMatrix.pushMatrix();
	//modelViewProjectionMatrix.print("Identity");

	// Setting the view and Projection matrices
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	modelViewProjectionMatrix.Perspective(glm::radians(60.0f), float(width) / float(height), 0.1f, 100.0f);
	modelViewProjectionMatrix.LookAt(eye, center, up);
	
	// Model transformation for Cube 1
	//modelViewProjectionMatrix.pushMatrix();
	//modelViewProjectionMatrix.translate(-2.0f, 2.0f, 0.0f);
	//modelViewProjectionMatrix.rotateY(glm::radians(45.0f));
	//modelViewProjectionMatrix.scale(0.8);
	//DrawCube(modelViewProjectionMatrix.topMatrix());
	//modelViewProjectionMatrix.popMatrix();

	robot->Draw();
	
	// Model transformation for Cube 2
	//modelViewProjectionMatrix.pushMatrix();
	//modelViewProjectionMatrix.translate(-2.0f, -2.0f, 0.0f);
	//modelViewProjectionMatrix.rotateX(glm::radians(45.0f));
	//modelViewProjectionMatrix.scale(0.8);
	//DrawCube(modelViewProjectionMatrix.topMatrix());
	//modelViewProjectionMatrix.popMatrix();
	//
	//// Model transformation for Cube 3
	//modelViewProjectionMatrix.pushMatrix();
	//modelViewProjectionMatrix.translate(2.0f, 0.0f, 0.0f);
	//modelViewProjectionMatrix.rotateZ(glm::radians(45.0f));
	//modelViewProjectionMatrix.scale(0.8);
	//DrawCube(modelViewProjectionMatrix.topMatrix());
	//modelViewProjectionMatrix.popMatrix();

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
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Assignment2 - Dathan Johnson", NULL, NULL);
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

	robot = ConstructRobot();
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