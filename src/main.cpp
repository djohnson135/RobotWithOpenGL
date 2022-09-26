#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <vector>
#include <iostream>
#include "MatrixStack.h"
#include "Program.h"

//#include <cmath>
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

double prev_xpos = -10000;
double prev_ypos = -10000;


// Draw cube on screen
void DrawCube(glm::mat4& modelViewProjectionMatrix)
{
	program.SendUniformData(modelViewProjectionMatrix, "mvp");
	glDrawArrays(GL_TRIANGLES, 0, 36);
}


class RobotElements
{
private:
	glm::vec3 jointTranslationWrtParentJoint; //global
	glm::vec3 jointAngle;//global
	glm::vec3 translationWrtJoint;//local translate joint pos (0,0,0)
	glm::vec3 scalingFactors;//local
	std::string identity;

public:
	RobotElements* parent;
	std::vector<RobotElements*> children;
	bool isSelected = false;
	RobotElements() {}
	RobotElements(std::string _identity): identity() {}
	RobotElements(std::string _identity, glm::vec3 _jointTranslationWrtParentJoint, glm::vec3 _translationWrtJoint, glm::vec3 _scalingFactors, glm::vec3 _jointAngle) : 
		identity(_identity), jointTranslationWrtParentJoint(_jointTranslationWrtParentJoint), translationWrtJoint(_translationWrtJoint), scalingFactors(_scalingFactors), jointAngle(_jointAngle) 
	{}
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

	void incrementAngleX() {
		this->jointAngle.x++;
	}
	void incrementAngleY() {
		this->jointAngle.y++;
	}
	void incrementAngleZ() {
		this->jointAngle.z++;
	}

	void DecrementAngleX() {
		this->jointAngle.x--;
	}
	void DecrementAngleY() {
		this->jointAngle.y--;
	}
	void DecrementAngleZ() {
		this->jointAngle.z--;
	}

	//MatrixStack & _matrixStack
	void Draw(MatrixStack * _modelViewProjectionMatrix)
	{

		_modelViewProjectionMatrix->pushMatrix();
		//global first then draw children
		_modelViewProjectionMatrix->translate(this->jointTranslationWrtParentJoint);
		//how do we calculate this. The joint angle with respect to what vector
		_modelViewProjectionMatrix->rotateX(glm::radians(this->jointAngle.x));
		_modelViewProjectionMatrix->rotateY(glm::radians(this->jointAngle.y));
		_modelViewProjectionMatrix->rotateZ(glm::radians(this->jointAngle.z));
		//draw children
		for (auto robotElement : this->children) {
			/*MatrixStack* parentTransformation = MatrixStack(*_modelViewProjectionMatrix);
			parentTransformation = _modelViewProjectionMatrix;*/
			robotElement->Draw(_modelViewProjectionMatrix);
			//delete parentTransformation;
		}
		
		//move componenent to center of coordinate how do we calculat his
		_modelViewProjectionMatrix->translate(this->translationWrtJoint);
		//scale
		_modelViewProjectionMatrix->scale(this->scalingFactors);

		if (isSelected) {
			_modelViewProjectionMatrix->scale(1.2, 1.2, 1.2);
		}

		//modelViewProjectionMatrix.print("cube");
		DrawCube(_modelViewProjectionMatrix->topMatrix());
		//pop
		_modelViewProjectionMatrix->popMatrix();
	}
};

RobotElements* robot = nullptr;
RobotElements* selectedComponent = nullptr;
std::vector<RobotElements*> dfs_node_list;
std::vector<RobotElements*> visited_node_list;
int node_list_iter = 0;
RobotElements* ConstructRobot()
{

	//torso vectors
	//translate (right/left, up/down, forwards/backwards
	glm::vec3 torsoJoint(0.0f, 0.0f, 0.0f);
	glm::vec3 torsoParentJoint(0.0f, 0.0f, 0.0f);
	glm::vec3 torsoScalingFactors(0.7f, 1.1f, 0.7f);
	glm::vec3 torsoJointAngle(0.0f, 0.0f, 0.0f);

	//up one and 0.7 by 1.1 by o.7

	//head vectors

	glm::vec3 headJoint(0.0f, +0.4f, 0.0f);
	glm::vec3 headParentJoint(0.0f, 1.1f, 0.0f);
	glm::vec3 headScalingFactors(0.5f, 0.4f, 0.5f);
	glm::vec3 headJointAngle(0.0f, 0.0f, 0.0f); 


	//rightarm

	glm::vec3 rightArmJoint(0.35f, 0.0f, 0.0f);
	glm::vec3 rightArmParentJoint(0.6f, 0.7f, 0.0f);
	glm::vec3 rightArmScalingFactors(0.7f, 0.3f, 0.3f);
	glm::vec3 rightArmJointAngle(0.0f, 0.0f, 0.0f);

	//lowerRightArm
	glm::vec3 lRightArmJoint(0.35f, 0.0f, 0.0f);
	glm::vec3 lRightArmParentJoint(1.1f, 0.7f, 0.0f);
	glm::vec3 lRightArmScalingFactors(0.7f, 0.2f, 0.2f);
	glm::vec3 lRightArmJointAngle(0.0f, 0.0f, 0.0f);


	//leftarm
	glm::vec3 leftArmJoint(-0.35f, 0.0f, 0.0f);
	glm::vec3 leftArmParentJoint(-0.6f, 0.7f, 0.0f);
	glm::vec3 leftArmScalingFactors(0.7f, 0.3f, 0.3f);
	glm::vec3 leftArmJointAngle(0.0f, 0.0f, 0.0f);

	//lowerLeftArm
	glm::vec3 lLeftArmJoint(-0.35f, 0.0f, 0.0f);
	glm::vec3 lLeftArmParentJoint(-1.1f, 0.7f, 0.0f);
	glm::vec3 lLeftArmScalingFactors(0.7f, 0.2f, 0.2f);
	glm::vec3 lLeftArmJointAngle(0.0f, 0.0f, 0.0f);


	//LeftLeg
	glm::vec3 leftLegJoint(0.0f, -0.3f, 0.0f);
	glm::vec3 leftLegParentJoint(-0.4f, -1.2f, 0.0f);
	glm::vec3 leftLegScalingFactors(0.3f, 0.8f, 0.3f);
	glm::vec3 leftLegJointAngle(0.0f, 0.0f, 0.0f);

	//lowerLeftLeg
	glm::vec3 lLeftLegJoint(0.0f, -0.3f, 0.0f);
	glm::vec3 lLeftLegParentJoint(-0.4f, -2.0f, 0.0f);
	glm::vec3 lLeftLegScalingFactors(0.2f, 0.8f, 0.2f);
	glm::vec3 lLeftLegJointAngle(0.0f, 0.0f, 0.0f);


	//RightLeg
	glm::vec3 rightLegJoint(0.0f, -0.3f, 0.0f);
	glm::vec3 rightLegParentJoint(0.4f, -1.2f, 0.0f);
	glm::vec3 rightLegScalingFactors(0.3f, 0.8f, 0.3f);
	glm::vec3 rightLegJointAngle(0.0f, 0.0f, 0.0f);

	//lowerRightLeg
	glm::vec3 lrightLegJoint(0.0f, -0.3f, 0.0f);
	glm::vec3 lrightLegParentJoint(0.4f, -2.0f, 0.0f);
	glm::vec3 lrightLegScalingFactors(0.2f, 0.8f, 0.2f);
	glm::vec3 lrightLegJointAngle(0.0f, 0.0f, 0.0f);


	//tree with torso as root
	RobotElements* torsoRoot = new RobotElements("torso", torsoParentJoint, torsoJoint, torsoScalingFactors, torsoJointAngle);
	torsoRoot->parent = nullptr; //root has no parent
	torsoRoot->children.push_back((RobotElements*) new RobotElements("left arm", leftArmJoint, leftArmParentJoint, leftArmScalingFactors, leftArmJointAngle)); //leftarm
	torsoRoot->children[0]->children.push_back((RobotElements*) new RobotElements("lower left arm", lLeftArmJoint, lLeftArmParentJoint, lLeftArmScalingFactors, lLeftArmJointAngle));

	torsoRoot->children.push_back((RobotElements*) new RobotElements("right arm", rightArmJoint, rightArmParentJoint, rightArmScalingFactors, rightArmJointAngle)); //rightarm
	torsoRoot->children[1]->children.push_back((RobotElements*) new RobotElements("lower right arm", lRightArmJoint, lRightArmParentJoint, lRightArmScalingFactors, lRightArmJointAngle));

	torsoRoot->children.push_back((RobotElements*) new RobotElements("left leg", leftLegJoint, leftLegParentJoint, leftLegScalingFactors, leftLegJointAngle)); //leftleg
	torsoRoot->children[2]->children.push_back((RobotElements*) new RobotElements("lower left leg", lLeftLegJoint, lLeftLegParentJoint, lLeftLegScalingFactors, lLeftLegJointAngle));

	torsoRoot->children.push_back((RobotElements*) new RobotElements("right leg", rightLegJoint, rightLegParentJoint, rightLegScalingFactors, rightLegJointAngle)); //rightleg
	torsoRoot->children[3]->children.push_back((RobotElements*) new RobotElements("lower right leg", lrightLegJoint, lrightLegParentJoint, lrightLegScalingFactors, lrightLegJointAngle));

	torsoRoot->children.push_back((RobotElements*) new RobotElements("head", headJoint, headParentJoint, headScalingFactors, headJointAngle)); //head
	
	
	return torsoRoot;
}


void dfs() {
	//until root is hit build search order and add to node list
	RobotElements* curnode;
	while (!dfs_node_list.empty()) {
		curnode = dfs_node_list.back();
		dfs_node_list.pop_back();
		visited_node_list.push_back(curnode);
		//expand
		for (auto iter : curnode->children) {
			dfs_node_list.push_back(iter);
		}
	}

}

void dfsForwards() {
	if (visited_node_list.size() == node_list_iter) {
		node_list_iter = 0;
		
	}

	selectedComponent->isSelected = false;
	selectedComponent = visited_node_list[node_list_iter];
	selectedComponent->isSelected = true;
	node_list_iter++;
}

void dfsBackwards() {
	if (node_list_iter == -1) {
		node_list_iter = visited_node_list.size()-1;
	}

	selectedComponent->isSelected = false;
	selectedComponent = visited_node_list[node_list_iter];
	selectedComponent->isSelected = true;
	node_list_iter--;
}

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

	robot->Draw(&modelViewProjectionMatrix);

	modelViewProjectionMatrix.popMatrix();

	program.Unbind();
	
}

void cameraRotation(double xpos, double ypos) {

	//glm::vec3 lookat = glm::normalize(center - eye);
	
	//calculate phi and theta

	glm::vec3 a(eye.x - center.x, eye.y - center.y, eye.z - center.z);
	glm::vec3 right = glm::cross(a, up);

	//rotate on specific axis. glm::rotate(angle (radians), axis(vec3)). gives rotatation matrix. Multiply with the matrix to rotate. [rotation

	int xDiff = xpos - prev_xpos;
	if (xDiff > 0) {
		glm::vec3 Arotated = glm::rotate(a, glm::radians(-2.0f), up);
		eye = center + Arotated;
	}
	else if (xDiff < 0) {
		glm::vec3 Arotated = glm::rotate(a, glm::radians(2.0f), up);
		eye = center + Arotated;
	}

	int yDiff = ypos - prev_ypos;
	if (yDiff > 0) {
		glm::vec3 Arotated = glm::rotate(a, glm::radians(-2.0f), right);
		eye = center + Arotated;
		up = glm::normalize(glm::cross(right, Arotated)); //remove normalize
	}
	else if (yDiff < 0) {
		glm::vec3 Arotated = glm::rotate(a, glm::radians(2.0f), right);
		eye = center + Arotated;
		up = glm::normalize(glm::cross(right, Arotated));
	}
}

void cameraZoom(double yoffset) {

	//need to change this from center to look at vector
	//a = eye - lookat
	

	if (yoffset > 0) {
		//zoom in
		eye.x = (eye.x - center.x) * 0.9;
		eye.y = (eye.y - center.y) * 0.9;
		eye.z = (eye.z - center.z) * 0.9;
	}
	else if (yoffset < 0) {
		//zoom out
		eye.x = (eye.x - center.x) * 1.1;
		eye.y = (eye.y - center.y) * 1.1;
		eye.z = (eye.z - center.z) * 1.1;
	}
}

void cameraTranslate(double xpos, double ypos) {
	//change in x
	//windowwidth
	glm::vec3 a(eye.x - center.x, eye.y - center.y, eye.z - center.z);
	glm::vec3 right = glm::normalize(glm::cross(a, up));
	int xDiff = xpos - prev_xpos;
	if (xDiff > 0) {
		//translation factor = xdiff/windowWidth right direction
		//eye = eye + -right;
		eye.x = eye.x - right.x * 0.2;
		eye.y = eye.y - right.y * 0.2;
		eye.z = eye.z - right.z * 0.2;
		
		//center = center -right;
		center.x = center.x - right.x * 0.2;
		center.y = center.y - right.y * 0.2;
		center.z = center.z - right.z * 0.2;

	}
	else if (xDiff < 0) {
		//translation factor = xdiff/windowWidth left direction
		//eye = eye + right;
		eye.x = eye.x + right.x * 0.2;
		eye.y = eye.y + right.y * 0.2;
		eye.z = eye.z + right.z * 0.2;

		//center = center + right;
		center.x = center.x + right.x * 0.2;
		center.y = center.y + right.y * 0.2;
		center.z = center.z + right.z * 0.2;
	}

	int yDiff = ypos - prev_ypos;
	if (yDiff > 0) {
		//translation factor = ydiff/windowHeight up direction
		/*eye = eye + up;
		center = center + up;*/
		eye.x = eye.x + up.x * 0.2;
		eye.y = eye.y + up.y * 0.2;
		eye.z = eye.z + up.z * 0.2;

		center.x = center.x + up.x * 0.2;
		center.y = center.y + up.y * 0.2;
		center.z = center.z + up.z * 0.2;
	}
	else if (yDiff < 0) {
		//translation factor = ydiff/windowHeight down direction

		/*eye = eye + -up;
		center = center + -up;*/

		eye.x = eye.x - up.x * 0.2;
		eye.y = eye.y - up.y * 0.2;
		eye.z = eye.z - up.z * 0.2;

		center.x = center.x - up.x * 0.2;
		center.y = center.y - up.y * 0.2;
		center.z = center.z - up.z * 0.2;
	}

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
	//add stuff here
	int leftMouseState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	int rightMouseState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
	if (leftMouseState == GLFW_PRESS) {
		std::cout << "Mouse position is: x - " << xpos << ", y - " << ypos << std::endl;
		if (prev_xpos && prev_ypos != -10000) cameraRotation(xpos, ypos);
		prev_xpos = xpos;
		prev_ypos = ypos;
		
	}
	else if (rightMouseState == GLFW_PRESS) {
		if (prev_xpos && prev_ypos != -10000) cameraTranslate(xpos, ypos);
		prev_xpos = xpos;
		prev_ypos = ypos;
	}
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	cameraZoom(yoffset);
}


// Keyboard character callback function
void CharacterCallback(GLFWwindow* lWindow, unsigned int key)
{
	std::cout << "Key " << (char)key << " is pressed." << std::endl;

	switch ((char)key) {
	case '.':
		dfsForwards();
		break;
	case ',':
		dfsBackwards();
		break;
	case 'x':
		selectedComponent->incrementAngleX();
		break;
	case 'X':
		selectedComponent->DecrementAngleX();
		break;
	case 'y':
		selectedComponent->incrementAngleY();
		break;
	case 'Y':
		selectedComponent->DecrementAngleY();
		break;
	case 'z':
		selectedComponent->incrementAngleZ();
		break;
	case 'Z':
		selectedComponent->DecrementAngleZ();
		break;
	default:
		break;
	}
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
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	program.SetShadersFileName(vertShaderPath, fragShaderPath);
	program.Init();

	robot = ConstructRobot();
	selectedComponent = robot;
	dfs_node_list.push_back(robot);
	dfs();
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