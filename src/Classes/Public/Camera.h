#pragma once
#include "Shader.h"
#include "Renderer.h"

#include "glm/glm.hpp"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include "memory"

class Camera
{
private:

	glm::vec3 m_Position;
	glm::vec3 m_Orientation = glm::vec3(0.0f, 0.f, -1.0f);
	glm::vec3 m_Up = glm::vec3(0.0f, 1.0f, 0.0f);

	// Prevents the camera from jumping around when first clicking left click
	bool m_FirstClick = true;

	// Adjust the speed of the camera and it's sensitivity when looking around
	float m_Speed = 0.1f;
	float m_Sensitivity = 100.0f;

	static Camera* m_Camera;

	void IncreaseSpeed(float value);

	// called when scroll is triggered
	static void Scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
public:
	bool BlockInput = false;

	// Stores the width and height of the window
	static int Width;
	static int Height;

	static float DeltaSpeed;

	Camera(glm::vec3 position);
	~Camera() {};

	// Updates and exports the camera matrix to the Vertex Shader
	void Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const std::string& uniform);

	// Handles camera inputs
	void Inputs(GLFWwindow* window);

	// sets orientation for camera
	void SetOrientation(glm::vec3 nOrientation) { m_Orientation = normalize(nOrientation); };

	void SetPosition(glm::vec3 nPosition) { m_Position = nPosition; };
	glm::vec3 GetPosition() const { return m_Position; };

	static Camera* GetWorkingCamera() { return m_Camera; };
	static void SetWorkingCamera(Camera* cam) { m_Camera = cam; };

	// sets scroll reaction
	static void SetScrollInput(GLFWwindow* window, bool value = true);
};