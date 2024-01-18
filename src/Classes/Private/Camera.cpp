#include "../Public/Camera.h"
#include "../Public/Math.h"

int Camera::Height = 800;
int Camera::Width = 800;
float Camera::DeltaSpeed = 0.02f;
Camera* Camera::m_Camera = nullptr;

Camera::Camera(glm::vec3 m_Position) : m_Position(m_Position)
{
}

void Camera::IncreaseSpeed(float value)
{
	// potential new speed
	float pot_new_speed = m_Speed + value * DeltaSpeed;

	if(pot_new_speed > 0.0)
		m_Speed = pot_new_speed;
}

void Camera::Matrix(float FOVdeg, float nearPlane, float farPlane, Shader& shader, const std::string& uniform)
{
	// Initializes matrices since otherwise they will be the null matrix
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 projection = glm::mat4(1.0f);

	// Makes camera look in the right direction from the right Position
	view = ViewMatrix(m_Position, m_Position + m_Orientation, m_Up);
	// Adds perspective to the scene
	projection = ProjectionMatrix(glm::radians(FOVdeg), (float)Width / Height, nearPlane, farPlane);

	shader.SetUniformMatrix4fv(uniform.c_str(), glm::value_ptr(projection * view));
}

void Camera::Inputs(GLFWwindow* window)
{
	if (BlockInput)
		return;
	// Handles key inputs
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		m_Position += m_Speed * m_Orientation;
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		m_Position += m_Speed * -glm::normalize(glm::cross(m_Orientation, m_Up));
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		m_Position += m_Speed * -m_Orientation;
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		m_Position += m_Speed * glm::normalize(glm::cross(m_Orientation, m_Up));
	}
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		auto R = glm::normalize(glm::cross(m_Orientation, m_Up));
		auto D = glm::normalize(glm::cross(R, m_Orientation));
		m_Position += m_Speed * D;
	}
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
	{
		auto R = glm::normalize(glm::cross(m_Orientation, m_Up));
		auto D = glm::normalize(glm::cross(R, m_Orientation));
		m_Position += m_Speed * -D;
	}

	// Handles mouse inputs
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		// Hides mouse cursor
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

		// Prevents camera from jumping on the first click
		if (m_FirstClick)
		{
			glfwSetCursorPos(window, (Width / 2), (Height / 2));
			m_FirstClick = false;
		}

		// Stores the coordinates of the cursor
		double mouseX;
		double mouseY;
		// Fetches the coordinates of the cursor
		glfwGetCursorPos(window, &mouseX, &mouseY);

		// Normalizes and shifts the coordinates of the cursor such that they begin in the middle of the screen
		// and then "transforms" them into degrees 
		float rotX = m_Sensitivity * (float)(mouseY - (Height / 2)) / Height;
		float rotY = m_Sensitivity * (float)(mouseX - (Width / 2)) / Width;

		// Calculates m_Upcoming vertical change in the m_Orientation
		glm::vec3 newm_Orientation = glm::rotate(m_Orientation, glm::radians(-rotX), glm::normalize(glm::cross(m_Orientation, m_Up)));

		// Decides whether or not the next vertical m_Orientation is legal or not
		if (abs(glm::angle(newm_Orientation, m_Up) - glm::radians(90.0f)) <= glm::radians(95.0f))
		{
			m_Orientation = newm_Orientation;
		}

		// Rotates the m_Orientation left and right
		m_Orientation = glm::rotate(m_Orientation, glm::radians(-rotY), m_Up);

		// Sets mouse cursor to the middle of the screen so that it doesn't end m_Up roaming around
		glfwSetCursorPos(window, (Width / 2), (Height / 2));
	}
	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
	{
		// Unhides cursor since camera is not looking around anymore
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		// Makes sure the next time the camera looks around it doesn't jump
		m_FirstClick = true;
	}
}

void Camera::SetScrollInput(GLFWwindow* window, bool value)
{
	value ? glfwSetScrollCallback(window, Camera::Scroll_callback) : glfwSetScrollCallback(window, nullptr);
}

void Camera::Scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	GetWorkingCamera()->IncreaseSpeed(yoffset);
}
