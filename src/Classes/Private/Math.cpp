#include "../Public/Math.h"
#include <glm/trigonometric.hpp>
# define M_PI           3.14159265358979323846
glm::mat4 ViewMatrix(glm::vec3 const& CameraPos, glm::vec3 const& center, glm::vec3 const& up)
{
	glm::mat4 Result(1.f);

	glm::vec3 D(glm::normalize(center - CameraPos));
	glm::vec3 R(glm::normalize(glm::cross(D, up)));
	glm::vec3 U(glm::cross(R, D));

	Result[0][0] = R.x;
	Result[1][0] = R.y;
	Result[2][0] = R.z;
	Result[0][1] = U.x;
	Result[1][1] = U.y;
	Result[2][1] = U.z;
	Result[0][2] = -D.x;
	Result[1][2] = -D.y;
	Result[2][2] = -D.z;
	Result[3][0] = -glm::dot(R, CameraPos);
	Result[3][1] = -glm::dot(U, CameraPos);
	Result[3][2] = glm::dot(D, CameraPos);

	return Result;
}

glm::mat4 ProjectionMatrix(float Fov, float Aspect, float Near, float Far)
{
	glm::mat4 Result(0.f);

	Result[0][0] = 1 / (std::tan(Fov / 2.f) * Aspect);
	Result[1][1] = 1 / std::tan(Fov / 2.f);
	Result[2][2] = -(Far + Near) / (Far - Near);
	Result[2][3] = -1.f;
	Result[3][2] = -2.f * Far * Near / (Far - Near);

	return Result;
}

glm::mat4 RotateX(float angle)
{
	glm::mat4 matrix(0.0f);
	matrix[0][0] = 1;
	matrix[3][3] = 1;

	matrix[1][1] = std::cos(angle);
	matrix[2][2] = std::cos(angle);
	matrix[2][1] = -std::sin(angle);
	matrix[1][2] = std::sin(angle);
	return matrix;
}

glm::mat4 RotateY(float angle)
{
	glm::mat4 matrix(0.0f);

	matrix[1][1] = 1;
	matrix[3][3] = 1;
	matrix[0][0] = std::cos(angle);
	matrix[2][2] = std::cos(angle);
	matrix[0][2] = -std::sin(angle);
	matrix[2][0] = std::sin(angle);

	return matrix;
}

glm::mat4 RotateZ(float angle)
{
	glm::mat4 matrix(0.0f);
	matrix[0][0] = std::cos(angle);
	matrix[1][1] = std::cos(angle);
	matrix[1][0] = -std::sin(angle);
	matrix[0][1] = std::sin(angle);
	matrix[2][2] = matrix[3][3] = 1;
	return matrix;
}

glm::mat4 ScaleObject(glm::vec3 const& v)
{
	glm::mat4 Result(1.f);
	Result[0][0] = v.x;
	Result[1][1] = v.y;
	Result[2][2] = v.z;
	return Result;
}

glm::mat4 TransateObject(glm::vec3 const& v)
{
	glm::mat4 Result(1.f);
	Result[3][0] = v.x;
	Result[3][1] = v.y;
	Result[3][2] = v.z;
	return Result;
}

glm::mat4 RotateObject(glm::vec3 const& rotation)
{
	glm::mat4 result(1.f);
	return result = RotateX(rotation.x) * RotateY(rotation.y) * RotateZ(rotation.z);
}

// radians
float CalcAngleDegrees(float angle)
{
	if (angle > 0)
		return angle - (int)angle / 360 * 360;

	return angle + (int)-angle / 360 * 360;
}

float CalcAngleRadians(float angle)
{
	if (angle >= 2 * M_PI || angle <= -2 * M_PI)
		return angle;

	return angle;
}