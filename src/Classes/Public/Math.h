#pragma once
#ifndef _MATH_
#define _MATH_

#include <glm/ext/matrix_float4x4.hpp>
#include <iostream>

// glm::mat4[1] - first column


glm::mat4 ViewMatrix(glm::vec3 const& CameraPos, glm::vec3 const& center, glm::vec3 const& up);

glm::mat4 ProjectionMatrix(float Fov, float Aspect, float Near, float Far);

glm::mat4 RotateX(float angle);

glm::mat4 RotateY(float angle);

glm::mat4 RotateZ(float angle);

glm::mat4 ScaleObject(glm::vec3 const& v);

glm::mat4 TransateObject(glm::vec3 const& v);

glm::mat4 RotateObject(glm::vec3 const& rotation);

float CalcAngleDegrees(float angle);

float CalcAngleRadians(float angle);

#endif // ! _MATH
