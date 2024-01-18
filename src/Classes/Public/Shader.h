#pragma once

#include <iostream>
#include "Typedef.h"
#include <unordered_map>
#include "glm/glm.hpp"

struct ShaderSource
{
	std::string VertexSource;
	std::string FragmentSource;
};

class Shader
{
private:

	uint m_Renderer_Id;
	std::string m_Filepath;
	std::unordered_map<std::string, int> m_LocationCache;

public:
	Shader(const std::string& filepath);
	~Shader();

	void Bind() const;
	void UnBind() const;


	// Set uniforms
	void SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3);
	void SetUniform3f(const std::string& name, float v0, float v1, float v2);
	void SetUniform3f(const std::string& name, glm::vec3 vec);

	void SetUniformMatrix4fv(const std::string& name, const glm::f32* pointer);
	void SetUniform1i(const std::string& name, int value);

private:
	uint CompileShader(uint type, const std::string& source);
	uint CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	ShaderSource ParseShader(const std::string& file);
	int GetUniformLocation(const std::string& name);
};

