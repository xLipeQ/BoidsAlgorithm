#include "../Public/Renderer.h"
#include "../Public/IndexBuffer.h"
#include "../Public/VertexArray.h"
#include "../Public/VertexBufferLayout.h"

#include <glm/gtc/type_ptr.hpp>
bool Renderer::Stop = false;

void GLClearError()
{
	while (glGetError() != GL_NO_ERROR)
	{
		if (!Renderer::Stop)
			continue;
		else
			break;
	}
}

bool GLLogCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		if (Renderer::Stop)
			return true;
		std::cout << "[ERROR: ]" << error << " " <<
			function << " " << file << " " << line << std::endl;
		return false;
	}
	return true;
}

void Renderer::Clear() const
{
	// Clean the back buffer and assign the new color to it
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

	GLCall(glClearColor(0.07f, 0.13f, 0.17f, 1.0f));
}

void Renderer::Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader, uint instances) const
{
	shader.Bind();
	va.Bind();
	ib.Bind();

	if (instances <= 1)
	{
		GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr));
	}
	else
	{
		GLCall(glDrawElementsInstanced(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr, instances));
	}
}

void Renderer::DrawLines(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const
{
	shader.Bind();
	va.Bind();
	ib.Bind();

	GLCall(glDrawElements(GL_LINES, ib.GetCount(), GL_UNSIGNED_INT, nullptr));
	
}
