#include "../Public/IndexBuffer.h"
#include "../Public/Renderer.h"

IndexBuffer::IndexBuffer(const uint* data, uint count) : m_Count(count)
{
	ASSERT(sizeof(uint) == sizeof(GLuint));

	GLCall(glGenBuffers(1, &m_Renderer_ID));
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_Renderer_ID));
	GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(uint), data, GL_STATIC_DRAW));
}

IndexBuffer::~IndexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_Renderer_ID));
}

void IndexBuffer::Bind() const
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_Renderer_ID));
}

void IndexBuffer::UnBind() const
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
}
