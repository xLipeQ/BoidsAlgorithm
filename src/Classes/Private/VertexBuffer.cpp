#include "../Public/VertexBuffer.h"
#include "../Public/Renderer.h"

VertexBuffer::VertexBuffer(const void* data, uint size)
{
	GLCall(glGenBuffers(1, &m_Renderer_ID));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_Renderer_ID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
	UnBind();
}

VertexBuffer::~VertexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_Renderer_ID));
}

void VertexBuffer::Bind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_Renderer_ID));
}

void VertexBuffer::UnBind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}
