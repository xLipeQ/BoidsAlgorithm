#include "../Public/VertexArray.h"
#include "../Public/Renderer.h"

VertexArray::VertexArray()
{
	GLCall(glGenVertexArrays(1, &m_Renderer_Id));
}

VertexArray::~VertexArray()
{
	GLCall(glDeleteVertexArrays(1, &m_Renderer_Id));
}

void VertexArray::AddBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout, 
	const VertexBuffer* instances, const VertexBuffer* rotations)
{
	Bind();
	vb.Bind();
	const auto& elements = layout.GetElements();
	uint offset = 0;
	uint i = 0;
	for (; i < elements.size(); i++)
	{
		const auto& element = elements[i];
		GLCall(glEnableVertexAttribArray(i));
		GLCall(glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(),
			(const void*)offset));
		offset += element.count * VertexElement::GetSizeOfType(element.type);
	}
	// if we want to draw multiple instances
	if (instances == nullptr)
		return;
	// also set instance data
	GLCall(glEnableVertexAttribArray(i));
	instances->Bind();
	GLCall(glVertexAttribPointer(i, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0));
	instances->UnBind();
	glVertexAttribDivisor(i, 1); // tell OpenGL this is an instanced vertex attribute.

	if (rotations == nullptr)
		return;

	i++;
	// also set instance data
	GLCall(glEnableVertexAttribArray(i));
	GLCall(glEnableVertexAttribArray(i + 1));
	GLCall(glEnableVertexAttribArray(i + 2));
	GLCall(glEnableVertexAttribArray(i + 3));

	rotations->Bind();
	GLCall(glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, 64, (void*)0));
	GLCall(glVertexAttribPointer(i + 1, 4, GL_FLOAT, GL_FALSE, 64, (void*)16));
	GLCall(glVertexAttribPointer(i + 2, 4, GL_FLOAT, GL_FALSE, 64, (void*)32));
	GLCall(glVertexAttribPointer(i + 3, 4, GL_FLOAT, GL_FALSE, 64, (void*)48));
	rotations->UnBind();
	glVertexAttribDivisor(i, 1); // tell OpenGL this is an instanced vertex attribute.
}

void VertexArray::Bind() const
{
	GLCall(glBindVertexArray(m_Renderer_Id));
}

void VertexArray::UnBind() const
{
	GLCall(glBindVertexArray(0));
}
