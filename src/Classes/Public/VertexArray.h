#pragma once
#include "VertexBuffer.h"
#include "VertexBufferLayout.h"

class VertexArray
{
private:

	uint m_Renderer_Id;

public:
	VertexArray();
	~VertexArray();

	void AddBuffer(const VertexBuffer& vb, const VertexBufferLayout& layouot, 
		const VertexBuffer* instances = nullptr, const VertexBuffer* rotations = nullptr);
	void Bind() const;
	void UnBind() const;
};