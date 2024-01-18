#pragma once
#include "Typedef.h"

class VertexBuffer
{
private:
	uint m_Renderer_ID;
public:
	VertexBuffer(const void* data, uint size);
	~VertexBuffer();

	void Bind() const;
	void UnBind() const;

};

