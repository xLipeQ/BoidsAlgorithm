#pragma once

#include "Typedef.h"

class IndexBuffer
{
private:
	uint m_Renderer_ID;
	uint m_Count;
public:
	IndexBuffer(const uint* data, uint count);
	~IndexBuffer();

	void Bind() const;
	void UnBind() const;

	uint GetCount() const { return m_Count; };
};

