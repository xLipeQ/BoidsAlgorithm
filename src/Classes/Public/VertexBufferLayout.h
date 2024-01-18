#pragma once
#include <vector>
#include "Typedef.h"
#include <GL/glew.h>

#define TEMPASSERT(x) if(!x) __debugbreak()

struct VertexElement
{
	uint type;
	uint count;
	uchar normalized;

	static uint GetSizeOfType(uint Type)
	{
		switch (Type)
		{
		case GL_FLOAT: 
			return 4;
		case GL_UNSIGNED_INT:
			return 4;
		case GL_UNSIGNED_BYTE: 
			return 1;
		default:
			TEMPASSERT(false);
		}
	}
};

class VertexBufferLayout
{
private:
	std::vector<VertexElement> m_Elements;
	uint m_Stride = 0;

public:
	VertexBufferLayout() {};

	template<typename T>
	void Push(uint count)
	{
		//static_assert(false);
	}

	template<>
	void Push<float>(uint count)
	{
		m_Elements.push_back(VertexElement{ GL_FLOAT, count, GL_FALSE });
		m_Stride += count * VertexElement::GetSizeOfType(GL_FLOAT);
	}

	template<>
	void Push<uint>(uint count)
	{
		m_Elements.push_back(VertexElement{ GL_UNSIGNED_INT, count, GL_FALSE });
		m_Stride += count * VertexElement::GetSizeOfType(GL_UNSIGNED_INT);
	}

	template<>
	void Push<uchar>(uint count)
	{
		m_Elements.push_back(VertexElement{ GL_UNSIGNED_BYTE, count, GL_TRUE });
		m_Stride += count * VertexElement::GetSizeOfType(GL_UNSIGNED_BYTE);
	}
	
	inline std::vector<VertexElement> GetElements() const { return m_Elements; };

	inline uint GetStride() const { return m_Stride; };

};
