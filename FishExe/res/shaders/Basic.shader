#shader vertex
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 colorV;
layout(location = 2) in vec3 aOffset;
layout(location = 3) in vec4 col1;
layout(location = 4) in vec4 col2;
layout(location = 5) in vec4 col3;
layout(location = 6) in vec4 col4;


out vec4 v_Color;
// Imports the camera matrix from the main function
uniform mat4 u_camMatrix;

void main()
{
	if(col1[0] == 0)
		gl_Position = u_camMatrix * vec4(position + aOffset, 1.0);
	else
		gl_Position = u_camMatrix * mat4(col1, col2, col3, col4) *
			vec4(position + aOffset, 1.0);
	v_Color = vec4( (aOffset + 1.0f ) / 2.0f, 1.0);
};

#shader fragment
#version 330 core
layout(location = 0) out vec4 color;
	
in vec4 v_Color;

void main()
{
	color = v_Color;
};