#include "Classes/Public/Renderer.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>

#include "Classes/Public/VertexArray.h"
#include "Classes/Public/IndexBuffer.h"
#include "Classes/Public/Shader.h"
#include "Classes/Public/Camera.h"
#include "Classes/Public/Renderer.h"
#include "Classes/Public/Boids.h"
#include "Classes/Public/BoidAlgorithm.h"
#include "Cuda/kernel.h"

#include "ImGui/imgui.h"
#include "ImGui/imgui_impl_glfw.h"
#include "ImGui/imgui_impl_opengl3.h"

int main()
{
	#pragma region GLFW init
	// Initialize GLFW
	GLFWwindow* window;

	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(Camera::Width, Camera::Height, "Boids", NULL, NULL);
	if (!window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);

	glfwSwapInterval(1);

	if (glewInit() != GLEW_OK)
		return -2;

	std::cout << glGetString(GL_VERSION) << "\n";

	glEnable(GL_DEPTH_TEST);

	GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_ALPHA));
	glEnable(GL_BLEND);

#pragma endregion

	float positions[] =
	{
		// positions               // colors (RGB - blue)
		 0.0f,  0.02f,  0.0f,       0.0f, 0.0f, 1.0f,  // Top vertex
		-0.02f, -0.02f,  0.02f,     0.0f, 0.0f, 1.0f,  // Bottom-left corner
		 0.02f, -0.02f,  0.02f,     0.0f, 0.0f, 1.0f,  // Bottom-right corner
		-0.02f, -0.02f, -0.02f,     0.0f, 0.0f, 1.0f,  // Top-left corner
		 0.02f, -0.02f, -0.02f,     0.0f, 0.0f, 1.0f,  // Top-right corner
	};

	uint indices[] = {
		0, 1, 2,  // Front face
		0, 3, 1,  // Left face
		0, 4, 2,  // Right face
		0, 3, 4,  // Back face
		3, 1, 2,  // Bottom face
	};

	float BorderPos[] =
	{
		// positions           // colors
		 -1.0f, -1.0f, -1.0f,  1.0f, 0.0f, 0.0f,
		 -1.0f,  1.0f, -1.0f,  0.0f, 0.0f, 1.0f,
		  1.0f,	 1.0f, -1.0f,  0.0f, 1.0f, 1.0f,
		  1.0f,	-1.0f, -1.0f,  0.0f, 1.0f, 0.0f,

		 -1.0f, -1.0f, 1.0f,  1.0f, 0.0f, 0.0f,
		 -1.0f,  1.0f, 1.0f,  0.0f, 0.0f, 1.0f,
		  1.0f,	 1.0f, 1.0f,  0.0f, 1.0f, 1.0f,
		  1.0f,	-1.0f, 1.0f,  0.0f, 1.0f, 0.0f,
	};

	uint BorderIndices[] =
	{
		// back
		0, 1,
		1, 2,
		2, 3,
		3, 0,

		// front
		4, 5,
		5, 6,
		6, 7,
		7, 4,

		0, 4,
		1, 5,
		2, 6,
		3, 7
	};

	Position* translations = new Position[BOIDSCOUNT];
	int index = 0;
	float offset = 0.1f;
	for (int y = -10; y < 10; y += 2)
	{
		for (int x = -10; x < 10; x += 2)
		{
			translations[index].x = (float)x / 10.0f + offset;
			translations[index].z = (float)x / 10.0f + offset;
			translations[index++].y = (float)y / 10.0f + offset;
		}
	}

	Shader shader("res/shaders/Basic.shader");
	shader.Bind();

	VertexArray VA, VABorder;
	VertexBuffer VB(positions, 5 * 6 * sizeof(float)), VBborder(BorderPos, 8 * 6 * sizeof(float));
	VertexBuffer* InstanceVB = new VertexBuffer(translations, BOIDSCOUNT* sizeof(Position));
	glm::mat4* Rotations = new glm::mat4[BOIDSCOUNT];
	for (size_t i = 0; i < BOIDSCOUNT; i++)
	{
		Rotations[i] = glm::mat4(1.f);
	}
	VertexBuffer* RotateVB = new VertexBuffer(Rotations, BOIDSCOUNT*sizeof(glm::mat4));
	VertexBufferLayout VBL, VBLborder;

	VBL.Push<float>(3);
	VBL.Push<float>(3);

	VBLborder.Push<float>(3);
	VBLborder.Push<float>(3);

	VA.AddBuffer(VB, VBL, InstanceVB);

	IndexBuffer IB(indices, 5 * 3), IBborder(BorderIndices, 12 * 2);

	VA.UnBind();
	VB.UnBind();
	IB.UnBind();

	VABorder.AddBuffer(VBborder, VBLborder);

	Renderer renderer;
	Camera camera(glm::vec3(0.f, 0.f, 4.f));
	Camera::SetWorkingCamera(&camera);

	typedef std::chrono::high_resolution_clock clock;
	typedef std::chrono::duration<float, std::milli> duration;

	static clock::time_point start = clock::now();
	duration elapsed = clock::now() - start;

	
	Boid* Boids = new Boid[BOIDSCOUNT];
	Position* BoidsPositions = new Position[BOIDSCOUNT];

	srand(time(NULL));
	for (size_t i = 0; i < BOIDSCOUNT; i++)
	{
		Boids[i].speedx = 2;
		Boids[i].speedy = 2;
		Boids[i].speedz = 2;

		BoidsPositions[i].x = rand() % WIDTH;
		BoidsPositions[i].y = rand() % HEIGHT;
		BoidsPositions[i].z = rand() % DEPTH;
	}
	CudaAllocations CudaAlloc(BoidsPositions, Boids, BOIDSCOUNT);
	BoidAlgorithm BA(BoidsPositions, Boids, BOIDSCOUNT);

	const char* glsl_version = "#version 130";

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Main while loop
	while (!glfwWindowShouldClose(window))
	{
#ifdef GPU


		GetNewPositions(translations, Rotations, &CudaAlloc);

		delete InstanceVB;
		InstanceVB = new VertexBuffer(translations, BOIDSCOUNT * sizeof(Position));
		//delete RotateVB;
		//RotateVB = new VertexBuffer(Rotations, BOIDSCOUNT * sizeof(glm::mat4));

		VA.AddBuffer(VB, VBL, InstanceVB, RotateVB);


		renderer.Clear();
		
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		duration elapsed = clock::now() - start;

		ImGui::Begin("Properties");
		ImGui::Text(("FPS: " + std::to_string( (int)(1000 / elapsed.count()) )).c_str());
		ImGui::SliderFloat("Avoid Factor", &CudaAlloc.AvoidFactor, 0.0, 2.0);
		ImGui::SliderFloat("Matching Factor", &CudaAlloc.MatchingFactor, 0.0, 0.5);
		ImGui::SliderFloat("Centering Factor", &CudaAlloc.CenteringFactor, 0.0, 0.1);
		ImGui::End();

#endif // GPU
#ifndef GPU
		BA.GetNewPositions(translations);

		delete InstanceVB;
		InstanceVB = new VertexBuffer(translations, BOIDSCOUNT * sizeof(Position));
		//delete RotateVB;
		//RotateVB = new VertexBuffer(Rotations, BOIDSCOUNT * sizeof(glm::mat4));

		VA.AddBuffer(VB, VBL, InstanceVB, RotateVB);


		renderer.Clear();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		duration elapsed = clock::now() - start;

		ImGui::Begin("Properties");
		ImGui::Text(("FPS: " + std::to_string((int)(1000 / elapsed.count()))).c_str());
		ImGui::SliderFloat("Avoid Factor", &BA.AvoidFactor, 0.0, 2.0);
		ImGui::SliderFloat("Matching Factor", &BA.MatchingFactor, 0.0, 0.5);
		ImGui::SliderFloat("Centering Factor", &BA.CenteringFactor, 0.0, 0.1);
		ImGui::End();

#endif // !GPU

		shader.Bind();

		Camera::GetWorkingCamera()->Inputs(window);
		Camera::GetWorkingCamera()->Matrix(45.0f, 0.1f, 100.0f, shader, "u_camMatrix");

		start = clock::now();

		VA.Bind();
		IB.Bind();
		renderer.Draw(VA, IB, shader, BOIDSCOUNT);

		VABorder.Bind();
		IBborder.Bind();
		renderer.DrawLines(VABorder, IBborder, shader);


		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		// Swap the back buffer with the front buffer
		glfwSwapBuffers(window);
		
		// Take care of all GLFW events
		glfwPollEvents();

	}

	//DeleteCuda(&CudaAlloc);

	// Delete window before ending the program
	glfwDestroyWindow(window);

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// Terminate GLFW before ending the program
	glfwTerminate();
	return 0;
}
