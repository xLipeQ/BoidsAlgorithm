#pragma once

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "../Classes/Public/Boids.h"
#include "../Classes/Public/Typedef.h"
#include <glm/ext/matrix_float4x4.hpp>


class CudaAllocations
{
public:
	Position* Positions;
	Position* OpenGLPositions;
	Boid* Boids;
	glm::mat4* Rotations;
	uint BoidsCount;
	cudaError_t error;

	float AvoidFactor = 0.05;
	float MatchingFactor = 0.05;
	float CenteringFactor = 0.0003;

	// rows (x changes), columns (y changes) depth (z changes)
	Box Boxes[WIDTH / EDGELENGTH * DEPTH / EDGELENGTH * HEIGHT / EDGELENGTH];
	Box* CudaBoxes;

	const int BoxAmount = WIDTH / EDGELENGTH * DEPTH / EDGELENGTH * HEIGHT / EDGELENGTH;

	bool Initialized = false;
public:
	CudaAllocations(Position* m_pos, Boid* m_boids, uint m_boidsCount);
	~CudaAllocations();
};

void GetNewPositions(Position* m_Pos, glm::mat4* rotations, CudaAllocations* CudaAll);
__global__ void CalculateNewFrame(Position* m_Pos, Boid* m_Boids, Position* OPenglPos, uint m_BoidCount,
	float AvoidFactor, float MatchingFactor, float CenteringFactor, Box* Boxes, glm::mat4* rotations);

__device__ bool InRange(Position& Pos1, Position& Pos2, float Range);
__device__ float GetSpeed(Boid& Boid1);
__device__ int* GetSoroundingBoxes(Position* MyPosition, int* amount);
__device__ short GetIndexOfMyBox(Position* Position);
__device__ glm::mat4 GetModelMatrix(Position& pos, Boid& boid);


// sets amount of boxes to 0
__global__ void ClearBoxes(Box* Boxes, uint BoxAmount);
__global__ void AddBoxes(Box* Boxes, uint BoxAmount, Position* Positions, uint BoidCount);
