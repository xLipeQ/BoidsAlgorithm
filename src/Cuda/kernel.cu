
#include "kernel.h"
#include <stdio.h>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/matrix_transform.hpp>

#define CHECK_CUDA(call)                                            \
{                                                                   \
const cudaError_t error = call;                                     \
if (error != cudaSuccess)                                           \
{                                                                   \
printf("ERROR:: File: %s, Line: %d, ", __FILE__, __LINE__);         \
printf("code: %d, reason: %s\n", error, cudaGetErrorString(error)); \
exit(1);                                                            \
}                                                                   \
}

CudaAllocations::CudaAllocations(Position* m_positions, Boid* m_boids, uint m_BoidsCount)
{
    BoidsCount = m_BoidsCount;

    CHECK_CUDA(cudaSetDevice(0));

    // Allocate GPU buffers for Positions
    CHECK_CUDA(cudaMalloc((void**)&Positions, BoidsCount * sizeof(Position)));

    // Allocate GPU buffers for Positions
    CHECK_CUDA(cudaMalloc((void**)&OpenGLPositions, BoidsCount * sizeof(Position)));

    // Allocate GPU buffers for Positions
    CHECK_CUDA(cudaMalloc((void**)&Boids, BoidsCount * sizeof(Boid)));

    // Allocate rotation Buffer
    CHECK_CUDA(cudaMalloc((void**)&Rotations, BoidsCount * sizeof(glm::mat4)));

    for (int i = 0; i < BoxAmount; i++)
    {
        CHECK_CUDA(cudaMalloc((void**)&Boxes[i].IndexesOfBird, BoidsCount * sizeof(int)));
        CHECK_CUDA(cudaMemset(Boxes[i].IndexesOfBird, 0, BoidsCount * sizeof(int)));
    }
    
    // Copy input vectors from host memory to GPU buffers.
    CHECK_CUDA(cudaMemcpy(Positions, m_positions, BoidsCount * sizeof(Position), cudaMemcpyHostToDevice));

    // copy speeds for each boids
    CHECK_CUDA(cudaMemcpy(Boids, m_boids, BoidsCount * sizeof(Boid), cudaMemcpyHostToDevice));
}

CudaAllocations::~CudaAllocations()
{
    cudaFree(Positions);
    cudaFree(Boids);
    cudaFree(OpenGLPositions);
    for (int i = 0; i < BoxAmount; i++)
        cudaFree(Boxes[i].IndexesOfBird);
    cudaFree(Boxes);
}

void GetNewPositions(Position* m_Pos, glm::mat4* rotations, CudaAllocations* CudaAll)
{
    if (!CudaAll->Initialized)
    {
        Box* Boxes;
        CHECK_CUDA(CudaAll->error = cudaMalloc((void**)&Boxes, CudaAll->BoxAmount * sizeof(Box)));
        
        CudaAll->CudaBoxes = Boxes;
        
        CHECK_CUDA(CudaAll->error = cudaMemcpy(CudaAll->CudaBoxes, CudaAll->Boxes, 
            sizeof(Box) * CudaAll->BoxAmount, cudaMemcpyHostToDevice));

        CudaAll->Initialized = true;
    } 

    int ThreadsPerBlock = 128;
    int blocksPerGrid = CudaAll->BoidsCount / ThreadsPerBlock + 1;
    CalculateNewFrame << <blocksPerGrid, ThreadsPerBlock >> > (CudaAll->Positions, CudaAll->Boids, 
        CudaAll->OpenGLPositions, CudaAll->BoidsCount, CudaAll->AvoidFactor, CudaAll->MatchingFactor,
        CudaAll->CenteringFactor, CudaAll->CudaBoxes, CudaAll->Rotations);

    // Check for any errors launching the kernel
    CHECK_CUDA(cudaGetLastError());

    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    CHECK_CUDA(cudaDeviceSynchronize());

#ifdef BOXES

    ClearBoxes << <1, CudaAll->BoxAmount >> > (CudaAll->CudaBoxes, CudaAll->BoxAmount);

    // Check for any errors launching the kernel
    CHECK_CUDA(cudaGetLastError());

    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    CHECK_CUDA(cudaDeviceSynchronize());

    AddBoxes << <blocksPerGrid, ThreadsPerBlock >> > (CudaAll->CudaBoxes, CudaAll->BoxAmount,
        CudaAll->Positions, CudaAll->BoidsCount);

    // Check for any errors launching the kernel
    CHECK_CUDA(cudaGetLastError());

    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    CHECK_CUDA(cudaDeviceSynchronize());

#endif // BOXES


    // Copy output vector from GPU buffer to host memory.
    CHECK_CUDA(cudaMemcpy(m_Pos, CudaAll->OpenGLPositions, CudaAll->BoidsCount * sizeof(Position), cudaMemcpyDeviceToHost));

    CHECK_CUDA(cudaMemcpy(rotations, CudaAll->Rotations, 
        CudaAll->BoidsCount * sizeof(glm::mat4), cudaMemcpyDeviceToHost));

}

__global__ void CalculateNewFrame(Position* m_Pos, Boid* m_Boids, Position* OpenGLPos, uint m_BoidCount,
    float AvoidFactor, float MatchingFactor, float CenteringFactor, Box* Boxes, glm::mat4* rotations)
{
    #pragma region Variables

    int index = threadIdx.x + blockIdx.x * blockDim.x;
    if (index >= m_BoidCount)
        return;
    float close_dx = 0;
    float close_dy = 0;
    float close_dz = 0;

    float xvel_avg = 0;
    float yvel_avg = 0;
    float zvel_avg = 0;

    float xpos_avg = 0;
    float ypos_avg = 0;
    float zpos_avg = 0;

    int neighbours = 0;

    #pragma endregion

    //Boids Algorithm - see https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html
    #pragma region Boids Alogirithm loop 
    int size = 0;
    int* CloseBoxes = GetSoroundingBoxes(&m_Pos[index], &size);

#ifdef BOXES
    for (int i = 0; i < size; i++)
    {
        int BoxIndex = CloseBoxes[i];
        for (int j = 0; j < Boxes[BoxIndex].AmountOfBirds; j++)
        {
            int BirdIndex = Boxes[BoxIndex].IndexesOfBird[j];

            if (BirdIndex == index)
                continue;

            if (BirdIndex >= m_BoidCount)
            {
                continue;
            }
#endif // BOXES

    
#ifndef BOXES
    for (int i = 0; i < m_BoidCount; i++)
    {
        int BirdIndex = i;
#endif // !BOXES

            // within protection range
            // Separation
            if (InRange(m_Pos[BirdIndex], m_Pos[index], PROTECTIONRANGE))
            {
                close_dx += m_Pos[index].x - m_Pos[BirdIndex].x;
                close_dy += m_Pos[index].y - m_Pos[BirdIndex].y;
                close_dz += m_Pos[index].z - m_Pos[BirdIndex].z;
            }

            // within visual range
            if (InRange(m_Pos[BirdIndex], m_Pos[index], VISIBLERANGE))
            {
                // alignment
                xvel_avg += m_Boids[BirdIndex].speedx;
                yvel_avg += m_Boids[BirdIndex].speedy;
                zvel_avg += m_Boids[BirdIndex].speedz;
                neighbours++;

                // cohession
                xpos_avg += m_Pos[BirdIndex].x;
                ypos_avg += m_Pos[BirdIndex].y;
                zpos_avg += m_Pos[BirdIndex].z;
            }
#ifdef BOXES
        }
#endif // BOXES
      
    }
    if (neighbours > 0)
    {
        xvel_avg /= neighbours;
        yvel_avg /= neighbours;
        zvel_avg /= neighbours;

        xpos_avg /= neighbours;
        ypos_avg /= neighbours;
        zpos_avg /= neighbours;
    }

    free(CloseBoxes);

#pragma endregion

    #pragma region SetSpeed

        // add speed x
        m_Boids[index].speedx += close_dx * AvoidFactor +
            (xvel_avg - m_Boids[index].speedx) * MatchingFactor +
            (xpos_avg - m_Pos[index].x) * CenteringFactor;

        // add speed y
        m_Boids[index].speedy += close_dy * AvoidFactor +
            (yvel_avg - m_Boids[index].speedy) * MatchingFactor +
            (ypos_avg - m_Pos[index].y) * CenteringFactor;

        // add speed z
        m_Boids[index].speedz += close_dz * AvoidFactor +
            (zvel_avg - m_Boids[index].speedz) * MatchingFactor +
            (zpos_avg - m_Pos[index].z) * CenteringFactor;

        // get velocity
        float speed = GetSpeed(m_Boids[index]);

        if (speed < MINSPEED)
        {
            m_Boids[index].speedx = (m_Boids[index].speedx / speed) * MINSPEED;
            m_Boids[index].speedy = (m_Boids[index].speedy / speed) * MINSPEED;
            m_Boids[index].speedz = (m_Boids[index].speedz / speed) * MINSPEED;
        }
        if (speed > MAXSPEED)
        {
            m_Boids[index].speedx = (m_Boids[index].speedx / speed) * MAXSPEED;
            m_Boids[index].speedy = (m_Boids[index].speedy / speed) * MAXSPEED;
            m_Boids[index].speedz = (m_Boids[index].speedz / speed) * MAXSPEED;
        }


        if (m_Pos[index].x < MARGIN)
            m_Boids[index].speedx += TURNFACTOR;

        if (m_Pos[index].x > WIDTH - MARGIN)
            m_Boids[index].speedx -= TURNFACTOR;

        if (m_Pos[index].y > HEIGHT - MARGIN)
            m_Boids[index].speedy -= TURNFACTOR;

        if (m_Pos[index].y < MARGIN)
            m_Boids[index].speedy += TURNFACTOR;

        if (m_Pos[index].z > DEPTH - MARGIN)
            m_Boids[index].speedz -= TURNFACTOR;

        if (m_Pos[index].z < MARGIN)
            m_Boids[index].speedz += TURNFACTOR;

    #pragma endregion

    __syncthreads();

    m_Pos[index].x += m_Boids[index].speedx;
    m_Pos[index].y += m_Boids[index].speedy;
    m_Pos[index].z += m_Boids[index].speedz;

    if (m_Pos[index].x >= WIDTH)
        m_Pos[index].x = WIDTH - 1;

    else if (m_Pos[index].x <= 0)
        m_Pos[index].x = 1;

    if (m_Pos[index].y >= HEIGHT)
        m_Pos[index].y = HEIGHT - 1;

    else if (m_Pos[index].y <= 0)
        m_Pos[index].y = 1;

    if (m_Pos[index].z >= DEPTH)
        m_Pos[index].z = DEPTH - 1;

    else if (m_Pos[index].z <= 0)
        m_Pos[index].z = 1;

    OpenGLPos[index].x = 2 * m_Pos[index].x / WIDTH - 1.0;
    OpenGLPos[index].y = 2 * m_Pos[index].y / HEIGHT - 1.0;
    OpenGLPos[index].z = 2 * m_Pos[index].z / DEPTH - 1.0;

    rotations[index] = GetModelMatrix(m_Pos[index], m_Boids[index]);
}

__device__ bool InRange(Position& Pos1, Position& Pos2, float Range)
{
    return sqrtf( (Pos1.x - Pos2.x) * (Pos1.x - Pos2.x) + 
                  (Pos1.y - Pos2.y) * (Pos1.y - Pos2.y) + 
                  (Pos1.z - Pos2.z) * (Pos1.z - Pos2.z) ) < Range;
}

__device__ float GetSpeed(Boid& Boid1)
{
    return sqrtf(Boid1.speedx * Boid1.speedx + 
                 Boid1.speedy * Boid1.speedy + 
                 Boid1.speedz * Boid1.speedz);
}

__device__ int* GetSoroundingBoxes(Position* MyPosition, int* amount)
{
    int* CloseBoxes = (int*)malloc(27 * sizeof(int));

    (*amount) = 0;
    short BoxAmountInZ = DEPTH / EDGELENGTH;
    short BoxAmountInY = HEIGHT / EDGELENGTH;
    short BoxAmountInX = WIDTH / EDGELENGTH;
    short my_Index = (int)MyPosition->z / EDGELENGTH * BoxAmountInX * BoxAmountInY
        + (int)MyPosition->y / EDGELENGTH * BoxAmountInX
        + (int)MyPosition->x / EDGELENGTH;

    // mine
    CloseBoxes[*amount] = my_Index;
    (*amount)++;

    // bool if next box in X,Y,Z direction is within radius
    bool NextX = false, PrevX = false;
    bool NextY = false, PrevY = false;
    bool NextZ = false, PrevZ = false;

    // Next in Z is in our visual range
    // and we are not in back wall (front in opengl, but it does not matter here)
    if ((short)((MyPosition->z + VISIBLERANGE) / EDGELENGTH) > (short)(MyPosition->z / EDGELENGTH) &&
        my_Index + BoxAmountInX*BoxAmountInY < BoxAmountInX*BoxAmountInY*BoxAmountInZ)
    {
        NextZ = true;
        CloseBoxes[*amount] = my_Index + BoxAmountInX * BoxAmountInY;
        (*amount)++;
    }
    // Prev in Z is in our visual range
    if ((short)((MyPosition->z - VISIBLERANGE) / EDGELENGTH) < (short)(MyPosition->z / EDGELENGTH) &&
        my_Index - BoxAmountInX * BoxAmountInY >= 0)
    {
        PrevZ = true;
        CloseBoxes[*amount] = my_Index - BoxAmountInX * BoxAmountInY;
        (*amount)++;
    }
    
    // Next in Y is in our visual range
    if ((short)((MyPosition->y + VISIBLERANGE) / EDGELENGTH) > (short)(MyPosition->y / EDGELENGTH) &&
        my_Index + BoxAmountInX < BoxAmountInX * BoxAmountInY * BoxAmountInZ)
    {
        NextY = true;
        CloseBoxes[*amount] = my_Index + BoxAmountInX;
        (*amount)++;
    }

    // Prev in Y is in our visual range
    if ((short)((MyPosition->y - VISIBLERANGE) / EDGELENGTH) < (short)(MyPosition->y / EDGELENGTH) &&
        my_Index - BoxAmountInX >= 0)
    {
        PrevY = true;
        CloseBoxes[*amount] = my_Index - BoxAmountInX;
        (*amount)++;
    }

    // Next in X is in our visual range
    if ((short)((MyPosition->x + VISIBLERANGE) / EDGELENGTH) > (short)(MyPosition->x / EDGELENGTH) &&
        my_Index + 1 < BoxAmountInX * BoxAmountInY * BoxAmountInZ)
    {
        NextX = true;
        CloseBoxes[*amount] = my_Index + 1;
        (*amount)++;
    }

    // Prev in X is in our visual range
    if ((short)((MyPosition->x - VISIBLERANGE) / EDGELENGTH) < (short)(MyPosition->x / EDGELENGTH) &&
        my_Index - 1 >= 0)
    {
        PrevX = true;
        CloseBoxes[*amount] = my_Index - 1;
        (*amount)++;
    }

    return CloseBoxes;
}

__device__ short GetIndexOfMyBox(Position* MyPosition)
{
    short BoxAmountInZ = DEPTH / EDGELENGTH;
    short BoxAmountInY = HEIGHT / EDGELENGTH;
    short BoxAmountInX = WIDTH / EDGELENGTH;
    if ((int)MyPosition->z >= DEPTH || (int)MyPosition->y >= HEIGHT || (int)MyPosition->x >= WIDTH)
        printf("tu");
    short my_Index = (int)MyPosition->z / EDGELENGTH * BoxAmountInX * BoxAmountInY
        + (int)MyPosition->y / EDGELENGTH * BoxAmountInX
        + (int)MyPosition->x / EDGELENGTH;

    return my_Index;
}

__device__ glm::mat4 GetModelMatrix(Position& pos, Boid& boid)
{
    glm::vec3 position(pos.x / WIDTH, pos.y / HEIGHT, pos.z / DEPTH);
    glm::vec3 speed = glm::normalize(glm::vec3(boid.speedx, boid.speedy, boid.speedz));

    // Create a translation matrix based on the scaled position
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);

    // Calculate the rotation matrix directly based on the normalized speed vector
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f); // Up vector for reference
    glm::vec3 right = glm::cross(up, speed);
    glm::vec3 newUp = glm::cross(speed, right);

    glm::mat4 rotation = glm::mat4(
        glm::vec4(right, 0.0f),
        glm::vec4(newUp, 0.0f),
        glm::vec4(-speed, 0.0f),
        glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
    );

    // Combine the translation and rotation matrices to get the final model matrix
    glm::mat4 modelMatrix = translation * rotation;

    return modelMatrix;
}

__global__ void ClearBoxes(Box* Boxes, uint BoxAmount)
{
    // clear Box tab to create new one there
    // memset(Boxes[threadIdx.x].IndexesOfBird, 0, BOIDSCOUNT * sizeof(int));
    Boxes[threadIdx.x].AmountOfBirds = 0;
}

__global__ void AddBoxes(Box* Boxes, uint BoxAmount, Position* Positions, uint BoidCount)
{
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index >= BoidCount)
        return;

    short myBox = GetIndexOfMyBox(&Positions[index]);
    int MyPlace = atomicAdd(&Boxes[myBox].AmountOfBirds, 1);
    Boxes[myBox].IndexesOfBird[MyPlace] = index;

}