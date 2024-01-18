#include "../Public/BoidAlgorithm.h"
#include <cstring>
#include <cmath>

BoidAlgorithm::BoidAlgorithm(Position* m_pos, Boid* m_boids, uint m_boidsCount)
{
	BoidsCount = m_boidsCount;

	BoidPositions = new Position[BoidsCount];
	std::memcpy(BoidPositions, m_pos, BoidsCount * sizeof(Position));

	OpenGLPositions = new Position[BoidsCount];

	BoidSpeeds = new Boid[BoidsCount];
	std::memcpy(BoidSpeeds, m_boids, BoidsCount * sizeof(Boid));

    for (int i = 0; i < BoxAmount; i++)
    {
        Boxes[i].IndexesOfBird = new int[BoidsCount];
    }
}

BoidAlgorithm::~BoidAlgorithm()
{
	delete[] BoidPositions;
	delete[] OpenGLPositions;
	delete[] BoidSpeeds;
    for (int i = 0; i < BoxAmount; i++)
    {
        delete[] Boxes[i].IndexesOfBird;
    }
}

void BoidAlgorithm::GetNewPositions(Position* DestPositions) 
{
    #pragma region Variables

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
    for (int index = 0; index < BoidsCount; index++)
    {
        close_dx = close_dy = close_dz = xvel_avg = yvel_avg = zvel_avg = xpos_avg = ypos_avg = zpos_avg = neighbours = 0;
        int size = 0;
        int* CloseBoxes = GetSoroundingBoxes(&this->BoidPositions[index], &size);

        for (int i = 0; i < size; i++)
        {
            int BoxIndex = CloseBoxes[i];
            for (int j = 0; j < Boxes[BoxIndex].AmountOfBirds; j++)
            {
                int BirdIndex = Boxes[BoxIndex].IndexesOfBird[j];

                if (BirdIndex == index)
                    continue;

                if (BirdIndex >= BoidsCount)
                {
                    continue;
                }

                // within protection range
                // Separation
                if (InRange(this->BoidPositions[BirdIndex], this->BoidPositions[index], PROTECTIONRANGE))
                {
                    close_dx += this->BoidPositions[index].x - this->BoidPositions[BirdIndex].x;
                    close_dy += this->BoidPositions[index].y - this->BoidPositions[BirdIndex].y;
                    close_dz += this->BoidPositions[index].z - this->BoidPositions[BirdIndex].z;
                }

                // within visual range
                if (InRange(this->BoidPositions[BirdIndex], this->BoidPositions[index], VISIBLERANGE))
                {
                    // alignment
                    xvel_avg += this->BoidSpeeds[BirdIndex].speedx;
                    yvel_avg += this->BoidSpeeds[BirdIndex].speedy;
                    zvel_avg += this->BoidSpeeds[BirdIndex].speedz;
                    neighbours++;

                    // cohession
                    xpos_avg += this->BoidPositions[BirdIndex].x;
                    ypos_avg += this->BoidPositions[BirdIndex].y;
                    zpos_avg += this->BoidPositions[BirdIndex].z;
                }
            }
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

#pragma region SetSpeed

        // add speed x
        this->BoidSpeeds[index].speedx += close_dx * AvoidFactor +
            (xvel_avg - this->BoidSpeeds[index].speedx) * MatchingFactor +
            (xpos_avg - this->BoidPositions[index].x) * CenteringFactor;

        // add speed y
        this->BoidSpeeds[index].speedy += close_dy * AvoidFactor +
            (yvel_avg - this->BoidSpeeds[index].speedy) * MatchingFactor +
            (ypos_avg - this->BoidPositions[index].y) * CenteringFactor;

        // add speed z
        this->BoidSpeeds[index].speedz += close_dz * AvoidFactor +
            (zvel_avg - this->BoidSpeeds[index].speedz) * MatchingFactor +
            (zpos_avg - this->BoidPositions[index].z) * CenteringFactor;

        // get velocity
        float speed = GetSpeed(this->BoidSpeeds[index]);

        if (speed < MINSPEED)
        {
            this->BoidSpeeds[index].speedx = (this->BoidSpeeds[index].speedx / speed) * MINSPEED;
            this->BoidSpeeds[index].speedy = (this->BoidSpeeds[index].speedy / speed) * MINSPEED;
            this->BoidSpeeds[index].speedz = (this->BoidSpeeds[index].speedz / speed) * MINSPEED;
        }
        if (speed > MAXSPEED)
        {
            this->BoidSpeeds[index].speedx = (this->BoidSpeeds[index].speedx / speed) * MAXSPEED;
            this->BoidSpeeds[index].speedy = (this->BoidSpeeds[index].speedy / speed) * MAXSPEED;
            this->BoidSpeeds[index].speedz = (this->BoidSpeeds[index].speedz / speed) * MAXSPEED;
        }


        if (this->BoidPositions[index].x < MARGIN)
            this->BoidSpeeds[index].speedx += TURNFACTOR;

        if (this->BoidPositions[index].x > WIDTH - MARGIN)
            this->BoidSpeeds[index].speedx -= TURNFACTOR;

        if (this->BoidPositions[index].y > HEIGHT - MARGIN)
            this->BoidSpeeds[index].speedy -= TURNFACTOR;

        if (this->BoidPositions[index].y < MARGIN)
            this->BoidSpeeds[index].speedy += TURNFACTOR;

        if (this->BoidPositions[index].z > DEPTH - MARGIN)
            this->BoidSpeeds[index].speedz -= TURNFACTOR;

        if (this->BoidPositions[index].z < MARGIN)
            this->BoidSpeeds[index].speedz += TURNFACTOR;

#pragma endregion

        this->BoidPositions[index].x += this->BoidSpeeds[index].speedx;
        this->BoidPositions[index].y += this->BoidSpeeds[index].speedy;
        this->BoidPositions[index].z += this->BoidSpeeds[index].speedz;

        if (this->BoidPositions[index].x >= WIDTH)
            this->BoidPositions[index].x = WIDTH - 1;

        else if (this->BoidPositions[index].x <= 0)
            this->BoidPositions[index].x = 1;

        if (this->BoidPositions[index].y >= HEIGHT)
            this->BoidPositions[index].y = HEIGHT - 1;

        else if (this->BoidPositions[index].y <= 0)
            this->BoidPositions[index].y = 1;

        if (this->BoidPositions[index].z >= DEPTH)
            this->BoidPositions[index].z = DEPTH - 1;

        else if (this->BoidPositions[index].z <= 0)
            this->BoidPositions[index].z = 1;

        this->OpenGLPositions[index].x = 2 * this->BoidPositions[index].x / WIDTH - 1.0;
        this->OpenGLPositions[index].y = 2 * this->BoidPositions[index].y / HEIGHT - 1.0;
        this->OpenGLPositions[index].z = 2 * this->BoidPositions[index].z / DEPTH - 1.0;

    }
   
    #pragma endregion

    memcpy(DestPositions, this->OpenGLPositions, sizeof(Position)* BoidsCount);

    ClearBoxes();
    AddBoxes();
}

bool BoidAlgorithm::InRange(Position& Pos1, Position& Pos2, float Range)
{
	return sqrt((Pos1.x - Pos2.x) * (Pos1.x - Pos2.x) +
				(Pos1.y - Pos2.y) * (Pos1.y - Pos2.y) +
				(Pos1.z - Pos2.z) * (Pos1.z - Pos2.z)) < Range;
}

float BoidAlgorithm::GetSpeed(Boid& Boid1)
{
	return sqrt(Boid1.speedx * Boid1.speedx +
				Boid1.speedy * Boid1.speedy +
				Boid1.speedz * Boid1.speedz);
}

int* BoidAlgorithm::GetSoroundingBoxes(Position* MyPosition, int* amount)
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
        my_Index + BoxAmountInX * BoxAmountInY < BoxAmountInX * BoxAmountInY * BoxAmountInZ)
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

short BoidAlgorithm::GetIndexOfMyBox(Position* Position)
{
    short BoxAmountInZ = DEPTH / EDGELENGTH;
    short BoxAmountInY = HEIGHT / EDGELENGTH;
    short BoxAmountInX = WIDTH / EDGELENGTH;
    short my_Index = (int)Position->z / EDGELENGTH * BoxAmountInX * BoxAmountInY
        + (int)Position->y / EDGELENGTH * BoxAmountInX
        + (int)Position->x / EDGELENGTH;

    return my_Index;
}

void BoidAlgorithm::ClearBoxes()
{
    // clear Box tab to create new one there
    for(int i=0; i<BoxAmount; i++)
        Boxes[i].AmountOfBirds = 0;
}

void BoidAlgorithm::AddBoxes()
{
    for (int i = 0; i < BoidsCount; i++)
    {
        short myBox = GetIndexOfMyBox(&this->BoidPositions[i]);
        Boxes[myBox].IndexesOfBird[Boxes[myBox].AmountOfBirds++] = i;
    }
}