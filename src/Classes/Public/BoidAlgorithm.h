#pragma once
#include "Boids.h"
#include "Typedef.h"

class BoidAlgorithm
{
public:
	Position* BoidPositions;
	Position* OpenGLPositions;
	Boid* BoidSpeeds;
	uint BoidsCount;
	float AvoidFactor = 0.05;
	float MatchingFactor = 0.05;
	float CenteringFactor = 0.0003;

	const int BoxAmount = WIDTH / EDGELENGTH * DEPTH / EDGELENGTH * HEIGHT / EDGELENGTH;

	Box Boxes[WIDTH / EDGELENGTH * DEPTH / EDGELENGTH * HEIGHT / EDGELENGTH];

	BoidAlgorithm(Position* m_pos, Boid* m_boids, uint m_boidsCount);
	~BoidAlgorithm();

	void GetNewPositions(Position* DestPositions);
	bool InRange(Position& Pos1, Position& Pos2, float Range);
	float GetSpeed(Boid& Boid1);
	int* GetSoroundingBoxes(Position* MyPosition, int* amount);
	short GetIndexOfMyBox(Position* Position);

	void ClearBoxes();
	void AddBoxes();

};
