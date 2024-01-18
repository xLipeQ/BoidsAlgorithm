#pragma once
#ifndef _BOIDS
#define _BOIDS

#define BOXES
//#define GPU

#define AQUARIGHT 1.0
#define AQUALEFT -1.0
#define AQUATOP 1.0
#define AQUADOWN -1.0

#define WIDTH 800
#define HEIGHT 800
#define DEPTH 800

#define EDGELENGTH 200

#define MARGIN 50

#define BOIDSCOUNT 10000
#define MAXSPEED 6
#define MINSPEED 2
#define VISIBLERANGE 40
#define PROTECTIONRANGE 8
#define TURNFACTOR 0.2

struct Position
{
	float x;
	float y;
	float z;
};

struct Boid
{
	float speedx;
	float speedy;
	float speedz;
};

struct Box
{
	int *IndexesOfBird;
	int AmountOfBirds = 0;
};



#endif // !_BOIDS