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

#define EDGELENGTH 100

#define MARGIN 50

#define MAXSPEED 6
#define MINSPEED 2
#define VISIBLERANGE 100
#define PROTECTIONRANGE 30
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