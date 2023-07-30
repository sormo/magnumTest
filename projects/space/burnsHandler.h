#pragma once
#include "utils.h"
#include "point.h"
#include "common.h"
#include "vectorHandler.h"

struct Ship;
struct Trajectory;

struct BurnsHandler
{
	BurnsHandler(Ship* s, Trajectory* t);

	void Draw();
	UpdateResult Update();
	void NewBurn(Burn* burn);
	void Refresh();

	VectorHandler vectorHandler;
	Ship* ship;
	Trajectory* trajectory;
	std::optional<size_t> burnAddIndex;
};
