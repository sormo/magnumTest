#pragma once
#include "utils.h"
#include "point.h"
#include "common.h"
#include "vectorHandler.h"

struct Trajectory;

struct BurnsHandler
{
	BurnsHandler(Trajectory* t);

	void Draw();
	UpdateResult Update();
	void NewBurn(Burn* burn);
	void Refresh();

	VectorHandler vectorHandler;
	Trajectory* trajectory;
	std::optional<size_t> burnAddIndex;
};
