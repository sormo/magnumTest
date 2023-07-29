#pragma once
#include "utils.h"
#include "point.h"
#include "common.h"

struct Trajectory;

struct BurnsHandler
{
	BurnsHandler(Trajectory* t);

	void Draw();
	UpdateResult Update();
	void NewBurn(Burn* burn);
	void Refresh();

	Utils::VectorHandler vectorHandler;
	Trajectory* trajectory;
	std::optional<size_t> burnAddIndex;
};
