#pragma once
#include "utils.h"
#include "point.h"
#include "common.h"
#include "vectorHandler.h"

struct Ship;

struct BurnsHandler
{
	BurnsHandler(Ship* t);

	void Draw();
	UpdateResult Update();
	void NewBurn(Burn* burn);
	void Refresh();

	VectorHandler vectorHandler;
	Ship* ship;
	std::optional<size_t> burnAddIndex;
};
