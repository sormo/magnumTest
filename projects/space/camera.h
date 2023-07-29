#pragma once
#include <Magnum2D.h>

struct Camera
{
	void Setup();
	void Update(bool allowMove);

	Magnum2D::vec2 originalCameraSize;
	float zoomFactor = 1.0f;
};
