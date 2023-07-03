#pragma once
#include <Magnum2D.h>

namespace Collisions
{
	void applyRectangleCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to);
	void applyCircleCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to);
	void applyCollisions(Magnum2D::vec2& from, Magnum2D::vec2& to);

	void applyCollisions(Magnum2D::vec2& point);
}
