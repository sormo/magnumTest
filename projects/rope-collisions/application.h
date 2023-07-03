#pragma once
#include "rope.h"
#include <Magnum2D.h>
#include <memory>

struct Movable
{
	virtual void SetCenter(const Magnum2D::vec2& c) = 0;
};

struct Rectangle : public Movable
{
	static Rectangle FromCenterSize(const Magnum2D::vec2& center, const Magnum2D::vec2& size);
	static Rectangle FromMinMax(const Magnum2D::vec2& min, const Magnum2D::vec2& max);

	Rectangle(const Magnum2D::vec2& center, const Magnum2D::vec2& size, const Magnum2D::vec2& min, const Magnum2D::vec2& max);

	Magnum2D::vec2 center;
	Magnum2D::vec2 size;

	Magnum2D::vec2 min;
	Magnum2D::vec2 max;

	bool IsInside(const Magnum2D::vec2& p) const;
	void SetCenter(const Magnum2D::vec2& c) override;
};

struct Circle : public Movable
{
	Circle(const Magnum2D::vec2& center, float radius);

	bool IsInside(const Magnum2D::vec2& p) const;
	void SetCenter(const Magnum2D::vec2& c) override;

	Magnum2D::vec2 center;
	float radius;
};

struct Application
{
	Application();
	
	Rope rope;
	std::vector<Circle> circles;
	std::vector<Rectangle> rectangles;

	void SetupRope();
	void SetupCircle();
	void SetupRectangle();
	
};

extern std::unique_ptr<Application> g_app;
