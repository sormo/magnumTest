#pragma once
#include "rope.h"
#include <Magnum2D.h>
#include <memory>
#include <vector>

struct Movable
{
	virtual void SetCenter(const Magnum2D::vec2& c) = 0;
};

struct Polygon : public Movable
{
	Polygon(std::vector<Magnum2D::vec2>&& points);
	Polygon(const std::vector<Magnum2D::vec2>& points);

	bool IsInside(const Magnum2D::vec2& p) const;
	void SetCenter(const Magnum2D::vec2& c) override;
	void Draw();

	std::vector<Magnum2D::vec2> points;
	Magnum2D::vec2 center;
	std::vector<Magnum2D::vec2> pointsMoved;
};

struct Rectangle : public Movable
{
	static Rectangle FromCenterSize(const Magnum2D::vec2& center, const Magnum2D::vec2& size);
	static Rectangle FromMinMax(const Magnum2D::vec2& min, const Magnum2D::vec2& max);

	Rectangle(const Magnum2D::vec2& center, const Magnum2D::vec2& size, const Magnum2D::vec2& min, const Magnum2D::vec2& max);

	Magnum2D::vec2 center;
	Magnum2D::vec2 size;
	Magnum2D::vec2 hsize;
	float angle = 0.0f;

	Magnum2D::vec2 min;
	Magnum2D::vec2 max;

	bool IsInside(const Magnum2D::vec2& p) const;
	void SetCenter(const Magnum2D::vec2& c) override;
	void Draw();

	Magnum2D::vec2 ConvertToLocal(const Magnum2D::vec2& p) const;
	Magnum2D::vec2 ConvertToGlobal(const Magnum2D::vec2& p) const;
};

struct Circle : public Movable
{
	Circle(const Magnum2D::vec2& center, float radius);

	bool IsInside(const Magnum2D::vec2& p) const;
	void SetCenter(const Magnum2D::vec2& c) override;
	void Draw();

	Magnum2D::vec2 center;
	float radius;
};

struct Application
{
	Application();
	
	Rope rope;
	std::vector<Circle> circles;
	std::vector<Rectangle> rectangles;
	std::vector<Polygon> polygons;

	void SetupRope();
	void SetupCircle();
	void SetupRectangle();
	void SetupPolygon();
	
};

extern std::unique_ptr<Application> g_app;
