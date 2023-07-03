#include "application.h"
#include "utils.h"

std::unique_ptr<Application> g_app;

Application::Application()
	: rope({0.0f, 0.0f}, {0.0f, 0.0f})
{
	SetupRope();
	SetupCircle();
	SetupRectangle();
}

void Application::SetupRope()
{
	rope = Rope({ 0.0f, 10.0f }, { -14.0f, 4.0f });
	rope.nodes[0].mass = 0.0f;
}

void Application::SetupRectangle()
{
	float offset = 0.01f * Magnum2D::getWindowSize().x();
	Magnum2D::vec2 min = Magnum2D::convertWindowToCamera({ offset, offset });
	Magnum2D::vec2 max = utils::getWindowRelativeCamera({ 0.08f, 0.08f });
	rectangles.push_back(Rectangle::FromMinMax(min, max));
}

void Application::SetupCircle()
{
	float offset = 0.06f * Magnum2D::getWindowSize().x();
	Magnum2D::vec2 center = Magnum2D::convertWindowToCamera({ Magnum2D::getWindowSize().x() - offset, offset });
	circles.push_back(Circle(center, 1.0f));
}

Rectangle Rectangle::FromCenterSize(const Magnum2D::vec2& center, const Magnum2D::vec2& size)
{
	return { center, size, center - size / 2.0f, center + size / 2.0f };
}

Rectangle Rectangle::FromMinMax(const Magnum2D::vec2& min, const Magnum2D::vec2& max)
{
	return { min + (max - min) / 2.0f, max - min, min, max };
}

Rectangle::Rectangle(const Magnum2D::vec2& center, const Magnum2D::vec2& size, const Magnum2D::vec2& min, const Magnum2D::vec2& max)
	: center(center), size(size), min(min), max(max)
{
}

bool Rectangle::IsInside(const Magnum2D::vec2& p) const
{
	return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
}

void Rectangle::SetCenter(const Magnum2D::vec2& c)
{
	center = c;
	min = c - size / 2.0f;
	max = c + size / 2.0f;
}

Circle::Circle(const Magnum2D::vec2& center, float radius)
	: center(center), radius(radius)
{
}

bool Circle::IsInside(const Magnum2D::vec2& p) const
{
	return (p - center).length() < radius;
}

void Circle::SetCenter(const Magnum2D::vec2& c)
{
	center = c;
}
