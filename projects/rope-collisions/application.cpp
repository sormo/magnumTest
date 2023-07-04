#include "application.h"
#include "utils.h"

std::unique_ptr<Application> g_app;

Application::Application()
	: rope({0.0f, 0.0f}, {0.0f, 0.0f})
{
	SetupRope();
	SetupCircle();
	SetupPolygon();
	SetupRectangle();
	
}

void Application::SetupRope()
{
	rope = Rope({ 0.0f, 10.0f }, { -14.0f, 4.0f });
	rope.nodes[0].mass = 0.0f;

	// setup the hanging object
	SetupHang();
}

void Application::SetupRectangle()
{
	float offset = 0.01f * Magnum2D::getWindowSize().x();
	Magnum2D::vec2 min = Magnum2D::convertWindowToCamera({ offset, offset });
	Magnum2D::vec2 max = utils::getWindowRelativeCamera({ 0.08f, 0.08f });
	rectangles.push_back(Rectangle::FromMinMax(min, max));

	auto rect = Rectangle::FromCenterSize({ 0.0f, -6.0f }, { 2.0f, 1.2f });
	rect.angle = 45.0f;

	rectangles.push_back(std::move(rect));
}

void Application::SetupCircle()
{
	float offset = 0.06f * Magnum2D::getWindowSize().x();
	Magnum2D::vec2 center = Magnum2D::convertWindowToCamera({ Magnum2D::getWindowSize().x() - offset, offset });
	circles.push_back(Circle(center, 1.0f));
}

void Application::SetupPolygon()
{
	polygons.push_back(Polygon({ {1,-1}, {-1, -1}, {-1, 1}, {0, 1.5f}, {1,1} }));
	polygons.push_back(Polygon({ {0, 0}, {-2, -1}, {-2, 2}, {0, 1}, {2, 2}, {2, -1} }));
	polygons.back().SetCenter({ -3, 3 });
}

void Application::Draw()
{
	for (auto& r : rectangles)
		r.Draw();

	for (auto& c : circles)
		c.Draw();

	for (auto& p : polygons)
		p.Draw();

	rope.Draw();

	DrawHang();
}

void Application::SetupHang()
{
	auto& parent = rope.nodes.back();
	auto position = parent.position + Magnum2D::vec2{ 1.0f, 0.0f };

	rope.nodes.emplace_back(position, 1.0f);
	rope.nodes.back().mass = 10.0f;
	rope.nodes.back().childs.push_back(parent);
	parent.childs.push_back(rope.nodes.back());
}

void Application::DrawHang()
{
	auto& node = rope.nodes.back();
	auto& parent = (Rope::RopeNode&)node.childs[0];

	float angle = utils::angle(parent.position - node.position);

	Magnum2D::drawRectangle(rope.nodes.back().position, angle, 2.0f, 2.0f, Magnum2D::rgb(34, 50, 97));
}

Polygon::Polygon(std::vector<Magnum2D::vec2>&& points)
	: points(std::forward<std::vector<Magnum2D::vec2>>(points))
{
	pointsMoved = this->points;
}

Polygon::Polygon(const std::vector<Magnum2D::vec2>& points)
	: points(points), pointsMoved(points)
{
}

bool Polygon::IsInside(const Magnum2D::vec2& p) const
{
	return utils::isPointInsidePolygon(p, pointsMoved);
}

void Polygon::SetCenter(const Magnum2D::vec2& c)
{
	center = c;
	pointsMoved = points;
	for (auto& p : pointsMoved)
		p += c;
}

void Polygon::Draw()
{
	Magnum2D::drawPolygon(pointsMoved, Magnum2D::rgb(50, 50, 50));
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
	: center(center), size(size), hsize(size/2.0f), min(min), max(max)
{
}

bool Rectangle::IsInside(const Magnum2D::vec2& p) const
{
	if (angle != 0.0f)
	{
		auto local = ConvertToLocal(p);

		return local.x() >= -hsize.x() && local.x() <= hsize.x() && local.y() >= -hsize.y() && local.y() <= hsize.y();
	}

	return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
}

void Rectangle::SetCenter(const Magnum2D::vec2& c)
{
	center = c;
	min = c - hsize;
	max = c + hsize;
}

void Rectangle::Draw()
{
	Magnum2D::drawRectangle(center, angle, size.x(), size.y(), Magnum2D::rgb(50, 50, 50));
}

Magnum2D::vec2 Rectangle::ConvertToLocal(const Magnum2D::vec2& p) const
{
	Magnum2D::vec2 result = p - center;
	if (angle != 0.0f)
		return utils::rotate(result, -angle);
	return result;
}

Magnum2D::vec2 Rectangle::ConvertToGlobal(const Magnum2D::vec2& p) const
{
	Magnum2D::vec2 result = p;

	if (angle != 0.0f)
		result = utils::rotate(result, angle);

	return result + center;
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

void Circle::Draw()
{
	Magnum2D::drawCircle(center, radius, Magnum2D::rgb(50, 50, 50));
}
