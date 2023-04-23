#include <vector>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector2.h>

namespace Application2D
{
	using Color = Magnum::Math::Color3<float>;
	Color rgb(uint8_t r, uint8_t g, uint8_t b);

	using Point = Magnum::Math::Vector2<float>;

	void setCameraCenter(Point center);
	Point getCameraCenter();

	void setCameraSize(Point size);
	Point getCameraSize();

	Point convertCameraToWindow(const Point& p);
	Point convertWindowToCamera(const Point& p);

	Point getMousePositionWindow();
	bool isMousePressed(); // true only in frame when mouse was pressed
	bool isMouseReleased(); // true only in frame when mouse was released
	bool isMouseDown();

	void drawPolyline(const std::vector<Point>& points, Color color);

	void drawCircle(Point center, float radius, Color color);
	void drawCircleOutline(Point center, float radius, Color color);
	
	void drawRectangle(Point center, float width, float height, Color color);
	void drawRectangle(Point center, float rotation, float width, float height, Color color);
	
	void drawPolygon(const std::vector<Point>& points, Color color);
}