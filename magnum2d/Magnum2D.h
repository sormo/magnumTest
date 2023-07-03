#include <vector>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector2.h>

// these two functions must be implemented by application
void setup();
void draw();

namespace Magnum2D
{
	using col3 = Magnum::Math::Color3<float>;
	col3 rgb(uint8_t r, uint8_t g, uint8_t b);

	using vec2 = Magnum::Math::Vector2<float>;

	vec2 getWindowSize();

	void setCameraCenter(vec2 center);
	vec2 getCameraCenter();

	void setCameraSize(vec2 size);
	vec2 getCameraSize();

	vec2 convertCameraToWindow(const vec2& p);
	vec2 convertWindowToCamera(const vec2& p);

	vec2 getMousePositionWindow();
	vec2 getMousePositionCamera();
	bool isMousePressed(); // true only in frame when mouse was pressed
	bool isMouseReleased(); // true only in frame when mouse was released
	bool isMouseDown();

	bool isKeyPressed(char key); // true only in frame when key was pressed
	bool isKeyReleased(char key); // true only in frame when key was released
	bool isKeyDown(char key);

	void drawPolyline(const std::vector<vec2>& points, col3 color);
	void drawLines(const std::vector<vec2>& points, col3 color);

	void drawCircle(vec2 center, float radius, col3 color);
	void drawCircleOutline(vec2 center, float radius, col3 color);
	
	void drawRectangle(vec2 center, float width, float height, col3 color);
	void drawRectangle(vec2 center, float rotation, float width, float height, col3 color);
	
	void drawPolygon(const std::vector<vec2>& points, col3 color);
}
