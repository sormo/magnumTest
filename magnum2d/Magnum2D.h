#pragma once
#include <vector>
#include <string>
#include <span>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Range.h>

// these two functions must be implemented by application
void setup();
void draw();

namespace Magnum2D
{
	using col3 = Magnum::Math::Color3<float>;
	col3 rgb(uint8_t r, uint8_t g, uint8_t b);

	using vec2 = Magnum::Math::Vector2<float>;
	using vec2d = Magnum::Math::Vector2<double>;
	using rectangle = Magnum::Math::Range2D<float>;

	vec2 getWindowSize();
	bool isWindowResized();

	void setCameraCenter(vec2 center);
	vec2 getCameraCenter();

	void setCameraSize(vec2 size);
	vec2 getCameraSize();

	vec2 convertCameraToWindow(const vec2& p);
	vec2 convertWindowToWorld(const vec2& p);
	vec2 convertWindowToWorldVector(const vec2& p); // will not take into account translation

	float getDeltaTimeMs(); // time since last frame
	float getTimeMs(); // time since start

	enum class Mouse { left, middle, right };
	vec2 getMousePositionWindow();
	vec2 getMousePositionWorld();
	vec2 getMouseDeltaWindow();
	float getMouseScroll();
	bool isMousePressed(Mouse mouse = Mouse::left); // true only in frame when mouse was pressed
	bool isMouseReleased(Mouse mouse = Mouse::left); // true only in frame when mouse was released
	bool isMouseDown(Mouse mouse = Mouse::left);

	bool isKeyPressed(char key); // true only in frame when key was pressed
	bool isKeyReleased(char key); // true only in frame when key was released
	bool isKeyDown(char key);

	void drawPolyline(const std::vector<vec2>& points, col3 color);
	void drawPolyline(std::span<vec2> points, col3 color);
	void drawPolyline2(std::span<vec2> points, float width, col3 color);
	void drawLines(const std::vector<vec2>& points, col3 color);
	void drawLines2(std::vector<vec2>& points, float width, col3 color);

	void drawCircle(vec2 center, float radius, col3 color);
	void drawCircleOutline(vec2 center, float radius, col3 color);
	void drawCircleOutline2(vec2 center, float radius, float width, col3 color);
	
	void drawRectangle(vec2 center, float width, float height, col3 color);
	void drawRectangle(vec2 center, float rotation, float width, float height, col3 color);
	
	void drawPolygon(const std::vector<vec2>& points, col3 color);

	enum class TextAlign { TopLeft, TopMiddle, TopRight, MiddleLeft, MiddleMiddle, MiddleRight, BottomLeft, BottomMiddle, BottomRight };
	void drawText(vec2 position, const std::string& text, float height, col3 color, TextAlign align = TextAlign::BottomLeft);
	rectangle getTextRectangle(vec2 position, const std::string& text, float height, TextAlign align);
}
