#include "camera.h"
#include <Magnum2D.h>

using namespace Magnum2D;

static const float MinimumZoom = 0.001f;
static const float ScrollSpeed = 0.1f; // 10% of width

void Camera::Setup()
{
	originalCameraSize = getCameraSize();
}

void drawCoordinateLines()
{
	vec2 bottomLeft = getCameraCenter() - getCameraSize() / 2.0f;
	vec2 upperRight = getCameraCenter() + getCameraSize() / 2.0f;

	std::vector<vec2> points;
	for (float x = std::ceil(bottomLeft.x()); x < upperRight.x(); x += 1.0f)
	{
		points.push_back({ x, bottomLeft.y() });
		points.push_back({ x, upperRight.y() });
	}
	for (float y = std::ceil(bottomLeft.y()); y < upperRight.y(); y += 1.0f)
	{
		points.push_back({ bottomLeft.x(), y });
		points.push_back({ upperRight.x(), y });
	}

	drawLines(points, rgb(50, 50, 50));
}

void Camera::Draw()
{
	drawCoordinateLines();
}

void Camera::Update(bool allowMove)
{
	if (isWindowResized())
		Setup();

	if (isMouseDown() && allowMove)
	{
		auto mouseDeltaWorld = convertWindowToWorldVector(getMouseDeltaWindow());
		auto cameraCenter = getCameraCenter();

		setCameraCenter(cameraCenter - mouseDeltaWorld);
	}

	auto scrollX = getMouseScroll();
	if (scrollX == 0.0f)
		return;

	auto cameraSize = getCameraSize();
	scrollX = ScrollSpeed * cameraSize.x() * scrollX;

	float scrollY = (cameraSize.y() / cameraSize.x()) * scrollX;
	auto newCameraSize = getCameraSize() + vec2{ scrollX, scrollY };

	if (newCameraSize.x() < MinimumZoom || newCameraSize.y() < MinimumZoom)
		return;

	zoomFactor =  newCameraSize.x() / originalCameraSize.x();

	auto mousePosition = getMousePositionWorld();

	setCameraSize(newCameraSize);

	if (allowMove)
	{
		auto offset = mousePosition - getMousePositionWorld();
		setCameraCenter(getCameraCenter() + offset);
	}
}
