#include "camera.h"
#include <Magnum2D.h>

using namespace Magnum2D;

void Camera::Setup()
{
	originalCameraSize = getCameraSize();
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
	auto cameraSize = getCameraSize();
	float scrollY = (cameraSize.y() / cameraSize.x()) * scrollX;
	auto newCameraSize = getCameraSize() + vec2{ scrollX, scrollY };

	if (newCameraSize.x() < 0.1f || newCameraSize.y() < 0.1f)
		return;

	zoomFactor =  newCameraSize.x() / originalCameraSize.x();

	setCameraSize(newCameraSize);
}
