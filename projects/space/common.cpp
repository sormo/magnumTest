#include "common.h"
#include "camera.h"

extern Camera camera;

namespace Common
{
	float GetZoomIndependentSize(float size)
	{
		return size * camera.zoomFactor;
	}
}