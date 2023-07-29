#pragma once

enum class UpdateResult
{
	None,
	Modified,
	InputGrab,
};

namespace Common
{
	float GetZoomIndependentSize(float size);
}
