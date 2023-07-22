#include <Magnum2D.h>
#include <optional>

namespace Utils
{
	const float Deg2Rad = 0.0174533f;

	Magnum2D::vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax); // ranges are inclusive
	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians);

	void DrawVector(const Magnum2D::vec2& position, const Magnum2D::vec2& vector, const Magnum2D::col3& color);
}