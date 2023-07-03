#include <Magnum2D.h>

namespace Globals
{
	static float RopeNodeDistance = 0.2f;
	static float RopeSimmulationDelta = 0.02f;
	static int32_t RopeConstraintIterations = 10;
	static Magnum2D::vec2 Gravity = Magnum2D::vec2(0.0f, -9.89f);

	enum
	{
		InteractionCut = 0,
		InteractionGrab = 1,
		InteractionAttract = 2,

	} static Interaction;
}
