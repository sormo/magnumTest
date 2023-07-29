#include "vectorHandler.h"
#include "utils.h"
#include "common.h"

bool VectorHandler::UpdateHighlight()
{
	auto position = Magnum2D::getMousePositionWorld();
	highlightVec.reset();

	for (size_t i = 0; i < vectors.size(); i++)
	{
		Magnum2D::vec2 offsetTo = position - vectors[i].to;
		if (offsetTo.length() < GrabToRadius)
		{
			highlightVec = i;
			highlightType = HandleType::To;
			return true;
		}

		Magnum2D::vec2 offsetFrom = position - vectors[i].from;
		if (offsetFrom.length() < GrabFromRadius)
		{
			highlightVec = i;
			highlightType = HandleType::From;
			return true;
		}
	}

	return false;
}

bool VectorHandler::Update()
{
	if (Magnum2D::isMousePressed())
	{
		auto position = Magnum2D::getMousePositionWorld();

		for (size_t i = 0; i < vectors.size(); i++)
		{
			Magnum2D::vec2 offsetTo = position - vectors[i].to;
			if (offsetTo.length() < GrabToRadius)
			{
				grabVec = i;
				grabType = HandleType::To;
				grabOffset = offsetTo;

				return true;
			}

			Magnum2D::vec2 offsetFrom = position - vectors[i].from;
			if (offsetFrom.length() < GrabFromRadius)
			{
				grabVec = i;
				grabType = HandleType::From;
				grabOffset = offsetFrom;

				return true;
			}
		}
	}
	else if (Magnum2D::isMouseReleased() && grabVec)
	{
		grabVec.reset();
		return true;
	}
	else if (grabVec)
	{
		auto position = Magnum2D::getMousePositionWorld() - grabOffset;

		switch (grabType)
		{
		case HandleType::To:
			vectors[*grabVec].to = vectors[*grabVec].onToChange(position, vectors[*grabVec].context);
			break;
		case HandleType::From:
			vectors[*grabVec].from = vectors[*grabVec].onFromChange(position, vectors[*grabVec].context);
			break;
		}

		return true;
	}

	return false;
}

void VectorHandler::Draw()
{
	for (size_t i = 0; i < vectors.size(); i++)
	{
		Magnum2D::col3 circleColorFrom = highlightVec && *highlightVec == i && highlightType == HandleType::From ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);
		Magnum2D::col3 circleColorTo = highlightVec && *highlightVec == i && highlightType == HandleType::To ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);

		Utils::DrawVector(vectors[i].from, vectors[i].to - vectors[i].from, Magnum2D::rgb(255, 255, 255));
		Magnum2D::drawCircleOutline(vectors[i].to, Common::GetZoomIndependentSize(GrabToRadius), circleColorTo);
		Magnum2D::drawCircleOutline(vectors[i].from, Common::GetZoomIndependentSize(GrabFromRadius), circleColorFrom);
		Magnum2D::drawCircle(vectors[i].from, Common::GetZoomIndependentSize(0.03f), Magnum2D::rgb(10, 200, 10));
	}
}

void VectorHandler::Push(Vector&& v)
{
	vectors.push_back(std::move(v));
}

void VectorHandler::Clear()
{
	vectors.clear();
	grabVec.reset();
}

void VectorHandler::ClearOnlyVectors()
{
	vectors.clear();
}
