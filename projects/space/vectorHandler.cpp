#include "vectorHandler.h"
#include "utils.h"
#include "common.h"

VectorHandler::Vector VectorHandler::VectorCounter = 0;

bool VectorHandler::UpdateHighlight()
{
	auto position = Magnum2D::getMousePositionWorld();
	highlightVec.reset();

	for (auto& [vector, data] : vectors)
	{
		if (data.onToChange)
		{
			Magnum2D::vec2 offsetTo = position - data.to;
			if (offsetTo.length() < Common::GetZoomIndependentSize(GrabToRadius))
			{
				highlightVec = vector;
				highlightType = HandleType::To;
				return true;
			}
		}

		if (data.onFromChange)
		{
			Magnum2D::vec2 offsetFrom = position - data.from;
			if (offsetFrom.length() < Common::GetZoomIndependentSize(GrabToRadius))
			{
				highlightVec = vector;
				highlightType = HandleType::From;
				return true;
			}
		}
	}

	return false;
}

VectorHandler::UpdateResult VectorHandler::Update()
{
	if (Magnum2D::isMousePressed())
	{
		auto position = Magnum2D::getMousePositionWorld();

		for (auto& [vector, data] : vectors)
		{
			if (data.onToChange)
			{
				Magnum2D::vec2 offsetTo = position - data.to;
				if (offsetTo.length() < Common::GetZoomIndependentSize(GrabToRadius))
				{
					grabVec = vector;
					grabType = HandleType::To;
					grabOffset = offsetTo;

					return { true, false };
				}
			}

			if (data.onFromChange)
			{
				Magnum2D::vec2 offsetFrom = position - data.from;
				if (offsetFrom.length() < Common::GetZoomIndependentSize(GrabFromRadius))
				{
					grabVec = vector;
					grabType = HandleType::From;
					grabOffset = offsetFrom;

					return { true, false };
				}
			}
		}
	}
	else if (Magnum2D::isMouseReleased() && grabVec)
	{
		grabVec.reset();
		return { true, false };
	}
	else if (grabVec)
	{
		auto& vec = vectors[*grabVec];
		auto position = Magnum2D::getMousePositionWorld() - grabOffset;

		bool changed = false;

		switch (grabType)
		{
		case HandleType::To:
			if ((position - vec.to).length() > Common::GetZoomIndependentSize(thresholdDistanceZoomIndependent))
			{
				vec.to = vec.onToChange(position, vec.context);
				changed = true;
			}
			break;
		case HandleType::From:
			if ((position - vec.from).length() > Common::GetZoomIndependentSize(thresholdDistanceZoomIndependent))
			{
				vec.from = vec.onFromChange(position, vec.context);
				changed = true;
			}
			break;
		}

		return { true, changed };
	}

	return { false, false };
}

void VectorHandler::Draw()
{
	for (auto& [vector, data] : vectors)
	{
		Utils::DrawVector(data.from, data.to - data.from, Magnum2D::rgb(120, 120, 120));

		if (data.onFromChange)
		{
			Magnum2D::col3 circleColorFrom = highlightVec && *highlightVec == vector && highlightType == HandleType::From ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);
			Common::DrawCircleOutline(data.from, Common::GetZoomIndependentSize(GrabFromRadius), Common::GetZoomIndependentSize(0.03f), circleColorFrom);
		}
		if (data.onToChange)
		{
			Magnum2D::col3 circleColorTo = highlightVec && *highlightVec == vector && highlightType == HandleType::To ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);
			Common::DrawCircleOutline(data.to, Common::GetZoomIndependentSize(GrabToRadius), Common::GetZoomIndependentSize(0.03f), circleColorTo);
		}
		Magnum2D::drawCircle(data.from, Common::GetZoomIndependentSize(0.03f), Magnum2D::rgb(10, 200, 10));
	}
}

VectorHandler::Vector VectorHandler::Push(Magnum2D::vec2 from, Magnum2D::vec2 to, void* context, OnChange onFromChange, OnChange onToChange)
{
	VectorData newVector;
	newVector.from = from;
	newVector.to = to;
	newVector.context = context;
	newVector.onFromChange = onFromChange;
	newVector.onToChange = onToChange;

	Vector vector = ++VectorCounter;
	vectors.insert({ vector, std::move(newVector) });

	return vector;
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

void VectorHandler::ChangeFrom(Vector vector, const Magnum2D::vec2& from)
{
	auto& vec = vectors[vector];
	if (from == vec.from)
		return;

	vec.to = from + (vec.to - vec.from);
	vec.from = from;
}

void VectorHandler::SetTo(Vector vector, const Magnum2D::vec2& to)
{
	auto& vec = vectors[vector];
	vec.to = to;
}

void VectorHandler::ClearGrab()
{
	grabVec = {};
}

bool VectorHandler::IsGrab(Vector vector)
{
	return grabVec == vector;
}

bool VectorHandler::IsGrab()
{
	return grabVec.has_value();
}

std::optional<VectorHandler::Vector> VectorHandler::GetGrab()
{
	return grabVec;
}
