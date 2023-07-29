#pragma once
#include <Magnum2D.h>
#include <optional>
#include <functional>

namespace Utils
{
	const float Deg2Rad = 0.0174533f;

	Magnum2D::vec2 GetRandomPosition(float xmin, float xmax, float ymin, float ymax); // ranges are inclusive
	Magnum2D::vec2 RotateVector(const Magnum2D::vec2& vector, float radians);
	float DistanceSqr(const Magnum2D::vec2& p1, const Magnum2D::vec2& p2);

	void DrawVector(const Magnum2D::vec2& position, const Magnum2D::vec2& vector, const Magnum2D::col3& color);

	struct VectorHandler
	{
		const float GrabFromRadius = 0.1f;
		const float GrabToRadius = 0.3f;

		struct Vector
		{
			Magnum2D::vec2 from;
			Magnum2D::vec2 to;
			void* context;

			std::function<Magnum2D::vec2(const Magnum2D::vec2&, void*)> onFromChange;
			std::function<Magnum2D::vec2(const Magnum2D::vec2&, void*)> onToChange;
		};

		// return true if some vector handle is highlighted
		bool UpdateHighlight()
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

		// return true if mouse is captured
		bool Update()
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

		void Draw()
		{
			for (size_t i = 0; i < vectors.size(); i++)
			{
				Magnum2D::col3 circleColorFrom = highlightVec && *highlightVec == i && highlightType == HandleType::From ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);
				Magnum2D::col3 circleColorTo = highlightVec && *highlightVec == i && highlightType == HandleType::To ? Magnum2D::rgb(50, 255, 50) : Magnum2D::rgb(50, 50, 50);

				Utils::DrawVector(vectors[i].from, vectors[i].to - vectors[i].from, Magnum2D::rgb(255, 255, 255));
				Magnum2D::drawCircleOutline(vectors[i].to, GrabToRadius, circleColorTo);
				Magnum2D::drawCircleOutline(vectors[i].from, GrabFromRadius, circleColorFrom);
				Magnum2D::drawCircle(vectors[i].from, 0.03f, Magnum2D::rgb(10, 200, 10));
			}
		}

		void Push(Vector&& v)
		{
			vectors.push_back(std::move(v));
		}

		void Clear()
		{
			vectors.clear();
			grabVec.reset();
		}

		void ClearOnlyVectors()
		{
			vectors.clear();
		}

		auto begin()
		{
			return vectors.begin();
		}

		auto end()
		{
			return vectors.end();
		}
		
	private:
		std::vector<Vector> vectors;

		enum class HandleType { From, To };

		std::optional<size_t> grabVec;
		HandleType grabType;
		Magnum2D::vec2 grabOffset;

		std::optional<size_t> highlightVec;
		HandleType highlightType;

	};
}