#pragma once
#include "Magnum2D.h"
#include <optional>
#include <functional>
#include <map>

struct VectorHandler
{
	const float GrabFromRadius = 0.1f;
	const float GrabToRadius = 0.3f;

	using OnChange = std::function<Magnum2D::vec2(const Magnum2D::vec2&, void*)>;

	using Vector = uint64_t;

	// return true if some vector handle is highlighted
	bool UpdateHighlight();
	
	struct UpdateResult
	{
		bool isInputGrabbed;
		bool isVectorChanged;
	};
	UpdateResult Update();

	void Draw();

	Vector Push(Magnum2D::vec2 from, Magnum2D::vec2 to, void* context, OnChange onFromChange, OnChange onToChange);
	void ChangeFrom(Vector vector, const Magnum2D::vec2& from); // preserve the vector size
	void SetTo(Vector vector, const Magnum2D::vec2& to); // does not preserve, do not call callback, just set to to to (: (like give me my black backpack back)

	bool IsGrab(Vector vector);

	void Clear();
	void ClearGrab();
	void ClearOnlyVectors();

	auto begin()
	{
		return vectors.begin();
	}

	auto end()
	{
		return vectors.end();
	}

	// do not call callbacks until the grab distance is not more than threshold
	float thresholdDistanceZoomIndependent = 0.0f;

private:
	static Vector VectorCounter;

	struct VectorData
	{
		Magnum2D::vec2 from;
		Magnum2D::vec2 to;
		void* context = nullptr;

		OnChange onFromChange;
		OnChange onToChange;
	};

	std::map<Vector, VectorData> vectors;

	enum class HandleType { From, To };

	std::optional<Vector> grabVec;
	HandleType grabType;
	Magnum2D::vec2 grabOffset;

	std::optional<Vector> highlightVec;
	HandleType highlightType;
};
