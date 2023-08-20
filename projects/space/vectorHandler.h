#pragma once
#include "Magnum2D.h"
#include <optional>
#include <functional>

struct VectorHandler
{
	const float GrabFromRadius = 0.1f;
	const float GrabToRadius = 0.3f;

	using OnChange = std::function<Magnum2D::vec2(const Magnum2D::vec2&, void*)>;

	// return true if some vector handle is highlighted
	bool UpdateHighlight();
	// return true if mouse is captured
	bool Update();

	void Draw();

	void Push(Magnum2D::vec2 from, Magnum2D::vec2 to, void* context, OnChange onFromChange, OnChange onToChange);

	void Clear();

	void ClearOnlyVectors();

	auto begin()
	{
		return vectors.begin();
	}

	auto end()
	{
		return vectors.end();
	}

private:
	struct Vector
	{
		Magnum2D::vec2 from;
		Magnum2D::vec2 to;
		void* context = nullptr;

		OnChange onFromChange;
		OnChange onToChange;
	};

	std::vector<Vector> vectors;

	enum class HandleType { From, To };

	std::optional<size_t> grabVec;
	HandleType grabType;
	Magnum2D::vec2 grabOffset;

	std::optional<size_t> highlightVec;
	HandleType highlightType;
};
