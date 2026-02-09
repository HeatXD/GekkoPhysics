#pragma once

#include "gekko_math.h"

#include <cstdint>

namespace GekkoPhysics {
	using namespace GekkoMath;

	enum DebugDrawFlags : uint32_t {
		DrawFlag_Shapes   = 1 << 0,
		DrawFlag_AABBs    = 1 << 1,
		DrawFlag_Contacts = 1 << 2,
		DrawFlag_BodyAxes = 1 << 3,
		DrawFlag_All      = 0xFFFFFFFF,
	};

	struct Color {
		float r, g, b, a;
		Color(float rr, float gg, float bb, float aa = 1.0f) : r(rr), g(gg), b(bb), a(aa) {}
	};

	class DebugDraw {
	public:
		uint32_t flags = DrawFlag_All;
		virtual ~DebugDraw() = default;

		virtual void DrawSphere(const Vec3F& center, float radius, const Color& color) = 0;
		virtual void DrawBox(const Vec3F& center, const Vec3F& half_extents, const Mat3F& rotation, const Color& color) = 0;
		virtual void DrawCapsule(const Vec3F& start, const Vec3F& end, float radius, const Color& color) = 0;
		virtual void DrawAABB(const Vec3F& min, const Vec3F& max, const Color& color) = 0;
		virtual void DrawLine(const Vec3F& from, const Vec3F& to, const Color& color) = 0;
		virtual void DrawPoint(const Vec3F& position, float size, const Color& color) = 0;
	};
}
