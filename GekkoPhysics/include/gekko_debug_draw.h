#pragma once

#include "gekko_math.h"

#include <cstdint>

namespace GekkoPhysics {
	using namespace GekkoMath;

	enum DebugDrawFlags : uint32_t {
		DrawFlag_Shapes      = 1 << 0,
		DrawFlag_AABBs       = 1 << 1,
		DrawFlag_Contacts    = 1 << 2,
		DrawFlag_BodyAxes    = 1 << 3,
		DrawFlag_BodyOrigins = 1 << 4,
		DrawFlag_All         = 0xFFFFFFFF,
	};

	class DebugDraw {
	public:
		uint32_t flags = DrawFlag_All;
		virtual ~DebugDraw() = default;

		virtual void DrawSphere(const Vec3F& center, float radius) = 0;
		virtual void DrawBox(const Vec3F& center, const Vec3F& half_extents, const Mat3F& rotation) = 0;
		virtual void DrawCapsule(const Vec3F& start, const Vec3F& end, float radius) = 0;
		virtual void DrawAABB(const Vec3F& min, const Vec3F& max) = 0;
		virtual void DrawLine(const Vec3F& from, const Vec3F& to) = 0;
		virtual void DrawPoint(const Vec3F& position, float size) = 0;
		virtual void DrawBodyOrigin(const Vec3F& position) = 0;
		virtual void DrawBodyAxes(const Vec3F& position, const Mat3F& rotation) = 0;
	};
}
