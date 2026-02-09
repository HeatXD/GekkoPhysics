#pragma once

#include "gekko_math.h"

namespace GekkoPhysics {
	using namespace GekkoMath;

	struct OBB {
		Vec3 center;
		Vec3 half_extents;
		Mat3 rotation;
	};

	struct Sphere {
		Vec3 center;
		Unit radius;
	};

	struct Capsule {
		Vec3 start, end;
		Unit radius;
	};

	struct AABB {
		Vec3 min;
		Vec3 max;
	};

	struct CollisionResult {
		bool hit = false;
		Vec3 normal;
		Unit depth;
	};
}
