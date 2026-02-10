#pragma once

#include "gekko_shapes.h"

namespace GekkoPhysics {
	using namespace GekkoMath;

	class Algo {
	public:
		static Vec3 ClosestPointOnSegment(const Vec3& point, const Vec3& seg_start, const Vec3& seg_end);
		static void ClosestPointsBetweenSegments(const Vec3& s1_start, const Vec3& s1_end, const Vec3& s2_start, const Vec3& s2_end, Vec3& out1, Vec3& out2);
		static Vec3 ClosestPointOnOBB(const Vec3& point, const OBB& obb);

		static CollisionResult CollideSpheres(const Sphere& a, const Sphere& b);
		static CollisionResult CollideSphereCapsule(const Sphere& a, const Capsule& b);
		static CollisionResult CollideCapsules(const Capsule& a, const Capsule& b);
		static CollisionResult CollideSphereOBB(const Sphere& a, const OBB& b);
		static CollisionResult CollideCapsuleOBB(const Capsule& a, const OBB& b);
		static CollisionResult CollideOBBs(const OBB& a, const OBB& b);

		static AABB ComputeAABB(const Sphere& sphere);
		static AABB ComputeAABB(const OBB& obb);
		static AABB ComputeAABB(const Capsule& capsule);
		static bool OverlapAABB(const AABB& a, const AABB& b);
		static AABB UnionAABB(const AABB& a, const AABB& b);
	};
}
