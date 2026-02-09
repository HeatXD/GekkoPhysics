#include "algo.h"

namespace GekkoPhysics {

	Vec3 Algo::ClosestPointOnSegment(const Vec3& point, const Vec3& seg_start, const Vec3& seg_end) {
		Vec3 ab = seg_end - seg_start;
		Unit ab_len_sq = ab.Dot(ab);
		if (ab_len_sq == Unit{0}) return seg_start;

		Unit t = (point - seg_start).Dot(ab) / ab_len_sq;
		t = clamp(t, Unit{0}, Unit{1});
		return seg_start + ab * t;
	}

	void Algo::ClosestPointsBetweenSegments(const Vec3& s1_start, const Vec3& s1_end,
		const Vec3& s2_start, const Vec3& s2_end, Vec3& out1, Vec3& out2)
	{
		Vec3 d1 = s1_end - s1_start;
		Vec3 d2 = s2_end - s2_start;
		Vec3 r = s1_start - s2_start;

		Unit a = d1.Dot(d1);
		Unit e = d2.Dot(d2);
		Unit f = d2.Dot(r);

		if (a == Unit{0} && e == Unit{0}) {
			out1 = s1_start;
			out2 = s2_start;
			return;
		}

		Unit s, t;

		if (a == Unit{0}) {
			s = Unit{0};
			t = clamp(f / e, Unit{0}, Unit{1});
		} else {
			Unit c = d1.Dot(r);
			if (e == Unit{0}) {
				t = Unit{0};
				s = clamp(-c / a, Unit{0}, Unit{1});
			} else {
				Unit b = d1.Dot(d2);
				Unit denom = a * e - b * b;

				if (denom != Unit{0}) {
					s = clamp((b * f - c * e) / denom, Unit{0}, Unit{1});
				} else {
					s = Unit{0};
				}

				t = (b * s + f) / e;

				if (t < Unit{0}) {
					t = Unit{0};
					s = clamp(-c / a, Unit{0}, Unit{1});
				} else if (t > Unit{1}) {
					t = Unit{1};
					s = clamp((b - c) / a, Unit{0}, Unit{1});
				}
			}
		}

		out1 = s1_start + d1 * s;
		out2 = s2_start + d2 * t;
	}

	Vec3 Algo::ClosestPointOnOBB(const Vec3& point, const OBB& obb) {
		Vec3 d = point - obb.center;
		Vec3 result = obb.center;

		for (int i = 0; i < 3; i++) {
			Unit dist = d.Dot(obb.rotation.cols[i]);
			Unit half = (i == 0) ? obb.half_extents.x : (i == 1) ? obb.half_extents.y : obb.half_extents.z;
			dist = clamp(dist, Unit{0} - half, half);
			result += obb.rotation.cols[i] * dist;
		}

		return result;
	}

	CollisionResult Algo::CollideSpheres(const Sphere& a, const Sphere& b) {
		CollisionResult result;
		Vec3 ab = b.center - a.center;
		Unit dist = length(ab);
		Unit sum_radii = a.radius + b.radius;

		result.depth = sum_radii - dist;
		if (result.depth < Unit{0}) return result;

		result.hit = true;
		if (dist == Unit{0}) {
			result.normal = Vec3(Unit{0}, Unit{1}, Unit{0});
		} else {
			result.normal = normalize(ab);
		}
		return result;
	}

	CollisionResult Algo::CollideSphereCapsule(const Sphere& a, const Capsule& b) {
		Vec3 closest = ClosestPointOnSegment(a.center, b.start, b.end);
		Sphere capsule_sphere;
		capsule_sphere.center = closest;
		capsule_sphere.radius = b.radius;
		return CollideSpheres(a, capsule_sphere);
	}

	CollisionResult Algo::CollideCapsules(const Capsule& a, const Capsule& b) {
		Vec3 closest_a, closest_b;
		ClosestPointsBetweenSegments(a.start, a.end, b.start, b.end, closest_a, closest_b);
		Sphere sa;
		sa.center = closest_a;
		sa.radius = a.radius;
		Sphere sb;
		sb.center = closest_b;
		sb.radius = b.radius;
		return CollideSpheres(sa, sb);
	}

	CollisionResult Algo::CollideSphereOBB(const Sphere& a, const OBB& b) {
		CollisionResult result;
		Vec3 closest = ClosestPointOnOBB(a.center, b);
		Vec3 diff = closest - a.center;
		Unit dist_sq = diff.Dot(diff);
		Unit radius_sq = a.radius * a.radius;

		// check if sphere center is inside OBB
		Vec3 local = a.center - b.center;
		Unit lx = GekkoMath::abs(local.Dot(b.rotation.cols[0]));
		Unit ly = GekkoMath::abs(local.Dot(b.rotation.cols[1]));
		Unit lz = GekkoMath::abs(local.Dot(b.rotation.cols[2]));
		bool inside = (lx <= b.half_extents.x) && (ly <= b.half_extents.y) && (lz <= b.half_extents.z);

		if (inside) {
			result.hit = true;
			Unit pen_x = b.half_extents.x - lx;
			Unit pen_y = b.half_extents.y - ly;
			Unit pen_z = b.half_extents.z - lz;

			Unit min_pen = pen_x;
			int min_axis = 0;
			if (pen_y < min_pen) { min_pen = pen_y; min_axis = 1; }
			if (pen_z < min_pen) { min_pen = pen_z; min_axis = 2; }

			Unit sign = local.Dot(b.rotation.cols[min_axis]) < Unit{0} ? Unit{-1} : Unit{1};
			result.normal = b.rotation.cols[min_axis] * (Unit{0} - sign);
			result.depth = min_pen + a.radius;
			return result;
		}

		if (dist_sq > radius_sq) return result;

		Unit dist = sqrt(dist_sq);
		result.hit = true;
		result.depth = a.radius - dist;

		if (dist == Unit{0}) {
			result.normal = Vec3(Unit{0}, Unit{1}, Unit{0});
		} else {
			result.normal = normalize(diff);
		}
		return result;
	}

	CollisionResult Algo::CollideCapsuleOBB(const Capsule& a, const OBB& b) {
		Vec3 closest_on_seg = ClosestPointOnSegment(b.center, a.start, a.end);
		Vec3 closest_on_obb = ClosestPointOnOBB(closest_on_seg, b);

		// refine: find closest point on segment to that OBB point
		closest_on_seg = ClosestPointOnSegment(closest_on_obb, a.start, a.end);

		Sphere s;
		s.center = closest_on_seg;
		s.radius = a.radius;
		return CollideSphereOBB(s, b);
	}

	CollisionResult Algo::CollideOBBs(const OBB& a, const OBB& b) {
		CollisionResult result;

		Vec3 axes_a[3] = { a.rotation.cols[0], a.rotation.cols[1], a.rotation.cols[2] };
		Vec3 axes_b[3] = { b.rotation.cols[0], b.rotation.cols[1], b.rotation.cols[2] };

		Vec3 d = b.center - a.center;

		Unit min_overlap = Unit{32000};
		Vec3 min_axis;

		auto test_axis = [&](const Vec3& axis) -> bool {
			Unit axis_len = length(axis);
			if (axis_len < Unit{0} + (Unit{1} / Unit{1000})) return true;

			Vec3 n = axis / axis_len;

			Unit proj_a =
				GekkoMath::abs(axes_a[0].Dot(n)) * a.half_extents.x +
				GekkoMath::abs(axes_a[1].Dot(n)) * a.half_extents.y +
				GekkoMath::abs(axes_a[2].Dot(n)) * a.half_extents.z;

			Unit proj_b =
				GekkoMath::abs(axes_b[0].Dot(n)) * b.half_extents.x +
				GekkoMath::abs(axes_b[1].Dot(n)) * b.half_extents.y +
				GekkoMath::abs(axes_b[2].Dot(n)) * b.half_extents.z;

			Unit distance = GekkoMath::abs(d.Dot(n));
			Unit overlap = proj_a + proj_b - distance;

			if (overlap < Unit{0}) return false;

			if (overlap < min_overlap) {
				min_overlap = overlap;
				if (d.Dot(n) < Unit{0}) {
					min_axis = Vec3(Unit{0}, Unit{0}, Unit{0}) - n;
				} else {
					min_axis = n;
				}
			}
			return true;
		};

		for (int i = 0; i < 3; i++) {
			if (!test_axis(axes_a[i])) return result;
		}

		for (int i = 0; i < 3; i++) {
			if (!test_axis(axes_b[i])) return result;
		}

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				Vec3 cross = axes_a[i].Cross(axes_b[j]);
				if (!test_axis(cross)) return result;
			}
		}

		result.hit = true;
		result.depth = min_overlap;
		result.normal = min_axis;
		return result;
	}

	AABB Algo::ComputeAABB(const Sphere& sphere) {
		AABB aabb;
		aabb.min = sphere.center - sphere.radius;
		aabb.max = sphere.center + sphere.radius;
		return aabb;
	}

	AABB Algo::ComputeAABB(const OBB& obb) {
		// extent[i] = sum_j |cols[j].component_i| * half_extents[j]
		Unit ex = GekkoMath::abs(obb.rotation.cols[0].x) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].x) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].x) * obb.half_extents.z;
		Unit ey = GekkoMath::abs(obb.rotation.cols[0].y) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].y) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].y) * obb.half_extents.z;
		Unit ez = GekkoMath::abs(obb.rotation.cols[0].z) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].z) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].z) * obb.half_extents.z;

		Vec3 extent(ex, ey, ez);
		AABB aabb;
		aabb.min = obb.center - extent;
		aabb.max = obb.center + extent;
		return aabb;
	}

	AABB Algo::ComputeAABB(const Capsule& capsule) {
		Unit min_x = (capsule.start.x < capsule.end.x ? capsule.start.x : capsule.end.x) - capsule.radius;
		Unit min_y = (capsule.start.y < capsule.end.y ? capsule.start.y : capsule.end.y) - capsule.radius;
		Unit min_z = (capsule.start.z < capsule.end.z ? capsule.start.z : capsule.end.z) - capsule.radius;
		Unit max_x = (capsule.start.x > capsule.end.x ? capsule.start.x : capsule.end.x) + capsule.radius;
		Unit max_y = (capsule.start.y > capsule.end.y ? capsule.start.y : capsule.end.y) + capsule.radius;
		Unit max_z = (capsule.start.z > capsule.end.z ? capsule.start.z : capsule.end.z) + capsule.radius;

		AABB aabb;
		aabb.min = Vec3(min_x, min_y, min_z);
		aabb.max = Vec3(max_x, max_y, max_z);
		return aabb;
	}

	bool Algo::OverlapAABB(const AABB& a, const AABB& b) {
		if (a.max.x < b.min.x || b.max.x < a.min.x) return false;
		if (a.max.y < b.min.y || b.max.y < a.min.y) return false;
		if (a.max.z < b.min.z || b.max.z < a.min.z) return false;
		return true;
	}

	AABB Algo::UnionAABB(const AABB& a, const AABB& b) {
		AABB result;
		result.min = Vec3(
			a.min.x < b.min.x ? a.min.x : b.min.x,
			a.min.y < b.min.y ? a.min.y : b.min.y,
			a.min.z < b.min.z ? a.min.z : b.min.z
		);
		result.max = Vec3(
			a.max.x > b.max.x ? a.max.x : b.max.x,
			a.max.y > b.max.y ? a.max.y : b.max.y,
			a.max.z > b.max.z ? a.max.z : b.max.z
		);
		return result;
	}
}
