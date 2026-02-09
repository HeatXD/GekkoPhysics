#include "algo.h"

namespace GekkoPhysics {

	Vec3 Algo::ClosestPointOnSegment(const Vec3& point, const Vec3& seg_start, const Vec3& seg_end) {
		Vec3 segment_dir = seg_end - seg_start;
		Unit segment_len_sq = segment_dir.Dot(segment_dir);
		if (segment_len_sq == Unit{0}) return seg_start;

		Unit t = (point - seg_start).Dot(segment_dir) / segment_len_sq;
		t = clamp(t, Unit{0}, Unit{1});
		return seg_start + segment_dir * t;
	}

	void Algo::ClosestPointsBetweenSegments(const Vec3& s1_start, const Vec3& s1_end,
		const Vec3& s2_start, const Vec3& s2_end, Vec3& out1, Vec3& out2)
	{
		Vec3 dir1 = s1_end - s1_start;
		Vec3 dir2 = s2_end - s2_start;
		Vec3 offset = s1_start - s2_start;

		Unit len_sq1 = dir1.Dot(dir1);
		Unit len_sq2 = dir2.Dot(dir2);
		Unit dot_dir2_offset = dir2.Dot(offset);

		if (len_sq1 == Unit{0} && len_sq2 == Unit{0}) {
			out1 = s1_start;
			out2 = s2_start;
			return;
		}

		Unit param1, param2;

		if (len_sq1 == Unit{0}) {
			param1 = Unit{0};
			param2 = clamp(dot_dir2_offset / len_sq2, Unit{0}, Unit{1});
		} else {
			Unit dot_dir1_offset = dir1.Dot(offset);
			if (len_sq2 == Unit{0}) {
				param2 = Unit{0};
				param1 = clamp(-dot_dir1_offset / len_sq1, Unit{0}, Unit{1});
			} else {
				Unit dot_dirs = dir1.Dot(dir2);
				Unit denom = len_sq1 * len_sq2 - dot_dirs * dot_dirs;

				if (denom != Unit{0}) {
					param1 = clamp((dot_dirs * dot_dir2_offset - dot_dir1_offset * len_sq2) / denom, Unit{0}, Unit{1});
				} else {
					param1 = Unit{0};
				}

				param2 = (dot_dirs * param1 + dot_dir2_offset) / len_sq2;

				if (param2 < Unit{0}) {
					param2 = Unit{0};
					param1 = clamp(-dot_dir1_offset / len_sq1, Unit{0}, Unit{1});
				} else if (param2 > Unit{1}) {
					param2 = Unit{1};
					param1 = clamp((dot_dirs - dot_dir1_offset) / len_sq1, Unit{0}, Unit{1});
				}
			}
		}

		out1 = s1_start + dir1 * param1;
		out2 = s2_start + dir2 * param2;
	}

	Vec3 Algo::ClosestPointOnOBB(const Vec3& point, const OBB& obb) {
		Vec3 offset = point - obb.center;
		Vec3 result = obb.center;

		for (int i = 0; i < 3; i++) {
			Unit projection = offset.Dot(obb.rotation.cols[i]);
			Unit half_extent = (i == 0) ? obb.half_extents.x : (i == 1) ? obb.half_extents.y : obb.half_extents.z;
			projection = clamp(projection, Unit{0} - half_extent, half_extent);
			result += obb.rotation.cols[i] * projection;
		}

		return result;
	}

	CollisionResult Algo::CollideSpheres(const Sphere& a, const Sphere& b) {
		CollisionResult result;
		Vec3 between_centers = b.center - a.center;
		Unit distance = length(between_centers);
		Unit sum_radii = a.radius + b.radius;

		result.depth = sum_radii - distance;
		if (result.depth < Unit{0}) return result;

		result.hit = true;
		if (distance == Unit{0}) {
			result.normal = Vec3(Unit{0}, Unit{1}, Unit{0});
		} else {
			result.normal = normalize(between_centers);
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
		Sphere sphere_a;
		sphere_a.center = closest_a;
		sphere_a.radius = a.radius;
		Sphere sphere_b;
		sphere_b.center = closest_b;
		sphere_b.radius = b.radius;
		return CollideSpheres(sphere_a, sphere_b);
	}

	CollisionResult Algo::CollideSphereOBB(const Sphere& a, const OBB& b) {
		CollisionResult result;
		Vec3 closest = ClosestPointOnOBB(a.center, b);
		Vec3 sphere_to_closest = closest - a.center;
		Unit distance_sq = sphere_to_closest.Dot(sphere_to_closest);
		Unit radius_sq = a.radius * a.radius;

		// check if sphere center is inside OBB
		Vec3 local_center = a.center - b.center;
		Unit local_x = GekkoMath::abs(local_center.Dot(b.rotation.cols[0]));
		Unit local_y = GekkoMath::abs(local_center.Dot(b.rotation.cols[1]));
		Unit local_z = GekkoMath::abs(local_center.Dot(b.rotation.cols[2]));
		bool inside = (local_x <= b.half_extents.x) && (local_y <= b.half_extents.y) && (local_z <= b.half_extents.z);

		if (inside) {
			result.hit = true;
			Unit penetration_x = b.half_extents.x - local_x;
			Unit penetration_y = b.half_extents.y - local_y;
			Unit penetration_z = b.half_extents.z - local_z;

			Unit min_penetration = penetration_x;
			int min_axis = 0;
			if (penetration_y < min_penetration) { min_penetration = penetration_y; min_axis = 1; }
			if (penetration_z < min_penetration) { min_penetration = penetration_z; min_axis = 2; }

			Unit axis_sign = local_center.Dot(b.rotation.cols[min_axis]) < Unit{0} ? Unit{-1} : Unit{1};
			result.normal = b.rotation.cols[min_axis] * (Unit{0} - axis_sign);
			result.depth = min_penetration + a.radius;
			return result;
		}

		if (distance_sq > radius_sq) return result;

		Unit distance = sqrt(distance_sq);
		result.hit = true;
		result.depth = a.radius - distance;

		if (distance == Unit{0}) {
			result.normal = Vec3(Unit{0}, Unit{1}, Unit{0});
		} else {
			result.normal = normalize(sphere_to_closest);
		}
		return result;
	}

	CollisionResult Algo::CollideCapsuleOBB(const Capsule& a, const OBB& b) {
		Vec3 closest_on_seg = ClosestPointOnSegment(b.center, a.start, a.end);
		Vec3 closest_on_obb = ClosestPointOnOBB(closest_on_seg, b);

		// refine: find closest point on segment to that OBB point
		closest_on_seg = ClosestPointOnSegment(closest_on_obb, a.start, a.end);

		Sphere sphere;
		sphere.center = closest_on_seg;
		sphere.radius = a.radius;
		return CollideSphereOBB(sphere, b);
	}

	CollisionResult Algo::CollideOBBs(const OBB& a, const OBB& b) {
		CollisionResult result;

		Vec3 axes_a[3] = { a.rotation.cols[0], a.rotation.cols[1], a.rotation.cols[2] };
		Vec3 axes_b[3] = { b.rotation.cols[0], b.rotation.cols[1], b.rotation.cols[2] };

		Vec3 center_offset = b.center - a.center;

		Unit min_overlap = Unit{32000};
		Vec3 min_overlap_axis;

		auto test_axis = [&](const Vec3& axis) -> bool {
			Unit axis_len = length(axis);
			if (axis_len < Unit{0} + (Unit{1} / Unit{1000})) return true;

			Vec3 normalized = axis / axis_len;

			Unit projection_a =
				GekkoMath::abs(axes_a[0].Dot(normalized)) * a.half_extents.x +
				GekkoMath::abs(axes_a[1].Dot(normalized)) * a.half_extents.y +
				GekkoMath::abs(axes_a[2].Dot(normalized)) * a.half_extents.z;

			Unit projection_b =
				GekkoMath::abs(axes_b[0].Dot(normalized)) * b.half_extents.x +
				GekkoMath::abs(axes_b[1].Dot(normalized)) * b.half_extents.y +
				GekkoMath::abs(axes_b[2].Dot(normalized)) * b.half_extents.z;

			Unit center_distance = GekkoMath::abs(center_offset.Dot(normalized));
			Unit overlap = projection_a + projection_b - center_distance;

			if (overlap < Unit{0}) return false;

			if (overlap < min_overlap) {
				min_overlap = overlap;
				if (center_offset.Dot(normalized) < Unit{0}) {
					min_overlap_axis = Vec3(Unit{0}, Unit{0}, Unit{0}) - normalized;
				} else {
					min_overlap_axis = normalized;
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
		result.normal = min_overlap_axis;
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
		Unit extent_x = GekkoMath::abs(obb.rotation.cols[0].x) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].x) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].x) * obb.half_extents.z;
		Unit extent_y = GekkoMath::abs(obb.rotation.cols[0].y) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].y) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].y) * obb.half_extents.z;
		Unit extent_z = GekkoMath::abs(obb.rotation.cols[0].z) * obb.half_extents.x
				+ GekkoMath::abs(obb.rotation.cols[1].z) * obb.half_extents.y
				+ GekkoMath::abs(obb.rotation.cols[2].z) * obb.half_extents.z;

		Vec3 extent(extent_x, extent_y, extent_z);
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
