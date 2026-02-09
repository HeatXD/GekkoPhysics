#pragma once

#include "gekko_ds.h"
#include "gekko_shapes.h"
#include "gekko_debug_draw.h"

namespace GekkoPhysics {
	using Identifier = int16_t;
	using namespace GekkoDS;
	using namespace GekkoMath;

	static const Identifier INVALID_ID = -1;

	struct Shape {
		Identifier shape_type_id = INVALID_ID;
		enum Type : uint8_t {
			None,
			OBB,
			Sphere,
			Capsule,
		} type = None;
	};

	struct ShapeGroup { 
		Identifier owner_body = INVALID_ID;
		Identifier link_shapes = INVALID_ID;
		uint32_t layer = 0, mask = 0;
	};

	struct Body {
		Vec3 position;
		Vec3 velocity;
		Vec3 acceleration;

		Mat3 rotation;

		Identifier link_shape_groups = INVALID_ID;

		bool is_static = false;
	};

	struct Link {
		static const uint8_t NUM_LINKS = 8;
		Identifier children[NUM_LINKS];
		void Reset();
	};

	struct ContactPair {
		Identifier body_a = INVALID_ID;
		Identifier body_b = INVALID_ID;
		Identifier shape_a = INVALID_ID;
		Identifier shape_b = INVALID_ID;
		Vec3 normal;
		Unit depth;
	};

	class World {
		SparseSet<Identifier, Body> _bodies;
		SparseSet<Identifier, ShapeGroup> _shape_groups;
		SparseSet<Identifier, Shape> _shapes;

		SparseSet<Identifier, Link> _links;

		SparseSet<Identifier, OBB> _obbs;
		SparseSet<Identifier, Sphere> _spheres;
		SparseSet<Identifier, Capsule> _capsules;

		Vec<ContactPair> _contacts;

		Vec3 _origin, _up;
		Unit _update_rate { 60 };

		struct GroupAABB {
			Identifier group_id = INVALID_ID;
			AABB aabb;
		};

		Vec<GroupAABB> _group_aabbs;

		DebugDraw* _debug_draw = nullptr;

	public:
		void SetOrientation(const Vec3& up);
		void SetOrigin(const Vec3& origin);

		// Sets the expected number of iterations per second (default 60)
		void SetUpdateRate(const Unit& rate);

		Identifier CreateBody();
		// Adds a shapegroup to a body.
		// Currently limited to NUM_LINK shapegroups per body.
		// And NUM_LINK shapes per shapegroup.
		Identifier AddShapeGroup(Identifier body_id);
		// Returns a shape containing the selected shape.
		Identifier AddShape(Identifier shape_group_id, Shape::Type shape_type);

		void RemoveBody(Identifier id);
		void RemoveShapeGroup(Identifier body_id, Identifier shape_group_id);
		void RemoveShape(Identifier shape_group_id, Identifier shape_id);

		void Save(MemStream& stream);
		void Load(MemStream& stream);

		void Update();

		Body& GetBody(Identifier id);
		ShapeGroup& GetShapeGroup(Identifier id);
		Shape& GetShape(Identifier id);
		Sphere& GetSphere(Identifier id);
		OBB& GetOBB(Identifier id);
		Capsule& GetCapsule(Identifier id);
		const Vec<ContactPair>& GetContacts() const;

		void SetDebugDraw(DebugDraw* dd);
		void DrawDebug() const;

	private:
		// Create a link between entites.
		Identifier CreateLink();

		// Transform body-local shapes to world space.
		Sphere WorldSphere(const Sphere& local, const Body& body) const;
		OBB WorldOBB(const OBB& local, const Body& body) const;
		Capsule WorldCapsule(const Capsule& local, const Body& body) const;

		void CheckCollisions();
		void BuildGroupAABBs();
		bool BroadphaseFilter(const ShapeGroup& group_a, const ShapeGroup& group_b, const AABB& aabb_a, const AABB& aabb_b) const;
		void NarrowphaseGroupPair(const ShapeGroup& group_a, const ShapeGroup& group_b);
		CollisionResult CollideShapes(const Shape& a, const Body& body_a, const Shape& b, const Body& body_b) const;
		AABB ComputeShapeGroupAABB(const ShapeGroup& group, const Body& body) const;
	};
}