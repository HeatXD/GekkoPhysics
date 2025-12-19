#pragma once

#include "gekko_ds.h"
#include "gekko_math.h"

namespace GekkoPhysics {
	using Identifier = int16_t;
	using namespace GekkoDS;
	using namespace GekkoMath;

	static const Identifier INVALID_ID = -1;

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
		OBB bounds;
		Identifier owner_body = INVALID_ID;
		Identifier link_shapes = INVALID_ID;
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

	struct CollisionPair {
		Identifier group_a;
		Identifier group_b;
	};

	class World {
		SparseSet<Identifier, Body> _bodies;
		SparseSet<Identifier, ShapeGroup> _shape_groups;
		SparseSet<Identifier, Shape> _shapes;

		SparseSet<Identifier, Link> _links;

		SparseSet<Identifier, OBB> _obbs;
		SparseSet<Identifier, Sphere> _spheres;
		SparseSet<Identifier, Capsule> _capsules;

		Vec<CollisionPair> _coll_pairs;

		Vec3 _origin, _up;
		Unit _update_rate { 60 };

	public:
		void SetOrientation(const Vec3& up);
		void SetOrigin(const Vec3& origin);

		// Sets the expected number of iterations per second (default 60)
		void SetUpdateRate(const Unit& rate);

		Identifier CreateBody();
		// Adds a shapegroup to a body.
		// Currently limited to 8 shapegroups per body.
		// And 8 shapes per shapegroup.
		Identifier AddShapeGroup(Identifier body_id);
		// Returns a shape containing the selected shape.
		Identifier AddShape(Identifier shape_group_id, Shape::Type shape_type);

		void RemoveBody(Identifier id);
		void RemoveShapeGroup(Identifier body_id, Identifier shape_group_id);
		void RemoveShape(Identifier shape_group_id, Identifier shape_id);

		void Save(MemStream& stream);
		void Load(MemStream& stream);
		void Update();

	private:
		// Create a 1 to 8 link between entites.
		Identifier CreateLink();

		void CheckCollisions();
	};
}