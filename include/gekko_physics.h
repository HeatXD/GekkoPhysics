#pragma once

#include "gekko_ds.h"
#include "gekko_math.h"

namespace GekkoPhysics {
	using Identifier = int16_t;
	using namespace GekkoDS;
	using namespace GekkoMath;

	static const Identifier INVALID_ID = -1;

	struct Sphere {
		Vec3 position;
		Unit radius;
	};

	struct Shape {
		Identifier shape_type_id = INVALID_ID;
		enum Type : uint8_t {
			None,
			Sphere,
		} type = None;
	};

	struct ShapeGroup {
		Identifier link_shapes = INVALID_ID;
	};

	struct Body {
		Vec3 position;
		Vec3 velocity;
		Vec3 acceleration;

		Identifier link_shape_groups = INVALID_ID;
	};

	struct L1T8 {
		Identifier children[8];

		void Reset() const;
	};

	class World {
		SparseSet<Identifier, Body> _bodies;
		SparseSet<Identifier, ShapeGroup> _shape_groups;
		SparseSet<Identifier, Shape> _shapes;
		SparseSet<Identifier, L1T8> _links;
		SparseSet<Identifier, Sphere> _spheres;

		Vec3 _origin = {}, _up = {};

	public:
		void SetOrientation(const Vec3& up);
		void SetOrigin(const Vec3& origin);

		Identifier CreateBody();
		// Adds a shapegroup to a body.
		// Currently limited to 8 shapegroups per body.
		// And 8 shapes per shapegroup.
		Identifier AddShapeGroup(Identifier body_id);
		// Returns a shape containing the selected shape.
		Identifier AddShape(Identifier shape_group_id, Shape::Type shape_type);

		void RemoveBody(Identifier id);

		void Save(MemStream& stream);
		void Load(MemStream& stream);

	private:
		// Create a 1 to 8 link between entites.
		Identifier CreateLink();
	};
}