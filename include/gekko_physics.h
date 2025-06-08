#pragma once

#include "gekko_ds.h"
#include "gekko_math.h"

namespace GekkoPhysics {
	using Identifier = int16_t;
	using namespace GekkoDS;
	using namespace GekkoMath;

	struct Sphere {
		Vec3 position;
		Unit radius;
	};

	struct Shape {
		enum Type : uint8_t {
			None,
			Sphere,
		} type;
	private:
		Identifier _shape_id = -1;
	};

	struct ShapeGroup {
	private:
		Identifier _rel_shapes_set = -1;
	};

	struct Body {
		Vec3 position;
		Vec3 velocity;
		Vec3 acceleration;
	private:
		Identifier _rel_shape_group_set = -1;
	};

	struct R1T8 {
		Identifier children[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
	};

	class World {
		SparseSet<Identifier, Body> _bodies;
		SparseSet<Identifier, ShapeGroup> _shape_groups;
		SparseSet<Identifier, Shape> _shapes;
		SparseSet<Identifier, R1T8> _relations;
		SparseSet<Identifier, Sphere> _spheres;

		Vec3 _origin, _up;

	public:
		void SetOrientation(const Vec3& up);
		void SetOrigin(const Vec3& origin);

		Identifier CreateBody(const Body& body);
		void RemoveBody(Identifier id);

		void Save(MemStream& stream);
		void Load(MemStream& stream);
	};
}