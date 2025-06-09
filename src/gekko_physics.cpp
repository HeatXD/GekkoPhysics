#include "gekko_physics.h"

namespace GekkoPhysics {
	void World::SetOrientation(const Vec3& up) {
		_up = up;
	}

	void World::SetOrigin(const Vec3& origin) {
		_origin = origin;
	}

	Identifier World::CreateBody() {	
		return _bodies.insert({});
	}

	Identifier World::AddShapeGroup(Identifier body_id) {
		if (!_bodies.contains(body_id)) return INVALID_ID;
		
		// no shapegroups? create a link.
		auto& body = _bodies.get(body_id);
		if (body.link_shape_groups == INVALID_ID) {
			body.link_shape_groups = CreateLink();
			// still invalid? return.
			if (body.link_shape_groups == INVALID_ID) {
				return INVALID_ID;
			}
		}

		Identifier group_id = INVALID_ID;
		auto& link = _links.get(body.link_shape_groups);
		for (size_t i = 0; i < 8; i++) {
			if (link.children[i] == INVALID_ID) {
				link.children[i] = _shape_groups.insert({});
				group_id = link.children[i];
				break;
			}
		}

		return group_id;
	}

	Identifier World::AddShape(Identifier shape_group_id, Shape::Type shape_type) {
		if (shape_type == Shape::Type::None || !_shape_groups.contains(shape_group_id)) {
			return INVALID_ID;
		}

		// no shapes? create a link.
		auto& shape_group = _shape_groups.get(shape_group_id);
		if (shape_group.link_shapes == INVALID_ID) {
			shape_group.link_shapes = CreateLink();
			// still invalid? return.
			if (shape_group.link_shapes == INVALID_ID) {
				return INVALID_ID;
			}
		}

		Identifier shape_id = INVALID_ID;
		auto& link = _links.get(shape_group.link_shapes);
		for (size_t i = 0; i < 8; i++) {
			if (link.children[i] == INVALID_ID) {
				auto shape = Shape();

				shape.type = shape_type;
				if (shape.type == Shape::Sphere) {
					shape.shape_type_id = _spheres.insert({});
				} 

				link.children[i] = _shapes.insert(shape);
				shape_id = link.children[i];
				break;
			}
		}

		return shape_id;
	}

	void World::Save(MemStream& stream) {
		_bodies.save(stream);
		_shape_groups.save(stream);
		_shapes.save(stream);
		_links.save(stream);
		_spheres.save(stream);

		stream.write_chunk(&_origin, sizeof(Vec3));
		stream.write_chunk(&_up, sizeof(Vec3));
	}

	void World::Load(MemStream& stream) {
		_bodies.load(stream);
		_shape_groups.load(stream);
		_shapes.load(stream);
		_links.load(stream);
		_spheres.load(stream);

		uint32_t out_size = 0;

		auto data = stream.read_chunk(out_size);
		std::memcpy(&_origin, data, out_size);

		data = stream.read_chunk(out_size);
		std::memcpy(&_up, data, out_size);
	}

	Identifier World::CreateLink() {
		auto link = L1T8();
		link.Reset();
		return _links.insert(link);
	}

	void L1T8::Reset() const {
		std::memset((void*) children, INVALID_ID, 8 * sizeof(Identifier));
	}
}