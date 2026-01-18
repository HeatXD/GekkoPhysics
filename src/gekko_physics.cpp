#include "gekko_physics.h"

namespace GekkoPhysics {
	void World::SetOrientation(const Vec3& up) {
		_up = up;
	}

	void World::SetOrigin(const Vec3& origin) {
		_origin = origin;
	}

	void World::SetUpdateRate(const Unit& rate) {
		_update_rate = rate;
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
		for (size_t i = 0; i < Link::NUM_LINKS; i++) {
			if (link.children[i] == INVALID_ID) {
				group_id = _shape_groups.insert({});
				_shape_groups.get(group_id).owner_body = body_id;
				link.children[i] = group_id;
				break;
			}
		}

		return group_id;
	}

	Identifier World::AddShape(Identifier shape_group_id, Shape::Type shape_type) {
		if (shape_type == Shape::None || !_shape_groups.contains(shape_group_id)) {
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
		for (size_t i = 0; i < Link::NUM_LINKS; i++) {
			if (link.children[i] == INVALID_ID) {
				auto shape = Shape();
				shape.type = shape_type;

				switch (shape.type) {
				case Shape::OBB:
					shape.shape_type_id = _obbs.insert({});
					break;
				case Shape::Sphere:
					shape.shape_type_id = _spheres.insert({});
					break;
				case Shape::Capsule:
					shape.shape_type_id = _capsules.insert({});
					break;
				}

				link.children[i] = _shapes.insert(shape);
				shape_id = link.children[i];
				break;
			}
		}

		return shape_id;
	}

	void World::RemoveBody(Identifier id) {
		// when removing a body also remove all its links, shapegroups and shapes
		if (!_bodies.contains(id)) return;

		auto& body = _bodies.get(id);
		// cleanup shapegroups
		if (body.link_shape_groups != INVALID_ID) {
			auto& link = _links.get(body.link_shape_groups);
			for (size_t i = 0; i < Link::NUM_LINKS; i++) {
				RemoveShapeGroup(id, link.children[i]);
			}
			_links.remove(body.link_shape_groups);
		}

		// cleanup body
		_bodies.remove(id);
	}

	void World::RemoveShapeGroup(Identifier body_id, Identifier shape_group_id) {
		if (body_id == INVALID_ID ||
			shape_group_id == INVALID_ID ||
			!_shape_groups.contains(shape_group_id)) return;

		// cleanup shapegroup link in body
		auto& body = _bodies.get(body_id);
		auto& shapegroup_link = _links.get(body.link_shape_groups);
		bool found_group = false;
		for (size_t i = 0; i < Link::NUM_LINKS; i++){
			if (shapegroup_link.children[i] == shape_group_id) {
				// mark link as free/open
				shapegroup_link.children[i] = INVALID_ID;
				found_group = true;
				break;
			}
		}

		// no group found in the right body? dont process
		if (!found_group) return;

		// remove shapes within the group
		auto& group = _shape_groups.get(shape_group_id);
		if (group.link_shapes != INVALID_ID) {
			auto& shapes_link = _links.get(group.link_shapes);
			for (size_t i = 0; i < Link::NUM_LINKS; i++) {
				RemoveShape(shape_group_id, shapes_link.children[i]);
			}
			_links.remove(group.link_shapes);
		}
		
		// remove shapegroup
		_shape_groups.remove(shape_group_id);
	}

	void World::RemoveShape(Identifier shape_group_id, Identifier shape_id) {
		if (shape_id == INVALID_ID ||
			shape_group_id == INVALID_ID ||
			!_shapes.contains(shape_id)) return;

		// cleanup shapes link in shapegroup
		auto& shape_group = _shape_groups.get(shape_group_id);
		auto& shapes_link = _links.get(shape_group.link_shapes);
		bool found_shape = false;
		for (size_t i = 0; i < Link::NUM_LINKS; i++) {
			if (shapes_link.children[i] == shape_id) {
				// mark link as free/open
				shapes_link.children[i] = INVALID_ID;
				found_shape = true;
				break;
			}
		}

		// no shape found? return.
		if (!found_shape) return;

		auto& shape = _shapes.get(shape_id);

		// remove collision shape based on shapetype
		switch (shape.type) {
		case Shape::OBB:
			_obbs.remove(shape.shape_type_id);
			break;
		case Shape::Sphere:
			_spheres.remove(shape.shape_type_id);
			break;
		case Shape::Capsule:
			_capsules.remove(shape.shape_type_id);
			break;
		}

		// cleanup shape
		_shapes.remove(shape_id);
	}

	void World::Save(MemStream& stream) {
		_bodies.save(stream);
		_shape_groups.save(stream);
		_shapes.save(stream);

		_links.save(stream);
		
		_obbs.save(stream);
		_spheres.save(stream);
		_capsules.save(stream);

		stream.write_chunk(&_origin, sizeof(Vec3));
		stream.write_chunk(&_up, sizeof(Vec3));
		stream.write_chunk(&_update_rate, sizeof(Unit));
	}

	void World::Load(MemStream& stream) {
		_bodies.load(stream);
		_shape_groups.load(stream);
		_shapes.load(stream);

		_links.load(stream);

		_obbs.load(stream);
		_spheres.load(stream);
		_capsules.load(stream);

		uint32_t out_size = 0;

		auto data = stream.read_chunk(out_size);
		std::memcpy(&_origin, data, out_size);

		data = stream.read_chunk(out_size);
		std::memcpy(&_up, data, out_size);

		data = stream.read_chunk(out_size);
		std::memcpy(&_update_rate, data, out_size);
	}

	void World::Update() {
		const Unit dt = 1 / _update_rate;
		for (auto& body : _bodies) {
			if (body.is_static) continue;
			body.velocity += body.acceleration * dt;
			body.position += body.velocity * dt;
		}

		CheckCollisions();
	}

	Identifier World::CreateLink() {
		auto link = Link();
		link.Reset();
		return _links.insert(link);
	}

	void World::CheckCollisions() {
		const ShapeGroup* groups = _shape_groups.begin();
		const uint32_t count = _shape_groups.active_size();

		// remove stale pairs.
		int ijk = 0;
		for (uint32_t i = 0; i < count; i++) {
			const ShapeGroup& a = groups[i];
			for (uint32_t j = i + 1; j < count; j++) {
				const ShapeGroup& b = groups[j];
				// same-body collisions are not supported.
				if (a.owner_body == b.owner_body) continue;
				// layers and mask check. skip when it dont match or not defined.
				if ((a.layer & b.mask) == 0 || (b.layer & a.mask) == 0) continue;
				// todo impl oob check and then app the found
				ijk++;
			}
		}
		printf("ijk:%d\n", ijk);
	}

	void Link::Reset() {
		for (size_t i = 0; i < NUM_LINKS; i++) {
			children[i] = INVALID_ID;
		}
	}
}