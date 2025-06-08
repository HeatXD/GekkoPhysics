#include "gekko_physics.h"

namespace GekkoPhysics {
	void World::SetOrientation(const Vec3& up) {
		_up = up;
	}

	void World::SetOrigin(const Vec3& origin) {
		_origin = origin;
	}

	Identifier World::CreateBody(const Body& body) {	
		return _bodies.insert(body);
	}

	void World::RemoveBody(Identifier id) {
		if (!_bodies.contains(id)) return;
		auto& body = _bodies.get(id);

		if (body._rel_shape_group_set != -1) {
			auto& shape_group_set = _relations.get(body._rel_shape_group_set);
			for (Identifier shape_group_id : shape_group_set.children) {
				if (shape_group_id == -1) continue;
				std::cout << shape_group_id << "\n";
			}
		}
	}

	void World::Save(MemStream& stream) {
		_bodies.save(stream);
		_shape_groups.save(stream);
		_shapes.save(stream);
		_relations.save(stream);
		_spheres.save(stream);

		stream.write_chunk(&_origin, sizeof(Vec3));
		stream.write_chunk(&_up, sizeof(Vec3));
	}

	void World::Load(MemStream& stream) {
		_bodies.load(stream);
		_shape_groups.load(stream);
		_shapes.load(stream);
		_relations.load(stream);
		_spheres.load(stream);

		uint32_t out_size = 0;

		auto data = stream.read_chunk(out_size);
		std::memcpy(&_origin, data, out_size);

		data = stream.read_chunk(out_size);
		std::memcpy(&_up, data, out_size);
	}
}