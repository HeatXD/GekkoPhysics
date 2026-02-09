#include "gekko_physics.h"
#include "algo.h"

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
			!_shapes.contains(shape_id) ||
			!_shape_groups.contains(shape_group_id)) return;

		// cleanup shapes link in shapegroup
		auto& shape_group = _shape_groups.get(shape_group_id);
		if (shape_group.link_shapes == INVALID_ID) return;
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

		uint32_t chunk_size = 0;

		auto chunk_data = stream.read_chunk(chunk_size);
		std::memcpy(&_origin, chunk_data, chunk_size);

		chunk_data = stream.read_chunk(chunk_size);
		std::memcpy(&_up, chunk_data, chunk_size);

		chunk_data = stream.read_chunk(chunk_size);
		std::memcpy(&_update_rate, chunk_data, chunk_size);
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

	Body& World::GetBody(Identifier id) {
		return _bodies.get(id);
	}

	ShapeGroup& World::GetShapeGroup(Identifier id) {
		return _shape_groups.get(id);
	}

	Shape& World::GetShape(Identifier id) {
		return _shapes.get(id);
	}

	Sphere& World::GetSphere(Identifier id) {
		return _spheres.get(id);
	}

	OBB& World::GetOBB(Identifier id) {
		return _obbs.get(id);
	}

	Capsule& World::GetCapsule(Identifier id) {
		return _capsules.get(id);
	}

	const Vec<ContactPair>& World::GetContacts() const {
		return _contacts;
	}

	void World::SetDebugDraw(DebugDraw* dd) {
		_debug_draw = dd;
	}

	void World::DrawDebug() const {
		if (!_debug_draw) return;

		const uint32_t flags = _debug_draw->flags;

		// AABBs
		if (flags & DrawFlag_AABBs) {
			for (const auto& ga : _group_aabbs) {
				_debug_draw->DrawAABB(ga.aabb.min.AsFloat(), ga.aabb.max.AsFloat());
			}
		}

		// Iterate bodies
		const uint32_t body_count = _bodies.active_size();
		for (uint32_t body_idx = 0; body_idx < body_count; body_idx++) {
			Identifier body_id = _bodies.entity_id(body_idx);
			const Body& body = _bodies.get(body_id);

			// Body origins
			if (flags & DrawFlag_BodyOrigins) {
				_debug_draw->DrawBodyOrigin(body.position.AsFloat());
			}

			// Body axes
			if (flags & DrawFlag_BodyAxes) {
				_debug_draw->DrawBodyAxes(body.position.AsFloat(), body.rotation.AsFloat());
			}

			// Iterate shape groups
			if (body.link_shape_groups == INVALID_ID) continue;
			const auto& group_link = _links.get(body.link_shape_groups);

			for (size_t group_idx = 0; group_idx < Link::NUM_LINKS; group_idx++) {
				Identifier group_id = group_link.children[group_idx];
				if (group_id == INVALID_ID || !_shape_groups.contains(group_id)) continue;
				const ShapeGroup& group = _shape_groups.get(group_id);

				// Shapes
				if (flags & DrawFlag_Shapes) {
					if (group.link_shapes == INVALID_ID) continue;
					const auto& shape_link = _links.get(group.link_shapes);

					for (size_t shape_idx = 0; shape_idx < Link::NUM_LINKS; shape_idx++) {
						Identifier shape_id = shape_link.children[shape_idx];
						if (shape_id == INVALID_ID || !_shapes.contains(shape_id)) continue;
						const Shape& shape = _shapes.get(shape_id);

						switch (shape.type) {
						case Shape::Sphere: {
							Sphere world_sphere = WorldSphere(_spheres.get(shape.shape_type_id), body);
							_debug_draw->DrawSphere(world_sphere.center.AsFloat(), static_cast<float>(world_sphere.radius));
						} break;
						case Shape::OBB: {
							OBB world_obb = WorldOBB(_obbs.get(shape.shape_type_id), body);
							_debug_draw->DrawBox(world_obb.center.AsFloat(), world_obb.half_extents.AsFloat(), world_obb.rotation.AsFloat());
						} break;
						case Shape::Capsule: {
							Capsule world_capsule = WorldCapsule(_capsules.get(shape.shape_type_id), body);
							_debug_draw->DrawCapsule(world_capsule.start.AsFloat(), world_capsule.end.AsFloat(), static_cast<float>(world_capsule.radius));
						} break;
						default: break;
						}
					}
				}
			}
		}

		// Contacts
		if (flags & DrawFlag_Contacts) {
			for (uint32_t contact_idx = 0; contact_idx < _contacts.size(); contact_idx++) {
				const ContactPair& contact = _contacts[contact_idx];
				Vec3F point = contact.point.AsFloat();
				_debug_draw->DrawPoint(point, 5.0f);

				Vec3F normal = contact.normal.AsFloat();
				_debug_draw->DrawLine(point, Vec3F(point.x + normal.x, point.y + normal.y, point.z + normal.z));
			}
		}
	}

	Sphere World::WorldSphere(const Sphere& local, const Body& body) const {
		Sphere world;
		world.center = body.rotation.TransformPoint(local.center, body.position);
		world.radius = local.radius;
		return world;
	}

	OBB World::WorldOBB(const OBB& local, const Body& body) const {
		OBB world;
		world.center = body.rotation.TransformPoint(local.center, body.position);
		world.rotation = body.rotation * local.rotation;
		world.half_extents = local.half_extents;
		return world;
	}

	Capsule World::WorldCapsule(const Capsule& local, const Body& body) const {
		Capsule world;
		world.start = body.rotation.TransformPoint(local.start, body.position);
		world.end = body.rotation.TransformPoint(local.end, body.position);
		world.radius = local.radius;
		return world;
	}

	static constexpr uint8_t ShapePair(uint8_t a, uint8_t b) {
		return (a << 2) | b;
	}

	CollisionResult World::CollideShapes(const Shape& a, const Body& body_a, const Shape& b, const Body& body_b) const {
		// normalize order so first.type <= second.type (OBB=1 < Sphere=2 < Capsule=3)
		bool swapped = a.type > b.type;
		const Shape& first  = swapped ? b : a;
		const Shape& second = swapped ? a : b;
		const Body& first_body  = swapped ? body_b : body_a;
		const Body& second_body = swapped ? body_a : body_b;

		CollisionResult result;
		switch (ShapePair(first.type, second.type)) {
		case ShapePair(Shape::OBB, Shape::OBB): {
			result = Algo::CollideOBBs(
				WorldOBB(_obbs.get(first.shape_type_id), first_body),
				WorldOBB(_obbs.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::OBB, Shape::Sphere): {
			result = Algo::CollideSphereOBB(
				WorldSphere(_spheres.get(second.shape_type_id), second_body),
				WorldOBB(_obbs.get(first.shape_type_id), first_body));
			swapped = !swapped;
		} break;
		case ShapePair(Shape::OBB, Shape::Capsule): {
			result = Algo::CollideCapsuleOBB(
				WorldCapsule(_capsules.get(second.shape_type_id), second_body),
				WorldOBB(_obbs.get(first.shape_type_id), first_body));
			swapped = !swapped;
		} break;
		case ShapePair(Shape::Sphere, Shape::Sphere): {
			result = Algo::CollideSpheres(
				WorldSphere(_spheres.get(first.shape_type_id), first_body),
				WorldSphere(_spheres.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::Sphere, Shape::Capsule): {
			result = Algo::CollideSphereCapsule(
				WorldSphere(_spheres.get(first.shape_type_id), first_body),
				WorldCapsule(_capsules.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::Capsule, Shape::Capsule): {
			result = Algo::CollideCapsules(
				WorldCapsule(_capsules.get(first.shape_type_id), first_body),
				WorldCapsule(_capsules.get(second.shape_type_id), second_body));
		} break;
		default: return result;
		}

		if (swapped) result.normal = Vec3(Unit{0}, Unit{0}, Unit{0}) - result.normal;
		return result;
	}

	AABB World::ComputeShapeGroupAABB(const ShapeGroup& group, const Body& body) const {
		AABB result;
		bool first = true;

		if (group.link_shapes == INVALID_ID) return result;
		const auto& link = _links.get(group.link_shapes);

		for (size_t i = 0; i < Link::NUM_LINKS; i++) {
			Identifier shape_id = link.children[i];
			if (shape_id == INVALID_ID) continue;
			if (!_shapes.contains(shape_id)) continue;

			const auto& shape = _shapes.get(shape_id);
			AABB shape_aabb;

			switch (shape.type) {
			case Shape::Sphere:
				shape_aabb = Algo::ComputeAABB(WorldSphere(_spheres.get(shape.shape_type_id), body));
				break;
			case Shape::Capsule:
				shape_aabb = Algo::ComputeAABB(WorldCapsule(_capsules.get(shape.shape_type_id), body));
				break;
			case Shape::OBB:
				shape_aabb = Algo::ComputeAABB(WorldOBB(_obbs.get(shape.shape_type_id), body));
				break;
			default:
				continue;
			}

			if (first) {
				result = shape_aabb;
				first = false;
			} else {
				result = Algo::UnionAABB(result, shape_aabb);
			}
		}

		return result;
	}

	void World::CheckCollisions() {
		_contacts.clear();
		_group_aabbs.clear();

		BuildGroupAABBs();

		for (uint32_t i = 0; i < _group_aabbs.size(); i++) {
			const ShapeGroup& group_a = _shape_groups.get(_group_aabbs[i].group_id);
			for (uint32_t j = i + 1; j < _group_aabbs.size(); j++) {
				const ShapeGroup& group_b = _shape_groups.get(_group_aabbs[j].group_id);

				if (!BroadphaseFilter(group_a, group_b, _group_aabbs[i].aabb, _group_aabbs[j].aabb)) continue;

				NarrowphaseGroupPair(group_a, group_b);
			}
		}
	}

	void World::BuildGroupAABBs() {
		const uint32_t group_count = _shape_groups.active_size();
		for (uint32_t i = 0; i < group_count; i++) {
			Identifier group_id = _shape_groups.entity_id(i);
			const ShapeGroup& group = _shape_groups.get(group_id);
			const Body& body = _bodies.get(group.owner_body);

			GroupAABB group_aabb;
			group_aabb.group_id = group_id;
			group_aabb.aabb = ComputeShapeGroupAABB(group, body);
			_group_aabbs.push_back(group_aabb);
		}
	}

	bool World::BroadphaseFilter(const ShapeGroup& group_a, const ShapeGroup& group_b, const AABB& aabb_a, const AABB& aabb_b) const {
		if (group_a.owner_body == group_b.owner_body) return false;
		if ((group_a.layer & group_b.mask) == 0 || (group_b.layer & group_a.mask) == 0) return false;
		if (_bodies.get(group_a.owner_body).is_static && _bodies.get(group_b.owner_body).is_static) return false;
		if (!Algo::OverlapAABB(aabb_a, aabb_b)) return false;
		return true;
	}

	void World::NarrowphaseGroupPair(const ShapeGroup& group_a, const ShapeGroup& group_b) {
		if (group_a.link_shapes == INVALID_ID || group_b.link_shapes == INVALID_ID) return;

		const auto& link_a = _links.get(group_a.link_shapes);
		const auto& link_b = _links.get(group_b.link_shapes);
		const Body& body_a = _bodies.get(group_a.owner_body);
		const Body& body_b = _bodies.get(group_b.owner_body);

		for (size_t shape_idx_a = 0; shape_idx_a < Link::NUM_LINKS; shape_idx_a++) {
			Identifier shape_a_id = link_a.children[shape_idx_a];
			if (shape_a_id == INVALID_ID || !_shapes.contains(shape_a_id)) continue;
			const auto& shape_a = _shapes.get(shape_a_id);

			for (size_t shape_idx_b = 0; shape_idx_b < Link::NUM_LINKS; shape_idx_b++) {
				Identifier shape_b_id = link_b.children[shape_idx_b];
				if (shape_b_id == INVALID_ID || !_shapes.contains(shape_b_id)) continue;
				const auto& shape_b = _shapes.get(shape_b_id);

				auto result = CollideShapes(shape_a, body_a, shape_b, body_b);
				if (result.hit) {
					ContactPair contact;
					contact.body_a = group_a.owner_body;
					contact.body_b = group_b.owner_body;
					contact.shape_a = shape_a_id;
					contact.shape_b = shape_b_id;
					contact.normal = result.normal;
					contact.point = result.point;
					contact.depth = result.depth;
					_contacts.push_back(contact);
				}
			}
		}
	}

	void Link::Reset() {
		for (size_t i = 0; i < NUM_LINKS; i++) {
			children[i] = INVALID_ID;
		}
	}
}
