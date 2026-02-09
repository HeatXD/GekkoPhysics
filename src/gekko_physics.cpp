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

		const uint32_t f = _debug_draw->flags;

		// Colors
		const Color green(0.0f, 1.0f, 0.0f);
		const Color yellow(1.0f, 1.0f, 0.0f);
		const Color red(1.0f, 0.0f, 0.0f);
		const Color orange(1.0f, 0.5f, 0.0f);
		const Color axis_r(1.0f, 0.0f, 0.0f);
		const Color axis_g(0.0f, 1.0f, 0.0f);
		const Color axis_b(0.0f, 0.0f, 1.0f);

		// Iterate bodies
		const uint32_t body_count = _bodies.active_size();
		for (uint32_t bi = 0; bi < body_count; bi++) {
			Identifier body_id = _bodies.entity_id(bi);
			const Body& body = _bodies.get(body_id);

			// Body axes
			if (f & DrawFlag_BodyAxes) {
				Vec3F pos = body.position.AsFloat();
				Mat3F rot = body.rotation.AsFloat();
				_debug_draw->DrawLine(pos, Vec3F(pos.x + rot.cols[0].x, pos.y + rot.cols[0].y, pos.z + rot.cols[0].z), axis_r);
				_debug_draw->DrawLine(pos, Vec3F(pos.x + rot.cols[1].x, pos.y + rot.cols[1].y, pos.z + rot.cols[1].z), axis_g);
				_debug_draw->DrawLine(pos, Vec3F(pos.x + rot.cols[2].x, pos.y + rot.cols[2].y, pos.z + rot.cols[2].z), axis_b);
			}

			// Iterate shape groups
			if (body.link_shape_groups == INVALID_ID) continue;
			const auto& group_link = _links.get(body.link_shape_groups);

			for (size_t gi = 0; gi < Link::NUM_LINKS; gi++) {
				Identifier group_id = group_link.children[gi];
				if (group_id == INVALID_ID || !_shape_groups.contains(group_id)) continue;
				const ShapeGroup& group = _shape_groups.get(group_id);

				// AABB
				if (f & DrawFlag_AABBs) {
					AABB aabb = ComputeShapeGroupAABB(group, body);
					_debug_draw->DrawAABB(aabb.min.AsFloat(), aabb.max.AsFloat(), yellow);
				}

				// Shapes
				if (f & DrawFlag_Shapes) {
					if (group.link_shapes == INVALID_ID) continue;
					const auto& shape_link = _links.get(group.link_shapes);

					for (size_t si = 0; si < Link::NUM_LINKS; si++) {
						Identifier shape_id = shape_link.children[si];
						if (shape_id == INVALID_ID || !_shapes.contains(shape_id)) continue;
						const Shape& shape = _shapes.get(shape_id);

						switch (shape.type) {
						case Shape::Sphere: {
							Sphere ws = WorldSphere(_spheres.get(shape.shape_type_id), body);
							_debug_draw->DrawSphere(ws.center.AsFloat(), static_cast<float>(ws.radius), green);
						} break;
						case Shape::OBB: {
							OBB wo = WorldOBB(_obbs.get(shape.shape_type_id), body);
							_debug_draw->DrawBox(wo.center.AsFloat(), wo.half_extents.AsFloat(), wo.rotation.AsFloat(), green);
						} break;
						case Shape::Capsule: {
							Capsule wc = WorldCapsule(_capsules.get(shape.shape_type_id), body);
							_debug_draw->DrawCapsule(wc.start.AsFloat(), wc.end.AsFloat(), static_cast<float>(wc.radius), green);
						} break;
						default: break;
						}
					}
				}
			}
		}

		// Contacts
		if (f & DrawFlag_Contacts) {
			for (uint32_t ci = 0; ci < _contacts.size(); ci++) {
				const ContactPair& cp = _contacts[ci];
				const Body& ba = _bodies.get(cp.body_a);
				const Body& bb = _bodies.get(cp.body_b);
				Vec3F pa = ba.position.AsFloat();
				Vec3F pb = bb.position.AsFloat();
				Vec3F mid(
					(pa.x + pb.x) * 0.5f,
					(pa.y + pb.y) * 0.5f,
					(pa.z + pb.z) * 0.5f
				);
				_debug_draw->DrawPoint(mid, 5.0f, red);

				Vec3F n = cp.normal.AsFloat();
				_debug_draw->DrawLine(mid, Vec3F(mid.x + n.x, mid.y + n.y, mid.z + n.z), orange);
			}
		}
	}

	Sphere World::WorldSphere(const Sphere& local, const Body& body) const {
		Sphere w;
		w.center = body.position + body.rotation * local.center;
		w.radius = local.radius;
		return w;
	}

	OBB World::WorldOBB(const OBB& local, const Body& body) const {
		OBB w;
		w.center = body.position + body.rotation * local.center;
		w.rotation = body.rotation * local.rotation;
		w.half_extents = local.half_extents;
		return w;
	}

	Capsule World::WorldCapsule(const Capsule& local, const Body& body) const {
		Capsule w;
		w.start = body.position + body.rotation * local.start;
		w.end = body.position + body.rotation * local.end;
		w.radius = local.radius;
		return w;
	}

	static constexpr uint8_t ShapePair(uint8_t a, uint8_t b) {
		return (a << 2) | b;
	}

	CollisionResult World::CollideShapes(const Shape& a, const Body& ba, const Shape& b, const Body& bb) const {
		// normalize order so first.type <= second.type (OBB=1 < Sphere=2 < Capsule=3)
		bool swapped = a.type > b.type;
		const Shape& first  = swapped ? b : a;
		const Shape& second = swapped ? a : b;
		const Body& first_body  = swapped ? bb : ba;
		const Body& second_body = swapped ? ba : bb;

		CollisionResult r;
		switch (ShapePair(first.type, second.type)) {
		case ShapePair(Shape::OBB, Shape::OBB): {
			r = Algo::CollideOBBs(
				WorldOBB(_obbs.get(first.shape_type_id), first_body),
				WorldOBB(_obbs.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::OBB, Shape::Sphere): {
			r = Algo::CollideSphereOBB(
				WorldSphere(_spheres.get(second.shape_type_id), second_body),
				WorldOBB(_obbs.get(first.shape_type_id), first_body));
			swapped = !swapped;
		} break;
		case ShapePair(Shape::OBB, Shape::Capsule): {
			r = Algo::CollideCapsuleOBB(
				WorldCapsule(_capsules.get(second.shape_type_id), second_body),
				WorldOBB(_obbs.get(first.shape_type_id), first_body));
			swapped = !swapped;
		} break;
		case ShapePair(Shape::Sphere, Shape::Sphere): {
			r = Algo::CollideSpheres(
				WorldSphere(_spheres.get(first.shape_type_id), first_body),
				WorldSphere(_spheres.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::Sphere, Shape::Capsule): {
			r = Algo::CollideSphereCapsule(
				WorldSphere(_spheres.get(first.shape_type_id), first_body),
				WorldCapsule(_capsules.get(second.shape_type_id), second_body));
		} break;
		case ShapePair(Shape::Capsule, Shape::Capsule): {
			r = Algo::CollideCapsules(
				WorldCapsule(_capsules.get(first.shape_type_id), first_body),
				WorldCapsule(_capsules.get(second.shape_type_id), second_body));
		} break;
		default: return r;
		}

		if (swapped) r.normal = Vec3(Unit{0}, Unit{0}, Unit{0}) - r.normal;
		return r;
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

		const uint32_t count = _shape_groups.active_size();

		// compute AABBs for all active shape groups
		struct GroupAABB {
			Identifier group_id = INVALID_ID;
			AABB aabb;
		};

		Vec<GroupAABB> group_aabbs;
		for (uint32_t i = 0; i < count; i++) {
			Identifier gid = _shape_groups.entity_id(i);
			const ShapeGroup& g = _shape_groups.get(gid);
			GroupAABB ga;
			ga.group_id = gid;
			ga.aabb = ComputeShapeGroupAABB(g, _bodies.get(g.owner_body));
			group_aabbs.push_back(ga);
		}

		for (uint32_t i = 0; i < group_aabbs.size(); i++) {
			const ShapeGroup& a = _shape_groups.get(group_aabbs[i].group_id);
			for (uint32_t j = i + 1; j < group_aabbs.size(); j++) {
				const ShapeGroup& b = _shape_groups.get(group_aabbs[j].group_id);

				// same-body collisions are not supported.
				if (a.owner_body == b.owner_body) continue;
				// layers and mask check. skip when it dont match or not defined.
				if ((a.layer & b.mask) == 0 || (b.layer & a.mask) == 0) continue;
				// skip static-vs-static
				if (_bodies.get(a.owner_body).is_static && _bodies.get(b.owner_body).is_static) continue;
				// broadphase AABB overlap check
				if (!Algo::OverlapAABB(group_aabbs[i].aabb, group_aabbs[j].aabb)) continue;

				// narrowphase: iterate shape pairs
				if (a.link_shapes == INVALID_ID || b.link_shapes == INVALID_ID) continue;
				const auto& link_a = _links.get(a.link_shapes);
				const auto& link_b = _links.get(b.link_shapes);

				for (size_t si = 0; si < Link::NUM_LINKS; si++) {
					Identifier sa_id = link_a.children[si];
					if (sa_id == INVALID_ID || !_shapes.contains(sa_id)) continue;
					const auto& sa = _shapes.get(sa_id);

					for (size_t sj = 0; sj < Link::NUM_LINKS; sj++) {
						Identifier sb_id = link_b.children[sj];
						if (sb_id == INVALID_ID || !_shapes.contains(sb_id)) continue;
						const auto& sb = _shapes.get(sb_id);

						auto result = CollideShapes(sa, _bodies.get(a.owner_body), sb, _bodies.get(b.owner_body));
						if (result.hit) {
							ContactPair cp;
							cp.body_a = a.owner_body;
							cp.body_b = b.owner_body;
							cp.shape_a = sa_id;
							cp.shape_b = sb_id;
							cp.normal = result.normal;
							cp.depth = result.depth;
							_contacts.push_back(cp);
						}
					}
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