#pragma once
#include "gekko_math.h"
#include "gekko_ds.h"

namespace Gekko::Physics {
    // ==== Todo / Goals ====
    // - Sphere, Capsule Collision Detection
    // - Sphere, Capsule Collision Resolution
    // - Collision Layers
    // - Collision Events
    // - Modifiable Physics World Origin
    // - Easily Saved and Loaded World State

    struct Object {
        enum Type : uint16_t {
            Sphere = 1 << 0,
            Capsule = 1 << 1,
            AABB = 1 << 2,
        } type;

        int16_t shape_id;
    };

    struct ObjectGroup {
        uint16_t group_layers;
        uint16_t detect_layers;
        uint16_t resolve_layers;
        uint16_t react_layers;

        DS::Vec<int16_t>* object_ids;
    };

    struct Body {
        // used for cleanup later. dont touch.
        int16_t id;

        bool is_static;

        Math::Vec3 position;
        Math::Vec3 velocity;
        Math::Vec3 acceleration;

        DS::Vec<int16_t>* group_ids;
    };

    // collision shapes
    struct Sphere {
        Math::Vec3 position;
        Math::Unit radius;
    };

    struct Capsule {
        Math::Vec3 start, end;
        Math::Unit radius;
    };

    struct AABB {
        Math::Vec3 min, max;
    };

    struct World {
    private:
        Math::Vec3 _origin, _up;

        // physics bodies 
        DS::SparseSet<int16_t, Body> _bodies;

        // collision object groups
        DS::SparseSet<int16_t, ObjectGroup> _groups;

        // collision objects
        DS::SparseSet<int16_t, Object> _objects;

        // collision shapes
        DS::SparseSet<int16_t, Sphere> _spheres;
        DS::SparseSet<int16_t, Capsule> _capsules;
        DS::SparseSet<int16_t, AABB> _aabbs;

        struct CInfo {
            bool collided, swapped;
            // collision depth
            Math::Unit depth;
            // collision normal
            Math::Vec3 normal;
            // contact points
            Math::Vec3 con_a, con_b, con_sym;
        };

        struct CPair {
            uint32_t bodies_hash;
            uint32_t groups_hash;

            CInfo info;
        };

        DS::Vec<CPair> _pairs;

    public:
        World();

        ~World();

        void SetOrientation(const Math::Vec3& up);

        void SetOrigin(const Math::Vec3& origin);

        bool CreateBody(int16_t& new_body_id);

        bool DestroyBody(int16_t body_id);

        bool CreateGroup(int16_t body_id, int16_t& new_group_id);

        bool DestoryGroup(int16_t body_id, int16_t group_id);

        bool AddObject(int16_t group_id, Object::Type type, int16_t& new_object_id);

        bool RemoveObject(int16_t group_id, int16_t object_id);

        bool SetBodyState(int16_t body_id, bool state);

        bool SetGroupState(int16_t group_id, bool state);

        void Update();

    private:
        void DetectPairs();

        void ResolvePairs();

        void ReactPairs();

        void IntegrateBodies();

        uint32_t HashPair(int16_t a, int16_t b);

        void UnhashPair(uint32_t hash, int16_t &a, int16_t &b);

        bool HashContainsId(uint32_t hash, int16_t id);

        void DoGroupsCollide(CPair& pair, const Body& body_a, const Body& body_b, const ObjectGroup& group_a, const ObjectGroup& group_b);

        void CheckSphereSphere(CInfo& info, const Object* obj_a, const Object* obj_b, const Body* body_a, const Body* body_b);

        void CheckSphereCapsule(CInfo& info, const Object* obj_a, const Object* obj_b, const Body* body_a, const Body* body_b);

        void CheckCapsuleCapsule(CInfo& info, const Object* obj_a, const Object* obj_b, const Body* body_a, const Body* body_b);

        void CheckAABBAABB(CInfo& info, const Object* obj_a, const Object* obj_b, const Body* body_a, const Body* body_b);

        void CalculateDepthNorm(CInfo& info, const Math::Unit& distSq, const Math::Unit& radSum, const Math::Vec3& diff);
    };
}
