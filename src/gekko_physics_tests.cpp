#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <chrono>
#include <sstream>
#include <vector>
#include <cmath>

#include "gekko_math.h"
#include "gekko_ds.h"
#include "gekko_physics.h"
#include "gekko_debug_draw.h"

using namespace GekkoMath;
using namespace GekkoDS;
using namespace GekkoPhysics;

// ============================================================================
// Vec tests
// ============================================================================

TEST_SUITE("Vec") {
    TEST_CASE("push_back and size") {
        Vec<int> v;
        CHECK(v.size() == 0);
        CHECK(v.empty());

        v.push_back(10);
        CHECK(v.size() == 1);
        CHECK(!v.empty());
        CHECK(v[0] == 10);

        v.push_back(20);
        v.push_back(30);
        CHECK(v.size() == 3);
        CHECK(v[1] == 20);
        CHECK(v[2] == 30);
    }

    TEST_CASE("pop_back") {
        Vec<int> v;
        v.push_back(1);
        v.push_back(2);
        v.push_back(3);

        v.pop_back();
        CHECK(v.size() == 2);

        v.pop_back();
        v.pop_back();
        CHECK(v.size() == 0);

        // pop on empty should not crash
        v.pop_back();
        CHECK(v.size() == 0);
    }

    TEST_CASE("back") {
        Vec<int> v;
        v.push_back(5);
        CHECK(v.back() == 5);

        v.push_back(99);
        CHECK(v.back() == 99);
    }

    TEST_CASE("remove_at swaps with last") {
        Vec<int> v;
        v.push_back(10);
        v.push_back(20);
        v.push_back(30);

        v.remove_at(0); // swaps 10 with 30, then pops
        CHECK(v.size() == 2);
        CHECK(v[0] == 30);
        CHECK(v[1] == 20);
    }

    TEST_CASE("remove_at last element") {
        Vec<int> v;
        v.push_back(10);
        v.push_back(20);

        v.remove_at(1);
        CHECK(v.size() == 1);
        CHECK(v[0] == 10);
    }

    TEST_CASE("remove_at out of bounds is safe") {
        Vec<int> v;
        v.push_back(1);
        v.remove_at(5);
        CHECK(v.size() == 1);
    }

    TEST_CASE("remove_first") {
        Vec<int> v;
        v.push_back(1);
        v.push_back(2);
        v.push_back(3);

        v.remove_first(2); // swaps 2 with 3, pops
        CHECK(v.size() == 2);
        CHECK(v.contains(1));
        CHECK(v.contains(3));
        CHECK(!v.contains(2));
    }

    TEST_CASE("remove_first nonexistent value is safe") {
        Vec<int> v;
        v.push_back(1);
        v.remove_first(99);
        CHECK(v.size() == 1);
    }

    TEST_CASE("contains") {
        Vec<int> v;
        CHECK(!v.contains(0));

        v.push_back(42);
        CHECK(v.contains(42));
        CHECK(!v.contains(43));
    }

    TEST_CASE("clear") {
        Vec<int> v;
        v.push_back(1);
        v.push_back(2);
        v.clear();
        CHECK(v.size() == 0);
        CHECK(v.empty());
    }

    TEST_CASE("push_back_range from another Vec") {
        Vec<int> a;
        a.push_back(1);
        a.push_back(2);

        Vec<int> b;
        b.push_back(3);
        b.push_back(4);

        a.push_back_range(b);
        CHECK(a.size() == 4);
        CHECK(a[0] == 1);
        CHECK(a[1] == 2);
        CHECK(a[2] == 3);
        CHECK(a[3] == 4);
    }

    TEST_CASE("push_back_range from raw pointer") {
        Vec<int> v;
        int data[] = { 10, 20, 30 };
        v.push_back_range(data, 3);
        CHECK(v.size() == 3);
        CHECK(v[0] == 10);
        CHECK(v[1] == 20);
        CHECK(v[2] == 30);
    }

    TEST_CASE("iterator") {
        Vec<int> v;
        v.push_back(1);
        v.push_back(2);
        v.push_back(3);

        int sum = 0;
        for (int val : v) {
            sum += val;
        }
        CHECK(sum == 6);
    }

    TEST_CASE("capacity grows") {
        Vec<int> v;
        for (int i = 0; i < 100; i++) {
            v.push_back(i);
        }
        CHECK(v.size() == 100);
        CHECK(v.capacity() >= 100);
        CHECK(v[0] == 0);
        CHECK(v[99] == 99);
    }

    TEST_CASE("push_back after clear reuses capacity") {
        Vec<int> v;
        for (int i = 0; i < 20; i++) v.push_back(i);
        uint32_t cap = v.capacity();
        v.clear();
        CHECK(v.capacity() == cap);

        for (int i = 100; i < 110; i++) v.push_back(i);
        CHECK(v.size() == 10);
        CHECK(v[0] == 100);
        CHECK(v[9] == 109);
    }
}

// ============================================================================
// MemStream tests
// ============================================================================

TEST_SUITE("MemStream") {
    TEST_CASE("write and read multiple chunks") {
        MemStream stream;
        int a = 111, b = 222;
        stream.write_chunk(&a, sizeof(a));
        stream.write_chunk(&b, sizeof(b));

        stream.rewind();

        uint32_t out_size = 0;
        auto data = stream.read_chunk(out_size);
        REQUIRE(data != nullptr);
        CHECK(out_size == sizeof(int));
        int ra;
        std::memcpy(&ra, data, sizeof(int));
        CHECK(ra == 111);

        data = stream.read_chunk(out_size);
        REQUIRE(data != nullptr);
        int rb;
        std::memcpy(&rb, data, sizeof(int));
        CHECK(rb == 222);
    }

    TEST_CASE("read past end returns nullptr") {
        MemStream stream;
        int val = 1;
        stream.write_chunk(&val, sizeof(val));
        stream.rewind();

        uint32_t out_size = 0;
        stream.read_chunk(out_size); // consume the one chunk
        auto data = stream.read_chunk(out_size); // nothing left
        CHECK(data == nullptr);
    }

    TEST_CASE("rewind and tell") {
        MemStream stream;
        CHECK(stream.tell() == 0);

        int val = 42;
        stream.write_chunk(&val, sizeof(val));
        CHECK(stream.tell() > 0);

        stream.rewind();
        CHECK(stream.tell() == 0);
    }

    TEST_CASE("seek") {
        MemStream stream;
        int val = 42;
        stream.write_chunk(&val, sizeof(val));

        size_t after_write = stream.tell();
        stream.rewind();
        CHECK(stream.tell() == 0);

        stream.seek(after_write);
        CHECK(stream.tell() == after_write);
    }

    TEST_CASE("external buffer") {
        Vec<uint8_t> buffer;
        MemStream stream(&buffer);

        int val = 999;
        stream.write_chunk(&val, sizeof(val));
        CHECK(buffer.size() > 0);

        stream.rewind();
        uint32_t out_size = 0;
        auto data = stream.read_chunk(out_size);
        REQUIRE(data != nullptr);
        int result;
        std::memcpy(&result, data, sizeof(int));
        CHECK(result == 999);
    }

    TEST_CASE("read from empty stream returns nullptr") {
        MemStream stream;
        uint32_t out_size = 0;
        auto data = stream.read_chunk(out_size);
        CHECK(data == nullptr);
    }

    TEST_CASE("write zero-size chunk") {
        MemStream stream;
        int dummy = 0;
        stream.write_chunk(&dummy, 0);

        stream.rewind();
        uint32_t out_size = 99;
        auto data = stream.read_chunk(out_size);
        REQUIRE(data != nullptr);
        CHECK(out_size == 0);
    }

    TEST_CASE("seek beyond size clamps to end") {
        MemStream stream;
        int val = 1;
        stream.write_chunk(&val, sizeof(val));

        stream.seek(999999);
        CHECK(stream.tell() == stream.size());
    }
}

// ============================================================================
// SparseSet tests
// ============================================================================

TEST_SUITE("SparseSet") {
    TEST_CASE("insert and get") {
        SparseSet<int16_t, int> set;
        auto id = set.insert(42);
        CHECK(id != SparseSet<int16_t, int>::INVALID_ID);
        CHECK(set.get(id) == 42);
        CHECK(set.size() == 1);
        CHECK(set.active_size() == 1);
    }

    TEST_CASE("multiple inserts get sequential ids") {
        SparseSet<int16_t, int> set;
        auto id0 = set.insert(10);
        auto id1 = set.insert(20);
        auto id2 = set.insert(30);
        CHECK(id0 == 0);
        CHECK(id1 == 1);
        CHECK(id2 == 2);
        CHECK(set.get(id0) == 10);
        CHECK(set.get(id1) == 20);
        CHECK(set.get(id2) == 30);
    }

    TEST_CASE("contains and is_valid") {
        SparseSet<int16_t, int> set;
        auto id = set.insert(1);
        CHECK(set.contains(id));
        CHECK(set.is_valid(id));
        CHECK(!set.contains(99));
        CHECK(!set.is_valid(-1));
    }

    TEST_CASE("remove") {
        SparseSet<int16_t, int> set;
        auto id0 = set.insert(10);
        auto id1 = set.insert(20);

        set.remove(id0);
        CHECK(!set.contains(id0));
        CHECK(set.contains(id1));
        CHECK(set.size() == 1);
        CHECK(set.get(id1) == 20);
    }

    TEST_CASE("remove invalid id is safe") {
        SparseSet<int16_t, int> set;
        set.remove(-1);
        set.remove(0);
        CHECK(set.size() == 0);
    }

    TEST_CASE("enable and disable") {
        SparseSet<int16_t, int> set;
        auto id = set.insert(42);
        CHECK(set.is_enabled(id));
        CHECK(set.active_size() == 1);
        CHECK(set.disabled_size() == 0);

        set.disable(id);
        CHECK(!set.is_enabled(id));
        CHECK(set.is_valid(id));
        CHECK(set.active_size() == 0);
        CHECK(set.disabled_size() == 1);

        set.enable(id);
        CHECK(set.is_enabled(id));
        CHECK(set.active_size() == 1);
    }

    TEST_CASE("iterators only cover active elements") {
        SparseSet<int16_t, int> set;
        set.insert(10);
        auto id1 = set.insert(20);
        set.insert(30);

        set.disable(id1);

        int sum = 0;
        for (auto it = set.begin(); it != set.end(); ++it) {
            sum += *it;
        }
        CHECK(set.active_size() == 2);
        CHECK(sum == 40);
    }

    TEST_CASE("clear") {
        SparseSet<int16_t, int> set;
        set.insert(1);
        set.insert(2);
        set.insert(3);

        set.clear();
        CHECK(set.size() == 0);
        CHECK(set.active_size() == 0);
    }

    TEST_CASE("get invalid id throws") {
        SparseSet<int16_t, int> set;
        CHECK_THROWS_AS(set.get(0), std::out_of_range);
        CHECK_THROWS_AS(set.get(-1), std::out_of_range);
    }

    TEST_CASE("save and load roundtrip") {
        SparseSet<int16_t, int> original;
        auto id0 = original.insert(100);
        auto id1 = original.insert(200);
        auto id2 = original.insert(300);
        original.disable(id1);

        MemStream stream;
        original.save(stream);
        stream.rewind();

        SparseSet<int16_t, int> loaded;
        loaded.load(stream);

        CHECK(loaded.size() == 3);
        CHECK(loaded.active_size() == 2);
        CHECK(loaded.get(id0) == 100);
        CHECK(loaded.get(id1) == 200);
        CHECK(loaded.get(id2) == 300);
        CHECK(loaded.is_enabled(id0));
        CHECK(!loaded.is_enabled(id1));
        CHECK(loaded.is_enabled(id2));
    }

    TEST_CASE("remove a disabled entity") {
        SparseSet<int16_t, int> set;
        auto id0 = set.insert(10);
        auto id1 = set.insert(20);
        auto id2 = set.insert(30);

        set.disable(id1);
        CHECK(set.active_size() == 2);
        CHECK(set.disabled_size() == 1);

        set.remove(id1);
        CHECK(set.size() == 2);
        CHECK(set.active_size() == 2);
        CHECK(set.disabled_size() == 0);
        CHECK(!set.contains(id1));
        CHECK(set.contains(id0));
        CHECK(set.contains(id2));
    }

    TEST_CASE("disable already disabled is a no-op") {
        SparseSet<int16_t, int> set;
        auto id = set.insert(42);
        set.disable(id);
        CHECK(set.active_size() == 0);

        set.disable(id); // second disable
        CHECK(set.active_size() == 0);
        CHECK(set.disabled_size() == 1);
        CHECK(set.is_valid(id));
    }

    TEST_CASE("enable already enabled is a no-op") {
        SparseSet<int16_t, int> set;
        auto id = set.insert(42);
        CHECK(set.active_size() == 1);

        set.enable(id); // already enabled
        CHECK(set.active_size() == 1);
        CHECK(set.disabled_size() == 0);
    }

    TEST_CASE("interleaved insert remove enable disable") {
        SparseSet<int16_t, int> set;
        auto a = set.insert(1);
        auto b = set.insert(2);
        auto c = set.insert(3);
        auto d = set.insert(4);

        set.disable(b);
        set.disable(d);
        CHECK(set.active_size() == 2);
        CHECK(set.disabled_size() == 2);

        set.remove(a);
        CHECK(set.active_size() == 1);
        CHECK(set.size() == 3);

        set.enable(b);
        CHECK(set.active_size() == 2);

        // insert should reuse id 'a'
        auto e = set.insert(5);
        CHECK(e == a);
        CHECK(set.get(e) == 5);
        CHECK(set.active_size() == 3);
        CHECK(set.size() == 4);

        // verify all remaining data accessible
        CHECK(set.get(b) == 2);
        CHECK(set.get(c) == 3);
        CHECK(set.get(d) == 4);
        CHECK(set.get(e) == 5);
    }

    TEST_CASE("multiple removes then inserts reuse ids") {
        SparseSet<int16_t, int> set;
        auto id0 = set.insert(10);
        auto id1 = set.insert(20);
        auto id2 = set.insert(30);

        set.remove(id0);
        set.remove(id2);
        CHECK(set.size() == 1);

        // free_ids stack: [id0, id2] - pops from back
        auto new1 = set.insert(40);
        CHECK(new1 == id2);
        auto new2 = set.insert(50);
        CHECK(new2 == id0);

        CHECK(set.get(new1) == 40);
        CHECK(set.get(new2) == 50);
        CHECK(set.get(id1) == 20);
    }

    TEST_CASE("remove all then reinsert") {
        SparseSet<int16_t, int> set;
        auto a = set.insert(1);
        auto b = set.insert(2);
        auto c = set.insert(3);

        set.remove(a);
        set.remove(b);
        set.remove(c);
        CHECK(set.size() == 0);
        CHECK(set.active_size() == 0);

        auto d = set.insert(99);
        CHECK(set.size() == 1);
        CHECK(set.active_size() == 1);
        CHECK(set.get(d) == 99);
    }
}

// ============================================================================
// Vec3 math tests
// ============================================================================

TEST_SUITE("Vec3") {
    TEST_CASE("default construction") {
        Vec3 v;
        CHECK(v.x == Unit{0});
        CHECK(v.y == Unit{0});
        CHECK(v.z == Unit{0});
    }

    TEST_CASE("addition") {
        Vec3 a(Unit{1}, Unit{2}, Unit{3});
        Vec3 b(Unit{4}, Unit{5}, Unit{6});
        Vec3 c = a + b;
        CHECK(c.x == Unit{5});
        CHECK(c.y == Unit{7});
        CHECK(c.z == Unit{9});
    }

    TEST_CASE("subtraction") {
        Vec3 a(Unit{5}, Unit{7}, Unit{9});
        Vec3 b(Unit{1}, Unit{2}, Unit{3});
        Vec3 c = a - b;
        CHECK(c.x == Unit{4});
        CHECK(c.y == Unit{5});
        CHECK(c.z == Unit{6});
    }

    TEST_CASE("scalar addition") {
        Vec3 a(Unit{1}, Unit{2}, Unit{3});
        Vec3 b = a + Unit{10};
        CHECK(b.x == Unit{11});
        CHECK(b.y == Unit{12});
        CHECK(b.z == Unit{13});
    }

    TEST_CASE("scalar subtraction") {
        Vec3 a(Unit{10}, Unit{20}, Unit{30});
        Vec3 b = a - Unit{5};
        CHECK(b.x == Unit{5});
        CHECK(b.y == Unit{15});
        CHECK(b.z == Unit{25});
    }

    TEST_CASE("component-wise multiplication") {
        Vec3 a(Unit{2}, Unit{3}, Unit{4});
        Vec3 b(Unit{5}, Unit{6}, Unit{7});
        Vec3 c = a * b;
        CHECK(c.x == Unit{10});
        CHECK(c.y == Unit{18});
        CHECK(c.z == Unit{28});
    }

    TEST_CASE("scalar multiplication") {
        Vec3 a(Unit{2}, Unit{3}, Unit{4});
        Vec3 b = a * Unit{3};
        CHECK(b.x == Unit{6});
        CHECK(b.y == Unit{9});
        CHECK(b.z == Unit{12});
    }

    TEST_CASE("component-wise division") {
        Vec3 a(Unit{10}, Unit{20}, Unit{30});
        Vec3 b(Unit{2}, Unit{5}, Unit{10});
        Vec3 c = a / b;
        CHECK(c.x == Unit{5});
        CHECK(c.y == Unit{4});
        CHECK(c.z == Unit{3});
    }

    TEST_CASE("scalar division") {
        Vec3 a(Unit{10}, Unit{20}, Unit{30});
        Vec3 b = a / Unit{10};
        CHECK(b.x == Unit{1});
        CHECK(b.y == Unit{2});
        CHECK(b.z == Unit{3});
    }

    TEST_CASE("dot product") {
        Vec3 a(Unit{1}, Unit{2}, Unit{3});
        Vec3 b(Unit{4}, Unit{5}, Unit{6});
        // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
        CHECK(a.Dot(b) == Unit{32});
    }

    TEST_CASE("cross product") {
        Vec3 x(Unit{1}, Unit{0}, Unit{0});
        Vec3 y(Unit{0}, Unit{1}, Unit{0});
        Vec3 z = x.Cross(y);
        CHECK(z.x == Unit{0});
        CHECK(z.y == Unit{0});
        CHECK(z.z == Unit{1});
    }

    TEST_CASE("equality and inequality") {
        Vec3 a(Unit{1}, Unit{2}, Unit{3});
        Vec3 b(Unit{1}, Unit{2}, Unit{3});
        Vec3 c(Unit{1}, Unit{2}, Unit{4});
        CHECK(a == b);
        CHECK(a != c);
    }
}

// ============================================================================
// Mat3 tests
// ============================================================================

TEST_SUITE("Mat3") {
    TEST_CASE("identity matrix") {
        Mat3 m;
        CHECK(m.cols[0] == Vec3(Unit{1}, Unit{0}, Unit{0}));
        CHECK(m.cols[1] == Vec3(Unit{0}, Unit{1}, Unit{0}));
        CHECK(m.cols[2] == Vec3(Unit{0}, Unit{0}, Unit{1}));
    }

    TEST_CASE("matrix-vector multiply") {
        // 90-degree rotation around X axis: y->z, z->-y
        Mat3 rotX(
            Vec3(Unit{1}, Unit{0}, Unit{0}),
            Vec3(Unit{0}, Unit{0}, Unit{1}),
            Vec3(Unit{0}, Unit{-1}, Unit{0})
        );
        Vec3 v(Unit{0}, Unit{1}, Unit{0});
        Vec3 result = rotX * v;
        CHECK(result.x == Unit{0});
        CHECK(result.y == Unit{0});
        CHECK(result.z == Unit{1});
    }

    TEST_CASE("equality") {
        Mat3 a, b;
        CHECK(a == b);
    }
}

// ============================================================================
// Math utility tests
// ============================================================================

TEST_SUITE("Math Utils") {
    TEST_CASE("abs") {
        CHECK(GekkoMath::abs(Unit{5}) == Unit{5});
        CHECK(GekkoMath::abs(Unit{-5}) == Unit{5});
        CHECK(GekkoMath::abs(Unit{0}) == Unit{0});
    }

    TEST_CASE("clamp") {
        CHECK(GekkoMath::clamp(Unit{5}, Unit{0}, Unit{10}) == Unit{5});
        CHECK(GekkoMath::clamp(Unit{-1}, Unit{0}, Unit{10}) == Unit{0});
        CHECK(GekkoMath::clamp(Unit{15}, Unit{0}, Unit{10}) == Unit{10});
        CHECK(GekkoMath::clamp(Unit{0}, Unit{0}, Unit{10}) == Unit{0});
        CHECK(GekkoMath::clamp(Unit{10}, Unit{0}, Unit{10}) == Unit{10});
    }

    TEST_CASE("length of unit vectors") {
        Vec3 x(Unit{1}, Unit{0}, Unit{0});
        CHECK(GekkoMath::length(x) == Unit{1});

        Vec3 y(Unit{0}, Unit{1}, Unit{0});
        CHECK(GekkoMath::length(y) == Unit{1});

        Vec3 z(Unit{0}, Unit{0}, Unit{1});
        CHECK(GekkoMath::length(z) == Unit{1});
    }

    TEST_CASE("length of zero vector") {
        Vec3 zero;
        CHECK(GekkoMath::length(zero) == Unit{0});
    }

    TEST_CASE("sqrt") {
        CHECK(GekkoMath::sqrt(Unit{4}) == Unit{2});
        CHECK(GekkoMath::sqrt(Unit{1}) == Unit{1});
        CHECK(GekkoMath::sqrt(Unit{0}) == Unit{0});
    }
}

// ============================================================================
// Link tests
// ============================================================================

TEST_SUITE("Link") {
    TEST_CASE("reset sets all children to INVALID_ID") {
        Link link;
        link.Reset();
        for (int i = 0; i < Link::NUM_LINKS; i++) {
            CHECK(link.children[i] == INVALID_ID);
        }
    }
}

// ============================================================================
// World tests
// ============================================================================

TEST_SUITE("World") {
    TEST_CASE("create multiple bodies returns unique ids") {
        World world;
        auto id0 = world.CreateBody();
        auto id1 = world.CreateBody();
        auto id2 = world.CreateBody();
        CHECK(id0 != id1);
        CHECK(id1 != id2);
        CHECK(id0 != id2);
    }

    TEST_CASE("add shape group to invalid body returns INVALID_ID") {
        World world;
        auto group = world.AddShapeGroup(INVALID_ID);
        CHECK(group == INVALID_ID);

        auto group2 = world.AddShapeGroup(999);
        CHECK(group2 == INVALID_ID);
    }

    TEST_CASE("add multiple shape groups to body") {
        World world;
        auto body = world.CreateBody();
        Identifier groups[Link::NUM_LINKS];
        for (int i = 0; i < Link::NUM_LINKS; i++) {
            groups[i] = world.AddShapeGroup(body);
            CHECK(groups[i] != INVALID_ID);
        }

        // 9th should fail (limit is NUM_LINKS = 8)
        auto overflow = world.AddShapeGroup(body);
        CHECK(overflow == INVALID_ID);
    }

    TEST_CASE("add shape to shape group") {
        World world;
        auto body = world.CreateBody();
        auto group = world.AddShapeGroup(body);

        auto obb = world.AddShape(group, Shape::OBB);
        CHECK(obb != INVALID_ID);

        auto sphere = world.AddShape(group, Shape::Sphere);
        CHECK(sphere != INVALID_ID);

        auto capsule = world.AddShape(group, Shape::Capsule);
        CHECK(capsule != INVALID_ID);
    }

    TEST_CASE("add shape with None type returns INVALID_ID") {
        World world;
        auto body = world.CreateBody();
        auto group = world.AddShapeGroup(body);
        auto shape = world.AddShape(group, Shape::None);
        CHECK(shape == INVALID_ID);
    }

    TEST_CASE("add shape to invalid group returns INVALID_ID") {
        World world;
        auto shape = world.AddShape(INVALID_ID, Shape::Sphere);
        CHECK(shape == INVALID_ID);
    }

    TEST_CASE("remove body with no shape groups") {
        World world;
        auto body = world.CreateBody();
        world.RemoveBody(body);
        auto group = world.AddShapeGroup(body);
        CHECK(group == INVALID_ID);
    }

    TEST_CASE("remove invalid body is safe") {
        World world;
        world.RemoveBody(INVALID_ID);
        world.RemoveBody(999);
    }

    TEST_CASE("save and load roundtrip") {
        World world1;
        for (int i = 0; i < 5; i++) {
            auto body = world1.CreateBody();
            for (int j = 0; j < 3; j++) {
                auto group = world1.AddShapeGroup(body);
                world1.AddShape(group, Shape::OBB);
                world1.AddShape(group, Shape::Sphere);
                world1.AddShape(group, Shape::Capsule);
            }
        }

        MemStream stream;
        world1.Save(stream);
        stream.rewind();

        World world2;
        world2.Load(stream);

        // Save world2 and compare binary output
        MemStream stream1, stream2;
        world1.Save(stream1);
        world2.Save(stream2);
        CHECK(stream1.size() == stream2.size());
    }

    TEST_CASE("update does not crash on empty world") {
        World world;
        world.Update();
    }

    TEST_CASE("remove shape group then add new one to same body") {
        World world;
        auto body = world.CreateBody();

        // fill all 8 slots
        Identifier groups[Link::NUM_LINKS];
        for (int i = 0; i < Link::NUM_LINKS; i++) {
            groups[i] = world.AddShapeGroup(body);
            REQUIRE(groups[i] != INVALID_ID);
        }

        // full - can't add more
        CHECK(world.AddShapeGroup(body) == INVALID_ID);

        // remove one in the middle
        world.RemoveShapeGroup(body, groups[3]);

        // should be able to add one more now (reuses the freed slot)
        auto new_group = world.AddShapeGroup(body);
        CHECK(new_group != INVALID_ID);

        // but still full again
        CHECK(world.AddShapeGroup(body) == INVALID_ID);
    }

    TEST_CASE("remove shape then add new one to same group") {
        World world;
        auto body = world.CreateBody();
        auto group = world.AddShapeGroup(body);

        // fill all 8 shape slots
        Identifier shapes[Link::NUM_LINKS];
        for (int i = 0; i < Link::NUM_LINKS; i++) {
            shapes[i] = world.AddShape(group, Shape::Sphere);
            REQUIRE(shapes[i] != INVALID_ID);
        }

        CHECK(world.AddShape(group, Shape::OBB) == INVALID_ID);

        // remove one in the middle
        world.RemoveShape(group, shapes[4]);

        // should be able to add again
        auto new_shape = world.AddShape(group, Shape::Capsule);
        CHECK(new_shape != INVALID_ID);

        // full again
        CHECK(world.AddShape(group, Shape::OBB) == INVALID_ID);
    }

    TEST_CASE("remove middle body of several") {
        World world;
        auto b0 = world.CreateBody();
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        auto g0 = world.AddShapeGroup(b0);
        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.AddShape(g0, Shape::Sphere);
        world.AddShape(g1, Shape::OBB);
        world.AddShape(g2, Shape::Capsule);

        // remove the middle body
        world.RemoveBody(b1);

        // b0 and b2 should still work
        auto g0_new = world.AddShapeGroup(b0);
        CHECK(g0_new != INVALID_ID);
        auto g2_new = world.AddShapeGroup(b2);
        CHECK(g2_new != INVALID_ID);

        // b1 should be gone
        CHECK(world.AddShapeGroup(b1) == INVALID_ID);
    }

    TEST_CASE("double remove is safe") {
        World world;
        auto body = world.CreateBody();
        auto group = world.AddShapeGroup(body);
        auto shape = world.AddShape(group, Shape::Sphere);

        world.RemoveShape(group, shape);
        world.RemoveShape(group, shape);

        world.RemoveShapeGroup(body, group);
        world.RemoveShapeGroup(body, group);

        world.RemoveBody(body);
        world.RemoveBody(body);
    }

    TEST_CASE("remove shape with wrong group id does nothing") {
        World world;
        auto body = world.CreateBody();
        auto group_a = world.AddShapeGroup(body);
        auto group_b = world.AddShapeGroup(body);
        auto shape = world.AddShape(group_a, Shape::Sphere);

        // try removing shape from the wrong group
        world.RemoveShape(group_b, shape);

        // fill group_a to prove the shape wasn't removed
        int added = 0;
        for (int i = 0; i < Link::NUM_LINKS; i++) {
            auto s = world.AddShape(group_a, Shape::Sphere);
            if (s != INVALID_ID) added++;
        }
        // we already had 1 shape, so only 7 more should fit
        CHECK(added == 7);
    }

    TEST_CASE("save load then continue mutating") {
        World world1;
        auto body = world1.CreateBody();
        auto group = world1.AddShapeGroup(body);
        world1.AddShape(group, Shape::Sphere);
        world1.AddShape(group, Shape::OBB);

        MemStream stream;
        world1.Save(stream);
        stream.rewind();

        World world2;
        world2.Load(stream);

        // the loaded world should support further mutations
        auto new_body = world2.CreateBody();
        CHECK(new_body != INVALID_ID);

        auto new_group = world2.AddShapeGroup(new_body);
        CHECK(new_group != INVALID_ID);

        auto new_shape = world2.AddShape(new_group, Shape::Capsule);
        CHECK(new_shape != INVALID_ID);

        // and removal
        world2.RemoveBody(body);
        CHECK(world2.AddShapeGroup(body) == INVALID_ID);

        // update should still work
        world2.Update();
    }

    TEST_CASE("remove all bodies then create new ones") {
        World world;
        Identifier bodies[5];
        for (int i = 0; i < 5; i++) {
            bodies[i] = world.CreateBody();
            auto g = world.AddShapeGroup(bodies[i]);
            world.AddShape(g, Shape::Sphere);
        }

        for (int i = 0; i < 5; i++) {
            world.RemoveBody(bodies[i]);
        }

        // should be able to create fresh bodies
        auto b = world.CreateBody();
        CHECK(b != INVALID_ID);
        auto g = world.AddShapeGroup(b);
        CHECK(g != INVALID_ID);
        auto s = world.AddShape(g, Shape::OBB);
        CHECK(s != INVALID_ID);

        world.Update();
    }
}

// ============================================================================
// Collision Pipeline tests
// ============================================================================

TEST_SUITE("Collision Pipeline") {
    TEST_CASE("overlapping spheres produce contact") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        // Place bodies in world space
        world.GetBody(b2).position = Vec3(Unit{3}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        // Shape centers at local origin, radius set
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{2};

        world.Update();

        auto& contacts = world.GetContacts();
        CHECK(contacts.size() == 1);
        CHECK(contacts[0].body_a != contacts[0].body_b);
        CHECK(contacts[0].depth == Unit{1});
    }

    TEST_CASE("separated spheres produce no contact") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b2).position = Vec3(Unit{10}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{1};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{1};

        world.Update();

        CHECK(world.GetContacts().size() == 0);
    }

    TEST_CASE("static vs static skipped") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b1).is_static = true;
        world.GetBody(b2).is_static = true;
        world.GetBody(b2).position = Vec3(Unit{1}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{2};

        world.Update();

        CHECK(world.GetContacts().size() == 0);
    }

    TEST_CASE("layer mask filtering") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b2).position = Vec3(Unit{1}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        // different layers, masks don't match
        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 2;
        world.GetShapeGroup(g2).mask = 2;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{5};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{5};

        world.Update();

        CHECK(world.GetContacts().size() == 0);
    }

    TEST_CASE("same body skip") {
        World world;
        auto b1 = world.CreateBody();

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b1);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        // local offsets within same body
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{5};
        world.GetSphere(world.GetShape(s2).shape_type_id).center = Vec3(Unit{1}, Unit{0}, Unit{0});
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{5};

        world.Update();

        CHECK(world.GetContacts().size() == 0);
    }

    TEST_CASE("contacts cleared between frames") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b2).position = Vec3(Unit{3}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{2};

        world.Update();
        CHECK(world.GetContacts().size() == 1);

        // separate them by moving body
        world.GetBody(b2).position = Vec3(Unit{20}, Unit{0}, Unit{0});
        world.Update();
        CHECK(world.GetContacts().size() == 0);
    }

    TEST_CASE("sphere vs OBB mixed types") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b1).position = Vec3(Unit{3}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);

        world.GetShapeGroup(g1).layer = 1;
        world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1;
        world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::OBB);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{1};

        auto& obb = world.GetOBB(world.GetShape(s2).shape_type_id);
        obb.half_extents = Vec3(Unit{2}, Unit{2}, Unit{2});

        world.Update();

        auto& contacts = world.GetContacts();
        CHECK(contacts.size() == 1);
        CHECK(contacts[0].depth == Unit{0}); // just touching
    }

    TEST_CASE("body transform applies to spheres") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        // Bodies at origin, shapes with local offset
        world.GetBody(b1).position = Vec3(Unit{0}, Unit{0}, Unit{0});
        world.GetBody(b2).position = Vec3(Unit{5}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);
        world.GetShapeGroup(g1).layer = 1; world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1; world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        // Local offset of 1 unit along X for each
        world.GetSphere(world.GetShape(s1).shape_type_id).center = Vec3(Unit{1}, Unit{0}, Unit{0});
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{1};
        world.GetSphere(world.GetShape(s2).shape_type_id).center = Vec3(Unit{-1}, Unit{0}, Unit{0});
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{1};

        // World positions: sphere1 at (1,0,0), sphere2 at (4,0,0). Distance=3, sum_r=2 => no contact
        world.Update();
        CHECK(world.GetContacts().size() == 0);

        // Move b2 closer so they overlap
        world.GetBody(b2).position = Vec3(Unit{3}, Unit{0}, Unit{0});
        // sphere1 at (1,0,0), sphere2 at (2,0,0). Distance=1, sum_r=2 => depth=1
        world.Update();
        CHECK(world.GetContacts().size() == 1);
        CHECK(world.GetContacts()[0].depth == Unit{1});
    }

    TEST_CASE("body rotation applies to shapes") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        // Body 1 at origin, rotated 90 deg around Z (X->Y, Y->-X)
        world.GetBody(b1).rotation = Mat3::RotateZ(90);

        // Body 2 at (0, 3, 0)
        world.GetBody(b2).position = Vec3(Unit{0}, Unit{3}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);
        world.GetShapeGroup(g1).layer = 1; world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1; world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);

        // Sphere 1 at local (2,0,0), after 90Z rotation -> world (0,2,0)
        world.GetSphere(world.GetShape(s1).shape_type_id).center = Vec3(Unit{2}, Unit{0}, Unit{0});
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{1};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{1};

        // World: s1 at (0,2,0), s2 at (0,3,0). Distance=1, sum_r=2 => depth=1
        world.Update();
        CHECK(world.GetContacts().size() == 1);
        CHECK(world.GetContacts()[0].depth == Unit{1});
    }

    TEST_CASE("body transform applies to OBB") {
        World world;
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();

        world.GetBody(b1).position = Vec3(Unit{0}, Unit{0}, Unit{0});
        world.GetBody(b2).position = Vec3(Unit{4}, Unit{0}, Unit{0});
        // Rotate body2 90 deg around Y, so OBB's local X-axis (half=3) maps to world Z
        world.GetBody(b2).rotation = Mat3::RotateY(90);

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);
        world.GetShapeGroup(g1).layer = 1; world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1; world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::OBB);

        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        auto& obb = world.GetOBB(world.GetShape(s2).shape_type_id);
        obb.half_extents = Vec3(Unit{3}, Unit{1}, Unit{1});
        // OBB local rotation is identity, body rotation rotates it

        // World OBB at (4,0,0), rotated 90Y: projects 1 along world X from center
        // Closest on OBB X-axis = 4-1 = 3, sphere at 0 with r=2, dist=3 > 2 => no hit
        world.Update();
        CHECK(world.GetContacts().size() == 0);

        // Move body2 closer
        world.GetBody(b2).position = Vec3(Unit{2}, Unit{0}, Unit{0});
        // Closest on OBB = 2-1 = 1, dist=1, r=2 => depth=1
        world.Update();
        CHECK(world.GetContacts().size() == 1);
        CHECK(world.GetContacts()[0].depth == Unit{1});
    }
}

// ============================================================================
// Integration tests
// ============================================================================

TEST_SUITE("Integration") {
    TEST_CASE("iteration benchmark") {
        World world;
        const int N = 100;

        // Create N bodies in a 10x10 grid, spacing 3, radius 2 => neighbors overlap
        for (int i = 0; i < N; i++) {
            auto bid = world.CreateBody();
            int row = i / 10;
            int col = i % 10;
            world.GetBody(bid).position = Vec3(Unit{col * 3}, Unit{0}, Unit{row * 3});

            auto gid = world.AddShapeGroup(bid);
            world.GetShapeGroup(gid).layer = 1;
            world.GetShapeGroup(gid).mask = 1;

            auto sid = world.AddShape(gid, Shape::Sphere);
            world.GetSphere(world.GetShape(sid).shape_type_id).radius = Unit{2};
        }

        const int M = 100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < M; i++) {
            world.Update();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        auto& contacts = world.GetContacts();

        std::ostringstream log;
        log << "bodies=" << N
            << " iterations=" << M
            << " total_us=" << ms
            << " per_iter_us=" << (ms / M)
            << " contacts=" << contacts.size();
        MESSAGE(log.str());

        CHECK(contacts.size() > 0);
    }

    TEST_CASE("save load roundtrip with contacts") {
        World world;

        // Body A: sphere at origin
        auto ba = world.CreateBody();
        auto ga = world.AddShapeGroup(ba);
        world.GetShapeGroup(ga).layer = 1;
        world.GetShapeGroup(ga).mask = 1;
        auto sa = world.AddShape(ga, Shape::Sphere);
        world.GetSphere(world.GetShape(sa).shape_type_id).radius = Unit{2};

        // Body B: OBB at (3,0,0)
        auto bb = world.CreateBody();
        world.GetBody(bb).position = Vec3(Unit{3}, Unit{0}, Unit{0});
        auto gb = world.AddShapeGroup(bb);
        world.GetShapeGroup(gb).layer = 1;
        world.GetShapeGroup(gb).mask = 1;
        auto sb = world.AddShape(gb, Shape::OBB);
        auto& obb = world.GetOBB(world.GetShape(sb).shape_type_id);
        obb.half_extents = Vec3(Unit{2}, Unit{2}, Unit{2});

        // Body C: capsule at (0,0,3)
        auto bc = world.CreateBody();
        world.GetBody(bc).position = Vec3(Unit{0}, Unit{0}, Unit{3});
        auto gc = world.AddShapeGroup(bc);
        world.GetShapeGroup(gc).layer = 1;
        world.GetShapeGroup(gc).mask = 1;
        auto sc = world.AddShape(gc, Shape::Capsule);
        auto& cap = world.GetCapsule(world.GetShape(sc).shape_type_id);
        cap.start = Vec3(Unit{-2}, Unit{0}, Unit{0});
        cap.end = Vec3(Unit{2}, Unit{0}, Unit{0});
        cap.radius = Unit{1};

        world.Update();
        uint32_t contacts_before = world.GetContacts().size();
        CHECK(contacts_before > 0);

        // Save
        MemStream stream;
        auto save_start = std::chrono::high_resolution_clock::now();
        world.Save(stream);
        auto save_end = std::chrono::high_resolution_clock::now();
        uint32_t save_size = (uint32_t)stream.size();
        stream.rewind();

        // Load
        World world2;
        auto load_start = std::chrono::high_resolution_clock::now();
        world2.Load(stream);
        auto load_end = std::chrono::high_resolution_clock::now();
        world2.Update();
        uint32_t contacts_after = world2.GetContacts().size();

        auto save_us = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count();
        auto load_us = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count();

        std::ostringstream log;
        log << "save_bytes=" << save_size
            << " save_us=" << save_us
            << " load_us=" << load_us
            << " contacts_before=" << contacts_before
            << " contacts_after=" << contacts_after;
        MESSAGE(log.str());

        CHECK(contacts_before == contacts_after);

        // Re-save loaded world and memcmp
        MemStream stream2;
        world2.Save(stream2);
        CHECK(stream.size() == stream2.size());
        CHECK(std::memcmp(stream.data(), stream2.data(), stream.size()) == 0);
    }

    TEST_CASE("save load large world") {
        World world;
        const int BODY_COUNT = 50;

        for (int i = 0; i < BODY_COUNT; i++) {
            auto bid = world.CreateBody();
            world.GetBody(bid).position = Vec3(Unit{i * 2}, Unit{0}, Unit{0});

            auto gid = world.AddShapeGroup(bid);
            world.GetShapeGroup(gid).layer = 1;
            world.GetShapeGroup(gid).mask = 1;

            // Alternate shape types
            Shape::Type type;
            switch (i % 3) {
            case 0: type = Shape::Sphere; break;
            case 1: type = Shape::OBB; break;
            default: type = Shape::Capsule; break;
            }

            auto sid = world.AddShape(gid, type);
            switch (type) {
            case Shape::Sphere:
                world.GetSphere(world.GetShape(sid).shape_type_id).radius = Unit{1};
                break;
            case Shape::OBB: {
                auto& o = world.GetOBB(world.GetShape(sid).shape_type_id);
                o.half_extents = Vec3(Unit{1}, Unit{1}, Unit{1});
                break;
            }
            case Shape::Capsule: {
                auto& c = world.GetCapsule(world.GetShape(sid).shape_type_id);
                c.start = Vec3(Unit{0}, Unit{-1}, Unit{0});
                c.end = Vec3(Unit{0}, Unit{1}, Unit{0});
                c.radius = Unit{1};
                break;
            }
            default: break;
            }
        }

        // Save
        MemStream stream1;
        auto save_start = std::chrono::high_resolution_clock::now();
        world.Save(stream1);
        auto save_end = std::chrono::high_resolution_clock::now();
        stream1.rewind();

        // Load
        World world2;
        auto load_start = std::chrono::high_resolution_clock::now();
        world2.Load(stream1);
        auto load_end = std::chrono::high_resolution_clock::now();

        // Re-save and compare
        MemStream stream2;
        world2.Save(stream2);

        auto save_us = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count();
        auto load_us = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count();

        std::ostringstream log;
        log << "bodies=" << BODY_COUNT
            << " save1_bytes=" << stream1.size()
            << " save2_bytes=" << stream2.size()
            << " save_us=" << save_us
            << " load_us=" << load_us;
        MESSAGE(log.str());

        CHECK(stream1.size() == stream2.size());
        CHECK(std::memcmp(stream1.data(), stream2.data(), stream1.size()) == 0);
    }
}

// ============================================================================
// Debug Draw tests
// ============================================================================

struct DrawCall {
    enum Type : uint8_t { SPHERE, BOX, CAPSULE, AABB_T, LINE, POINT } type;
    Vec3F pos, pos2, half_ext;
    Mat3F rot;
    float radius;
    float size;
    Color color;

    DrawCall() : type(SPHERE), pos(), pos2(), half_ext(), rot(), radius(0), size(0), color(0,0,0,0) {}
};

class MockDebugDraw : public DebugDraw {
public:
    std::vector<DrawCall> calls;

    void DrawSphere(const Vec3F& center, float radius, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::SPHERE;
        c.pos = center;
        c.radius = radius;
        c.color = color;
        calls.push_back(c);
    }

    void DrawBox(const Vec3F& center, const Vec3F& half_extents, const Mat3F& rotation, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::BOX;
        c.pos = center;
        c.half_ext = half_extents;
        c.rot = rotation;
        c.color = color;
        calls.push_back(c);
    }

    void DrawCapsule(const Vec3F& start, const Vec3F& end, float radius, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::CAPSULE;
        c.pos = start;
        c.pos2 = end;
        c.radius = radius;
        c.color = color;
        calls.push_back(c);
    }

    void DrawAABB(const Vec3F& min, const Vec3F& max, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::AABB_T;
        c.pos = min;
        c.pos2 = max;
        c.color = color;
        calls.push_back(c);
    }

    void DrawLine(const Vec3F& from, const Vec3F& to, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::LINE;
        c.pos = from;
        c.pos2 = to;
        c.color = color;
        calls.push_back(c);
    }

    void DrawPoint(const Vec3F& position, float size, const Color& color) override {
        DrawCall c;
        c.type = DrawCall::POINT;
        c.pos = position;
        c.size = size;
        c.color = color;
        calls.push_back(c);
    }

    int CountByType(DrawCall::Type t) const {
        int count = 0;
        for (const auto& c : calls) {
            if (c.type == t) count++;
        }
        return count;
    }
};

// Helper: create a world with one sphere body and attach debug draw
static void MakeSingleSphereWorld(World& world, MockDebugDraw& dd) {
    auto b = world.CreateBody();
    auto g = world.AddShapeGroup(b);
    world.GetShapeGroup(g).layer = 1;
    world.GetShapeGroup(g).mask = 1;
    auto s = world.AddShape(g, Shape::Sphere);
    world.GetSphere(world.GetShape(s).shape_type_id).radius = Unit{2};
    world.SetDebugDraw(&dd);
}

TEST_SUITE("DebugDraw") {
    TEST_CASE("DrawDebug with no debug draw set does not crash") {
        World world;
        world.CreateBody();
        world.DrawDebug(); // should be a no-op
    }

    TEST_CASE("DrawDebug with empty world produces no calls") {
        World world;
        MockDebugDraw dd;
        world.SetDebugDraw(&dd);
        world.DrawDebug();
        CHECK(dd.calls.size() == 0);
    }

    TEST_CASE("single sphere body produces one DrawSphere call") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        MakeSingleSphereWorld(world, dd);
        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::SPHERE) == 1);
    }

    TEST_CASE("single OBB body produces one DrawBox call") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        world.SetDebugDraw(&dd);

        auto b = world.CreateBody();
        auto g = world.AddShapeGroup(b);
        world.GetShapeGroup(g).layer = 1;
        world.GetShapeGroup(g).mask = 1;
        auto s = world.AddShape(g, Shape::OBB);
        world.GetOBB(world.GetShape(s).shape_type_id).half_extents = Vec3(Unit{1}, Unit{1}, Unit{1});

        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::BOX) == 1);
    }

    TEST_CASE("single capsule body produces one DrawCapsule call") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        world.SetDebugDraw(&dd);

        auto b = world.CreateBody();
        auto g = world.AddShapeGroup(b);
        world.GetShapeGroup(g).layer = 1;
        world.GetShapeGroup(g).mask = 1;
        auto s = world.AddShape(g, Shape::Capsule);
        auto& cap = world.GetCapsule(world.GetShape(s).shape_type_id);
        cap.start = Vec3(Unit{0}, Unit{-1}, Unit{0});
        cap.end = Vec3(Unit{0}, Unit{1}, Unit{0});
        cap.radius = Unit{1};

        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::CAPSULE) == 1);
    }

    TEST_CASE("body with multiple shapes produces correct draw calls") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        world.SetDebugDraw(&dd);

        auto b = world.CreateBody();
        auto g = world.AddShapeGroup(b);
        world.GetShapeGroup(g).layer = 1;
        world.GetShapeGroup(g).mask = 1;

        world.AddShape(g, Shape::Sphere);
        world.AddShape(g, Shape::OBB);
        world.AddShape(g, Shape::Capsule);

        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::SPHERE) == 1);
        CHECK(dd.CountByType(DrawCall::BOX) == 1);
        CHECK(dd.CountByType(DrawCall::CAPSULE) == 1);
        CHECK(dd.calls.size() == 3);
    }

    TEST_CASE("flag filtering: shapes only") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        MakeSingleSphereWorld(world, dd);
        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::SPHERE) == 1);
        CHECK(dd.CountByType(DrawCall::AABB_T) == 0);
        CHECK(dd.CountByType(DrawCall::LINE) == 0);
        CHECK(dd.CountByType(DrawCall::POINT) == 0);
    }

    TEST_CASE("flag filtering: AABBs only") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_AABBs;
        MakeSingleSphereWorld(world, dd);
        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::AABB_T) == 1);
        CHECK(dd.CountByType(DrawCall::SPHERE) == 0);
        CHECK(dd.CountByType(DrawCall::LINE) == 0);
    }

    TEST_CASE("flag filtering: body axes only") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_BodyAxes;
        MakeSingleSphereWorld(world, dd);
        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::LINE) == 3);
        CHECK(dd.CountByType(DrawCall::SPHERE) == 0);
        CHECK(dd.CountByType(DrawCall::AABB_T) == 0);
    }

    TEST_CASE("flag filtering: contacts only") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Contacts;
        world.SetDebugDraw(&dd);

        // Create two overlapping spheres to produce a contact
        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();
        world.GetBody(b2).position = Vec3(Unit{3}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);
        world.GetShapeGroup(g1).layer = 1; world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1; world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{2};

        world.Update();
        REQUIRE(world.GetContacts().size() == 1);

        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::POINT) == 1);
        CHECK(dd.CountByType(DrawCall::LINE) == 1);
        CHECK(dd.CountByType(DrawCall::SPHERE) == 0);
        CHECK(dd.CountByType(DrawCall::AABB_T) == 0);
    }

    TEST_CASE("flag filtering: none produces no calls") {
        World world;
        MockDebugDraw dd;
        dd.flags = 0;
        MakeSingleSphereWorld(world, dd);
        world.Update();
        world.DrawDebug();
        CHECK(dd.calls.size() == 0);
    }

    TEST_CASE("contact produces DrawPoint and DrawLine") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Contacts;
        world.SetDebugDraw(&dd);

        auto b1 = world.CreateBody();
        auto b2 = world.CreateBody();
        world.GetBody(b2).position = Vec3(Unit{3}, Unit{0}, Unit{0});

        auto g1 = world.AddShapeGroup(b1);
        auto g2 = world.AddShapeGroup(b2);
        world.GetShapeGroup(g1).layer = 1; world.GetShapeGroup(g1).mask = 1;
        world.GetShapeGroup(g2).layer = 1; world.GetShapeGroup(g2).mask = 1;

        auto s1 = world.AddShape(g1, Shape::Sphere);
        auto s2 = world.AddShape(g2, Shape::Sphere);
        world.GetSphere(world.GetShape(s1).shape_type_id).radius = Unit{2};
        world.GetSphere(world.GetShape(s2).shape_type_id).radius = Unit{2};

        world.Update();
        REQUIRE(world.GetContacts().size() == 1);

        world.DrawDebug();
        // 1 point + 1 line per contact
        CHECK(dd.CountByType(DrawCall::POINT) == 1);
        CHECK(dd.CountByType(DrawCall::LINE) == 1);

        // midpoint should be at (1.5, 0, 0)
        auto& pt = dd.calls[0];
        CHECK(std::abs(pt.pos.x - 1.5f) < 0.01f);
        CHECK(std::abs(pt.pos.y) < 0.01f);
    }

    TEST_CASE("body axes produce 3 DrawLine calls per body") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_BodyAxes;
        world.SetDebugDraw(&dd);

        world.CreateBody();
        world.CreateBody();

        world.DrawDebug();
        CHECK(dd.CountByType(DrawCall::LINE) == 6); // 3 per body * 2 bodies
    }

    TEST_CASE("world-space transform applied to drawn shapes") {
        World world;
        MockDebugDraw dd;
        dd.flags = DrawFlag_Shapes;
        world.SetDebugDraw(&dd);

        auto b = world.CreateBody();
        world.GetBody(b).position = Vec3(Unit{10}, Unit{5}, Unit{3});

        auto g = world.AddShapeGroup(b);
        world.GetShapeGroup(g).layer = 1;
        world.GetShapeGroup(g).mask = 1;
        auto s = world.AddShape(g, Shape::Sphere);
        world.GetSphere(world.GetShape(s).shape_type_id).radius = Unit{1};
        // sphere local center at (1, 0, 0)
        world.GetSphere(world.GetShape(s).shape_type_id).center = Vec3(Unit{1}, Unit{0}, Unit{0});

        world.DrawDebug();
        REQUIRE(dd.CountByType(DrawCall::SPHERE) == 1);

        // world center should be body.position + local.center = (11, 5, 3)
        auto& call = dd.calls[0];
        CHECK(std::abs(call.pos.x - 11.0f) < 0.01f);
        CHECK(std::abs(call.pos.y - 5.0f) < 0.01f);
        CHECK(std::abs(call.pos.z - 3.0f) < 0.01f);
    }
}
