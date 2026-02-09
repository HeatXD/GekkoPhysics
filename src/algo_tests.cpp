#include "doctest/doctest.h"
#include "algo.h"

using namespace GekkoMath;
using namespace GekkoPhysics;

static Unit U(int v) { return Unit{v}; }
static Unit UF(int num, int den) { return Unit{num} / Unit{den}; }  // e.g. UF(3,2) = 1.5

// ============================================================================
// Collision: Sphere vs Sphere
// ============================================================================

TEST_SUITE("Collision: Sphere vs Sphere") {
    TEST_CASE("separated spheres") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(1);
        Sphere b; b.center = Vec3(U(5), U(0), U(0)); b.radius = U(1);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("just touching") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(1);
        Sphere b; b.center = Vec3(U(2), U(0), U(0)); b.radius = U(1);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
        CHECK(r.normal.x == U(1));
        CHECK(r.normal.y == U(0));
        CHECK(r.normal.z == U(0));
    }

    TEST_CASE("overlapping") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(2);
        Sphere b; b.center = Vec3(U(3), U(0), U(0)); b.radius = U(2);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.x == U(1));
        CHECK(r.normal.y == U(0));
        CHECK(r.normal.z == U(0));
    }

    TEST_CASE("concentric spheres - fallback normal") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(1);
        Sphere b; b.center = Vec3(U(0), U(0), U(0)); b.radius = U(2);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(3));
        // fallback normal is up
        CHECK(r.normal.y == U(1));
    }

    TEST_CASE("different radii") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(3);
        Sphere b; b.center = Vec3(U(4), U(0), U(0)); b.radius = U(2);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
    }

    TEST_CASE("overlap along Y axis") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = U(2);
        Sphere b; b.center = Vec3(U(0), U(3), U(0)); b.radius = U(2);
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.y == U(1));
        CHECK(r.normal.x == U(0));
    }

    TEST_CASE("fractional radii - overlapping") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = UF(3, 2); // 1.5
        Sphere b; b.center = Vec3(U(2), U(0), U(0)); b.radius = UF(3, 2); // 1.5
        // sum_radii = 3, dist = 2, depth = 1
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
    }

    TEST_CASE("fractional radii - separated") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = UF(1, 4); // 0.25
        Sphere b; b.center = Vec3(U(1), U(0), U(0)); b.radius = UF(1, 4); // 0.25
        // sum_radii = 0.5, dist = 1 => no hit
        auto r = Algo::CollideSpheres(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("fractional positions - overlapping") {
        Sphere a; a.center = Vec3(UF(1, 2), U(0), U(0)); a.radius = U(1); // center at 0.5
        Sphere b; b.center = Vec3(UF(3, 2), U(0), U(0)); b.radius = U(1); // center at 1.5
        // dist = 1, sum_radii = 2, depth = 1
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
    }

    TEST_CASE("fractional - just touching") {
        Sphere a; a.center = Vec3(U(0), U(0), U(0)); a.radius = UF(3, 4); // 0.75
        Sphere b; b.center = Vec3(UF(3, 2), U(0), U(0)); b.radius = UF(3, 4); // 0.75
        // sum_radii = 1.5, dist = 1.5 => depth = 0
        auto r = Algo::CollideSpheres(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }
}

// ============================================================================
// Collision: Sphere vs Capsule
// ============================================================================

TEST_SUITE("Collision: Sphere vs Capsule") {
    TEST_CASE("sphere near capsule middle") {
        Sphere s; s.center = Vec3(U(0), U(2), U(0)); s.radius = U(1);
        Capsule c; c.start = Vec3(U(-5), U(0), U(0)); c.end = Vec3(U(5), U(0), U(0)); c.radius = U(1);
        // distance from sphere center to segment = 2, sum radii = 2 => just touching
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("sphere near capsule endpoint") {
        Sphere s; s.center = Vec3(U(7), U(0), U(0)); s.radius = U(1);
        Capsule c; c.start = Vec3(U(0), U(0), U(0)); c.end = Vec3(U(5), U(0), U(0)); c.radius = U(1);
        // closest point on segment is (5,0,0), dist = 2, sum radii = 2 => just touching
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("sphere far away") {
        Sphere s; s.center = Vec3(U(0), U(10), U(0)); s.radius = U(1);
        Capsule c; c.start = Vec3(U(-5), U(0), U(0)); c.end = Vec3(U(5), U(0), U(0)); c.radius = U(1);
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(!r.hit);
    }

    TEST_CASE("sphere overlapping capsule") {
        Sphere s; s.center = Vec3(U(0), U(1), U(0)); s.radius = U(1);
        Capsule c; c.start = Vec3(U(-5), U(0), U(0)); c.end = Vec3(U(5), U(0), U(0)); c.radius = U(1);
        // dist = 1, sum radii = 2, depth = 1
        // normal points from sphere(A) toward capsule point(B), so (0,-1,0)
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.y == U(-1));
    }

    TEST_CASE("sphere on capsule axis") {
        Sphere s; s.center = Vec3(U(3), U(0), U(0)); s.radius = U(1);
        Capsule c; c.start = Vec3(U(0), U(0), U(0)); c.end = Vec3(U(10), U(0), U(0)); c.radius = U(1);
        // closest point is (3,0,0), dist = 0 => concentric, depth = 2
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(r.hit);
        CHECK(r.depth == U(2));
    }

    TEST_CASE("fractional radius capsule") {
        Sphere s; s.center = Vec3(U(0), UF(3, 2), U(0)); s.radius = UF(1, 2); // y=1.5, r=0.5
        Capsule c; c.start = Vec3(U(-5), U(0), U(0)); c.end = Vec3(U(5), U(0), U(0)); c.radius = UF(1, 2); // r=0.5
        // dist to segment = 1.5, sum radii = 1.0 => no hit
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(!r.hit);
    }

    TEST_CASE("fractional positions - partial overlap") {
        Sphere s; s.center = Vec3(U(0), UF(3, 4), U(0)); s.radius = UF(1, 2); // y=0.75, r=0.5
        Capsule c; c.start = Vec3(U(-3), U(0), U(0)); c.end = Vec3(U(3), U(0), U(0)); c.radius = UF(1, 2); // r=0.5
        // dist = 0.75, sum radii = 1.0, depth = 0.25
        auto r = Algo::CollideSphereCapsule(s, c);
        CHECK(r.hit);
        CHECK(r.depth == UF(1, 4));
    }
}

// ============================================================================
// Collision: Capsule vs Capsule
// ============================================================================

TEST_SUITE("Collision: Capsule vs Capsule") {
    TEST_CASE("parallel capsules separated") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(10), U(0), U(0)); a.radius = U(1);
        Capsule b; b.start = Vec3(U(0), U(5), U(0)); b.end = Vec3(U(10), U(5), U(0)); b.radius = U(1);
        auto r = Algo::CollideCapsules(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("parallel capsules overlapping") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(10), U(0), U(0)); a.radius = U(1);
        Capsule b; b.start = Vec3(U(0), U(1), U(0)); b.end = Vec3(U(10), U(1), U(0)); b.radius = U(1);
        // dist between segments = 1, sum radii = 2, depth = 1
        auto r = Algo::CollideCapsules(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
    }

    TEST_CASE("perpendicular capsules") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(10), U(0), U(0)); a.radius = U(1);
        Capsule b; b.start = Vec3(U(5), U(0), U(-5)); b.end = Vec3(U(5), U(0), U(5)); b.radius = U(1);
        // closest points: (5,0,0) and (5,0,0), dist = 0, sum radii = 2
        auto r = Algo::CollideCapsules(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(2));
    }

    TEST_CASE("collinear capsules end to end") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(5), U(0), U(0)); a.radius = U(1);
        Capsule b; b.start = Vec3(U(7), U(0), U(0)); b.end = Vec3(U(12), U(0), U(0)); b.radius = U(1);
        // closest: (5,0,0) and (7,0,0), dist = 2, sum radii = 2 => just touching
        auto r = Algo::CollideCapsules(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("capsules crossing in an X") {
        // Use smaller values to avoid fixed-point overflow
        Capsule a; a.start = Vec3(U(-3), U(0), U(-3)); a.end = Vec3(U(3), U(0), U(3)); a.radius = U(1);
        Capsule b; b.start = Vec3(U(-3), U(0), U(3)); b.end = Vec3(U(3), U(0), U(-3)); b.radius = U(1);
        // Cross at origin, dist = 0, depth = 2
        auto r = Algo::CollideCapsules(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(2));
    }

    TEST_CASE("fractional radii - parallel near miss") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(4), U(0), U(0)); a.radius = UF(1, 4); // 0.25
        Capsule b; b.start = Vec3(U(0), U(1), U(0)); b.end = Vec3(U(4), U(1), U(0)); b.radius = UF(1, 4); // 0.25
        // dist = 1, sum radii = 0.5 => no hit
        auto r = Algo::CollideCapsules(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("fractional radii - parallel overlap") {
        Capsule a; a.start = Vec3(U(0), U(0), U(0)); a.end = Vec3(U(4), U(0), U(0)); a.radius = UF(3, 4); // 0.75
        Capsule b; b.start = Vec3(U(0), U(1), U(0)); b.end = Vec3(U(4), U(1), U(0)); b.radius = UF(3, 4); // 0.75
        // dist = 1, sum radii = 1.5, depth = 0.5
        auto r = Algo::CollideCapsules(a, b);
        CHECK(r.hit);
        CHECK(r.depth == UF(1, 2));
    }
}

// ============================================================================
// Collision: Sphere vs OBB
// ============================================================================

TEST_SUITE("Collision: Sphere vs OBB") {
    TEST_CASE("sphere outside OBB face") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(5), U(0), U(0)); s.radius = U(1);
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(!r.hit);
    }

    TEST_CASE("sphere touching OBB face") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(3), U(0), U(0)); s.radius = U(1);
        // closest point on OBB = (2,0,0), dist = 1 = radius => just touching
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("sphere overlapping OBB face") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(2), U(0), U(0)); s.radius = U(1);
        // closest on OBB = (2,0,0), dist = 0 but center exactly on face => inside?
        // lx = 2 = half_ext.x, so inside check: 2 <= 2 => inside
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("sphere center inside OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(5), U(5), U(5));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(1), U(0), U(0)); s.radius = U(1);
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        // inside: min pen axis is x (pen_x=4), y (pen_y=5), z (pen_z=5) => min is x with 4
        // depth = 4 + 1 = 5
        // normal points from sphere toward nearest OBB face (negative x direction)
        CHECK(r.depth == U(5));
        CHECK(r.normal.x == U(-1));
    }

    TEST_CASE("sphere outside OBB corner") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(1), U(1), U(1));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(10), U(10), U(10)); s.radius = U(1);
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(!r.hit);
    }

    TEST_CASE("rotated OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(3), U(1), U(1));
        box.rotation = Mat3::RotateY(90);
        // The box is 6 units long along world-Z, 2 along world-Y, 2 along world-X

        Sphere s; s.center = Vec3(U(0), U(0), U(4)); s.radius = U(1);
        // OBB extends 3 along its local-X, which is world-Z
        // Closest point on OBB = (0,0,3), dist = 1 = radius => just touching
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("fractional half-extents") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(UF(3, 2), UF(3, 2), UF(3, 2)); // 1.5 each
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(U(2), U(0), U(0)); s.radius = UF(1, 2); // r=0.5
        // closest on OBB = (1.5,0,0), dist = 0.5 = radius => just touching
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("fractional sphere inside OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Sphere s; s.center = Vec3(UF(1, 2), U(0), U(0)); s.radius = UF(1, 4); // center 0.5, r=0.25
        // inside: pen_x = 2-0.5 = 1.5, pen_y = 2, pen_z = 2 => min is x with 1.5
        // depth = 1.5 + 0.25 = 1.75
        // normal points from sphere toward nearest OBB face (negative x direction)
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth == UF(7, 4));
        CHECK(r.normal.x == U(-1));
    }

    TEST_CASE("sphere overlapping 30-deg-Z rotated OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(3), U(1), U(1));
        box.rotation = Mat3::RotateZ(30);
        // Box long axis tilted 30 deg from X toward Y

        Sphere s; s.center = Vec3(U(0), U(2), U(0)); s.radius = U(1);
        // Closest on OBB ~0.55 away from sphere center (< radius 1)
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("sphere separated from 30-deg-Z rotated OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(3), U(1), U(1));
        box.rotation = Mat3::RotateZ(30);

        Sphere s; s.center = Vec3(U(0), U(4), U(0)); s.radius = U(1);
        // OBB max Y extent ~2.37, sphere at y=4 with r=1 => gap > 0.6
        auto r = Algo::CollideSphereOBB(s, box);
        CHECK(!r.hit);
    }
}

// ============================================================================
// Collision: Capsule vs OBB
// ============================================================================

TEST_SUITE("Collision: Capsule vs OBB") {
    TEST_CASE("capsule parallel to OBB face - separated") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Capsule c; c.start = Vec3(U(-5), U(5), U(0)); c.end = Vec3(U(5), U(5), U(0)); c.radius = U(1);
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(!r.hit);
    }

    TEST_CASE("capsule parallel to OBB face - overlapping") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Capsule c; c.start = Vec3(U(-5), U(2), U(0)); c.end = Vec3(U(5), U(2), U(0)); c.radius = U(1);
        // closest on seg to OBB center is (0,2,0), closest on OBB = (0,2,0), dist = 0
        // sphere center on box surface => inside
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(r.hit);
    }

    TEST_CASE("capsule endpoint near OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(2), U(2));
        box.rotation = Mat3();

        Capsule c; c.start = Vec3(U(10), U(0), U(0)); c.end = Vec3(U(3), U(0), U(0)); c.radius = U(1);
        // closest point on seg to OBB center is (3,0,0), closest on OBB = (2,0,0), dist = 1 = radius
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("capsule far from OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(1), U(1), U(1));
        box.rotation = Mat3();

        Capsule c; c.start = Vec3(U(10), U(10), U(10)); c.end = Vec3(U(15), U(10), U(10)); c.radius = U(1);
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(!r.hit);
    }

    TEST_CASE("rotated OBB with capsule") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(3), U(1), U(1));
        box.rotation = Mat3::RotateY(90);

        Capsule c; c.start = Vec3(U(0), U(0), U(5)); c.end = Vec3(U(0), U(0), U(10)); c.radius = U(1);
        // Box extends 3 along world-Z. Capsule starts at z=5. Closest on seg = (0,0,5).
        // Closest on OBB = (0,0,3). dist = 2, radius = 1 => separated
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(!r.hit);
    }

    TEST_CASE("fractional capsule radius near OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(1), U(1), U(1));
        box.rotation = Mat3();

        Capsule c; c.start = Vec3(U(-3), UF(5, 4), U(0)); c.end = Vec3(U(3), UF(5, 4), U(0)); c.radius = UF(1, 4);
        // closest on seg to box center = (0, 1.25, 0). closest on OBB = (0, 1, 0).
        // dist = 0.25 = radius => just touching
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("capsule overlapping 30-deg-Z rotated OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(1), U(1));
        box.rotation = Mat3::RotateZ(30);

        Capsule c; c.start = Vec3(U(-3), U(2), U(0)); c.end = Vec3(U(3), U(2), U(0)); c.radius = U(1);
        // Capsule runs horizontally at y=2. OBB tilted 30 deg extends ~1.87 along Y.
        // Closest OBB surface ~0.55 from capsule axis, well within radius 1.
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("capsule separated from 60-deg-Y rotated OBB") {
        OBB box;
        box.center = Vec3(U(0), U(0), U(0));
        box.half_extents = Vec3(U(2), U(1), U(1));
        box.rotation = Mat3::RotateY(60);

        Capsule c; c.start = Vec3(U(-3), U(0), U(4)); c.end = Vec3(U(3), U(0), U(4)); c.radius = U(1);
        // OBB max Z extent ~2.23, capsule at z=4 with r=1 => gap ~0.77
        auto r = Algo::CollideCapsuleOBB(c, box);
        CHECK(!r.hit);
    }
}

// ============================================================================
// Collision: OBB vs OBB
// ============================================================================

TEST_SUITE("Collision: OBB vs OBB") {
    TEST_CASE("separated along one axis") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(5), U(0), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("face-to-face overlap") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(2), U(2), U(2));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(3), U(0), U(0));
        b.half_extents = Vec3(U(2), U(2), U(2));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.x == U(1));
    }

    TEST_CASE("just touching") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(2), U(0), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("identical OBBs - full overlap") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(2), U(3), U(4));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(0), U(0));
        b.half_extents = Vec3(U(2), U(3), U(4));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        // depth = 2*min_half_extent = 4 (along x)
        CHECK(r.depth == U(4));
    }

    TEST_CASE("separated along Y") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(10), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("overlap along Y") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(2), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(3), U(0));
        b.half_extents = Vec3(U(1), U(2), U(1));
        b.rotation = Mat3();

        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.y == U(1));
    }

    TEST_CASE("fractional half-extents - separated") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(UF(3, 4), UF(3, 4), UF(3, 4)); // 0.75
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(2), U(0), U(0));
        b.half_extents = Vec3(UF(3, 4), UF(3, 4), UF(3, 4)); // 0.75
        b.rotation = Mat3();

        // proj_a + proj_b = 0.75 + 0.75 = 1.5, dist = 2 => separated
        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("fractional half-extents - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(UF(3, 2), UF(1, 2), UF(1, 2)); // 1.5, 0.5, 0.5
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(2), U(0), U(0));
        b.half_extents = Vec3(UF(3, 2), UF(1, 2), UF(1, 2)); // 1.5, 0.5, 0.5
        b.rotation = Mat3();

        // along x: 1.5 + 1.5 - 2 = 1
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(1));
        CHECK(r.normal.x == U(1));
    }

    TEST_CASE("one OBB rotated 90 deg around Z - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(4), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(3), U(0));
        b.half_extents = Vec3(U(4), U(1), U(1));
        b.rotation = Mat3::RotateZ(90);
        // b extends 4 along world-Y from center y=3
        // proj_a on Y = 1, proj_b on Y = 4, dist = 3, overlap = 1+4-3 = 2
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
    }

    TEST_CASE("one OBB rotated 90 deg around Z - separated") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(6), U(0));
        b.half_extents = Vec3(U(4), U(1), U(1));
        b.rotation = Mat3::RotateZ(90);
        // b extends 4 along world-Y from y=6: y=2 to y=10
        // a extends 1 along world-Y: y=-1 to y=1
        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("both OBBs rotated 45 deg - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(2), U(1), U(1));
        a.rotation = Mat3::RotateZ(45);

        OBB b;
        b.center = Vec3(U(3), U(0), U(0));
        b.half_extents = Vec3(U(2), U(1), U(1));
        b.rotation = Mat3::RotateZ(-45);

        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("both OBBs rotated 45 deg - separated") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3::RotateZ(45);

        OBB b;
        b.center = Vec3(U(5), U(0), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3::RotateZ(-45);

        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("edge-edge cross product axis") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(5), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(0), U(3));
        b.half_extents = Vec3(U(1), U(5), U(1));
        b.rotation = Mat3::RotateX(90);
        // b is long along world-Z (half=5), centered at z=3
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
    }

    TEST_CASE("one OBB rotated 90 deg around Y - just touching") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(3), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(4), U(0), U(0));
        b.half_extents = Vec3(U(3), U(1), U(1));
        b.rotation = Mat3::RotateY(90);
        // b long axis now along world-Z. Along X: b projects 1
        // dist on X = 4, a projects 3, overlap = 3+1-4 = 0
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("fractional center offset") {
        OBB a;
        a.center = Vec3(UF(1, 4), U(0), U(0)); // 0.25
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(UF(9, 4), U(0), U(0)); // 2.25
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3();

        // dist = 2.0, sum half = 2, overlap = 0 => just touching
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth == U(0));
    }

    TEST_CASE("one OBB rotated 30 deg around Z - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(2), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(3), U(0), U(0));
        b.half_extents = Vec3(U(2), U(1), U(1));
        b.rotation = Mat3::RotateZ(30);

        // b's X-axis is (cos30,sin30,0). Projections on a's X give overlap ~1.2
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("one OBB rotated 30 deg around Z - separated") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(4), U(0), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3::RotateZ(30);

        // Along a's X: proj_a=1, proj_b=cos30+sin30~1.37, dist=4 => gap ~1.63
        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("one OBB rotated 60 deg around Y - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(3), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(4), U(0), U(0));
        b.half_extents = Vec3(U(3), U(1), U(1));
        b.rotation = Mat3::RotateY(60);

        // b's long axis rotated 60 deg into XZ plane.
        // All 15 SAT axes show overlap (min ~0.63 on cross-product axis a[1]xb[0])
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("compound rotation (30X then 60Y) - overlapping") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(2), U(2), U(2));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(3), U(1), U(0));
        b.half_extents = Vec3(U(2), U(1), U(1));
        b.rotation = Mat3::RotateY(60) * Mat3::RotateX(30);

        // Arbitrary orientation. a is a large cube, b is close enough to overlap.
        auto r = Algo::CollideOBBs(a, b);
        CHECK(r.hit);
        CHECK(r.depth > U(0));
    }

    TEST_CASE("both OBBs with non-45 rotations - separated") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(1), U(1), U(1));
        a.rotation = Mat3::RotateZ(15);

        OBB b;
        b.center = Vec3(U(5), U(0), U(0));
        b.half_extents = Vec3(U(1), U(1), U(1));
        b.rotation = Mat3::RotateZ(60);

        // Max projection of unit cube ~1.41. Sum ~2.83. Distance = 5 => clear gap.
        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }

    TEST_CASE("parallel long OBBs, one rotated 30X - separated by oblique axis") {
        OBB a;
        a.center = Vec3(U(0), U(0), U(0));
        a.half_extents = Vec3(U(4), U(1), U(1));
        a.rotation = Mat3();

        OBB b;
        b.center = Vec3(U(0), U(2), U(2));
        b.half_extents = Vec3(U(4), U(1), U(1));
        b.rotation = Mat3::RotateX(30);

        // Both long along X. Along b's Y-axis (0,cos30,sin30):
        // proj_a = cos30+sin30 ~1.37, proj_b = 1, dist = |2*cos30+2*sin30| ~2.73
        // overlap = 1.37+1-2.73 = -0.37 => separated on this oblique axis
        auto r = Algo::CollideOBBs(a, b);
        CHECK(!r.hit);
    }
}
