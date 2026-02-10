#include "raylib-cpp.hpp"
#include "rlgl.h"

#include "gekko_physics.h"

static Vector3 ToRL(const GekkoMath::Vec3F& v) { return { v.x, v.y, v.z }; }

class RaylibDebugDraw : public GekkoPhysics::DebugDraw {
    using Vec3F = GekkoMath::Vec3F;
    using Mat3F = GekkoMath::Mat3F;

    static constexpr ::Color COL_SHAPE      = { 30, 60, 180, 255 };    // strong blue
    static constexpr ::Color COL_SHAPE_FILL = { 30, 60, 180, 50 };
    static constexpr ::Color COL_AABB       = { 0, 160, 0, 255 };      // green
    static constexpr ::Color COL_CONTACT    = { 220, 0, 0, 255 };      // red
    static constexpr ::Color COL_NORMAL     = { 200, 0, 0, 255 };      // red (same as contact)
    static constexpr ::Color COL_LINE       = { 40, 40, 40, 255 };     // near-black

public:
    void DrawSphere(const Vec3F& center, float radius) override {
        DrawSphereEx(ToRL(center), radius, 12, 12, COL_SHAPE_FILL);
        DrawSphereWires(ToRL(center), radius, 12, 12, COL_SHAPE);
    }

    void DrawBox(const Vec3F& center, const Vec3F& half_extents, const Mat3F& rot) override {
        Vector3 size = { half_extents.x * 2, half_extents.y * 2, half_extents.z * 2 };

        rlPushMatrix();
        rlTranslatef(center.x, center.y, center.z);
        float m[16] = {
            rot.cols[0].x, rot.cols[0].y, rot.cols[0].z, 0,
            rot.cols[1].x, rot.cols[1].y, rot.cols[1].z, 0,
            rot.cols[2].x, rot.cols[2].y, rot.cols[2].z, 0,
            0, 0, 0, 1,
        };
        rlMultMatrixf(m);
        DrawCubeV({ 0, 0, 0 }, size, COL_SHAPE_FILL);
        DrawCubeWiresV({ 0, 0, 0 }, size, COL_SHAPE);
        rlPopMatrix();
    }

    void DrawCapsule(const Vec3F& start, const Vec3F& end, float radius) override {
        ::DrawCapsule(ToRL(start), ToRL(end), radius, 8, 8, COL_SHAPE_FILL);
        DrawCapsuleWires(ToRL(start), ToRL(end), radius, 8, 8, COL_SHAPE);
    }

    void DrawAABB(const Vec3F& min, const Vec3F& max) override {
        const float r = 0.02f;
        Vector3 v[8] = {
            {min.x, min.y, min.z}, {max.x, min.y, min.z},
            {max.x, min.y, max.z}, {min.x, min.y, max.z},
            {min.x, max.y, min.z}, {max.x, max.y, min.z},
            {max.x, max.y, max.z}, {min.x, max.y, max.z},
        };
        int edges[][2] = {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7},
        };
        for (auto& e : edges) {
            DrawCylinderEx(v[e[0]], v[e[1]], r, r, 4, COL_AABB);
        }
    }

    void DrawLine(const Vec3F& from, const Vec3F& to) override {
        DrawCylinderEx(ToRL(from), ToRL(to), 0.05f, 0.05f, 4, COL_NORMAL);
    }

    void DrawPoint(const Vec3F& position, float size) override {
        DrawSphereEx(ToRL(position), 0.2f, 6, 6, COL_CONTACT);
    }

    void DrawBodyOrigin(const Vec3F& position) override {
        static constexpr ::Color COL_ORIGIN = { 140, 40, 200, 255 }; // purple
        DrawSphereEx(ToRL(position), 0.12f, 6, 6, COL_ORIGIN);
    }

    void DrawBodyAxes(const Vec3F& position, const Mat3F& rot) override {
        Vector3 pos = ToRL(position);
        Vector3 x = { pos.x + rot.cols[0].x, pos.y + rot.cols[0].y, pos.z + rot.cols[0].z };
        Vector3 y = { pos.x + rot.cols[1].x, pos.y + rot.cols[1].y, pos.z + rot.cols[1].z };
        Vector3 z = { pos.x + rot.cols[2].x, pos.y + rot.cols[2].y, pos.z + rot.cols[2].z };
        DrawCylinderEx(pos, x, 0.03f, 0.03f, 4, {220, 40, 40, 255});   // X = red
        DrawCylinderEx(pos, y, 0.03f, 0.03f, 4, {40, 180, 40, 255});   // Y = green
        DrawCylinderEx(pos, z, 0.03f, 0.03f, 4, {40, 80, 220, 255});   // Z = blue
    }
};


static GekkoPhysics::Identifier BuildScene(GekkoPhysics::World& world) {
    using namespace GekkoPhysics;
    using namespace GekkoMath;

    Vec3 gravity(Unit{0}, Unit{-10}, Unit{0});

    // Helper: create body + shape group
    auto makeBody = [&](Vec3 pos, bool is_static, Mat3 rot = Mat3()) {
        Identifier bid = world.CreateBody();
        Body& b = world.GetBody(bid);
        b.position = pos;
        b.is_static = is_static;
        b.rotation = rot;
        if (!is_static) b.acceleration = gravity;
        Identifier gid = world.AddShapeGroup(bid);
        ShapeGroup& sg = world.GetShapeGroup(gid);
        sg.layer = 1; sg.mask = 0xFFFFFFFF;
        return std::pair<Identifier, Identifier>{bid, gid};
    };

    auto addSphereShape = [&](Identifier gid, Vec3 center, Unit radius) {
        Identifier sid = world.AddShape(gid, Shape::Sphere);
        Sphere& s = world.GetSphere(world.GetShape(sid).shape_type_id);
        s.center = center;
        s.radius = radius;
    };

    auto addOBBShape = [&](Identifier gid, Vec3 center, Vec3 half, Mat3 rot = Mat3()) {
        Identifier sid = world.AddShape(gid, Shape::OBB);
        OBB& o = world.GetOBB(world.GetShape(sid).shape_type_id);
        o.center = center;
        o.half_extents = half;
        o.rotation = rot;
    };

    auto addCapsuleShape = [&](Identifier gid, Vec3 start, Vec3 end, Unit radius) {
        Identifier sid = world.AddShape(gid, Shape::Capsule);
        Capsule& c = world.GetCapsule(world.GetShape(sid).shape_type_id);
        c.start = start;
        c.end = end;
        c.radius = radius;
    };

    Vec3 zero;

    // Floor: top surface at y=0, extends 20 units in XZ
    {
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{-1}, Unit{0}), true);
        addOBBShape(g, zero, Vec3(Unit{20}, Unit{1}, Unit{20}));
    }

    // Wall 1 (back wall): along X axis at z=-10, 4 units tall
    {
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{2}, Unit{-11}), true);
        addOBBShape(g, zero, Vec3(Unit{20}, Unit{4}, Unit{5}));
    }

    // Wall 2 (left wall): along Z axis at x=-10, 4 units tall — forms corner with wall 1
    {
        auto [b, g] = makeBody(Vec3(Unit{-11}, Unit{2}, Unit{0}), true);
        addOBBShape(g, zero, Vec3(Unit{1}, Unit{4}, Unit{20}));
    }

    Identifier controlled_body;
    {
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{2}, Unit{0}), false);
        addSphereShape(g, zero, Unit{1});
        controlled_body = b;
    }

    {
        auto [b, g] = makeBody(Vec3(Unit{-5}, Unit{2}, Unit{0}), false);
        addSphereShape(g, zero, Unit{1});
    }
    {
        // Tall box
        auto [b, g] = makeBody(Vec3(Unit{5}, Unit{3}, Unit{0}), false);
        addOBBShape(g, zero, Vec3(Unit{1}, Unit{2}, Unit{1}));
    }
    {
        // Diagonal capsule
        auto [b, g] = makeBody(Vec3(Unit{0}, Unit{2}, Unit{-5}), false);
        addCapsuleShape(g, Vec3(Unit{-1}, Unit{-1}, Unit{0}), Vec3(Unit{1}, Unit{1}, Unit{0}), Unit{1} / Unit{2});
    }
    {
        // Rotated wide box near corner
        auto [b, g] = makeBody(Vec3(Unit{-7}, Unit{2}, Unit{-7}), false, Mat3::RotateY(45));
        addOBBShape(g, zero, Vec3(Unit{2}, Unit{1}, Unit{1}));
    }
    {
        // Three spheres in a line
        auto [b, g] = makeBody(Vec3(Unit{5}, Unit{2}, Unit{5}), false);
        addSphereShape(g, Vec3(Unit{-2}, Unit{0}, Unit{0}), Unit{1});
        addSphereShape(g, Vec3(Unit{0}, Unit{0}, Unit{0}), Unit{1});
        addSphereShape(g, Vec3(Unit{2}, Unit{0}, Unit{0}), Unit{1});
    }

    return controlled_body;
}

// ── Main ────────────────────────────────────────────────────────────

int main() {
    raylib::Window window(1600, 900, "GekkoPhysics Test Scene");

    Camera3D camera = {};
    camera.position = { 10.0f, 8.0f, 10.0f };
    camera.target   = { 0.0f, 1.0f, 0.0f };
    camera.up       = { 0.0f, 1.0f, 0.0f };
    camera.fovy     = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    GekkoPhysics::World world;
    auto controlled = BuildScene(world);

    RaylibDebugDraw debug_draw;
    debug_draw.flags = GekkoPhysics::DrawFlag_All;
    world.SetDebugDraw(&debug_draw);

    const GekkoMath::Unit move_speed{5};
    const GekkoMath::Unit jump_speed{8};
    SetTargetFPS(60);

    while (!window.ShouldClose()) {
        // Arrow keys set horizontal velocity, space to jump
        {
            auto& body = world.GetBody(controlled);
            GekkoMath::Unit vx{0}, vz{0};
            if (IsKeyDown(KEY_RIGHT)) vx += move_speed;
            if (IsKeyDown(KEY_LEFT))  vx -= move_speed;
            if (IsKeyDown(KEY_UP))    vz -= move_speed;
            if (IsKeyDown(KEY_DOWN))  vz += move_speed;
            body.velocity.x = vx;
            body.velocity.z = vz;

            // Jump: only when on or near the floor (y near 1 = sphere radius above floor)
            if (IsKeyPressed(KEY_SPACE) && body.position.y < GekkoMath::Unit{2}) {
                body.velocity.y = jump_speed;
            }
        }

        world.Update();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        // Pass 1: shapes + AABBs (with depth testing)
        debug_draw.flags = GekkoPhysics::DrawFlag_Shapes | GekkoPhysics::DrawFlag_AABBs;
        world.DrawDebug();

        // Pass 2: overlays (no depth testing — always on top)
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        debug_draw.flags = GekkoPhysics::DrawFlag_Contacts | GekkoPhysics::DrawFlag_BodyOrigins | GekkoPhysics::DrawFlag_BodyAxes;
        world.DrawDebug();
        rlDrawRenderBatchActive();
        rlEnableDepthTest();

        EndMode3D();

        DrawText("GekkoPhysics - Arrow keys: move | Space: jump", 10, 10, 20, DARKGRAY);
        DrawText(TextFormat("Contacts: %d", (int)world.GetContacts().size()), 10, 35, 20, RED);
        {
            auto pos = world.GetBody(controlled).position.AsFloat();
            DrawText(TextFormat("Pos: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z), 10, 60, 20, DARKGRAY);
        }
        auto& contacts = world.GetContacts();
        for (int i = 0; i < (int)contacts.size() && i < 4; i++) {
            auto p = contacts[i].point.AsFloat();
            DrawText(TextFormat("  C%d: (%.2f, %.2f, %.2f)", i, p.x, p.y, p.z), 10, 85 + i * 25, 20, RED);
        }
        DrawFPS(1500, 10);

        EndDrawing();
    }

    return 0;
}
